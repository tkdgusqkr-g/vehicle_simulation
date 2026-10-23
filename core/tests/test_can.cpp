#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "test_support.hpp"
#include "vsim/can/dbc.hpp"
#include "vsim/can/frame.hpp"
#include "vsim/can/network.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/input_hub.hpp"
#include "vsim/sim/rng.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

const std::string kExampleDbc = std::string(VSIM_SOURCE_DIR) + "/templates/projects/example_fs_ev/can/fs_ev.dbc";

// --- DBC ----------------------------------------------------------------------------------

TEST(Dbc, ParsesTheExampleNetwork) {
  const Dbc dbc = Dbc::from_file(kExampleDbc);
  EXPECT_EQ(dbc.nodes.size(), 8u);
  ASSERT_NE(dbc.message("VCU_TorqueCmd"), nullptr);
  const DbcMessage& m = *dbc.message("VCU_TorqueCmd");
  EXPECT_EQ(m.id, 0x100u);
  EXPECT_EQ(m.length, 8);
  EXPECT_EQ(m.transmitter, "VCU");
  EXPECT_DOUBLE_EQ(m.cycle_time, 0.010);
  EXPECT_EQ(m.comment, "Torque commands to the four in-wheel inverters (motor shaft torque).");
  ASSERT_EQ(m.signals.size(), 4u);
  const DbcSignal& s = m.signals[0];
  EXPECT_EQ(s.name, "TorqueFL");
  EXPECT_TRUE(s.is_signed);
  EXPECT_TRUE(s.little_endian);
  EXPECT_DOUBLE_EQ(s.factor, 0.01);
  EXPECT_EQ(s.unit, "N*m");
  EXPECT_EQ(s.receivers, std::vector<std::string>{"INV_FL"});
  EXPECT_EQ(dbc.message("INV_FL_Status")->signal("MotorTemp")->offset, -40.0);
  EXPECT_EQ(dbc.message(0x300, false)->name, "BMS_Status");
}

TEST(Dbc, EncodesIntelMotorolaSignedFloatAndMultiplexed) {
  const Dbc dbc = Dbc::parse(R"(
VERSION ""
BU_: A B
BO_ 2147484188 Ext: 8 A
 SG_ Intel : 0|16@1+ (1,0) [0|65535] "" B
 SG_ Moto : 23|16@0+ (1,0) [0|65535] "" B
 SG_ Neg : 32|12@1- (0.5,10) [0|0] "" B
 SG_ Mux M : 56|8@1+ (1,0) [0|255] "" B
 SG_ OnlyIn1 m1 : 44|12@1+ (1,0) [0|4095] "" B
BO_ 300 Floats: 8 A
 SG_ F : 0|32@1- (1,0) [0|0] "" B
SIG_VALTYPE_ 300 F : 1;
)");
  const DbcMessage& m = *dbc.message("Ext");
  EXPECT_TRUE(m.extended);
  EXPECT_EQ(m.id, 0x21Cu);
  std::vector<std::uint8_t> data(8, 0);
  m.signal("Intel")->write(data, 0x1234);
  m.signal("Moto")->write(data, 0xABCD);
  m.signal("Neg")->write(data, -100.0);  // raw (-100 - 10) / 0.5 = -220
  EXPECT_EQ(data[0], 0x34);
  EXPECT_EQ(data[1], 0x12);
  EXPECT_EQ(data[2], 0xAB);  // Motorola: MSB first from bit 23 (byte 2)
  EXPECT_EQ(data[3], 0xCD);
  EXPECT_DOUBLE_EQ(m.signal("Intel")->read(data), 0x1234);
  EXPECT_DOUBLE_EQ(m.signal("Moto")->read(data), 0xABCD);
  EXPECT_DOUBLE_EQ(m.signal("Neg")->read(data), -100.0);
  EXPECT_DOUBLE_EQ(m.signal("Intel")->decode(m.signal("Intel")->encode(1e9)), 65535.0);  // saturates
  EXPECT_TRUE(m.signal("Mux")->multiplexer);
  EXPECT_EQ(m.signal("OnlyIn1")->mux_value, 1);
  const DbcSignal& f = *dbc.message("Floats")->signal("F");
  EXPECT_EQ(f.kind, DbcSignal::Kind::Float32);
  std::vector<std::uint8_t> fd(8, 0);
  f.write(fd, -1.5);
  EXPECT_FLOAT_EQ(static_cast<float>(f.read(fd)), -1.5F);
}

TEST(Dbc, RoundTripsThroughText) {
  const Dbc a = Dbc::from_file(kExampleDbc);
  const Dbc b = Dbc::parse(a.to_text());
  ASSERT_EQ(a.messages.size(), b.messages.size());
  for (std::size_t i = 0; i < a.messages.size(); ++i) {
    const DbcMessage &x = a.messages[i], &y = b.messages[i];
    EXPECT_EQ(x.name, y.name);
    EXPECT_EQ(x.id, y.id);
    EXPECT_EQ(x.cycle_time, y.cycle_time);
    EXPECT_EQ(x.comment, y.comment);
    ASSERT_EQ(x.signals.size(), y.signals.size());
    for (std::size_t k = 0; k < x.signals.size(); ++k) {
      EXPECT_EQ(x.signals[k].start_bit, y.signals[k].start_bit);
      EXPECT_EQ(x.signals[k].factor, y.signals[k].factor);
      EXPECT_EQ(x.signals[k].offset, y.signals[k].offset);
      EXPECT_EQ(x.signals[k].unit, y.signals[k].unit);
      EXPECT_EQ(x.signals[k].receivers, y.signals[k].receivers);
    }
  }
}

TEST(Dbc, ReportsErrorsWithTheLine) {
  try {
    Dbc::parse("VERSION \"\"\nBU_: A\nBO_ 1 M: 2 A\n SG_ S : 8|16@1+ (1,0) [0|1] \"\" A\n", "bad.dbc");
    FAIL() << "no error";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("does not fit"), std::string::npos) << e.what();
  }
  EXPECT_THROW(Dbc::parse("BO_ 1 M: 8 A\n SG_ S : x|16@1+ (1,0) [0|1] \"\" A\n", "bad.dbc"), std::invalid_argument);
}

// --- Frames (V-08) --------------------------------------------------------------------------

// ID 0, no data: 34 bits from SOF to the CRC are all 0 (the CRC of zeros is
// 0), so a stuff bit after every fifth: 6; plus 13 bits of delimiters, ACK,
// EOF and interframe space: 53 bits.
TEST(CanFrame, AllZeroFrameMatchesTheHandCount) {
  CanFrame f;
  EXPECT_EQ(frame_bits(f).stuff, 6);
  EXPECT_EQ(frame_bits(f).nominal, 53);
  EXPECT_DOUBLE_EQ(frame_duration(f, 500e3, 2e6), 53.0 / 500e3);
}

TEST(CanFrame, StaysWithinTheTextbookBounds) {
  Rng rng(5);
  for (int i = 0; i < 2000; ++i) {
    CanFrame f;
    f.extended = i % 3 == 0;
    f.id = static_cast<std::uint32_t>(rng.next_u64() & (f.extended ? 0x1FFFFFFFU : 0x7FFU));
    f.data.resize(static_cast<std::size_t>(rng.next_u64() % 9));
    for (auto& b : f.data) {
      b = static_cast<std::uint8_t>(rng.next_u64() % 4 == 0 ? 0 : rng.next_u64());  // runs of zeros too
    }
    const int n = static_cast<int>(f.data.size());
    const int bits = frame_bits(f).nominal;
    EXPECT_GE(bits, (f.extended ? 67 : 47) + 8 * n);
    EXPECT_LE(bits, worst_case_frame_bits(n, f.extended));
  }
}

TEST(CanFrame, FdDataPhaseRunsAtTheDataRate) {
  CanFrame f;
  f.fd = true;
  f.data.assign(64, 0x55);
  const FrameBits b = frame_bits(f);
  EXPECT_GT(b.data, 64 * 8);
  EXPECT_LT(b.nominal, 40);
  EXPECT_NEAR(frame_duration(f, 1e6, 5e6), b.nominal / 1e6 + b.data / 5e6, 1e-15);
  f.brs = false;
  EXPECT_EQ(frame_bits(f).data, 0);  // all at the nominal rate
}

// --- A network on a hand-made bus -------------------------------------------------------

class TempDbc {
 public:
  explicit TempDbc(const std::string& text)
      : path_(std::filesystem::temp_directory_path() / ("vsim_test_" + std::to_string(counter_++) + ".dbc")) {
    std::ofstream(path_) << text;
  }
  ~TempDbc() { std::filesystem::remove(path_); }
  std::string path() const { return path_.string(); }

 private:
  static inline int counter_ = 0;
  std::filesystem::path path_;
};

class CanRig {
 public:
  CanRig(const std::string& yaml, const std::vector<std::string>& signals) {
    params_ = ParamTree::from_yaml_string(yaml);
    for (const std::string& s : signals) {
      bus_.declare(s, "", "");
    }
    ctx_ = std::make_unique<Context>(bus_, inputs_, RealismConfig::from_params(params_), 1, 0.001);
    net_.init(params_, *ctx_);
  }
  void run_for(double seconds) {
    for (auto n = std::lround(seconds / 0.001); n > 0; --n) {
      const double t = static_cast<double>(tick_++) * 0.001;
      net_.before_controllers(t, bus_);
      net_.after_controllers(t, bus_);
      net_.after_plant_inputs(bus_);
    }
  }
  double get(const std::string& name) const { return bus_.get(bus_.require(name)); }
  void set(const std::string& name, double v) { bus_.set(bus_.require(name), v); }
  nlohmann::json summary() const { return nlohmann::json::parse(net_.summary_json()); }

 private:
  ParamTree params_;
  SignalBus bus_;
  InputHub inputs_;
  std::unique_ptr<Context> ctx_;
  CanNetwork net_;
  long long tick_ = 0;
};

TEST(CanNetwork, ArbitrationSendsTheLowerIdFirst) {
  const TempDbc dbc(R"(
BU_: C D
BO_ 512 Late: 8 C
 SG_ X : 0|16@1+ (1,0) [0|65535] "" D
BO_ 256 Early: 8 C
 SG_ X : 0|16@1+ (1,0) [0|65535] "" D
BA_ "GenMsgCycleTime" BO_ 512 10;
BA_ "GenMsgCycleTime" BO_ 256 10;
)");
  CanRig rig("can: {fidelity: timing, buses: [{name: b, bitrate: 500000, dbc: ['" + dbc.path() +
                 "']}], nodes: [{name: C, role: controller}]}",
             {});
  rig.run_for(0.005);
  CanFrame f;
  f.data.assign(8, 0);
  f.id = 0x100;
  const double early = frame_duration(f, 500e3, 2e6);
  f.id = 0x200;
  const double late = frame_duration(f, 500e3, 2e6);
  EXPECT_NEAR(rig.get("can.Early.latency"), early, 1e-12);
  EXPECT_NEAR(rig.get("can.Late.latency"), early + late, 1e-12);  // waited for Early
}

// V-08: the bus load is the frames' bits over the bit rate.
TEST(CanNetwork, BusLoadMatchesTheFrameTimes) {
  CanRig rig("can: {fidelity: timing, buses: [{name: main, bitrate: 1000000, dbc: ['" + kExampleDbc + "']}]}", {});
  rig.run_for(2.0);
  // Expected from the messages' cycles with all-zero payloads (what is sent
  // without a map), frame by frame.
  const Dbc dbc = Dbc::from_file(kExampleDbc);
  double expected = 0.0;
  for (const DbcMessage& m : dbc.messages) {
    CanFrame f;
    f.id = m.id;
    f.data.assign(static_cast<std::size_t>(m.length), 0);
    expected += frame_duration(f, 1e6, 2e6) / m.cycle_time;
  }
  const auto s = rig.summary();
  EXPECT_NEAR(s["buses"]["main"]["load_mean"].get<double>(), expected, 0.01 * expected);
  EXPECT_EQ(s["buses"]["main"]["frames"], 2 * (7 * 100 + 10 + 2 * 50));
}

// --- On the example car ----------------------------------------------------------------

Scenario::Overrides with_can(Scenario::Overrides o) {
  o.emplace("can.enabled", "true");
  return o;
}

TEST(Can, IdealBusIsTransparent) {
  Simulation plain(load_example("acceleration"));
  Simulation can(load_example("acceleration", with_can({{"can.fidelity", "ideal"}})));
  plain.run_for(3.0);
  can.run_for(3.0);
  EXPECT_NEAR(signal(can, "veh.chassis.vx"), signal(plain, "veh.chassis.vx"), 0.01 * signal(plain, "veh.chassis.vx"));
}

// The plant gets its torque command only from the frames: with the frames
// gone the inverters time out and fall back to zero torque.
TEST(Can, TorqueCommandsReachThePlantOnlyThroughTheBus) {
  Simulation sim(load_example("acceleration", with_can({{"can.fidelity", "faults"},
                                                        {"can.faults.0.type", "drop"},
                                                        {"can.faults.0.message", "VCU_TorqueCmd"},
                                                        {"can.faults.0.at_s", "1.0"}})));
  sim.run_for(1.0);
  EXPECT_GT(signal(sim, "pt.motor.fl.torque"), 5.0);
  EXPECT_EQ(signal(sim, "can.VCU_TorqueCmd.timeout"), 0.0);
  sim.run_for(0.04);  // within the 50 ms timeout: the last command holds
  EXPECT_GT(signal(sim, "pt.motor.fl.torque"), 5.0);
  sim.run_for(0.03);
  EXPECT_EQ(signal(sim, "can.VCU_TorqueCmd.timeout"), 1.0);
  EXPECT_LT(std::abs(signal(sim, "pt.motor.fl.torque")), 0.5);
  EXPECT_GT(signal(sim, "pt.motor.fl.torque_cmd"), 5.0);  // what the VCU still asks for
  EXPECT_GT(signal(sim, "can.VCU_TorqueCmd.lost"), 3.0);
}

// PH-08: more traffic with higher priority delays the torque commands; past
// the bus's capacity they starve, the inverters time out and the car slows.
TEST(Can, BusyBusDelaysTheTorqueCommands) {
  const auto run = [](int load_frames) {
    Scenario::Overrides o = with_can({{"can.fidelity", "timing"}});
    if (load_frames > 0) {
      o.emplace("can.load.0.id", "0x050");
      o.emplace("can.load.0.count", std::to_string(load_frames));
      o.emplace("can.load.0.period_ms", "1");
    }
    SimulationOptions options;
    options.log_dir = std::filesystem::temp_directory_path() / "vsim_can_busy";
    Simulation sim(load_example("acceleration", o), options);
    sim.run_for(3.0);
    sim.finish();
    nlohmann::json s;
    {
      std::ifstream in(sim.run_path() / "summary.json");
      s = nlohmann::json::parse(in);
    }  // closed before the folder goes (Windows keeps open files)
    std::filesystem::remove_all(options.log_dir);
    return std::pair{s["can"], signal(sim, "veh.chassis.vx")};
  };
  const auto [quiet, v_quiet] = run(0);
  const auto [busy, v_busy] = run(6);      // about 85 % load
  const auto [jammed, v_jammed] = run(9);  // more than the bus can carry
  const auto max_latency = [](const nlohmann::json& c) {
    return c["messages"]["VCU_TorqueCmd"]["latency_ms"]["max"].get<double>();
  };
  EXPECT_LT(quiet["buses"]["can_main"]["load_mean"].get<double>(), 0.12);
  EXPECT_GT(busy["buses"]["can_main"]["load_mean"].get<double>(), 0.8);
  EXPECT_GT(max_latency(busy), 2.0 * max_latency(quiet));
  EXPECT_GT(jammed["messages"]["VCU_TorqueCmd"]["timeouts"].get<int>(), 0);
  EXPECT_LT(v_jammed, 0.5 * v_quiet);
  EXPECT_NEAR(v_busy, v_quiet, 0.05 * v_quiet);  // late, but they get through
}

// PH-09: with the inverter's frames arriving close to the VCU's task
// boundary, clock drift and task jitter make task runs find no frame or two.
TEST(Can, ClockDriftAndJitterMakeTasksMissFrames) {
  const auto counts = [](const Scenario::Overrides& extra) {
    Scenario::Overrides o = with_can({{"can.fidelity", "timing"}});
    for (const auto& [k, v] : extra) {
      o.insert_or_assign(k, v);
    }
    Simulation sim(load_example("skidpad_auto", o));
    sim.run_for(4.0);
    return std::pair{signal(sim, "can.INV_FL_Status.missed"), signal(sim, "can.INV_FL_Status.doubled")};
  };
  const auto [missed0, doubled0] = counts({{"can.nodes.0.jitter_us", "0"},
                                           {"can.nodes.0.clock_ppm", "0"},
                                           {"can.nodes.1.clock_ppm", "0"},
                                           {"can.nodes.1.jitter_us", "0"},
                                           {"can.nodes.1.offset_ms", "3"}});
  EXPECT_EQ(missed0, 0.0);
  EXPECT_EQ(doubled0, 0.0);
  const auto [missed, doubled] = counts({{"can.nodes.1.offset_ms", "9.75"}});  // frames land near 10 ms
  EXPECT_GT(missed, 5.0);
  EXPECT_GT(doubled, 5.0);
}

TEST(Can, BusOffNodeSilencesItsFramesUntilItRecovers) {
  Simulation sim(load_example("skidpad_auto", with_can({{"can.fidelity", "faults"},
                                                        {"can.faults.0.type", "bus_off"},
                                                        {"can.faults.0.node", "INV_FL"},
                                                        {"can.faults.0.at_s", "1.0"},
                                                        {"can.faults.0.until_s", "1.5"}})));
  sim.run_for(1.2);
  EXPECT_EQ(signal(sim, "can.INV_FL.bus_off"), 1.0);
  EXPECT_EQ(signal(sim, "can.INV_FL_Status.timeout"), 1.0);
  EXPECT_EQ(signal(sim, "can.INV_FR_Status.timeout"), 0.0);
  sim.run_for(0.5);
  EXPECT_EQ(signal(sim, "can.INV_FL.bus_off"), 0.0);
  EXPECT_EQ(signal(sim, "can.INV_FL_Status.timeout"), 0.0);
}

TEST(Can, ErrorFramesAreRetransmittedAndLogged) {
  SimulationOptions options;
  options.log_dir = std::filesystem::temp_directory_path() / "vsim_can_errors";
  std::filesystem::remove_all(options.log_dir);
  {
    Simulation sim(load_example("acceleration", with_can({{"can.fidelity", "faults"},
                                                          {"can.log", "both"},
                                                          {"can.faults.0.type", "error_rate"},
                                                          {"can.faults.0.bus", "can_main"},
                                                          {"can.faults.0.probability", "0.1"}})),
                   options);
    sim.run_for(2.0);
    sim.finish();
    std::ifstream in(sim.run_path() / "summary.json");
    const auto s = nlohmann::json::parse(in)["can"];
    EXPECT_GT(s["buses"]["can_main"]["error_frames"].get<int>(), 100);
    const auto& torque = s["messages"]["VCU_TorqueCmd"];
    EXPECT_GE(torque["delivered"].get<int>(), torque["sent"].get<int>() - 1);  // retried, not lost
    std::ifstream asc(sim.run_path() / "can_main.asc"), candump(sim.run_path() / "can_main.log");
    const auto lines = [](std::ifstream& f, const std::string& has) {
      int n = 0;
      for (std::string line; std::getline(f, line);) {
        n += line.find(has) != std::string::npos ? 1 : 0;
      }
      return n;
    };
    EXPECT_EQ(lines(asc, " Rx "), s["buses"]["can_main"]["frames"].get<int>());
    EXPECT_EQ(lines(candump, "can_main"), s["buses"]["can_main"]["frames"].get<int>());
  }
  std::filesystem::remove_all(options.log_dir);
}

TEST(Can, ReportsConfigurationMistakes) {
  const auto message = [](const Scenario::Overrides& o) {
    try {
      Simulation sim(load_example("acceleration", with_can(o)));
    } catch (const std::exception& e) {
      return std::string(e.what());
    }
    return std::string("no error");
  };
  EXPECT_NE(message({{"can.map.NoSuch.X", "veh.chassis.vx"}}).find("no message 'NoSuch'"), std::string::npos);
  EXPECT_NE(message({{"can.map.VCU_TorqueCmd.Nope", "veh.chassis.vx"}}).find("no signal 'Nope'"), std::string::npos);
  EXPECT_NE(message({{"can.map.VCU_TorqueCmd.TorqueFL", "veh.chassis.vx"}}).find("cannot convert"),
            std::string::npos);
  EXPECT_NE(message({{"can.fidelity", "perfect"}}).find("can.fidelity"), std::string::npos);
  EXPECT_NE(message({{"can.faults.0.type", "melt"}}).find("expected drop"), std::string::npos);
}

TEST(Can, IsDeterministic) {
  const auto run = [] {
    Simulation sim(load_example("acceleration", with_can({{"can.fidelity", "faults"},
                                                          {"can.faults.0.type", "drop"},
                                                          {"can.faults.0.probability", "0.2"}})));
    sim.run_for(1.0);
    return std::vector<double>{signal(sim, "veh.chassis.vx"), signal(sim, "can.VCU_TorqueCmd.lost"),
                               signal(sim, "can.INV_FL_Status.latency")};
  };
  EXPECT_EQ(run(), run());
}

}  // namespace
}  // namespace vsim

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "test_support.hpp"
#include "vsim/plugin/plugin.hpp"
#include "vsim/plugin/units.hpp"
#include "../src/util/tcp.hpp"

#define VSIM_EXTERNAL_IMPLEMENTATION
#include "vsim_external.h"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

std::vector<double> column(const Simulation& sim, const std::string& name) {
  const auto& names = sim.recorder().names();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    throw std::out_of_range("not logged: " + name);
  }
  return sim.recorder().columns()[static_cast<std::size_t>(it - names.begin())];
}

TEST(PluginUnits, ConvertBetweenUnitsOfOneKind) {
  EXPECT_NEAR(unit_conversion("rpm", "rad/s").apply(60.0), 2.0 * std::numbers::pi, 1e-12);
  EXPECT_NEAR(unit_conversion("rad/s", "rpm").apply(2.0 * std::numbers::pi), 60.0, 1e-12);
  EXPECT_NEAR(unit_conversion("km/h", "m/s").apply(36.0), 10.0, 1e-12);
  EXPECT_NEAR(unit_conversion("degC", "K").apply(25.0), 298.15, 1e-12);
  EXPECT_NEAR(unit_conversion("K", "degC").apply(300.0), 26.85, 1e-12);
  EXPECT_NEAR(unit_conversion("bar", "kPa").apply(1.5), 150.0, 1e-12);
  EXPECT_NEAR(unit_conversion("%", "-").apply(25.0), 0.25, 1e-12);
  EXPECT_NEAR(unit_conversion("N.m", "N*m").apply(3.0), 3.0, 0.0);
  EXPECT_TRUE(unit_conversion("", "rad/s").identity());
  EXPECT_TRUE(unit_conversion("furlong", "furlong").identity());
  EXPECT_THROW(unit_conversion("m/s", "rad/s"), std::invalid_argument);
  EXPECT_THROW(unit_conversion("furlong", "m"), std::invalid_argument);
}

TEST(PluginComponents, ReadSlotsOrderAndSwitches) {
  const ParamTree params = ParamTree::from_yaml_string(R"(
project: {dir: /tmp}
components:
  vcu: {type: native, path: x}
  b: {type: fmu, path: y, after: sensors, order: 2}
  a: {type: fmu, path: y, after: sensors, order: 1}
  off: {type: native, path: z, enabled: false}
)");
  const std::vector<ComponentSpec> c = components_from_params(params);
  ASSERT_EQ(c.size(), 3u);
  EXPECT_EQ(c[0].name, "vcu");
  EXPECT_EQ(c[0].slot, "vcu");
  EXPECT_EQ(c[1].name, "a");
  EXPECT_EQ(c[2].name, "b");
  EXPECT_EQ(c[1].after, "sensors");
  EXPECT_THROW(components_from_params(ParamTree::from_yaml_string("components: {x: {type: native, slot: brakes}}")),
               std::invalid_argument);
}

TEST(PluginComponents, ParamsBecomeJson) {
  const ParamTree params = ParamTree::from_yaml_string(R"(
components:
  x:
    params: {gain: 40, name: front, on: true, table: [1, 2.5], nested: {k: 3}}
)");
  ComponentSpec spec;
  spec.prefix = "components.x";
  const auto json = nlohmann::json::parse(component_params_json(params, spec));
  EXPECT_EQ(json["gain"], 40.0);
  EXPECT_EQ(json["name"], "front");
  EXPECT_EQ(json["on"], true);
  EXPECT_EQ(json["table"][1], 2.5);
  EXPECT_EQ(json["nested"]["k"], 3.0);
}

// A VCU in C takes the vcu slot: no regen, no slip control, a linear map.
TEST(Plugins, NativeVcuDrivesTheCar) {
  Simulation sim(load_example("acceleration", {{"components.simple_vcu.enabled", "true"},
                                               {"logging.signals", ""},
                                               {"logging.signals.0", "veh.*"}}));
  ASSERT_EQ(sim.plugins().size(), 1u);
  const PluginBlock& vcu = *sim.plugins().front();
  EXPECT_EQ(vcu.name(), "simple_vcu");
  EXPECT_EQ(vcu.info().type, "native");
  EXPECT_EQ(vcu.info().name, "simple_vcu");
  EXPECT_EQ(vcu.sha256().size(), 64u);
  EXPECT_FALSE(sim.bus().find("vcu.regen").has_value());  // the reference VCU is gone
  sim.run_for(2.0);
  EXPECT_EQ(signal(sim, "pt.motor.fl.torque_cmd"), 21.0);
  EXPECT_GT(signal(sim, "veh.chassis.vx"), 15.0);
  EXPECT_EQ(vcu.stats().calls, 200);  // t = 0, 0.01, ..., 1.99
}

// The same torque vectoring law as a C++ library and as an FMU: identical runs.
TEST(Plugins, NativeAndFmuTorqueVectoringAgree) {
  const auto run = [](const char* component) {
    Simulation sim(load_example("skidpad_auto", {{std::string("components.") + component + ".enabled", "true"},
                                                 {"logging.signals", ""},
                                                 {"logging.signals.0", "veh.chassis.*"},
                                                 {"logging.signals.1", "tv.*"},
                                                 {"logging.signals.2", "pt.motor.*"}}));
    sim.run_for(8.0);
    return std::pair{column(sim, "veh.chassis.yaw_rate"), column(sim, "tv.shift")};
  };
  const auto [r_native, shift_native] = run("tv_native");
  const auto [r_fmu, shift_fmu] = run("tv_fmu");
  EXPECT_EQ(r_native, r_fmu);
  EXPECT_EQ(shift_native, shift_fmu);
  EXPECT_GT(std::abs(*std::max_element(shift_native.begin(), shift_native.end(),
                                       [](double a, double b) { return std::abs(a) < std::abs(b); })),
            0.1);

  Simulation plain(load_example("skidpad_auto", {{"logging.signals", ""}, {"logging.signals.0", "veh.chassis.*"}}));
  plain.run_for(8.0);
  EXPECT_NE(column(plain, "veh.chassis.yaw_rate"), r_native);  // the plugin changed the run
}

TEST(Plugins, FmuParametersAndConstantInputs) {
  Simulation sim(load_example("skidpad_auto", {{"components.tv_fmu.enabled", "true"},
                                               {"components.tv_fmu.params.max_shift", "0"},
                                               {"components.tv_fmu.ports.in.tq_fl_in", ""},
                                               {"components.tv_fmu.ports.in.tq_fl_in.value", "5"}}));
  sim.run_for(1.0);
  EXPECT_EQ(signal(sim, "tv.shift"), 0.0);
  EXPECT_EQ(signal(sim, "pt.motor.fl.torque_cmd"), 5.0);
  const PluginBlock& p = *sim.plugins().front();
  EXPECT_EQ(p.info().type, "fmu");
  EXPECT_EQ(p.info().name, "tv_fmu");
  const auto& c = p.connections();
  const auto shift = std::find_if(c.begin(), c.end(), [](const auto& x) { return x.port == "shift"; });
  ASSERT_NE(shift, c.end());
  EXPECT_EQ(shift->unit, "N.m");  // from the FMU's declared type
}

TEST(Plugins, ReportProjectMistakes) {
  const auto load = [](const Scenario::Overrides& o) {
    Scenario::Overrides all = o;
    all.emplace("components.tv_native.enabled", "true");
    return Simulation(load_example("skidpad_auto", all));
  };
  const auto message = [&](const Scenario::Overrides& o) {
    try {
      load(o);
    } catch (const std::exception& e) {
      return std::string(e.what());
    }
    return std::string("no error");
  };
  EXPECT_NE(message({{"components.tv_native.ports.in.stear", "drv.cmd.steer"}}).find("no input port 'stear'"),
            std::string::npos);
  EXPECT_NE(message({{"components.tv_native.ports.in.vx", "veh.chassis.yaw_rate"}}).find("cannot convert"),
            std::string::npos);
  EXPECT_NE(message({{"components.tv_native.ports.in.vx", "no.such.signal"}}).find("no signal"), std::string::npos);
  EXPECT_NE(message({{"components.tv_native.ports.in.vx", ""}}).find("not connected: vx"), std::string::npos);
  EXPECT_NE(message({{"components.tv_native.path", "missing_plugin"}}).find("not found"), std::string::npos);
  EXPECT_NE(message({{"components.tv_native.type", "lua"}}).find("not available"), std::string::npos);
  EXPECT_NE(message({{"components.tv_fmu.enabled", "true"}, {"components.tv_fmu.slot", "vcu"},
                     {"components.simple_vcu.enabled", "true"}})
                .find("both"),
            std::string::npos);
}

TEST(Plugins, RecordedWithTheRun) {
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / "vsim_plugin_meta";
  std::filesystem::remove_all(dir);
  {
    SimulationOptions options;
    options.log_dir = dir;
    Simulation sim(load_example("skidpad_auto", {{"components.tv_native.enabled", "true"}}), options);
    sim.run_for(0.5);
    sim.finish();
    std::ifstream meta(sim.run_path() / "meta.json");
    const auto m = nlohmann::json::parse(meta);
    EXPECT_EQ(m["plugins"]["tv_native"]["type"], "native");
    EXPECT_EQ(m["plugins"]["tv_native"]["sha256"].get<std::string>().size(), 64u);
    EXPECT_EQ(m["plugins"]["tv_native"]["ports"]["in"]["yaw_rate"], "sens.imu.gz");
    std::ifstream summary(sim.run_path() / "summary.json");
    const auto s = nlohmann::json::parse(summary);
    EXPECT_EQ(s["plugins"]["tv_native"]["calls"], 50);
  }
  std::filesystem::remove_all(dir);
}

// What vsim_export builds from Embedded Coder output (sdk/simulink), on a
// stand-in model: typed root ports (double, single, boolean, int32) and the
// units the model's ports declare (%, rpm) converted from the bus.
TEST(Plugins, EmbeddedCoderWrapper) {
  Simulation sim(load_example("acceleration", {{"components.ert.type", "native"},
                                               {"components.ert.path", "fake_ert_vcu"},
                                               {"components.ert.ports.in.APPS", "drv.cmd.throttle"},
                                               {"components.ert.ports.in.WheelSpeedFL", "veh.wheel.fl.omega"},
                                               {"components.ert.ports.in.Enable.value", "1"}}));
  ASSERT_EQ(sim.plugins().size(), 1u);
  EXPECT_DOUBLE_EQ(sim.plugins().front()->period(), 0.01);  // the model's sample time
  sim.run_for(1.0);
  EXPECT_DOUBLE_EQ(signal(sim, "plugin.ert.TorqueCmd"), 21.0);  // 0.21 N*m per % at 100 %
  EXPECT_EQ(signal(sim, "plugin.ert.Count"), 100.0);
  const double rpm = signal(sim, "veh.wheel.fl.omega") * 60.0 / (2.0 * std::numbers::pi);
  EXPECT_NEAR(signal(sim, "plugin.ert.WheelSpeedEcho"), rpm, 0.02 * rpm);
  EXPECT_EQ(sim.bus().info(sim.bus().require("plugin.ert.WheelSpeedEcho")).unit, "rpm");

  // Generated code keeps its data in globals: a second instance is refused,
  // and a period other than the model's sample time too.
  EXPECT_THROW(Simulation(load_example("acceleration", {{"components.ert.type", "native"},
                                                        {"components.ert.path", "fake_ert_vcu"},
                                                        {"components.ert.period", "0.02"},
                                                        {"components.ert.ports.in.APPS", "drv.cmd.throttle"},
                                                        {"components.ert.ports.in.WheelSpeedFL.value", "0"},
                                                        {"components.ert.ports.in.Enable.value", "1"}})),
               std::runtime_error);
}

TEST(PluginHash, Sha256OfKnownText) {
  const std::filesystem::path file = std::filesystem::temp_directory_path() / "vsim_sha256.txt";
  {
    std::ofstream out(file, std::ios::binary);
    out << "abc";
  }
  EXPECT_EQ(sha256_file(file), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  std::filesystem::remove(file);
}


// An external component (sdk/c/include/vsim_external.h) in lockstep: here a
// thread with the C client; in practice an MCU emulator or a test bench.
TEST(Plugins, ExternalComponentInLockstep) {
  int port = 0;
  {
    TcpListener probe("127.0.0.1", 0);  // a free port
    port = probe.port();
  }
  std::vector<double> times;
  std::string setup;
  std::thread program([&] {
    VsimExternal link;
    const VsimExternalPort ports[] = {{"speed", "km/h", 1}, {"pedal", "-", 1}, {"twice", "km/h", 0}};
    const std::string address = "127.0.0.1:" + std::to_string(port);
    for (int attempt = 0; attempt < 500; ++attempt) {  // until vsim listens
      if (vsim_external_connect(&link, address.c_str(), "c_client", "0.1", ports, 3) == 0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    setup = link.setup;
    double t = 0.0, in[2] = {0.0, 0.0}, out[1] = {0.0};
    while (vsim_external_next(&link, &t, in) == 1) {
      times.push_back(t);
      out[0] = 2.0 * in[0];
      if (vsim_external_reply(&link, out) != 0) {
        break;
      }
    }
    vsim_external_close(&link);
  });
  {
    Simulation sim(load_example(
        "acceleration",
        {{"components.bench", "{type: external, after: vcu, period: 0.01, listen: \"127.0.0.1:" + std::to_string(port) +
                                  "\", params: {gain: 2}, ports: {in: {speed: veh.chassis.speed, pedal: drv.cmd.throttle},"
                                  " out: {twice: ext.twice_speed}}}"}}));
    ASSERT_EQ(sim.plugins().size(), 1u);
    EXPECT_EQ(sim.plugins().front()->info().type, "external");
    EXPECT_EQ(sim.plugins().front()->info().name, "c_client");
    sim.run_for(1.0);
    // Its input went out in km/h; its output (a new signal, in the port's unit) came back one
    // period behind.
    EXPECT_NEAR(signal(sim, "ext.twice_speed"), 2.0 * 3.6 * signal(sim, "veh.chassis.speed"), 2.0);
    EXPECT_EQ(sim.bus().info(sim.bus().require("ext.twice_speed")).unit, "km/h");
  }  // the run ends: END, and the program's loop returns
  program.join();
  ASSERT_EQ(times.size(), 100u);
  EXPECT_DOUBLE_EQ(times[1], 0.01);
  const nlohmann::json s = nlohmann::json::parse(setup);
  EXPECT_EQ(s["component"], "bench");
  EXPECT_DOUBLE_EQ(s["period"].get<double>(), 0.01);
  EXPECT_EQ(s["params"]["gain"], 2);
  EXPECT_EQ(s["inputs"], nlohmann::json::array({"speed", "pedal"}));
}

}  // namespace
}  // namespace vsim

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "test_support.hpp"
#include "vsim/elec/circuit.hpp"
#include "vsim/elec/elements.hpp"
#include "vsim/elec/network.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/input_hub.hpp"
#include "vsim/sim/params.hpp"

namespace vsim {
namespace {

using elec::Circuit;
using elec::kGround;
using test::load_example;
using test::signal;

// --- Circuit solver: analytic solutions (V-07) ---------------------------------------------

elec::Step transient(double t, double h, bool euler = false) {
  elec::Step s;
  s.t = t;
  s.h = h;
  s.euler = euler;
  return s;
}

TEST(Circuit, SeriesResistorsDropExactly) {
  Circuit c;
  c.gmin = 1e-12;
  const auto top = c.node("TOP"), mid = c.node("MID");
  c.emplace<elec::VoltageSource>("V", top, kGround, 12.0, 0.01);
  c.emplace<elec::Resistor>("R1", top, mid, 2.0);
  c.emplace<elec::Resistor>("R2", mid, kGround, 4.0);
  ASSERT_TRUE(c.operating_point(0.0).converged);
  const double i = 12.0 / 6.01;
  EXPECT_NEAR(c.voltage(top), 12.0 - 0.01 * i, 1e-9);
  EXPECT_NEAR(c.voltage(mid), 4.0 * i, 1e-9);
}

// Precharge: v(t) = V (1 - exp(-t / RC)) through the precharge resistor.
double precharge_error(double h) {
  Circuit c;
  c.gmin = 1e-12;
  const auto pack = c.node("PACK"), link = c.node("LINK"), mid = c.node("MID");
  c.emplace<elec::VoltageSource>("ACC", pack, kGround, 600.0, 1e-3);
  auto& relay = c.emplace<elec::Switch>("PRE_K", pack, mid, 1e-3);
  c.emplace<elec::Resistor>("PRE_R", mid, link, 600.0);
  auto& cap = c.emplace<elec::Capacitor>("C_LINK", link, kGround, 600e-6, 0.0);
  c.operating_point(0.0);
  cap.vc = 0.0;  // an empty DC link (the open relay leaves it undefined at DC)
  relay.closed = true;
  c.invalidate();
  const double tau = (600.0 + 2e-3) * 600e-6;
  double worst = 0.0;
  const int steps = static_cast<int>(std::lround(2.0 / h));
  for (int k = 1; k <= steps; ++k) {
    const double t = k * h;
    EXPECT_TRUE(c.solve(transient(t, h, k == 1)).converged);
    worst = std::max(worst, std::abs(c.voltage(link) - 600.0 * (1.0 - std::exp(-t / tau))));
  }
  return worst / 600.0;
}

TEST(Circuit, PrechargeFollowsTheRcExponential) {
  const double coarse = precharge_error(1e-3);
  const double fine = precharge_error(0.5e-3);
  EXPECT_LT(coarse, 2e-3);  // the backward-Euler first step dominates the error
  EXPECT_LT(fine, 0.6 * coarse);
}

TEST(Circuit, TrapezoidalIsSecondOrderAwayFromTheSwitchingInstant) {
  // Smooth drive (no discontinuity): sinusoidal source into an RC.
  const auto error = [](double h) {
    Circuit c;
    c.gmin = 1e-12;
    const auto in = c.node("IN"), out = c.node("OUT");
    auto& src = c.emplace<elec::VoltageSource>("V", in, kGround, 0.0, 1e-6);
    c.emplace<elec::Resistor>("R", in, out, 100.0);
    auto& cap = c.emplace<elec::Capacitor>("C", out, kGround, 1e-3, 0.0);
    const double w = 10.0, rc = 100.0 * 1e-3;
    // Steady-state response of v' = (sin(wt) - v) / RC.
    const auto exact = [&](double t) {
      const double k = 1.0 / (1.0 + w * w * rc * rc);
      return k * (std::sin(w * t) - w * rc * std::cos(w * t));
    };
    c.operating_point(0.0);
    cap.vc = exact(0.0);
    cap.i = 1e-3 * w / (1.0 + w * w * rc * rc);  // C dv/dt at t = 0
    double worst = 0.0;
    const int steps = static_cast<int>(std::lround(1.0 / h));
    for (int k = 1; k <= steps; ++k) {
      src.e = std::sin(w * k * h);
      c.solve(transient(k * h, h));
      worst = std::max(worst, std::abs(c.voltage(out) - exact(k * h)));
    }
    return worst;
  };
  const double e1 = error(2e-3), e2 = error(1e-3);
  EXPECT_GT(e1 / e2, 3.5);
  EXPECT_LT(e2, 1e-4);
}

TEST(Circuit, InductorCurrentRisesWithLOverR) {
  Circuit c;
  c.gmin = 1e-12;
  const auto top = c.node("TOP");
  auto& src = c.emplace<elec::VoltageSource>("V", top, kGround, 0.0, 1e-6);
  auto& l = c.emplace<elec::Inductor>("L", top, kGround, 0.1, 2.0);
  c.operating_point(0.0);
  src.e = 12.0;
  c.invalidate();
  for (int k = 1; k <= 200; ++k) {
    const double t = k * 1e-3;
    c.solve(transient(t, 1e-3, k == 1));
    EXPECT_NEAR(l.i, 6.0 * (1.0 - std::exp(-t * 20.0)), 0.02);
  }
}

TEST(Circuit, ConstantPowerLoadDrawsItsPower) {
  Circuit c;
  const auto bus = c.node("DC");
  c.emplace<elec::VoltageSource>("ACC", bus, kGround, 600.0, 0.1);
  auto& load = c.emplace<elec::Load>("INV", bus, kGround, elec::Load::Law::Power, 30e3);
  load.v_knee = 300.0;  // resistive below: Newton from 0 V reaches the high-voltage solution
  ASSERT_TRUE(c.operating_point(0.0).converged);
  const double v = 0.5 * (600.0 + std::sqrt(600.0 * 600.0 - 4.0 * 0.1 * 30e3));
  EXPECT_NEAR(c.voltage(bus), v, 1e-3);
  EXPECT_NEAR(c.voltage(bus) * load.current(c.view()), 30e3, 1.0);
}

TEST(Circuit, DiodeConductsOneWayAndRegulatorDropsOutAndLimits) {
  Circuit c;
  const auto in = c.node("IN"), out = c.node("OUT"), k = c.node("K");
  auto& src = c.emplace<elec::VoltageSource>("V", in, kGround, 12.0, 1e-3);
  auto& reg = c.emplace<elec::Regulator>("REG", in, out, kGround, 5.0);
  reg.dropout = 1.2;
  reg.i_max = 0.4;
  auto& load = c.emplace<elec::Resistor>("RL", out, kGround, 100.0);
  c.emplace<elec::Diode>("D", in, k, 0.7, 0.01);
  c.emplace<elec::Resistor>("RK", k, kGround, 1000.0);
  ASSERT_TRUE(c.operating_point(0.0).converged);
  EXPECT_NEAR(c.voltage(out), 5.0 * 100.0 / (100.0 + reg.r_out), 1e-3);
  EXPECT_NEAR(c.voltage(k), 12.0 - 0.7 - 0.01 * (11.3 / 1000.0), 0.01);
  src.e = 5.5;  // headroom 4.3 V
  ASSERT_TRUE(c.operating_point(0.0).converged);
  EXPECT_NEAR(c.voltage(out), 4.3 * 100.0 / (100.0 + reg.r_out), 0.01);
  src.e = -5.0;  // reversed: the diode blocks
  ASSERT_TRUE(c.operating_point(0.0).converged);
  EXPECT_NEAR(c.voltage(k), 0.0, 1e-3);
  src.e = 12.0;
  load.r = 5.0;  // 1 A wanted, 0.4 A allowed
  ASSERT_TRUE(c.operating_point(0.0).converged);
  EXPECT_NEAR(c.voltage(out), 0.4 * 5.0, 1e-3);
}

// --- Network devices -------------------------------------------------------------------------

struct Rig {
  SignalBus bus;
  InputHub inputs;
  std::unique_ptr<Context> ctx;
  ElectricalNetwork net;
  double t = 0.0;

  explicit Rig(const std::string& yaml, const std::vector<std::string>& signals = {}) {
    const ParamTree p = ParamTree::from_yaml_string(yaml);
    ctx = std::make_unique<Context>(bus, inputs, RealismConfig::from_params(p), 7, 1e-3);
    for (const std::string& s : signals) {
      bus.declare(s, "-");
    }
    net.init(p, *ctx);
  }
  void run(double until) {
    for (; t < until - 1e-9; t += 1e-3) {
      net.step(t, bus);
    }
  }
  double get(const std::string& name) const { return bus.get(bus.require(name)); }
  void set(const std::string& name, double v) { bus.set(bus.require(name), v); }
  nlohmann::json summary() const { return nlohmann::json::parse(net.summary_json()); }
};

// Time a fuse needs at a constant current: the melting integral with cooling.
double fuse_time(double i, double rating) {
  const double i2t = 1.6 * rating * rating, tau = i2t / std::pow(1.1 * rating, 2.0);
  return -tau * std::log(1.0 - i2t / (i * i * tau));
}

TEST(Elec, FuseBlowsAfterItsMeltingIntegral) {
  for (const double amps : {100.0, 15.0}) {
    Rig rig(R"(
electrical:
  components:
    - {id: SRC, type: voltage_source, pos: V, neg: GND, v: 12, r_ohm: 0.0001}
    - {id: F, type: fuse, a: V, b: L, rating_a: 10, r_ohm: 0.0001}
    - {id: LOAD, type: resistor, a: L, b: GND, r_ohm: )" + std::to_string(12.0 / amps - 0.0003) + "}\n");
    const double expected = fuse_time(amps, 10.0);
    rig.run(expected * 0.95);
    EXPECT_EQ(rig.get("elec.F.blown"), 0.0) << amps;
    rig.run(expected * 1.05 + 0.002);
    EXPECT_EQ(rig.get("elec.F.blown"), 1.0) << amps;
    EXPECT_NEAR(rig.get("elec.LOAD.i"), 0.0, 1e-6);
    EXPECT_EQ(rig.summary()["events"][0]["type"], "fuse_blown");
  }
}

// A small tractive system: pack, AIRs, precharge relay and resistor, DC link,
// shutdown supply and the precharge controller.
std::string tractive(const std::string& extra, const std::string& pcc = "threshold: 0.95") {
  return R"(
electrical:
  initial: off
  components:
    - {id: ACC, type: voltage_source, pos: ACC+, neg: ACC-, v: 600, r_ohm: 0.05}
    - {id: AIR_P, type: contactor, a: ACC+, b: TS+, coil_a: AIRP, coil_b: GND}
    - {id: AIR_N, type: contactor, a: ACC-, b: TS-, coil_a: SDC, coil_b: GND}
    - {id: PRE_K, type: relay, a: ACC+, b: PRE, coil_a: PREC, coil_b: GND}
    - {id: PRE_R, type: resistor, a: PRE, b: TS+, r_ohm: 600}
    - {id: C_LINK, type: capacitor, a: TS+, b: TS-, c_f: 600e-6}
    - {id: GLV, type: voltage_source, pos: LV, neg: GND, v: 12, r_ohm: 0.02}
    - {id: TSMS, type: switch, a: LV, b: SDC}
    - {id: PCC, type: precharge, supply: SDC, pack: [ACC+, ACC-], link: [TS+, TS-],
       pre_out: [SDC, PREC], air_out: [SDC, AIRP], air_relay: AIR_P, )" +
         pcc + "}\n" + extra;
}

TEST(Elec, PrechargeSequenceClosesTheAirAboveThreshold) {
  Rig rig(tractive(""));
  rig.run(3.0);
  const nlohmann::json s = rig.summary()["precharge"]["PCC"];
  EXPECT_EQ(s["state"], 3);
  // The loop closes with AIR- (25 ms; the precharge relay takes 10), then
  // 95 % of the RC charge: tau ln 20.
  const double tau = 600.06 * 600e-6;
  EXPECT_NEAR(s["precharge_s"].get<double>(), 0.025 + tau * std::log(20.0), 0.004);
  EXPECT_GE(s["close_ratio"].get<double>(), 0.95);
  EXPECT_FALSE(s["violation"].get<bool>());
  EXPECT_EQ(rig.get("elec.AIR_P.closed"), 1.0);
  EXPECT_EQ(rig.get("elec.PRE_K.closed"), 0.0);
  EXPECT_NEAR(rig.get("elec.C_LINK.v"), 600.0, 0.5);
  EXPECT_EQ(rig.get("elec.AIR_P.chatter"), 0.0);
}

// PH-16: precharge failures.
TEST(Elec, PrechargeFailsWithAnOpenResistorAndTimesOut) {
  Rig rig(tractive("  faults: [{type: open, target: PRE_R}]\n", "timeout_s: 2"));
  rig.run(3.0);
  EXPECT_EQ(rig.get("elec.PCC.state"), 4.0);
  EXPECT_EQ(rig.get("elec.AIR_P.closed"), 0.0);
  EXPECT_LT(rig.get("elec.C_LINK.v"), 1.0);
  EXPECT_EQ(rig.summary()["events"].back()["type"], "precharge_fail");
}

TEST(Elec, ShortTimedPrechargeClosesTheAirTooEarly) {
  Rig rig(tractive("", "mode: time, time_s: 0.3"));
  rig.run(1.0);
  const double ratio = rig.get("elec.PCC.close_ratio");
  // Charging from AIR- (25 ms) until AIR+ closes 25 ms after the 0.3 s timer.
  EXPECT_NEAR(ratio, 1.0 - std::exp(-0.3 / (600.06 * 600e-6)), 0.01);
  EXPECT_EQ(rig.get("elec.PCC.violation"), 1.0);
  bool reported = false;
  const nlohmann::json summary = rig.summary();
  for (const auto& e : summary["events"]) {
    reported = reported || e["type"] == "precharge_violation";
  }
  EXPECT_TRUE(reported);
}

// PH-16: a contactor whose coil supply sags under the load it switches.
TEST(Elec, SaggingCoilSupplyMakesTheContactorChatter) {
  Rig rig(R"(
electrical:
  initial: off
  components:
    - {id: SUP, type: voltage_source, pos: S, neg: GND, v: 12, r_ohm: 6}
    - {id: K, type: contactor, a: S, b: L, coil_a: S, coil_b: GND}
    - {id: LOAD, type: resistor, a: L, b: GND, r_ohm: 2}
)");
  rig.run(1.0);
  EXPECT_EQ(rig.get("elec.K.chatter"), 1.0);
  EXPECT_GT(rig.get("elec.K.closes"), 20.0);  // 25 ms pull-in + 10 ms drop-out
  EXPECT_LT(rig.get("elec.K.closes"), 35.0);
}

TEST(Elec, DcDcLoadStepDipsRecoversAndLimits) {
  Rig rig(R"(
electrical:
  components:
    - {id: HV, type: voltage_source, pos: HV+, neg: HV-, v: 500, r_ohm: 0.1}
    - {id: DCDC, type: dcdc, in_pos: HV+, in_neg: HV-, out_pos: LV, out_neg: GND, v_out: 13.5,
       r_out_ohm: 0.02, i_max_a: 20, response_s: 0.002, v_in_min: 200, efficiency: 0.9}
    - {id: BASE, type: resistor, a: LV, b: GND, r_ohm: 5}
    - {id: STEP, type: load, a: LV, b: GND, r_ohm: 1.35, control: test.step}
)",
          {"test.step"});
  rig.run(0.1);
  EXPECT_NEAR(rig.get("elec.v.LV"), 13.5, 0.01);
  const double p_in = rig.get("elec.DCDC.i_in") * (rig.get("elec.v.HV_P") - rig.get("elec.v.HV_N"));
  EXPECT_NEAR(p_in, rig.get("elec.DCDC.p_out") / 0.9 + 2.0, 0.05);
  rig.set("test.step", 1.0);  // +10 A: r_out x 10 A at once, then the regulator catches up
  rig.run(0.101);
  const double dip = 13.5 - rig.get("elec.v.LV");
  EXPECT_GT(dip, 0.15);
  EXPECT_LT(dip, 0.21);
  rig.run(0.13);  // 15 time constants
  EXPECT_NEAR(rig.get("elec.v.LV"), 13.5, 0.01);
  EXPECT_EQ(rig.get("elec.DCDC.limited"), 0.0);
}

TEST(Elec, DcDcCurrentLimitAndInputUndervoltage) {
  Rig rig(R"(
electrical:
  components:
    - {id: HV, type: voltage_source, pos: HV+, neg: GND, signal: test.hv, r_ohm: 0.1}
    - {id: DCDC, type: dcdc, in_pos: HV+, in_neg: GND, out_pos: LV, out_neg: GND, v_out: 13.5, i_max_a: 10,
       v_in_min: 200, v_in_on: 220}
    - {id: LOAD, type: resistor, a: LV, b: GND, r_ohm: 1.0}
)",
          {"test.hv"});
  rig.set("test.hv", 400.0);
  rig.run(0.05);
  EXPECT_NEAR(rig.get("elec.v.LV"), 10.0, 0.01);  // 10 A into 1 ohm
  EXPECT_EQ(rig.get("elec.DCDC.limited"), 1.0);
  rig.set("test.hv", 150.0);
  rig.run(0.1);
  EXPECT_EQ(rig.get("elec.DCDC.on"), 0.0);
  EXPECT_NEAR(rig.get("elec.v.LV"), 0.0, 1e-3);
  rig.set("test.hv", 210.0);  // above the cut-off but below the turn-on voltage
  rig.run(0.15);
  EXPECT_EQ(rig.get("elec.DCDC.on"), 0.0);
  rig.set("test.hv", 230.0);
  rig.run(0.2);
  EXPECT_EQ(rig.get("elec.DCDC.on"), 1.0);
}

TEST(Elec, EcuResetsBelowItsMinimumAndBootsAgain) {
  Rig rig(R"(
electrical:
  components:
    - {id: LV, type: voltage_source, pos: LV, neg: GND, signal: test.lv, r_ohm: 0.01}
    - {id: VCU, type: ecu, a: LV, b: GND, i_a: 0.3, v_min: 7.0, v_on: 7.5, boot_s: 0.2, can_node: VCU,
       forces: [test.cmd]}
)",
          {"test.lv", "test.cmd"});
  rig.set("test.lv", 12.0);
  rig.run(0.1);
  EXPECT_EQ(rig.get("elec.VCU.on"), 1.0);
  EXPECT_EQ(rig.get("elec.can_node.VCU.on"), 1.0);
  rig.set("test.lv", 6.5);  // brown-out
  rig.run(0.2);
  EXPECT_EQ(rig.get("elec.VCU.on"), 0.0);
  EXPECT_EQ(rig.get("elec.VCU.resets"), 1.0);
  rig.net.init_gate(rig.bus);
  rig.set("test.cmd", 5.0);
  rig.net.gate(rig.t, rig.bus);
  EXPECT_EQ(rig.get("test.cmd"), 0.0);
  rig.set("test.lv", 7.2);  // above v_min but below v_on: stays in reset
  rig.run(0.5);
  EXPECT_EQ(rig.get("elec.VCU.on"), 0.0);
  rig.set("test.lv", 12.0);
  rig.run(0.69);
  EXPECT_EQ(rig.get("elec.VCU.on"), 0.0);  // booting
  rig.run(0.72);
  EXPECT_EQ(rig.get("elec.VCU.on"), 1.0);
  rig.set("test.cmd", 5.0);
  rig.net.gate(rig.t, rig.bus);
  EXPECT_EQ(rig.get("test.cmd"), 5.0);
}

TEST(Elec, WireFaultsOpenShortAndContactResistance) {
  Rig rig(R"(
electrical:
  harness: {contact_r_ohm: 0.0}
  components:
    - {id: SRC, type: voltage_source, pos: V, neg: GND, v: 12, r_ohm: 0.001}
    - {id: F, type: fuse, a: V, b: VF, rating_a: 10}
    - {id: W, type: wire, a: VF, b: L, length_m: 2, area_mm2: 1.0, pins: [X1.4], thermal: false, temp_c: 20}
    - {id: LOAD, type: resistor, a: L, b: GND, r_ohm: 6}
  faults:
    - {type: contact_resistance, target: conn.X1.4, r_ohm: 0.5, at_s: 0.1, until_s: 0.2}
    - {type: open, target: W, at_s: 0.2, until_s: 0.3}
    - {type: short, target: L, r_ohm: 0.01, at_s: 0.4}
)");
  rig.run(0.05);
  const double r_wire = 1.724e-8 * 2.0 / 1e-6;
  const double i0 = rig.get("elec.LOAD.i");
  EXPECT_NEAR(rig.get("elec.W.drop"), i0 * r_wire, 1e-6);
  rig.run(0.15);
  EXPECT_NEAR(rig.get("elec.W.drop"), rig.get("elec.LOAD.i") * (r_wire + 0.5), 1e-6);
  rig.run(0.25);
  EXPECT_LT(rig.get("elec.LOAD.i"), 1e-6);
  rig.run(0.35);
  EXPECT_NEAR(rig.get("elec.LOAD.i"), i0, 1e-6);
  EXPECT_EQ(rig.get("elec.F.blown"), 0.0);
  rig.run(0.6);  // the short draws ~300 A through the 10 A fuse
  EXPECT_EQ(rig.get("elec.F.blown"), 1.0);
}

TEST(Elec, IntermittentContactIsReproducible) {
  const auto run = [] {
    Rig rig(R"(
electrical:
  components:
    - {id: SRC, type: voltage_source, pos: V, neg: GND, v: 12}
    - {id: W, type: wire, a: V, b: L, length_m: 1, area_mm2: 0.5}
    - {id: LOAD, type: resistor, a: L, b: GND, r_ohm: 12}
  faults: [{type: open, target: W, intermittent: {rate_hz: 20, duration_s: 0.01}}]
)");
    std::vector<int> pattern;
    for (int k = 0; k < 1000; ++k) {
      rig.run((k + 1) * 1e-3);
      pattern.push_back(rig.get("elec.LOAD.i") < 0.5 ? 1 : 0);
    }
    return pattern;
  };
  const std::vector<int> a = run(), b = run();
  EXPECT_EQ(a, b);
  int open = 0;
  for (const int v : a) {
    open += v;
  }
  EXPECT_GT(open, 100);  // about 20 outages of 10 ms
  EXPECT_LT(open, 330);
}

TEST(Elec, ReportsNetlistMistakes) {
  const auto error = [](const std::string& yaml) {
    try {
      Rig rig(yaml);
      rig.run(0.002);
    } catch (const std::invalid_argument& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  EXPECT_NE(error("electrical: {components: [{id: R, type: transistor, a: A, b: B}]}").find("transistor"),
            std::string::npos);
  EXPECT_NE(error("electrical: {components: [{id: R, type: resistor, a: A, b: B}]}").find("needs r_ohm"),
            std::string::npos);
  EXPECT_NE(error("electrical: {components: [{id: R, type: resistor, a: A, b: B, r_ohm: 1}, "
                  "{id: R, type: resistor, a: A, b: B, r_ohm: 1}]}")
                .find("used twice"),
            std::string::npos);
  EXPECT_NE(error("electrical: {components: [{id: bat, type: resistor, a: A, b: B, r_ohm: 1}]}").find("reserved"),
            std::string::npos);
  EXPECT_NE(error("electrical: {components: [{id: R, type: resistor, a: A, b: B, r_ohm: 1}], "
                  "faults: [{type: open, target: X}]}")
                .find("no such component"),
            std::string::npos);
  EXPECT_NE(error("electrical: {components: [{id: R, type: resistor, a: A, b: B, r_ohm: 1}], "
                  "faults: [{type: blow, target: R}]}")
                .find("does not take blow"),
            std::string::npos);
  EXPECT_NE(error("electrical: {components: [{id: S, type: switch, a: A, b: B, open_when: {signal: no.such}}]}")
                .find("no.such"),
            std::string::npos);
  EXPECT_NE(error("electrical: {components: [{id: K, type: relay, a: A, b: B, coil_a: C, coil_b: GND, "
                  "pull_in_v: 3, drop_out_v: 5}]}")
                .find("drop_out_v"),
            std::string::npos);
}

TEST(Elec, SignalNamesAreIdentifiers) {
  EXPECT_EQ(ElectricalNetwork::signal_name("HV+"), "HV_P");
  EXPECT_EQ(ElectricalNetwork::signal_name("INV_DC-"), "INV_DC_N");
  EXPECT_EQ(ElectricalNetwork::signal_name("12V"), "_12V");
  EXPECT_EQ(ElectricalNetwork::signal_name("LV.a"), "LV_a");
}

// --- The example car's network -------------------------------------------------------------

Scenario::Overrides with_elec(Scenario::Overrides o) {
  o.emplace("electrical.enabled", "true");
  return o;
}

// "electrical.components.<i>.<key>" of the example netlist's component `id`.
std::string component_key(const std::string& id, const std::string& key) {
  const Scenario s = load_example("acceleration", with_elec({}));
  for (std::size_t i = 0; s.params.has_prefix("electrical.components." + std::to_string(i)); ++i) {
    const std::string p = "electrical.components." + std::to_string(i);
    if (s.params.get_string(p + ".id", "") == id) {
      return p + "." + key;
    }
  }
  ADD_FAILURE() << "no component " << id;
  return "";
}

TEST(ElecVehicle, NetworkCarriesTheTractionCurrentTransparently) {
  Simulation plain(load_example("acceleration"));
  Simulation elec(load_example("acceleration", with_elec({})));
  plain.run_for(3.0);
  elec.run_for(3.0);
  EXPECT_NEAR(signal(elec, "veh.chassis.vx"), signal(plain, "veh.chassis.vx"), 0.002 * signal(plain, "veh.chassis.vx"));
  EXPECT_EQ(signal(elec, "bat.current"), signal(elec, "elec.bat.current"));
  // Accumulator to DC link: pack resistance, fuse, contactor, cables.
  EXPECT_LT(signal(elec, "bat.voltage") - signal(elec, "elec.ts_voltage"), 1.0);
  EXPECT_GT(signal(elec, "bat.voltage") - signal(elec, "elec.ts_voltage"), 0.0);
  EXPECT_NEAR(signal(elec, "energy.residual"), signal(plain, "energy.residual"), 0.5);
  EXPECT_NEAR(signal(elec, "elec.v.GLV"), 13.5, 0.05);  // DC-DC
  EXPECT_EQ(signal(elec, "elec.PCC.state"), 3.0);
  const nlohmann::json s = nlohmann::json::parse(elec.electrical_summary());
  EXPECT_EQ(s["solver"]["not_converged"], 0);
  EXPECT_LT(s["solver"]["iterations_mean"].get<double>(), 3.0);
  EXPECT_TRUE(s["events"].empty()) << s["events"].dump();
  EXPECT_GT(s["wires"]["HV_P"]["i_max_a"].get<double>(), 100.0);
  EXPECT_GT(s["lv_budget"]["p_mean_w"].get<double>(), 50.0);
}

TEST(ElecVehicle, TractionWaitsForThePrecharge) {
  Simulation sim(load_example("acceleration", with_elec({{"electrical.initial", "off"}})));
  sim.run_for(0.5);
  EXPECT_LT(signal(sim, "veh.chassis.vx"), 0.05);  // inverters below their undervoltage limit
  EXPECT_EQ(signal(sim, "elec.motor.fl.enabled"), 0.0);
  sim.run_for(1.5);
  EXPECT_EQ(signal(sim, "elec.PCC.state"), 3.0);
  EXPECT_GT(signal(sim, "veh.chassis.vx"), 5.0);
  const nlohmann::json s = nlohmann::json::parse(sim.electrical_summary());
  EXPECT_NEAR(s["precharge"]["PCC"]["precharge_s"].get<double>(), 0.025 + 0.36 * std::log(20.0), 0.03);
  EXPECT_FALSE(s["precharge"]["PCC"]["violation"].get<bool>());
}

// PH-07: the fans start (inrush) with the DC-DC dead, a low LV battery and a
// corroded main LV lead: the supply sags, the VCU and the inverter logic
// reset, torque drops out and the VCU's CAN node goes silent.
TEST(ElecVehicle, LvSagResetsTheVcuWhenTheFansStart) {
  Scenario::Overrides o = with_elec({{"initial.motor_temp_c", "59.5"},
                                     {"can.enabled", "true"},
                                     {"can.fidelity", "timing"},
                                     {"electrical.faults.0.type", "open"},
                                     {"electrical.faults.0.target", "DCDC"},
                                     {"electrical.faults.1.type", "contact_resistance"},
                                     {"electrical.faults.1.target", "W_GLV"},
                                     {"electrical.faults.1.r_ohm", "0.12"}});
  o.emplace(component_key("LV_BAT", "soc"), "0.03");
  Simulation sim(load_example("acceleration", o));
  Simulation healthy(load_example("acceleration", with_elec({{"initial.motor_temp_c", "59.5"}})));
  sim.run_for(4.0);
  healthy.run_for(4.0);
  EXPECT_EQ(signal(healthy, "elec.VCU.resets"), 0.0);
  EXPECT_EQ(signal(healthy, "elec.FANS.on"), 1.0);
  EXPECT_GE(signal(sim, "elec.VCU.resets"), 1.0);
  EXPECT_GE(signal(sim, "elec.INV_LOGIC.resets"), 1.0);
  EXPECT_LT(signal(sim, "veh.chassis.vx"), signal(healthy, "veh.chassis.vx") - 1.0);
  const nlohmann::json can = nlohmann::json::parse(sim.can_summary());
  EXPECT_GT(can["messages"]["VCU_TorqueCmd"]["skipped"].get<int>(), 5);
  bool reset = false;
  const nlohmann::json summary = nlohmann::json::parse(sim.electrical_summary());
  for (const auto& e : summary["events"]) {
    reset = reset || (e["type"] == "ecu_reset" && e["id"] == "VCU");
  }
  EXPECT_TRUE(reset);
}

// PH-10: a corroded APPS2 ground pin lifts its signal; the VCU sees the two
// tracks disagree and cuts torque. An open signal wire reads 0 V (pull-down).
TEST(ElecVehicle, AppsWiringFaultsTripThePlausibilityCheck) {
  const auto run = [](const Scenario::Overrides& faults) {
    Scenario::Overrides o = with_elec(faults);
    Simulation sim(load_example("acceleration", o));
    sim.run_for(1.0);
    const double before = signal(sim, "vcu.apps_implausible");
    sim.run_for(1.3);
    return std::pair{before, signal(sim, "vcu.apps_implausible")};
  };
  const auto healthy = run({});
  EXPECT_EQ(healthy.second, 0.0);
  const auto corroded = run({{"electrical.faults.0.type", "contact_resistance"},
                             {"electrical.faults.0.target", "conn.X6.2"},
                             {"electrical.faults.0.r_ohm", "1500"},
                             {"electrical.faults.0.at_s", "1.0"}});
  EXPECT_EQ(corroded.first, 0.0);
  EXPECT_EQ(corroded.second, 1.0);
  const auto broken = run({{"electrical.faults.0.type", "open"},
                           {"electrical.faults.0.target", "W_A1_SIG"},
                           {"electrical.faults.0.at_s", "1.0"}});
  EXPECT_EQ(broken.second, 1.0);
}

TEST(ElecVehicle, IsDeterministic) {
  const auto run = [] {
    Simulation sim(load_example("acceleration",
                                with_elec({{"electrical.faults.0.type", "open"},
                                           {"electrical.faults.0.target", "W_SDC"},
                                           {"electrical.faults.0.at_s", "0.5"},
                                           {"electrical.faults.0.intermittent.rate_hz", "5"},
                                           {"electrical.faults.0.intermittent.duration_s", "0.03"}})));
    sim.run_for(2.0);
    return std::pair{signal(sim, "veh.chassis.vx"), signal(sim, "elec.AIR_P.closes")};
  };
  EXPECT_EQ(run(), run());
}

TEST(ElecVehicle, NeedsAnInverterForEveryMotor) {
  Scenario::Overrides o = with_elec({});
  o.emplace(component_key("INV_R", "motors"), "rl");
  try {
    Simulation sim(load_example("acceleration", o));
    FAIL() << "accepted a motor without an inverter";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("motor rr has no inverter"), std::string::npos) << e.what();
  }
}

}  // namespace
}  // namespace vsim

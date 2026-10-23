#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include "test_support.hpp"
#include "vsim/models/powertrain/battery.hpp"
#include "vsim/models/powertrain/motor.hpp"
#include "vsim/models/powertrain/pmsm.hpp"

namespace vsim {
namespace {

const ParamTree& example() {
  static const ParamTree params = test::load_example("straight_accel").params;
  return params;
}

TEST(Motor, EnvelopeIsTorqueThenPowerLimited) {
  const MotorParams m = MotorParams::from_params(example());
  const double v = m.nominal_voltage;
  EXPECT_DOUBLE_EQ(m.envelope(0.0, v, 25.0), m.torque_max);
  const double base = m.power_max / m.torque_max;
  EXPECT_DOUBLE_EQ(m.envelope(0.5 * base, v, 25.0), m.torque_max);
  const double fast = 1.15 * base;  // above base speed, below the speed-limit fade
  ASSERT_LT(fast, 0.95 * m.speed_max);
  EXPECT_NEAR(m.envelope(fast, v, 25.0) * fast, m.power_max, 1e-6);
  EXPECT_DOUBLE_EQ(m.envelope(-fast, v, 25.0), m.envelope(fast, v, 25.0));
  // Less DC voltage, less power above base speed.
  EXPECT_NEAR(m.envelope(fast, 0.5 * v, 25.0), 0.5 * m.envelope(fast, v, 25.0), 1e-9);
  EXPECT_EQ(m.envelope(m.speed_max, v, 25.0), 0.0);
}

// PH-05: torque fades linearly between the derating temperatures.
TEST(Motor, ThermalDerating) {
  const MotorParams m = MotorParams::from_params(example());
  EXPECT_DOUBLE_EQ(m.derate(m.derate_start - 1.0), 1.0);
  EXPECT_DOUBLE_EQ(m.derate(0.5 * (m.derate_start + m.derate_end)), 0.5);
  EXPECT_DOUBLE_EQ(m.derate(m.derate_end + 1.0), 0.0);
  EXPECT_DOUBLE_EQ(m.envelope(0.0, m.nominal_voltage, m.derate_end), 0.0);
}

TEST(Motor, LossesArePositiveAndGrowWithTorque) {
  const MotorParams m = MotorParams::from_params(example());
  EXPECT_DOUBLE_EQ(m.losses(0.0, 0.0), m.loss_constant);
  EXPECT_GT(m.losses(20.0, 1000.0), m.losses(10.0, 1000.0));
  EXPECT_DOUBLE_EQ(m.losses(-20.0, -1000.0), m.losses(20.0, 1000.0));
}

TEST(Battery, OcvInterpolatesAndClamps) {
  const BatteryParams b = BatteryParams::from_params(example());
  EXPECT_DOUBLE_EQ(b.ocv_cell(-0.1), b.ocv_voltage.front());
  EXPECT_DOUBLE_EQ(b.ocv_cell(1.2), b.ocv_voltage.back());
  EXPECT_DOUBLE_EQ(b.ocv_cell(0.5), 3.74);
  EXPECT_NEAR(b.ocv_cell(0.55), 0.5 * (3.74 + 3.82), 1e-12);
  EXPECT_NEAR(b.pack_resistance(), 140 * 0.012 / 5 + 0.030, 1e-12);
}

TEST(Battery, PowerSolveSatisfiesTheCircuit) {
  const double open = 550.0;
  const double resistance = 0.4;
  for (const double power : {-40e3, 0.0, 10e3, 80e3}) {
    const BatteryOperatingPoint p = solve_battery_power(open, resistance, power);
    EXPECT_FALSE(p.limited);
    EXPECT_NEAR(p.voltage * p.current, power, 1e-6);
    EXPECT_NEAR(p.voltage, open - resistance * p.current, 1e-9);
  }
  // Beyond V^2 / 4R the pack cannot deliver; it sits at its maximum power point.
  const BatteryOperatingPoint over = solve_battery_power(open, resistance, 1e6);
  EXPECT_TRUE(over.limited);
  EXPECT_DOUBLE_EQ(over.voltage, 0.5 * open);
}

// PH-14: a nearly full pack accepts little charge current.
TEST(Battery, ChargeLimitShrinksNearFull) {
  const BatteryParams b = BatteryParams::from_params(example());
  const BatteryLimits mid = battery_limits(b, 0.6, 0.0, 25.0);
  const BatteryLimits full = battery_limits(b, 0.99, 0.0, 25.0);
  EXPECT_DOUBLE_EQ(mid.charge_current, b.charge_current_max);
  EXPECT_LT(full.charge_current, 0.15 * b.charge_current_max);
  EXPECT_DOUBLE_EQ(mid.discharge_current, b.discharge_current_max);
  // Hot pack: everything derates to zero at the cutoff.
  const BatteryLimits hot = battery_limits(b, 0.6, 0.0, b.cutoff);
  EXPECT_DOUBLE_EQ(hot.discharge_current, 0.0);
  EXPECT_DOUBLE_EQ(hot.charge_current, 0.0);
  // Empty pack: no discharge.
  EXPECT_DOUBLE_EQ(battery_limits(b, 0.0, 0.0, 25.0).discharge_current, 0.0);
}

TEST(Battery, RejectsInconsistentTables) {
  ParamTree params = example();
  params.set("vehicle.battery.ocv_v", "3.7");
  EXPECT_THROW(BatteryParams::from_params(params), std::invalid_argument);
  params = example();
  params.set("vehicle.battery.cells_parallel", "0");
  EXPECT_THROW(BatteryParams::from_params(params), std::invalid_argument);
}

}  // namespace
}  // namespace vsim

namespace vsim {
namespace {

TEST(CellSpread, LimitsComeFromTheWeakestGroup) {
  const BatteryParams p = BatteryParams::from_params(test::load_example("straight_accel").params);
  CellSpread spread;
  spread.capacity.assign(static_cast<std::size_t>(p.series), 1.0);
  spread.resistance.assign(static_cast<std::size_t>(p.series), 1.0);
  spread.soc_offset.assign(static_cast<std::size_t>(p.series), 0.0);
  spread.soc_start = 0.5;
  // Identical groups: the same limits as without spread.
  const BatteryLimits same = battery_limits(p, 0.2, 0.01, 25.0, &spread);
  const BatteryLimits none = battery_limits(p, 0.2, 0.01, 25.0);
  EXPECT_DOUBLE_EQ(same.discharge_current, none.discharge_current);
  EXPECT_DOUBLE_EQ(same.charge_current, none.charge_current);
  // A group with less capacity runs down faster: from 0.5, 30 % drawn from the mean leaves it
  // at 0.5 - 0.3 / 0.9.
  spread.capacity[3] = 0.9;
  EXPECT_NEAR(spread.group_soc(3, 0.2), 0.5 - 0.3 / 0.9, 1e-12);
  spread.resistance[3] = 1.3;
  const BatteryLimits weak = battery_limits(p, 0.2, 0.01, 25.0, &spread);
  EXPECT_LT(weak.discharge_current, none.discharge_current);
  const CellExtremes x = cell_extremes(p, spread, 0.2, 0.01, 100.0);
  EXPECT_EQ(x.weakest, 3u);
  EXPECT_LT(x.v_min, x.v_max);
  EXPECT_NEAR(x.soc_min, 0.5 - 0.3 / 0.9, 1e-12);
  EXPECT_NEAR(x.soc_max, 0.2, 1e-12);
}

TEST(CellSpread, SampledPerCarWhenVariationIsReal) {
  const Scenario::Overrides spread = {{"vehicle.battery.spread.capacity_pct", "2"},
                                      {"vehicle.battery.spread.resistance_pct", "6"},
                                      {"vehicle.model", "two_track"}};
  Scenario::Overrides real = spread;
  real.emplace("realism.variation", "real");
  const auto cell_min = [](const Scenario::Overrides& o) {
    Simulation sim(test::load_example("straight_accel", o));
    sim.run_for(1.0);
    return test::signal(sim, "bat.cell_v_min");
  };
  const auto cell_max = [](const Scenario::Overrides& o) {
    Simulation sim(test::load_example("straight_accel", o));
    sim.run_for(1.0);
    return test::signal(sim, "bat.cell_v_max");
  };
  EXPECT_DOUBLE_EQ(cell_min(spread), cell_max(spread));  // ideal: identical cells
  EXPECT_LT(cell_min(real), cell_max(real) - 0.01);       // a real pack spreads
  EXPECT_DOUBLE_EQ(cell_min(real), cell_min(real));       // the same car every run
  Scenario::Overrides other = real;
  other.emplace("seed", "2");
  EXPECT_NE(cell_min(real), cell_min(other));
}

}  // namespace
}  // namespace vsim

namespace vsim {
namespace {

TEST(Motor, TorqueRippleAndItsFade) {
  MotorParams m = MotorParams::from_params(test::load_example(
      "straight_accel", {{"vehicle.powertrain.motor.ripple.0.order", "30"},
                         {"vehicle.powertrain.motor.ripple.0.amplitude_pct", "3"},
                         {"vehicle.powertrain.motor.ripple.0.cogging_nm", "0.3"}}).params);
  ASSERT_EQ(m.ripple.size(), 1u);
  const double quarter = 3.14159265358979 / (2.0 * 30.0);  // sin = 1
  // At standstill the full ripple: 3 % of 10 N m plus 0.3 N m cogging.
  EXPECT_NEAR(m.ripple_torque(10.0, quarter, 0.0, 125.0), 0.6, 1e-12);
  // Far above what the plant step carries: faded out.
  EXPECT_LT(std::abs(m.ripple_torque(10.0, quarter, 1000.0, 125.0)), 1e-3);
  EXPECT_THROW(MotorParams::from_params(test::load_example(
                   "straight_accel", {{"vehicle.powertrain.motor.ripple.0.order", "0"}}).params),
               std::invalid_argument);
}


// --- Level B: PMSM under field oriented control --------------------------------------------

PmsmParams example_pmsm() {
  PmsmParams p;  // the example car's motor (5 pole pairs, 0.225 N m/A)
  p.dead_time = 0.0;
  return p;
}

// Amplitude of the f [Hz] component of samples taken every dt.
double amplitude(const std::vector<double>& v, double dt, double f) {
  double c = 0.0, s = 0.0, mean = 0.0;
  for (const double x : v) {
    mean += x / static_cast<double>(v.size());
  }
  for (std::size_t i = 0; i < v.size(); ++i) {
    const double w = 2.0 * std::numbers::pi * f * dt * static_cast<double>(i);
    c += (v[i] - mean) * std::cos(w);
    s += (v[i] - mean) * std::sin(w);
  }
  return 2.0 * std::hypot(c, s) / static_cast<double>(v.size());
}

std::vector<double> torque_trace(PmsmDrive& drive, const PmsmDrive::Input& start, double seconds,
                                 std::vector<double>* d_current = nullptr) {
  std::vector<double> torque;
  drive.trace = [&](const PmsmDrive::Sample& s) {
    torque.push_back(s.torque);
    if (d_current) {
      d_current->push_back(s.id);
    }
  };
  PmsmDrive::Input in = start;
  for (int k = 0; k < static_cast<int>(std::lround(seconds / 1e-3)); ++k) {
    drive.step(in, 1e-3);
    in.angle += in.speed * 1e-3;
  }
  drive.trace = nullptr;
  return torque;
}

TEST(Pmsm, CurrentLoopFollowsTheTorqueAtItsBandwidth) {
  PmsmDrive drive(example_pmsm());
  const std::vector<double> t = torque_trace(drive, {10.0, 20.0, 0.0, 520.0}, 5e-3);
  // 16 kHz control: one sample is 62.5 us. 63 % within the loop's time
  // constant (159 us) plus the computation and PWM delay (about 94 us).
  const auto first_above = [&](double level) {
    return static_cast<int>(std::find_if(t.begin(), t.end(), [&](double v) { return v >= level; }) - t.begin());
  };
  EXPECT_GE(first_above(6.3), 2);
  EXPECT_LE(first_above(6.3), 6);
  EXPECT_NEAR(t.back(), 10.0, 0.02);
  double peak = 0.0;
  for (const double v : t) {
    peak = std::max(peak, v);
  }
  EXPECT_LT(peak, 10.0 * 1.15);  // well damped
}

TEST(Pmsm, MtpaAndFieldWeakening) {
  const PmsmParams p = example_pmsm();
  PmsmDrive drive(p);
  double id = 0.0, iq = 0.0;
  // Low speed: MTPA uses the reluctance torque (Ld < Lq: id < 0) for less current.
  drive.references(18.0, 50.0, 520.0, id, iq);
  EXPECT_LT(id, 0.0);
  EXPECT_NEAR(p.torque(id, iq), 18.0, 1e-6);
  EXPECT_LT(std::hypot(id, iq), 18.0 / (1.5 * p.pole_pairs * p.flux));
  // Regenerating: the same current angle, negative q.
  double id_r = 0.0, iq_r = 0.0;
  drive.references(-18.0, 50.0, 520.0, id_r, iq_r);
  EXPECT_DOUBLE_EQ(id_r, id);
  EXPECT_DOUBLE_EQ(iq_r, -iq);

  // Near top speed the back-EMF (5 x 2000 rad/s x 0.03 Wb = 300 V) needs the
  // field weakened; at 400 V less torque is left than at 520 V.
  double id_hi = 0.0, iq_hi = 0.0, id_lo = 0.0, iq_lo = 0.0;
  drive.references(15.0, 2000.0, 520.0, id_hi, iq_hi);
  drive.references(15.0, 2000.0, 400.0, id_lo, iq_lo);
  EXPECT_LT(id_hi, -20.0);
  EXPECT_LT(id_lo, id_hi);
  EXPECT_LT(p.torque(id_lo, iq_lo), p.torque(id_hi, iq_hi) - 1.0);
  EXPECT_LE(std::hypot(id_lo, iq_lo), p.current_max + 1e-9);

  // The drive reaches the reference within the voltage it has.
  PmsmDrive::Input in{15.0, 2000.0, 0.0, 520.0};
  PmsmDrive::Output out;
  for (int k = 0; k < 20; ++k) {
    out = drive.step(in, 1e-3);
    in.angle += in.speed * 1e-3;
  }
  // 10 control samples per electrical turn at this speed: the currents swing
  // between samples and the average torque comes out about 2 % short.
  EXPECT_NEAR(out.torque, p.torque(id_hi, iq_hi), 0.035 * 15.0);
  EXPECT_NEAR(out.id, id_hi, 0.1);  // at the sample instants the loop holds the reference
  EXPECT_LE(out.modulation, 1.0);
  EXPECT_GT(out.modulation, 0.8);
  // Power into the inverter: shaft power plus its losses.
  EXPECT_NEAR(out.dc_power, out.torque * in.speed + out.loss, 0.02 * out.dc_power);
  EXPECT_GT(out.loss, 1.5 * p.resistance * (out.id * out.id + out.iq * out.iq) * 0.95);
}

// Dead time ripples the torque at 6x the electrical frequency; the
// resolver's angle error turns the controller's frame at its order of the
// rotor speed, so the d current (and a little torque) ripples there.
TEST(Pmsm, DeadTimeAndResolverRipple) {
  const double speed = 500.0;  // rotor: 79.6 rev/s, electrical 398 Hz
  const double rev = speed / (2.0 * std::numbers::pi), electrical = 5.0 * rev;
  const auto ripple = [&](double dead_us, double compensation, double resolver_deg) {
    PmsmParams p = example_pmsm();
    p.dead_time = dead_us * 1e-6;
    p.dead_time_compensation = compensation;
    p.resolver_error = resolver_deg * std::numbers::pi / 180.0;
    PmsmDrive drive(p);
    std::vector<double> d;
    std::vector<double> t = torque_trace(drive, {5.0, speed, 0.0, 520.0}, 0.25, &d);
    t.erase(t.begin(), t.begin() + static_cast<std::ptrdiff_t>(t.size() / 5));
    d.erase(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(d.size() / 5));
    return std::pair{amplitude(t, 1.0 / p.control_rate, 6.0 * electrical), amplitude(d, 1.0 / p.control_rate, 2.0 * rev)};
  };
  const auto clean = ripple(0.0, 0.0, 0.0);
  const auto dead = ripple(1.0, 0.0, 0.0);
  const auto compensated = ripple(1.0, 0.8, 0.0);
  const auto resolver = ripple(0.0, 0.0, 0.2);
  EXPECT_LT(clean.first, 0.01);
  EXPECT_LT(clean.second, 0.01);
  EXPECT_GT(dead.first, 0.1);
  EXPECT_LT(compensated.first, 0.6 * dead.first);
  EXPECT_GT(resolver.second, 1.0);  // [A], 0.2 deg x 5 pole pairs
  EXPECT_LT(resolver.first, 0.02);
}

TEST(Pmsm, ParametersAreChecked) {
  const ParamTree params = test::load_example("acceleration").params;
  const MotorParams motor = MotorParams::from_params(params);
  ParamTree bad = params;
  bad.set("vehicle.powertrain.motor.pmsm.current_bandwidth_hz", "5000");  // above control_rate / 8
  EXPECT_THROW(PmsmParams::from_params(bad, motor), std::invalid_argument);
  bad = params;
  bad.set("vehicle.powertrain.motor.pmsm.ld_mh", "0");
  EXPECT_THROW(PmsmParams::from_params(bad, motor), std::invalid_argument);
  EXPECT_NO_THROW(PmsmParams::from_params(params, motor));
}

}  // namespace
}  // namespace vsim

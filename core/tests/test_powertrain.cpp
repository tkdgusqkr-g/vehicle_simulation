#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "test_support.hpp"
#include "vsim/models/powertrain/battery.hpp"
#include "vsim/models/powertrain/motor.hpp"

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
  EXPECT_NEAR(b.pack_resistance(), 144 * 0.012 / 5 + 0.030, 1e-12);
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

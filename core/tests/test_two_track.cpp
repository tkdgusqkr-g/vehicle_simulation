#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "test_support.hpp"
#include "vsim/models/powertrain/motor.hpp"
#include "vsim/models/vehicle/two_track.hpp"

namespace vsim {
namespace {

using test::signal;

// These tests are about the L2 model (the example car runs the full model).
Scenario load_example(const std::string& name, Scenario::Overrides overrides = {}) {
  overrides.emplace("vehicle.model", "two_track");
  return test::load_example(name, overrides);
}

constexpr double kGravity = 9.81;

double rad_to_deg(double rad) { return rad * 180.0 / std::numbers::pi; }

// Recorded column of a logged signal (a copy).
std::vector<double> column(const Simulation& sim, const std::string& name) {
  const auto& names = sim.recorder().names();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    throw std::out_of_range("not logged: " + name);
  }
  return sim.recorder().columns()[static_cast<std::size_t>(it - names.begin())];
}

double max_of(const std::vector<double>& values) { return *std::max_element(values.begin(), values.end()); }
double min_of(const std::vector<double>& values) { return *std::min_element(values.begin(), values.end()); }

TEST(TwoTrack, IsAvailable) {
  Simulation sim(load_example("straight_accel"));
  EXPECT_NO_THROW(sim.bus().require("veh.wheel.rr.slip_ratio"));
  EXPECT_NO_THROW(sim.bus().require("bat.soc"));
}

TEST(TwoTrack, StandstillStaysAtRest) {
  Simulation sim(load_example("straight_accel", {{"driver.throttle.value", "0"}}));
  sim.run_for(5.0);
  EXPECT_EQ(signal(sim, "veh.chassis.x"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.y"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.vx"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.yaw_rate"), 0.0);
  // The auxiliary load still drains the pack.
  EXPECT_LT(signal(sim, "bat.soc"), sim.scenario().params.get_double("initial.soc", 0.9));
}

TEST(TwoTrack, FullSteeringLockAtRestDoesNotCreep) {
  Simulation sim(load_example("straight_accel", {{"driver.throttle.value", "0"},
                                                 {"driver.steer.type", "const"},
                                                 {"driver.steer.value", "110"}}));
  sim.run_for(5.0);
  EXPECT_EQ(signal(sim, "veh.chassis.vx"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.vy"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.yaw_rate"), 0.0);
}

// V-01: steady-state yaw rate in the linear range matches the analytic
// bicycle model r = v * delta / (L + K * v^2).
TEST(TwoTrack, SteadyStateYawRateMatchesLinearBicycle) {
  constexpr double kRoadWheelAngle = 0.01;  // [rad]
  const Scenario probe = load_example("sine_steer");
  const double ratio = probe.params.get_double("vehicle.steering.ratio");
  Simulation sim(load_example(
      "sine_steer", {{"driver.steer.type", "const"},
                     {"driver.steer.value", std::to_string(rad_to_deg(kRoadWheelAngle * ratio))},
                     {"driver.speed_hold_kph", "36"},
                     {"vehicle.aero.cla_m2", "0"},
                     {"vehicle.aero.cda_m2", "0"},
                     {"vehicle.tire.rolling_resistance", "0"},
                     {"vehicle.tire.simple.load_sensitivity", "0"},
                     {"vehicle.tire.simple.pneumatic_trail_m", "0"},
                     {"vehicle.steering.ackermann", "0"},
                     {"vehicle.tire.rear.simple.By", "16"}}));  // stiffer rear -> understeer
  sim.run_for(20.0);

  const ParamTree& params = sim.scenario().params;
  const auto p = TwoTrackModel::params_from(params);
  const auto front = SimpleTire::params_from(params, "front");
  const auto rear = SimpleTire::params_from(params, "rear");
  const double lf = p.cg_to_front;
  const double lr = p.cg_to_rear();
  const double c_front = front.by * front.cy * front.mu_y * p.mass * kGravity * lr / p.wheelbase;
  const double c_rear = rear.by * rear.cy * rear.mu_y * p.mass * kGravity * lf / p.wheelbase;
  const double understeer = p.mass / p.wheelbase * (lr / c_front - lf / c_rear);
  ASSERT_GT(understeer, 0.0);

  const double v = signal(sim, "veh.chassis.vx");
  EXPECT_NEAR(v, 10.0, 0.05);
  const double expected = v * kRoadWheelAngle / (p.wheelbase + understeer * v * v);
  EXPECT_NEAR(signal(sim, "veh.chassis.yaw_rate"), expected, 0.01 * expected);
}

// V-02: launch is traction limited (slip control holds the target slip),
// then power limited, where acceleration follows the motor torque through
// the gears minus drag and rolling resistance, with rotating inertia.
TEST(TwoTrack, StraightAccelerationTractionThenPowerLimited) {
  Simulation sim(load_example("straight_accel"));
  const ParamTree& params = sim.scenario().params;
  const auto p = TwoTrackModel::params_from(params);
  const MotorParams motor = MotorParams::from_params(params);
  const double gear = params.get_double("vehicle.driveline.0.gear_ratio");
  const double efficiency = params.get_double("vehicle.driveline.0.efficiency");

  sim.run_for(1.0);
  const double target = params.get_double("control.slip_control.target_slip", 0.1);
  for (const char* wheel : {"fl", "fr", "rl", "rr"}) {
    const std::vector<double> slip = column(sim, std::string("veh.wheel.") + wheel + ".slip_ratio");
    const std::vector<double> settled(slip.begin() + 30, slip.end());  // after 0.3 s
    EXPECT_LT(max_of(settled), 2.0 * target) << wheel;
  }
  EXPECT_GT(signal(sim, "veh.chassis.ax"), 1.1 * kGravity);  // 4WD launch

  while (signal(sim, "veh.chassis.vx") < 24.0) {
    sim.step();
    ASSERT_LT(sim.time(), 10.0) << "never reached 24 m/s";
  }
  sim.run_for(0.2);
  double torque = 0.0;
  for (const char* id : {"fl", "fr", "rl", "rr"}) {
    torque += signal(sim, std::string("pt.motor.") + id + ".torque");
  }
  const double v = signal(sim, "veh.chassis.vx");
  const double q = 0.5 * p.air_density * v * v;
  const double rolling = p.rolling_resistance * (p.mass * kGravity + p.cla * q);
  const double rotating = 4.0 * (p.wheel_inertia + motor.rotor_inertia * gear * gear);
  const double mass = p.mass + rotating / (p.wheel_radius * p.wheel_radius);
  const double expected = (torque * gear * efficiency / p.wheel_radius - p.cda * q - rolling) / mass;
  // Wheels and rotors spin up with the car, so the chassis signal ax
  // (forces on the chassis only) is not the whole story; use dv/dt.
  sim.run_for(0.01);
  const double measured = (signal(sim, "veh.chassis.vx") - v) / 0.01;
  EXPECT_NEAR(measured, expected, 0.02 * expected);
  // The VCU keeps the battery at the power limit (plus the auxiliary load).
  EXPECT_LE(signal(sim, "bat.power"), params.get_double("control.power_limit_kw") * 1e3 * 1.01);
  EXPECT_GE(signal(sim, "bat.power"), params.get_double("control.power_limit_kw") * 1e3 * 0.95);
}

// V-03: chemical energy from the cells = kinetic energy gain + every loss,
// through acceleration, cornering and braking with regen.
TEST(TwoTrack, EnergyIsConserved) {
  Simulation sim(load_example("straight_accel", {{"driver.throttle.type", "step"},
                                                 {"driver.throttle.before", "1"},
                                                 {"driver.throttle.at_s", "4"},
                                                 {"driver.throttle.value", "0"},
                                                 {"driver.brake.type", "step"},
                                                 {"driver.brake.at_s", "5"},
                                                 {"driver.brake.value", "0.6"},
                                                 {"driver.steer.type", "sine"},
                                                 {"driver.steer.amplitude", "20"},
                                                 {"driver.steer.freq_hz", "0.5"},
                                                 {"driver.steer.start_s", "1"}}));
  sim.run_for(10.0);
  const double released = max_of(column(sim, "energy.battery_chemical"));
  ASSERT_GT(released, 1e5);
  const double residual = std::max(std::abs(max_of(column(sim, "energy.residual"))),
                                   std::abs(min_of(column(sim, "energy.residual"))));
  EXPECT_LT(residual, 0.01 * released);  // V-03 criterion
  EXPECT_LT(residual, 1e-6 * released);  // bookkeeping is exact up to integration error
  for (const char* loss : {"slip_loss", "brake_loss", "drag_loss", "rolling_loss", "motor_loss",
                           "gear_loss", "battery_loss"}) {
    EXPECT_GT(signal(sim, std::string("energy.") + loss), 0.0) << loss;
  }
  // Regen put energy back: the chemical total fell during braking.
  EXPECT_LT(signal(sim, "energy.battery_chemical"), released);
}

// V-05: braking to a stop ends at rest without creeping or ringing, and a
// launch from rest accelerates monotonically.
TEST(TwoTrack, StopAndGoIsCalm) {
  Simulation sim(load_example("straight_accel", {{"driver.throttle.type", "step"},
                                                 {"driver.throttle.before", "1"},
                                                 {"driver.throttle.at_s", "3"},
                                                 {"driver.throttle.value", "0"},
                                                 {"driver.brake.type", "step"},
                                                 {"driver.brake.at_s", "3"},
                                                 {"driver.brake.value", "1"}}));
  sim.run_for(3.0);
  const std::vector<double> vx = column(sim, "veh.chassis.vx");
  for (std::size_t i = 1; i < vx.size(); ++i) {
    ASSERT_GE(vx[i], vx[i - 1]) << "at sample " << i;
  }
  sim.run_for(5.0);
  const double x = signal(sim, "veh.chassis.x");
  EXPECT_LT(std::abs(signal(sim, "veh.chassis.vx")), 1e-6);
  sim.run_for(2.0);
  EXPECT_LT(std::abs(signal(sim, "veh.chassis.x") - x), 1e-6);
  for (const char* wheel : {"fl", "fr", "rl", "rr"}) {
    EXPECT_LT(std::abs(signal(sim, std::string("veh.wheel.") + wheel + ".omega")), 1e-6);
  }
}

// PH-05: a hot motor delivers only its derated torque.
TEST(TwoTrack, HotMotorsDerate) {
  Simulation sim(load_example("straight_accel", {{"initial.motor_temp_c", "130"}}));
  const MotorParams motor = MotorParams::from_params(sim.scenario().params);
  sim.run_for(1.0);
  // The coolant pulls the winding temperature down while it runs.
  const double coolest = min_of(column(sim, "pt.motor.rl.temp"));
  ASSERT_LT(coolest, 130.0);
  ASSERT_GT(coolest, 125.0);
  const double torque = max_of(column(sim, "pt.motor.rl.torque"));
  EXPECT_LE(torque, motor.torque_max * motor.derate(coolest) * 1.001);
  EXPECT_GT(torque, motor.torque_max * motor.derate(130.0));
}

TEST(TwoTrack, MotorsHeatUpAndLoseTorque) {
  Simulation sim(load_example("straight_accel",
                              {{"vehicle.powertrain.motor.thermal.heat_capacity_j_per_k", "50"},
                               {"vehicle.powertrain.motor.thermal.resistance_k_per_w", "0.1"}}));
  const MotorParams motor = MotorParams::from_params(sim.scenario().params);
  sim.run_for(10.0);
  const double temp = signal(sim, "pt.motor.fl.temp");
  ASSERT_GT(temp, motor.derate_start);
  const double expected = motor.envelope(signal(sim, "pt.motor.fl.speed"), signal(sim, "bat.voltage"), temp);
  EXPECT_NEAR(signal(sim, "pt.motor.fl.torque_avail"), expected, 0.02 * expected);
}

// PH-14: regen is limited by the BMS when the pack is nearly full.
TEST(TwoTrack, RegenIsLimitedNearFullCharge) {
  const auto max_charge_current = [](const char* soc) {
    Simulation sim(load_example("straight_accel", {{"initial.speed_mps", "25"},
                                                   {"initial.soc", soc},
                                                   {"driver.throttle.value", "0"},
                                                   {"driver.brake.type", "const"},
                                                   {"driver.brake.value", "0.5"},
                                                   {"control.regen.torque_ratio", "1"}}));
    sim.run_for(1.0);
    return -min_of(column(sim, "bat.current"));
  };
  const double full = max_charge_current("0.99");
  const double mid = max_charge_current("0.6");
  const double limit = load_example("straight_accel").params.get_double(
      "vehicle.battery.charge_current_max_a");
  EXPECT_GT(mid, 0.9 * limit);   // the regen request exceeds the pack limit
  EXPECT_LT(mid, 1.05 * limit);  // ... and the VCU holds it there
  EXPECT_LT(full, 0.15 * limit);
}

TEST(TwoTrack, SteeringFeelsTheRoad) {
  Simulation sim(load_example("sine_steer", {{"driver.steer.type", "const"},
                                             {"driver.steer.value", "10"}}));
  sim.run_for(10.0);
  const double light = signal(sim, "veh.ffb.torque_sw");
  EXPECT_LT(light, 0.0);  // left turn: the road pushes the wheel back right
  Simulation harder(load_example("sine_steer", {{"driver.steer.type", "const"},
                                                {"driver.steer.value", "20"}}));
  harder.run_for(10.0);
  EXPECT_LT(signal(harder, "veh.ffb.torque_sw"), light);
}

// V-06 at vehicle level: one motor through an open differential.
TEST(TwoTrack, RearOpenDiffDrivesAndDifferentiates) {
  Simulation sim(load_example("sine_steer", {{"vehicle.driveline", ""},
                                             {"vehicle.driveline.0.motor", "rear"},
                                             {"vehicle.driveline.0.wheels.0", "rl"},
                                             {"vehicle.driveline.0.wheels.1", "rr"},
                                             {"vehicle.driveline.0.gear_ratio", "14"},
                                             {"vehicle.driveline.0.diff.type", "open"},
                                             {"driver.steer.type", "const"},
                                             {"driver.steer.value", "30"}}));
  sim.run_for(10.0);
  EXPECT_NEAR(signal(sim, "veh.chassis.vx"), 40.0 / 3.6, 0.2);
  const double left = signal(sim, "veh.wheel.rl.omega");
  const double right = signal(sim, "veh.wheel.rr.omega");
  EXPECT_GT(right, left * 1.05);  // outer wheel turns faster
  EXPECT_NEAR(signal(sim, "pt.motor.rear.speed"), 14.0 * 0.5 * (left + right), 1e-9);
}

// The same car on Magic Formula tires from a .tir file drives the same way.
TEST(TwoTrack, RunsOnMagicFormulaTires) {
  const auto run = [](const Scenario::Overrides& overrides) {
    Simulation sim(load_example("sine_steer", overrides));
    sim.run_for(12.0);
    return std::pair{signal(sim, "veh.chassis.yaw_rate"), signal(sim, "veh.chassis.vx")};
  };
  // The example .tir mirrors the simple tire except for load sensitivity.
  const auto [r_simple, v_simple] = run({{"vehicle.tire.simple.load_sensitivity", "-0.0625"}});
  const auto [r_mf, v_mf] = run({{"vehicle.tire.model", "mf"},
                                 {"vehicle.tire.file", "vehicle/tires/example_fs_10in.tir"}});
  EXPECT_NEAR(v_mf, v_simple, 0.1);
  EXPECT_NEAR(r_mf, r_simple, 0.1 * std::abs(r_simple));
}

TEST(TwoTrack, StaysFiniteUnderAbusiveInputs) {
  Simulation sim(load_example("straight_accel", {{"driver.steer.type", "sine"},
                                                 {"driver.steer.amplitude", "110"},
                                                 {"driver.steer.freq_hz", "0.5"},
                                                 {"driver.brake.type", "step"},
                                                 {"driver.brake.at_s", "6"},
                                                 {"driver.brake.value", "1"}}));
  sim.run_for(10.0);
  for (const auto& values : sim.recorder().columns()) {
    for (const double value : values) {
      ASSERT_TRUE(std::isfinite(value));
    }
  }
  EXPECT_LT(signal(sim, "veh.chassis.speed"), 40.0);
}

}  // namespace
}  // namespace vsim

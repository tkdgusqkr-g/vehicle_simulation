#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "test_support.hpp"
#include "vsim/models/driveline/driveline.hpp"
#include "vsim/models/vehicle/single_track.hpp"

namespace vsim {
namespace {

using test::signal;

// The example vehicle defaults to the L2 model; these tests pin L1, which has
// no battery or energy signals to log.
Scenario load_example(const std::string& name, Scenario::Overrides overrides = {}) {
  overrides.emplace("vehicle.model", "single_track");
  overrides.emplace("logging.signals", "*");
  return test::load_example(name, overrides);
}

constexpr double kGravity = 9.81;

double rad_to_deg(double rad) { return rad * 180.0 / std::numbers::pi; }

TEST(SingleTrack, StandstillStaysAtRest) {
  Simulation sim(load_example("straight_accel", {{"driver.throttle.value", "0"}}));
  sim.run_for(5.0);
  EXPECT_EQ(signal(sim, "veh.chassis.x"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.y"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.vx"), 0.0);
  EXPECT_EQ(signal(sim, "veh.chassis.yaw_rate"), 0.0);
}

TEST(SingleTrack, FullSteeringLockAtRestDoesNotCreep) {
  Simulation sim(load_example("straight_accel", {{"driver.throttle.value", "0"},
                                                 {"driver.steer.type", "const"},
                                                 {"driver.steer.value", "110"}}));
  sim.run_for(5.0);
  EXPECT_NEAR(signal(sim, "veh.chassis.vx"), 0.0, 1e-12);
  EXPECT_NEAR(signal(sim, "veh.chassis.x"), 0.0, 1e-12);
  EXPECT_NEAR(signal(sim, "veh.chassis.y"), 0.0, 1e-12);
}

// V-02 (part): once traction no longer limits, acceleration follows the
// power limit minus drag and rolling resistance.
TEST(SingleTrack, PowerLimitedAccelerationMatchesTheory) {
  Simulation sim(load_example("straight_accel"));
  const ParamTree& params = sim.scenario().params;
  const auto p = SingleTrackModel::params_from(params);
  // The VCU limits estimated battery power; this model has no motor losses.
  const double power =
      params.get_double("control.power_limit_kw") * 1e3 * params.get_double("control.efficiency_estimate");
  const double gear_efficiency = DrivelineConfig::from_params(params).entries.front().efficiency;
  while (signal(sim, "veh.chassis.vx") < 30.0) {
    sim.step();
    ASSERT_LT(sim.time(), 20.0) << "never reached 30 m/s";
  }
  const double v = signal(sim, "veh.chassis.vx");
  const double q = 0.5 * p.air_density * v * v;
  const double drag = p.cda * q;
  const double rolling = p.rolling_resistance * (p.mass * kGravity + p.cla * q);
  const double expected = (power * gear_efficiency / v - drag - rolling) / p.mass;
  EXPECT_NEAR(signal(sim, "veh.chassis.ax"), expected, 0.02 * expected);
  EXPECT_LE(signal(sim, "pt.power_mech"), power * 1.01);
}

// V-01: steady-state yaw rate in the linear range matches the analytic
// bicycle model r = v * delta / (L + K * v^2).
TEST(SingleTrack, SteadyStateYawRateMatchesLinearBicycle) {
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
                     {"vehicle.tire.rear.simple.By", "16"}}));  // stiffer rear -> understeer
  sim.run_for(20.0);

  const auto p = SingleTrackModel::params_from(sim.scenario().params);
  const double lf = p.cg_to_front;
  const double lr = p.cg_to_rear();
  const double fz_front = p.mass * kGravity * lr / p.wheelbase;
  const double fz_rear = p.mass * kGravity * lf / p.wheelbase;
  const double c_front = p.tire_front.B * p.tire_front.C * p.tire_front.mu * fz_front;
  const double c_rear = p.tire_rear.B * p.tire_rear.C * p.tire_rear.mu * fz_rear;
  const double understeer = p.mass / p.wheelbase * (lr / c_front - lf / c_rear);
  ASSERT_GT(understeer, 0.0);

  const double v = signal(sim, "veh.chassis.vx");
  EXPECT_NEAR(v, 10.0, 0.05);
  const double expected = v * kRoadWheelAngle / (p.wheelbase + understeer * v * v);
  EXPECT_NEAR(signal(sim, "veh.chassis.yaw_rate"), expected, 0.01 * expected);
}

TEST(SingleTrack, LowSpeedTurningIsKinematic) {
  Simulation sim(load_example("sine_steer", {{"driver.steer.type", "const"},
                                             {"driver.steer.value", "90"},
                                             {"driver.speed_hold_kph", "1.8"}}));
  sim.run_for(20.0);
  const double delta = signal(sim, "veh.steer.road_wheel_angle");
  const double wheelbase = sim.scenario().params.get_double("vehicle.wheelbase_m");
  const double vx = signal(sim, "veh.chassis.vx");
  ASSERT_GT(vx, 0.3);
  EXPECT_NEAR(signal(sim, "veh.chassis.yaw_rate") / vx, std::tan(delta) / wheelbase,
              0.02 * std::tan(delta) / wheelbase);
}

TEST(SingleTrack, StaysFiniteUnderAbusiveInputs) {
  Simulation sim(load_example("straight_accel", {{"driver.steer.type", "sine"},
                                                 {"driver.steer.amplitude", "110"},
                                                 {"driver.steer.freq_hz", "0.5"},
                                                 {"driver.brake.type", "step"},
                                                 {"driver.brake.at_s", "6"},
                                                 {"driver.brake.value", "1"}}));
  sim.run_for(10.0);
  for (const auto& column : sim.recorder().columns()) {
    for (const double value : column) {
      ASSERT_TRUE(std::isfinite(value));
    }
  }
  EXPECT_LT(signal(sim, "veh.chassis.speed"), 50.0);
}

}  // namespace
}  // namespace vsim

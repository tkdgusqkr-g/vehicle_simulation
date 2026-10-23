#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "test_support.hpp"
#include "vsim/models/vehicle/thermal.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

TEST(TireThermal, GripWindowAndIdealGasPressure) {
  const TireThermalParams p = TireThermalParams::from_params(load_example("straight_accel").params);
  ASSERT_TRUE(p.enabled);
  EXPECT_DOUBLE_EQ(p.temperature_grip(p.optimum), 1.0);
  EXPECT_NEAR(p.temperature_grip(p.optimum - p.window), 1.0 - p.grip_loss, 1e-12);
  EXPECT_NEAR(p.temperature_grip(p.optimum + p.window), 1.0 - p.grip_loss, 1e-12);
  EXPECT_DOUBLE_EQ(p.temperature_grip(-200.0), p.min_grip);
  // Gauge pressure at the cold temperature is the cold pressure; absolute pressure scales with
  // absolute temperature.
  EXPECT_NEAR(p.pressure(p.cold_temp), p.cold_pressure, 1e-9);
  const double hot = p.pressure(p.cold_temp + 30.0);
  EXPECT_NEAR((hot + 101.325) / (p.cold_pressure + 101.325), (p.cold_temp + 30.0 + 273.15) / (p.cold_temp + 273.15), 1e-12);
  EXPECT_DOUBLE_EQ(p.pressure_grip(p.optimum_pressure), 1.0);
  EXPECT_LT(p.pressure_grip(0.7 * p.optimum_pressure), 1.0);

  // Without the section nothing changes.
  const TireThermalParams off = TireThermalParams::from_params(ParamTree{});
  EXPECT_FALSE(off.enabled);
  EXPECT_THROW(TireThermalParams::from_params(
                   load_example("straight_accel", {{"vehicle.tire.thermal.window_c", "0"}}).params),
               std::invalid_argument);
}

TEST(BrakeThermal, PadCurveAndCooling) {
  const BrakeThermalParams p = BrakeThermalParams::from_params(load_example("straight_accel").params);
  ASSERT_TRUE(p.enabled);
  EXPECT_DOUBLE_EQ(p.pad_mu(-50.0), p.mu_factor.front());
  EXPECT_DOUBLE_EQ(p.pad_mu(5000.0), p.mu_factor.back());
  EXPECT_NEAR(p.pad_mu(0.5 * (p.mu_temp[0] + p.mu_temp[1])), 0.5 * (p.mu_factor[0] + p.mu_factor[1]), 1e-12);
  EXPECT_DOUBLE_EQ(p.cooling_power(25.0, 25.0, 20.0), 0.0);
  EXPECT_GT(p.cooling_power(400.0, 25.0, 20.0), p.cooling_power(400.0, 25.0, 0.0));  // air flow
  EXPECT_THROW(BrakeThermalParams::from_params(
                   load_example("straight_accel", {{"vehicle.brakes.thermal.pad_mu_factor", "[1, 1]"}}).params),
               std::invalid_argument);
}

// The heat in the discs is the brake work (no cooling): what energy.brake_loss counts.
TEST(BrakeThermal, DiscsStoreTheBrakeWork) {
  Simulation sim(load_example("straight_accel", {{"vehicle.brakes.thermal.cooling_w_per_k", "0"},
                                                 {"vehicle.brakes.thermal.cooling_w_per_k_per_mps", "0"},
                                                 {"vehicle.brakes.thermal.emissivity_area_m2", "0"},
                                                 {"vehicle.tire.thermal.rim_brake_w_per_k", "0"},
                                                 {"driver.throttle.type", "step"},
                                                 {"driver.throttle.before", "1"},
                                                 {"driver.throttle.at_s", "4"},
                                                 {"driver.throttle.value", "0"},
                                                 {"driver.brake.type", "step"},
                                                 {"driver.brake.at_s", "4.1"},
                                                 {"driver.brake.value", "0.7"}}));
  sim.run_for(9.0);
  const Scenario sc = load_example("straight_accel");
  const BrakeThermalParams p = BrakeThermalParams::from_params(sc.params);
  double stored = 0.0;
  for (const char* c : {"fl", "fr", "rl", "rr"}) {
    const bool front = c[0] == 'f';
    stored += p.heat_capacity[front ? 0 : 1] * (signal(sim, std::string("veh.wheel.") + c + ".brake_temp") - 25.0);
  }
  const double work = signal(sim, "energy.brake_loss");
  ASSERT_GT(work, 3e4);
  EXPECT_NEAR(stored, work, 1e-3 * work);
  // Front discs take the brake bias.
  EXPECT_GT(signal(sim, "veh.wheel.fl.brake_temp"), signal(sim, "veh.wheel.rl.brake_temp"));
  EXPECT_LT(signal(sim, "veh.brake.balance"), 1.0);
}

// PH-13 in the plant: cold tires grip less and warm up while working.
TEST(TireThermal, ColdTiresGripLessAndWarmUp) {
  Simulation cold(load_example("sine_steer", {{"initial.tire_temp_c", "20"}}));
  Simulation warm(load_example("sine_steer"));
  cold.run_for(10.0);
  warm.run_for(10.0);
  EXPECT_LT(signal(cold, "veh.wheel.fl.tire_grip"), signal(warm, "veh.wheel.fl.tire_grip") - 0.05);
  EXPECT_GT(signal(cold, "veh.wheel.fl.tire_temp"), 20.5);
  EXPECT_GT(signal(cold, "veh.wheel.fl.tire_pressure"), 62.0);
  EXPECT_GT(signal(cold, "veh.wheel.fl.tire_temp"), signal(cold, "veh.wheel.fl.tire_core_temp"));
  // The energy balance is untouched (heat is what the losses already count).
  EXPECT_LT(std::abs(signal(cold, "energy.residual")), 1.0);
}

}  // namespace
}  // namespace vsim

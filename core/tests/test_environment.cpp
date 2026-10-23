#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "test_support.hpp"
#include "vsim/env/wind.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/input_hub.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

struct WindRig {
  SignalBus bus;
  InputHub hub;
  Context ctx;
  WindBlock wind;
  WindRig(const Scenario& sc) : ctx(bus, hub, RealismConfig::from_params(sc.params), 7, 0.001) {
    wind.init(sc.params, ctx);
  }
  std::array<double, 2> at(double t) {
    wind.step(t, bus);
    return {bus.get(bus.require("env.wind.x")), bus.get(bus.require("env.wind.y"))};
  }
};

TEST(Wind, SteadyAndGustDirectionsAndShape) {
  const Scenario sc = load_example("crosswind", {{"environment.wind.speed_mps", "4"},
                                                 {"environment.wind.from_deg", "270"}});
  WindRig rig(sc);
  // From the west: the air moves east.
  std::array<double, 2> v = rig.at(1.0);
  EXPECT_NEAR(v[0], 4.0, 1e-12);
  EXPECT_NEAR(v[1], 0.0, 1e-12);
  // The gust from the north (12 m/s, 2 s from t = 5) peaks halfway, southwards.
  v = rig.at(6.0);
  EXPECT_NEAR(v[0], 4.0, 1e-9);
  EXPECT_NEAR(v[1], -12.0, 1e-9);
  v = rig.at(5.5);
  EXPECT_NEAR(v[1], -6.0, 1e-9);
  v = rig.at(7.5);
  EXPECT_NEAR(v[1], 0.0, 1e-12);
}

TEST(Wind, TurbulenceHasItsSigmaAndFollowsTheSeed) {
  const Scenario sc = load_example("crosswind", {{"environment.wind.gusts.0.speed_mps", "0"},
                                                 {"environment.wind.turbulence.sigma_mps", "1.5"},
                                                 {"environment.wind.turbulence.length_m", "5"}});
  const auto sample = [&] {
    WindRig rig(sc);
    rig.bus.set(rig.bus.require("veh.chassis.vx"), 20.0);
    std::vector<double> u;
    for (int i = 0; i < 200000; ++i) u.push_back(rig.at(i * 0.001)[0]);
    return u;
  };
  const std::vector<double> a = sample(), b = sample();
  EXPECT_EQ(a, b);
  double mean = 0.0, sq = 0.0;
  for (const double x : a) mean += x;
  mean /= static_cast<double>(a.size());
  for (const double x : a) sq += (x - mean) * (x - mean);
  EXPECT_NEAR(std::sqrt(sq / static_cast<double>(a.size())), 1.5, 0.15);
  EXPECT_NEAR(mean, 0.0, 0.15);

  // Explicitly ideal environment: still air.
  Scenario ideal = sc;
  ideal.params.set("realism.environment", "ideal");
  WindRig still(ideal);
  for (int i = 0; i < 100; ++i) EXPECT_EQ(still.at(i * 0.001)[0], 0.0);
}

// Crosswind on the car: side force downwind, nose into the wind (side force behind the CG),
// and the energy balance counts the work the wind does.
TEST(Wind, CrosswindPushesAndYawsTheCar) {
  Simulation sim(load_example("crosswind"));
  sim.run_for(6.0);  // gust peak
  EXPECT_LT(signal(sim, "veh.aero.side_force"), -200.0);
  EXPECT_GT(signal(sim, "veh.aero.yaw_angle"), 0.3);
  EXPECT_GT(signal(sim, "veh.aero.yaw_moment"), 0.0);
  sim.run_for(6.0);
  EXPECT_GT(signal(sim, "veh.chassis.yaw"), 0.0087);
  EXPECT_LT(std::abs(signal(sim, "energy.residual")), 1.0);

  // A headwind adds drag: at the same speed more power to hold it.
  Simulation calm(load_example("crosswind", {{"environment.wind.gusts.0.speed_mps", "0"}}));
  Simulation head(load_example("crosswind", {{"environment.wind.gusts.0.speed_mps", "0"},
                                             {"environment.wind.speed_mps", "8"},
                                             {"environment.wind.from_deg", "90"}}));
  calm.run_for(4.0);
  head.run_for(4.0);
  EXPECT_NEAR(signal(head, "veh.aero.airspeed"), signal(head, "veh.chassis.speed") + 8.0, 0.05);
  EXPECT_GT(signal(head, "veh.aero.drag"), 1.5 * signal(calm, "veh.aero.drag"));
  EXPECT_LT(std::abs(signal(head, "energy.residual")), 1.0);
}

}  // namespace
}  // namespace vsim

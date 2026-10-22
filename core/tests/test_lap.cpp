#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "test_support.hpp"
#include "vsim/driver/profile.hpp"
#include "vsim/track/speed_profile.hpp"
#include "vsim/track/track.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

constexpr double kPi = std::numbers::pi;

std::vector<double> column(const Simulation& sim, const std::string& name) {
  const auto& names = sim.recorder().names();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    throw std::out_of_range("not logged: " + name);
  }
  return sim.recorder().columns()[static_cast<std::size_t>(it - names.begin())];
}

TrackDefinition circle_track(double radius) {
  TrackDefinition def;
  def.name = "circle";
  for (int i = 0; i < 24; ++i) {
    const double a = 2.0 * kPi * i / 24 - kPi / 2;
    def.points.push_back({radius * std::cos(a), radius * (1.0 + std::sin(a)), 0.0, 2.0, 2.0, 0.0});
  }
  def.sector_points = {8, 16};
  return def;
}

TEST(SpeedProfile, CircleIsLimitedByLateralAcceleration) {
  const Track track(circle_track(15.0));
  SpeedLimits limits;
  limits.lateral = 12.0;
  limits.top = 40.0;
  const std::vector<double> v = quasi_steady_speed(track, limits);
  for (const double value : v) {
    ASSERT_NEAR(value, std::sqrt(12.0 * 15.0), 0.05 * std::sqrt(12.0 * 15.0));
  }
}

TEST(SpeedProfile, StraightAcceleratesAndStops) {
  TrackDefinition def;
  def.closed = false;
  def.points = {{0, 0, 0, 2, 2, 0}, {200, 0, 0, 2, 2, 0}};
  const Track track(def);
  SpeedLimits limits;
  limits.accel = 5.0;
  limits.brake = 10.0;
  limits.top = 25.0;
  const std::vector<double> v = quasi_steady_speed(track, limits);
  const auto& samples = track.samples();
  EXPECT_EQ(v.front(), 0.0);
  EXPECT_EQ(v.back(), 0.0);
  for (std::size_t i = 0; i < v.size(); ++i) {
    const double s = samples[i].s;
    const double expected = std::min({std::sqrt(2.0 * 5.0 * s), 25.0, std::sqrt(2.0 * 10.0 * (200.0 - s))});
    ASSERT_NEAR(v[i], expected, 1e-6) << "at s = " << s;
  }
}

TEST(Profile, SweepAndSineWithDwell) {
  const Profile sweep = Profile::from_params(
      ParamTree::from_yaml_string("{type: sweep, amplitude: 2, f0_hz: 1, f1_hz: 3, duration_s: 10, start_s: 1}"), "t");
  EXPECT_EQ(sweep.value(0.5), 0.0);
  EXPECT_NEAR(sweep.value(1.25), 2.0 * std::sin(2.0 * kPi * (0.25 + 2.0 * 0.0625 / 20.0)), 1e-12);
  EXPECT_EQ(sweep.value(11.5), 0.0);

  const Profile dwell = Profile::from_params(
      ParamTree::from_yaml_string("{type: sine_dwell, amplitude: 60, freq_hz: 0.5, dwell_s: 0.5}"), "t");
  EXPECT_NEAR(dwell.value(0.5), 60.0, 1e-9);   // first peak
  EXPECT_NEAR(dwell.value(1.5), -60.0, 1e-9);  // second peak ...
  EXPECT_NEAR(dwell.value(1.9), -60.0, 1e-9);  // ... held
  EXPECT_NEAR(dwell.value(2.5), 0.0, 1e-9);    // cycle ends after the dwell
  EXPECT_EQ(dwell.value(3.0), 0.0);
}

TEST(LapTimer, StartsOnTheTrackBehindTheLine) {
  Simulation sim(load_example("autocross_auto"));
  const std::shared_ptr<const Track> track = sim.track();
  ASSERT_TRUE(track);
  const CurveSample start = track->at(track->start_s() - 5.0);
  EXPECT_NEAR(signal(sim, "veh.chassis.x"), start.x, 1e-3);
  EXPECT_NEAR(signal(sim, "veh.chassis.y"), start.y, 1e-3);
  EXPECT_NEAR(std::remainder(signal(sim, "veh.chassis.yaw") - start.heading, 2.0 * kPi), 0.0, 1e-3);
  sim.step();
  EXPECT_NEAR(signal(sim, "track.distance"), -5.0, 0.05);
  // An explicit initial pose wins.
  Simulation placed(load_example("autocross_auto", {{"initial.x_m", "1"}, {"initial.y_m", "2"}}));
  EXPECT_DOUBLE_EQ(signal(placed, "veh.chassis.x"), 1.0);
}

// FS acceleration: the 75 m time is the time between crossing the start
// and the finish line, interpolated between steps.
TEST(LapTimer, TimesTheAccelerationRun) {
  Simulation sim(load_example("acceleration"));
  sim.run_to_end();
  EXPECT_EQ(signal(sim, "track.finished"), 1.0);
  EXPECT_EQ(signal(sim, "track.lap"), 1.0);
  const std::vector<double> t = sim.recorder().times();
  const std::vector<double> d = column(sim, "track.distance");
  const auto crossing = [&](double mark) {
    for (std::size_t i = 1; i < d.size(); ++i) {
      if (d[i - 1] < mark && d[i] >= mark) {
        return t[i - 1] + (mark - d[i - 1]) / (d[i] - d[i - 1]) * (t[i] - t[i - 1]);
      }
    }
    return -1.0;
  };
  const double expected = crossing(75.0) - crossing(0.0);
  EXPECT_NEAR(signal(sim, "track.last_lap"), expected, 1e-3);
  EXPECT_GT(expected, 3.0);
  EXPECT_LT(expected, 4.5);
}

// Autonomous laps: Pure Pursuit on the racing line with the QSS profile.
TEST(Pursuit, DrivesCleanRepeatableLaps) {
  Simulation sim(load_example("autocross_auto"));
  sim.run_to_end();
  ASSERT_GE(signal(sim, "track.lap"), 3.0);
  EXPECT_EQ(signal(sim, "track.off_track_count"), 0.0);
  EXPECT_EQ(signal(sim, "track.last_lap_valid"), 1.0);
  const std::vector<double> err = column(sim, "drv.line_offset");
  const std::vector<double> t = sim.recorder().times();
  double worst = 0.0;
  for (std::size_t i = 0; i < err.size(); ++i) {
    if (t[i] > 2.0) {
      worst = std::max(worst, std::abs(err[i]));
    }
  }
  EXPECT_LT(worst, 0.8);
  // Flying laps repeat to within a few milliseconds.
  std::vector<double> laps = column(sim, "track.last_lap");
  laps.erase(std::unique(laps.begin(), laps.end()), laps.end());
  ASSERT_GE(laps.size(), 4u);  // 0, lap 1, lap 2, lap 3
  EXPECT_NEAR(laps[2], laps[3], 0.02);
  EXPECT_DOUBLE_EQ(signal(sim, "track.best_lap"), std::min(laps[2], laps[3]) < laps[1] ? std::min(laps[2], laps[3])
                                                                                      : std::min({laps[1], laps[2], laps[3]}));
}

// Skidpad crosses itself; the timer follows the branch and splits the four
// laps into sectors.
TEST(Pursuit, DrivesTheSkidpadAndStopsAtTheEnd) {
  Simulation sim(load_example("skidpad_auto"));
  sim.run_to_end();
  EXPECT_EQ(signal(sim, "track.finished"), 1.0);
  EXPECT_EQ(signal(sim, "track.off_track_count"), 0.0);
  EXPECT_LT(signal(sim, "veh.chassis.speed"), 0.1);
  std::vector<double> sectors = column(sim, "track.last_sector");
  sectors.erase(std::unique(sectors.begin(), sectors.end()), sectors.end());
  ASSERT_EQ(sectors.size(), 5u);  // 0 then four laps
  for (std::size_t i = 2; i < sectors.size(); ++i) {
    EXPECT_GT(sectors[i], 4.0);
    EXPECT_LT(sectors[i], 7.0);
  }
  EXPECT_NEAR(sectors[1] + sectors[2] + sectors[3] + sectors[4], signal(sim, "track.last_lap"), 1e-6);
}

TEST(LapTimer, DetectsLeavingTheTrack) {
  // Driving straight on a circle pad leaves it.
  Simulation sim(load_example("straight_accel", {{"track.template", "circle_r15"},
                                                 {"driver.throttle.value", "0.3"}}));
  sim.run_for(5.0);
  EXPECT_EQ(signal(sim, "track.off_track"), 1.0);
  EXPECT_GE(signal(sim, "track.off_track_count"), 1.0);
  EXPECT_EQ(signal(sim, "track.lap_valid"), 0.0);
}

TEST(Simulation, TrackCanBeReplacedAtRunTime) {
  Simulation sim(load_example("autocross_auto"));
  sim.run_for(1.0);
  auto circle = std::make_shared<const Track>(circle_track(20.0));
  sim.set_track(circle);
  EXPECT_EQ(sim.tick(), 0);
  EXPECT_EQ(sim.track()->name(), "circle");
  sim.run_for(30.0);
  EXPECT_GE(signal(sim, "track.lap"), 2.0);
  sim.set_track(nullptr);
  EXPECT_EQ(sim.track()->name(), "autocross example");
}

TEST(Simulation, TrackErrors) {
  EXPECT_THROW(Simulation(load_example("straight_accel", {{"driver.type", "pursuit"}})), std::invalid_argument);
  EXPECT_THROW(Simulation(load_example("straight_accel", {{"track.template", "no_such_track"}})),
               std::invalid_argument);
}

}  // namespace
}  // namespace vsim

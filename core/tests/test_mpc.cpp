#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "test_support.hpp"
#include "vsim/driver/mpc.hpp"
#include "vsim/driver/qp.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/input_hub.hpp"
#include "vsim/track/track.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

// --- QP solver ------------------------------------------------------------------------------
TEST(Qp, UnconstrainedMinimum) {
  QuadraticProgram qp;
  qp.resize(2, 0);
  qp.h(0, 0) = 4.0;
  qp.h(0, 1) = qp.h(1, 0) = 1.0;
  qp.h(1, 1) = 2.0;
  qp.g = {1.0, 1.0};
  const QpResult r = solve_qp(qp);
  ASSERT_TRUE(r.converged);
  // H z = -g: z = -[[4, 1], [1, 2]]^-1 [1, 1] = -[1, 3] / 7.
  EXPECT_NEAR(r.z[0], -1.0 / 7.0, 1e-6);
  EXPECT_NEAR(r.z[1], -3.0 / 7.0, 1e-6);
}

TEST(Qp, BoundsAndRows) {
  // min (x - 3)^2 + (y - 3)^2  s.t.  x + y <= 2,  x <= 0.5.
  QuadraticProgram qp;
  qp.resize(2, 1);
  qp.h(0, 0) = qp.h(1, 1) = 2.0;
  qp.g = {-6.0, -6.0};
  qp.a(0, 0) = qp.a(0, 1) = 1.0;
  qp.b = {2.0};
  qp.ub[0] = 0.5;
  const QpResult r = solve_qp(qp);
  ASSERT_TRUE(r.converged);
  EXPECT_NEAR(r.z[0], 0.5, 1e-5);
  EXPECT_NEAR(r.z[1], 1.5, 1e-5);
  EXPECT_LT(r.iterations, 25);
}

TEST(Qp, DegenerateAndInfeasible) {
  // A free variable with no curvature but a bound: it sits on the bound the gradient points to.
  QuadraticProgram qp;
  qp.resize(1, 0);
  qp.g = {1.0};
  qp.lb[0] = -2.0;
  qp.ub[0] = 5.0;
  QpResult r = solve_qp(qp);
  ASSERT_TRUE(r.converged);
  EXPECT_NEAR(r.z[0], -2.0, 1e-5);

  // x >= 1 and x <= -1 cannot both hold.
  qp.resize(1, 2);
  qp.h(0, 0) = 1.0;
  qp.a(0, 0) = -1.0;
  qp.a(1, 0) = 1.0;
  qp.b = {-1.0, -1.0};
  r = solve_qp(qp);
  EXPECT_FALSE(r.converged);
}

// --- Model ----------------------------------------------------------------------------------
TEST(MpcModel, AxlesComeFromTheTyreData) {
  const Scenario sc = load_example("autocross_auto");
  const MpcModel m = MpcModel::from_params(sc.params);
  for (const MpcModel::Axle* a : {&m.front, &m.rear}) {
    // The example tyre: mu_y 1.6, load sensitivity -0.1; load transfer and the 0.95 the
    // driver keeps in hand bring the axle below that.
    EXPECT_GT(a->mu, 1.2);
    EXPECT_LT(a->mu, 1.6);
    EXPECT_LT(a->sensitivity, 0.0);
    EXPECT_GT(a->peak_slip(), 0.08);
    EXPECT_LT(a->peak_slip(), 0.3);
    EXPECT_GT(a->c, 1.0);
  }
  EXPECT_GT(m.grip(25.0), m.grip(5.0));  // downforce
  EXPECT_LE(m.ax_max(25.0), m.power / (m.mass * 25.0) + 1e-9);
  EXPECT_NEAR(m.drive_force, 4 * 21.0 * 14.0 * 0.97 / 0.203, 1.0);

  const MpcModel fixed = MpcModel::from_params(load_example("autocross_auto", {{"driver.model.mu", "1.2"}}).params);
  EXPECT_DOUBLE_EQ(fixed.front.mu, 1.2);
  EXPECT_DOUBLE_EQ(fixed.rear.mu, 1.2);
  EXPECT_THROW(MpcModel::from_params(load_example("autocross_auto", {{"driver.model.grip_scale", "0"}}).params),
               std::invalid_argument);
}

// Integrates the model at constant input on a straight centreline until it settles.
MpcModel::State settle(const MpcModel& m, MpcModel::State x, MpcModel::Input u, double seconds) {
  for (int i = 0; i < static_cast<int>(seconds / 1e-4); ++i) {
    const MpcModel::State d = m.derivative(x, u, 0.0);
    for (std::size_t j = 0; j < x.size(); ++j) x[j] += 1e-4 * d[j];
    x[MpcModel::Vx] -= 1e-4 * d[MpcModel::Vx];  // hold the speed
  }
  return x;
}

TEST(MpcModel, CornersLikeASingleTrack) {
  const MpcModel m = MpcModel::from_params(load_example("autocross_auto").params);
  const double wheelbase = m.lf + m.lr;
  // Walking speed: kinematic, it rolls where the wheels point.
  MpcModel::State x = settle(m, {0, 0, 0, 0.9, 0, 0}, {0.2, 0.0}, 3.0);
  const double beta = std::atan(m.lr * std::tan(0.2) / wheelbase);
  EXPECT_NEAR(x[MpcModel::R], 0.9 * std::cos(beta) * std::tan(0.2) / wheelbase, 1e-3);
  // A small angle at speed: close to kinematic, a little understeer or oversteer.
  x = settle(m, {0, 0, 0, 12.0, 0, 0}, {0.02, 0.0}, 3.0);
  EXPECT_NEAR(x[MpcModel::R] / (12.0 * 0.02 / wheelbase), 1.0, 0.15);
  // Full lock at speed: the lateral acceleration stops at what the tyres give.
  x = settle(m, {0, 0, 0, 15.0, 0, 0}, {0.4, 0.0}, 3.0);
  const double ay = x[MpcModel::Vx] * x[MpcModel::R];
  EXPECT_GT(ay, 0.8 * m.grip(15.0));
  EXPECT_LT(ay, 1.1 * m.grip(15.0) / 0.95);
  // Combined slip: driving hard leaves the tyres less to corner with.
  const MpcModel::State driving = settle(m, {0, 0, 0, 15.0, 0, 0}, {0.4, 0.8 * m.grip(15.0)}, 3.0);
  EXPECT_LT(driving[MpcModel::Vx] * driving[MpcModel::R], 0.75 * ay);
  // Past the peak slip angle more lock gains nothing (the driver's curve is flat there).
  const MpcModel::State more = settle(m, {0, 0, 0, 15.0, 0, 0}, {0.6, 0.0}, 3.0);
  EXPECT_LE(more[MpcModel::Vx] * more[MpcModel::R], ay);
}

// --- Driver on its own model ----------------------------------------------------------------
struct SelfLoop {
  Scenario sc;
  std::shared_ptr<Track> track;
  SignalBus bus;
  InputHub hub;
  std::unique_ptr<Context> ctx;
  MpcDriver driver;
  MpcModel::State x{};
  double max_offset = 0.0, max_speed_error = 0.0, max_slip = 0.0;

  explicit SelfLoop(Scenario::Overrides overrides) {
    overrides.emplace("driver.type", "mpc");
    overrides.emplace("driver.model.learn", "false");  // the plant is the driver's own model
    sc = load_example("autocross_auto", overrides);
    track = std::make_shared<Track>(
        TrackDefinition::from_file(std::string(VSIM_SOURCE_DIR) + "/templates/tracks/autocross.track.json"));
    ctx = std::make_unique<Context>(bus, hub, RealismConfig::from_params(sc.params), 1, 0.001);
    ctx->set_track(track);
    driver.init(sc.params, *ctx);
    x = {track->start_s() + 1.0, 0.0, 0.0, 8.0, 0.0, 0.0};
  }

  void run(double seconds) {
    const MpcModel& m = driver.model();
    const auto set = [&](const char* name, double v) { bus.set(bus.require(name), v); };
    for (int i = 0; i < static_cast<int>(seconds / 0.001); ++i) {
      const double t = i * 0.001;
      const CurveSample c = track->at(x[MpcModel::S]);
      set("veh.chassis.x", c.x - x[MpcModel::N] * std::sin(c.heading));
      set("veh.chassis.y", c.y + x[MpcModel::N] * std::cos(c.heading));
      set("veh.chassis.yaw", c.heading + x[MpcModel::Dpsi]);
      set("veh.chassis.vx", x[MpcModel::Vx]);
      set("veh.chassis.vy", x[MpcModel::Vy]);
      set("veh.chassis.yaw_rate", x[MpcModel::R]);
      if (i % 10 == 0) {
        driver.step(t, bus);
      }
      const MpcModel::Input u = {bus.get(bus.require("drv.cmd.steer")) / 4.5, bus.get(bus.require("drv.mpc.ax_cmd"))};
      const MpcModel::State d = m.derivative(x, u, c.curvature);
      set("veh.chassis.ax", d[MpcModel::Vx] - x[MpcModel::Vy] * x[MpcModel::R]);
      set("veh.chassis.ay", d[MpcModel::Vy] + x[MpcModel::Vx] * x[MpcModel::R]);
      for (std::size_t j = 0; j < x.size(); ++j) x[j] += 0.001 * d[j];
      if (t > 2.0) {
        max_offset = std::max(max_offset, std::abs(bus.get(bus.require("drv.line_offset"))));
        max_speed_error = std::max(max_speed_error, std::abs(x[MpcModel::Vx] - bus.get(bus.require("drv.mpc.v_ref"))));
        max_slip = std::max(max_slip, std::abs(x[MpcModel::Vy]) / std::max(x[MpcModel::Vx], 1.0));
      }
    }
  }
};

TEST(MpcDriver, FollowsTheLineOnItsOwnModel) {
  SelfLoop loop({{"driver.human.enabled", "false"}});
  loop.run(15.0);
  EXPECT_LT(loop.max_offset, 0.35);
  EXPECT_LT(loop.max_speed_error, 2.5);
  EXPECT_LT(loop.max_slip, 0.12);
  EXPECT_EQ(loop.driver.failures(), 0);
  EXPECT_EQ(loop.driver.fallbacks(), 0);
  EXPECT_GT(loop.x[MpcModel::S], 200.0);
  // The plan starts where the car is and spans the horizon.
  const auto plan = loop.driver.planned();
  ASSERT_EQ(plan.size(), 30u);
  EXPECT_GT(plan.back()[MpcModel::S], plan.front()[MpcModel::S] + 10.0);
}

TEST(MpcDriver, RacingGetsFurtherThanTracking) {
  SelfLoop tracking({{"driver.human.enabled", "false"}});
  SelfLoop racing({{"driver.mode", "race"}});
  tracking.run(12.0);
  racing.run(12.0);
  EXPECT_GT(racing.x[MpcModel::S], tracking.x[MpcModel::S] + 3.0);
  EXPECT_EQ(racing.driver.fallbacks(), 0);
  // Still on the track: the car's edge inside the borders (1.75 m half width).
  EXPECT_LT(racing.max_offset, 1.5);
}

TEST(MpcDriver, HumanCharacteristicsStillFollow) {
  SelfLoop loop({});  // tracking mode: delay, neuromuscular lag, rate limit, noise
  loop.run(15.0);
  EXPECT_LT(loop.max_offset, 0.8);
  EXPECT_EQ(loop.driver.fallbacks(), 0);
}

// --- On the car -----------------------------------------------------------------------------
TEST(MpcDriver, DrivesTheCarRepeatably) {
  const auto run = [] {
    Simulation sim(load_example("autocross_mpc"));
    sim.run_for(6.0);
    return std::vector<double>{signal(sim, "drv.cmd.steer"), signal(sim, "drv.cmd.throttle"),
                               signal(sim, "drv.cmd.brake"), signal(sim, "veh.chassis.x"),
                               signal(sim, "drv.mpc.front_gain"), signal(sim, "drv.mpc.rear_gain")};
  };
  const std::vector<double> a = run(), b = run();
  EXPECT_EQ(a, b);
  EXPECT_GT(a[3], 0.0);
  EXPECT_NE(a[4], 1.0);  // the driver has started to learn the car
  EXPECT_NE(a[5], 1.0);
}

TEST(MpcDriver, Errors) {
  EXPECT_THROW(Simulation(load_example("straight_accel", {{"driver.type", "mpc"}})), std::invalid_argument);
  EXPECT_THROW(Simulation(load_example("autocross_mpc", {{"driver.mode", "fast"}})), std::invalid_argument);
  EXPECT_THROW(Simulation(load_example("autocross_mpc", {{"driver.steps", "2"}})), std::invalid_argument);
  EXPECT_THROW(Simulation(load_example("autocross_mpc", {{"driver.profile.lateral_share", "1.1"}})),
               std::invalid_argument);
  EXPECT_THROW(Simulation(load_example("autocross_mpc", {{"driver.speed_profile", "missing.csv"}})),
               std::invalid_argument);
}

}  // namespace
}  // namespace vsim

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "test_support.hpp"
#include "vsim/models/vehicle/suspension.hpp"

namespace vsim {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

// Equal, parallel, horizontal wishbones: the upright translates without
// turning, so camber and toe stay put and the roll centre is on the ground.
WishboneGeometry parallel_arms() {
  WishboneGeometry g;
  g.lower_front = {0.15, 0.20, 0.10};
  g.lower_rear = {-0.15, 0.20, 0.10};
  g.lower_outer = {0.0, 0.55, 0.10};
  g.upper_front = {0.15, 0.20, 0.30};
  g.upper_rear = {-0.15, 0.20, 0.30};
  g.upper_outer = {0.0, 0.55, 0.30};
  g.tie_inner = {0.08, 0.20, 0.20};
  g.tie_outer = {0.08, 0.55, 0.20};
  g.push_outer = {0.0, 0.50, 0.10};
  g.push_inner = {0.0, 0.25, 0.45};
  g.wheel_centre = {0.0, 0.60, 0.20};
  return g;
}

TEST(Suspension, DesignPoseIsTheHardpoints) {
  WishboneGeometry g = WishboneGeometry::from_params(test::load_example("drive").params, true);
  g.static_camber = -1.5 * kDeg;
  g.static_toe = 0.2 * kDeg;
  const WishboneSolver solver(g);
  const CornerPose p = solver.solve(0.0, 0.0);
  EXPECT_NEAR((p.wheel_centre - g.wheel_centre).norm(), 0.0, 1e-9);
  EXPECT_NEAR((p.lower_ball - g.lower_outer).norm(), 0.0, 1e-9);
  EXPECT_NEAR((p.upper_ball - g.upper_outer).norm(), 0.0, 1e-9);
  EXPECT_NEAR(p.camber, -1.5 * kDeg, 1e-9);
  EXPECT_NEAR(p.steer, -0.2 * kDeg, 1e-9);  // toe-in turns the left wheel right
  EXPECT_NEAR(p.contact_patch.z, 0.0, 1e-3);
}

TEST(Suspension, ParallelEqualArmsKeepCamberAndTrack) {
  const WishboneSolver solver(parallel_arms());
  for (const double travel : {-0.04, -0.02, 0.02, 0.04}) {
    const CornerPose p = solver.solve(travel, 0.0);
    EXPECT_NEAR(p.wheel_centre.z - 0.20, travel, 1e-9);
    EXPECT_NEAR(p.camber, 0.0, 1e-9) << travel;
    EXPECT_NEAR(p.steer, 0.0, 1e-9) << travel;
    // The arms swing on 0.35 m radii: the wheel comes in by r (1 - cos).
    const double r = 0.35;
    EXPECT_NEAR(p.wheel_centre.y, 0.60 - (r - std::sqrt(r * r - travel * travel)), 1e-9);
  }
  const KcSummary s = summarize(solver, 0.6, 1.53, true);
  EXPECT_NEAR(s.roll_centre_height_m, 0.0, 1e-6);
  EXPECT_NEAR(s.camber_gain_deg_per_m, 0.0, 1e-6);
  EXPECT_NEAR(s.kingpin_inclination_deg, 0.0, 1e-9);
}

// Arms converging inboard: the roll centre is where the line from the
// contact patch through the front-view instant centre crosses the centreline.
TEST(Suspension, RollCentreFromTheInstantCentre) {
  WishboneGeometry g = parallel_arms();
  g.upper_front = {0.15, 0.25, 0.30};
  g.upper_rear = {-0.15, 0.25, 0.30};
  g.upper_outer = {0.0, 0.53, 0.31};
  const WishboneSolver solver(g);
  // Lines in the front view (y, z): lower z = 0.10; upper through (0.25, 0.30)
  // and (0.53, 0.31).
  const double slope = (0.31 - 0.30) / (0.53 - 0.25);
  const double ic_y = 0.25 + (0.10 - 0.30) / slope;
  const double ic_z = 0.10;
  const CornerPose p = solver.solve(0.0, 0.0);
  const double cp_y = p.contact_patch.y;
  const double rc = ic_z * cp_y / (cp_y - ic_y);
  const KcSummary s = summarize(solver, cp_y, 1.53, true);
  EXPECT_GT(rc, 0.005);
  EXPECT_NEAR(s.roll_centre_height_m, rc, 1e-4);
  EXPECT_LT(s.camber_gain_deg_per_m, 0.0);  // instant centre inboard: negative camber in bump
}

TEST(Suspension, SteeringAxisFigures) {
  // Example front corner: kingpin from (0, 0.555, 0.115) to (-0.012, 0.535, 0.315).
  WishboneGeometry g = WishboneGeometry::from_params(test::load_example("drive").params, true);
  g.static_camber = 0.0;  // contact patch right below the hub
  const WishboneSolver solver(g);
  const KcSummary s = summarize(solver, 0.61, 1.53, true);
  EXPECT_NEAR(s.kingpin_inclination_deg, std::atan2(0.020, 0.200) / kDeg, 1e-6);
  EXPECT_NEAR(s.caster_deg, std::atan2(0.012, 0.200) / kDeg, 1e-6);
  // The axis meets the ground at y = 0.555 + 0.02 * 0.115 / 0.2, x = 0.012 * 0.115 / 0.2.
  EXPECT_NEAR(s.scrub_radius_m, 0.61 - (0.555 + 0.02 * 0.575), 1e-6);
  EXPECT_NEAR(s.mechanical_trail_m, 0.012 * 0.575, 1e-6);
  EXPECT_GT(s.motion_ratio, 0.0);
  // Steering: moving the rack left turns the left wheel left.
  const CornerPose left = solver.solve(0.0, 0.01);
  EXPECT_GT(left.steer, 0.05);
}

// Rocker on an axis along x: a vertical push rod and a horizontal spring at
// right angles on the rocker give wheel / spring = (lower arm / push rod
// point radius) * (push arm / spring arm).
TEST(Suspension, RockerMotionRatio) {
  WishboneGeometry g = parallel_arms();
  g.push_outer = {0.0, 0.45, 0.10};   // 0.25 m out on the 0.35 m lower arm
  g.push_inner = {0.0, 0.45, 0.40};   // vertical push rod
  g.has_rocker = true;
  g.rocker_pivot = {0.0, 0.37, 0.40};  // push arm 0.08 m
  g.rocker_axis = {1.0, 0.0, 0.0};
  g.spring_rocker = {0.0, 0.37, 0.44};  // spring arm 0.04 m
  g.spring_chassis = {0.0, 0.07, 0.44};
  const WishboneSolver solver(g);
  const KcSummary s = summarize(solver, 0.6, 1.53, false);
  const double expected = (0.35 / 0.25) * (0.08 / 0.04);
  EXPECT_NEAR(s.motion_ratio, expected, 0.01 * expected);
  // Bump compresses the spring.
  EXPECT_GT(solver.solve(0.02, 0.0).spring, 0.0);
}

TEST(Suspension, TableMatchesTheSolverAndItsSlopes) {
  const WishboneSolver solver(WishboneGeometry::from_params(test::load_example("drive").params, true));
  const KcTable table = KcTable::build(solver, 0.06, 49, 0.04, 25);
  for (const double travel : {-0.05, 0.0, 0.0125, 0.035}) {
    for (const double rack : {-0.03, 0.0, 0.02}) {
      const CornerPose p = solver.solve(travel, rack);
      const KcPoint k = table.at(travel, rack);
      EXPECT_NEAR(k.steer, p.steer, 2e-4) << travel << " " << rack;
      EXPECT_NEAR(k.camber, p.camber, 2e-4);
      EXPECT_NEAR(k.spring, p.spring, 1e-5);
    }
  }
  // The travel slopes are the exact derivatives of the interpolated values.
  const double h = 1e-6;
  for (const double travel : {-0.0301, 0.0111, 0.07}) {  // inside cells, and beyond the table
    const KcPoint k = table.at(travel, 0.005);
    EXPECT_NEAR(k.dspring, (table.at(travel + h, 0.005).spring - table.at(travel - h, 0.005).spring) / (2 * h), 1e-6);
    EXPECT_NEAR(k.dcp_y, (table.at(travel + h, 0.005).cp_y - table.at(travel - h, 0.005).cp_y) / (2 * h), 1e-6);
  }
}

TEST(Suspension, ImpossibleLinkageIsReported) {
  WishboneGeometry g = parallel_arms();
  g.tie_inner = {0.08, -2.0, 0.20};  // tie rod longer than any upright position allows at large rack
  const WishboneSolver solver(g);
  EXPECT_THROW(solver.solve(0.0, 3.0), std::runtime_error);
}

}  // namespace
}  // namespace vsim

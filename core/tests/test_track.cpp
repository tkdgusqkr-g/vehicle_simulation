#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <stdexcept>

#include "vsim/track/track.hpp"

namespace vsim {
namespace {

constexpr double kPi = std::numbers::pi;

TrackDefinition circle(double radius, int points = 8, double width = 1.5) {
  TrackDefinition def;
  def.name = "circle";
  def.closed = true;
  for (int i = 0; i < points; ++i) {
    const double a = 2.0 * kPi * i / points;
    TrackDefinition::Point p;
    p.x = radius * std::cos(a);
    p.y = radius * std::sin(a);
    p.w_left = width;
    p.w_right = width;
    def.points.push_back(p);
  }
  return def;
}

TEST(Track, CircleHasTheRightLengthAndCurvature) {
  // 16 points: a cubic spline through them is within ~1.5 % of a circle.
  const Track track(circle(20.0, 16));
  EXPECT_NEAR(track.length(), 2.0 * kPi * 20.0, 0.005 * 2.0 * kPi * 20.0);
  for (const CurveSample& s : track.samples()) {
    ASSERT_NEAR(s.curvature, 1.0 / 20.0, 0.02 / 20.0) << "at s = " << s.s;
    ASSERT_NEAR(std::hypot(s.x, s.y), 20.0, 0.05);
    // Counter-clockwise: heading is the position angle + 90 degrees.
    ASSERT_NEAR(std::remainder(s.heading - std::atan2(s.y, s.x) - kPi / 2.0, 2.0 * kPi), 0.0, 0.01);
  }
  EXPECT_NEAR(track.samples()[1].s - track.samples()[0].s, Track::kStep, 0.01);
  EXPECT_DOUBLE_EQ(track.start_s(), 0.0);
  EXPECT_DOUBLE_EQ(track.finish_s(), 0.0);
  EXPECT_NEAR(track.point_s(4), track.length() / 4.0, 0.01 * track.length());
}

TEST(Track, StraightOpenTrack) {
  TrackDefinition def;
  def.closed = false;
  def.points = {{0, 0, 0, 2, 2, 0}, {100, 0, 0, 4, 2, 0}};
  const Track track(def);
  EXPECT_NEAR(track.length(), 100.0, 1e-9);
  EXPECT_DOUBLE_EQ(track.finish_s(), 100.0);
  for (const CurveSample& s : track.samples()) {
    ASSERT_NEAR(s.curvature, 0.0, 1e-9);
    ASSERT_NEAR(s.y, 0.0, 1e-9);
  }
  // Widths are linear between control points.
  EXPECT_NEAR(track.widths_at(50.0).left, 3.0, 1e-9);
  EXPECT_NEAR(track.widths_at(50.0).right, 2.0, 1e-9);
  // Beyond the ends: clamped.
  EXPECT_NEAR(track.at(150.0).x, 100.0, 1e-9);
}

TEST(Track, ProjectionGivesArcLengthAndSignedOffset) {
  const Track track(circle(20.0));
  // Outside a counter-clockwise circle is to the right of the direction of travel.
  const CurveProjection out = track.project(21.0 * std::cos(0.5), 21.0 * std::sin(0.5));
  EXPECT_NEAR(out.n, -1.0, 0.02);
  EXPECT_NEAR(out.s, 0.5 / (2.0 * kPi) * track.length(), 0.05);
  const CurveProjection in = track.project(18.5 * std::cos(3.0), 18.5 * std::sin(3.0), out.s + 40.0, 30.0);
  EXPECT_NEAR(in.n, 1.5, 0.02);
  // Wrapping around the start.
  const CurveProjection near_start = track.project(20.0, -0.5, 0.2);
  EXPECT_NEAR(near_start.s, track.length() - 0.5, 0.05);
}

// A figure eight crosses itself; the hint keeps the projection on the branch
// the car is on.
TEST(Track, HintKeepsTheBranchWhereTheCourseCrosses) {
  TrackDefinition def;
  def.closed = true;
  const double r = 10.0;
  for (int i = 0; i < 16; ++i) {
    const double a = 2.0 * kPi * i / 16;
    def.points.push_back({r * std::sin(a), r * std::sin(a) * std::cos(a), 0, 1, 1, 0});  // lemniscate-like
  }
  const Track track(def);
  const double first = track.project(0.0, 0.0, 0.5).s;
  const double second = track.project(0.0, 0.0, track.length() / 2.0 + 0.5).s;
  EXPECT_LT(std::abs(first - 0.0), 1.0);
  EXPECT_LT(std::abs(second - track.length() / 2.0), 1.0);
}

TEST(Track, LinesFollowTheirOffsets) {
  TrackDefinition def = circle(20.0, 12, 3.0);
  def.lines.push_back({"inside", {{0.0, 1.0}}});
  def.lines.push_back({"wave", {{0.0, 0.0}, {0.25, 1.0}, {0.5, 0.0}, {0.75, -1.0}}});
  const Track track(def);
  const SampledCurve inside = track.line("inside");
  EXPECT_NEAR(inside.length(), 2.0 * kPi * 19.0, 0.01 * 2.0 * kPi * 19.0);
  EXPECT_NEAR(track.line_offset(def.lines[1], 0.25 * track.length()), 1.0, 1e-9);
  EXPECT_NEAR(track.line_offset(def.lines[1], 0.75 * track.length()), -1.0, 1e-9);
  EXPECT_NEAR(track.line_offset(def.lines[1], 0.125 * track.length()), std::sin(kPi / 4.0), 0.1);
  EXPECT_EQ(track.line("center").samples().size(), track.samples().size());
  EXPECT_THROW(track.line("nope"), std::invalid_argument);
}

TEST(Track, JsonRoundTripAndCsv) {
  TrackDefinition def = circle(15.0);
  def.name = "round trip";
  def.sector_points = {2, 5};
  def.start_point = 1;
  def.lines.push_back({"racing", {{0.1, 0.5}, {0.6, -0.5}}});
  const TrackDefinition back = TrackDefinition::from_json(def.to_json());
  EXPECT_EQ(back.name, "round trip");
  EXPECT_EQ(back.points.size(), def.points.size());
  EXPECT_DOUBLE_EQ(back.points[3].x, def.points[3].x);
  EXPECT_EQ(back.sector_points, def.sector_points);
  EXPECT_EQ(back.start_point, 1);
  ASSERT_EQ(back.lines.size(), 1u);
  EXPECT_EQ(back.lines[0].offsets, def.lines[0].offsets);

  const auto file = std::filesystem::temp_directory_path() /
                    ("vsim_track_" + std::to_string(std::random_device{}()) + ".csv");
  {
    std::ofstream out(file);
    out << "# x_m,y_m,w_tr_right_m,w_tr_left_m\n0,0,2,3\n50,0,2,3\n50,30,2,3\n0,30,2,3\n";
  }
  const TrackDefinition csv = TrackDefinition::from_file(file);
  std::filesystem::remove(file);
  ASSERT_EQ(csv.points.size(), 4u);
  EXPECT_TRUE(csv.closed);
  EXPECT_DOUBLE_EQ(csv.points[1].w_right, 2.0);
  EXPECT_DOUBLE_EQ(csv.points[1].w_left, 3.0);
}

TEST(Track, RejectsBadDefinitions) {
  EXPECT_THROW(TrackDefinition::from_json("{\"schema_version\": 2, \"points\": []}"), std::invalid_argument);
  EXPECT_THROW(TrackDefinition::from_json("not json"), std::invalid_argument);
  TrackDefinition def = circle(10.0);
  def.points[3] = def.points[2];
  EXPECT_THROW(def.validate(), std::invalid_argument);
  def = circle(10.0);
  def.sector_points = {42};
  EXPECT_THROW(def.validate(), std::invalid_argument);
  def = circle(10.0);
  def.points.resize(2);
  EXPECT_THROW(def.validate(), std::invalid_argument);
  def = circle(10.0);
  def.lines.push_back({"center", {}});
  EXPECT_THROW(def.validate(), std::invalid_argument);
}

TEST(Track, WarnsAboutTightCornersAndFoldedBoundaries) {
  EXPECT_TRUE(Track(circle(20.0)).warnings().empty());
  const std::vector<std::string> tight = Track(circle(3.5, 8, 4.0)).warnings();
  EXPECT_EQ(tight.size(), 2u);
}

}  // namespace
}  // namespace vsim

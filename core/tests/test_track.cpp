#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

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

// --- Road surface (M5) ----------------------------------------------------------------

TrackDefinition straight_road(double length, double grade = 0.0) {
  TrackDefinition def;
  def.name = "straight road";
  def.closed = false;
  const int n = static_cast<int>(length / 50.0);
  for (int i = 0; i <= n; ++i) {
    TrackDefinition::Point p;
    p.x = 50.0 * i;
    p.z = grade * p.x;
    p.w_left = p.w_right = 3.0;
    def.points.push_back(p);
  }
  return def;
}

TEST(Track, ElevationAndBankShapeTheSurface) {
  TrackDefinition def = circle(30.0, 12, 4.0);
  for (std::size_t i = 0; i < def.points.size(); ++i) {
    def.points[i].z = 2.0 * std::sin(2.0 * kPi * static_cast<double>(i) / 12.0);
    def.points[i].bank_deg = 6.0;  // left edge (inside of the circle) up
  }
  const Track track(def);
  EXPECT_TRUE(track.has_relief());
  // The centreline passes through the control points' heights.
  EXPECT_NEAR(track.elevation(track.point_s(3)), def.points[3].z, 1e-4);  // 0.25 m samples
  EXPECT_NEAR(track.elevation(track.point_s(7)), def.points[7].z, 1e-4);
  // Cross slope: 1 m left of the centreline is tan(6 deg) higher.
  const CurveSample c = track.at(track.point_s(2));
  const double nx = -std::sin(c.heading), ny = std::cos(c.heading);
  const RoadPoint mid = track.road(c.x, c.y);
  const RoadPoint left = track.road(c.x + nx, c.y + ny);
  EXPECT_NEAR(left.z - mid.z, std::tan(6.0 * kPi / 180.0), 2e-3);
  EXPECT_EQ(mid.kind, SurfaceKind::Road);
  // The gradient is the slope of the height.
  for (const double s : {5.0, 40.0, 101.0, 150.0}) {
    const CurveSample p = track.at(s);
    const double x = p.x + 0.7 * nx, y = p.y + 0.7 * ny;
    const RoadPoint r = track.road(x, y);
    const double h = 1e-4;
    EXPECT_NEAR(r.dzdx, (track.road(x + h, y).z - track.road(x - h, y).z) / (2 * h), 0.01) << s;
    EXPECT_NEAR(r.dzdy, (track.road(x, y + h).z - track.road(x, y - h).z) / (2 * h), 0.01) << s;
  }
}

TEST(Track, KerbsPatchesAndGrass) {
  TrackDefinition def = straight_road(300.0);
  def.kerbs.push_back({1, 3, "right", 1.2, 0.04});
  def.patches.push_back({4, 5, -1.0, 3.0, 0.4});
  def.surface.grass_mu = 0.55;
  const Track track(def);
  // On the kerb, 0.6 m beyond the right edge, between x = 50 and 150 (ramped in over 1.5 m).
  const RoadPoint kerb = track.road(100.0, -3.6);
  EXPECT_EQ(kerb.kind, SurfaceKind::Kerb);
  EXPECT_GT(kerb.z, 0.03);
  EXPECT_LE(kerb.z, 0.04 + 1e-12);
  EXPECT_LT(kerb.mu, 1.0);
  // The same offset before the kerb starts is grass.
  const RoadPoint grass = track.road(20.0, -3.6);
  EXPECT_EQ(grass.kind, SurfaceKind::Grass);
  EXPECT_DOUBLE_EQ(grass.mu, 0.55);
  // A wet patch between x = 200 and 250, n from -1 to 3.
  EXPECT_DOUBLE_EQ(track.road(225.0, 0.0).mu, 0.4);
  EXPECT_DOUBLE_EQ(track.road(225.0, -2.0).mu, 1.0);
  EXPECT_DOUBLE_EQ(track.road(180.0, 0.0).mu, 1.0);
  // No steps anywhere across the edge (road -> kerb -> grass).
  double previous = track.road(100.0, -2.9).z;
  for (double y = -2.9; y > -6.0; y -= 0.01) {
    const double z = track.road(100.0, y).z;
    EXPECT_LT(std::abs(z - previous), 0.003) << y;
    previous = z;
  }
}

// Minimal radix-2 FFT for the PSD check.
void fft(std::vector<std::complex<double>>& a) {
  const std::size_t n = a.size();
  for (std::size_t i = 1, j = 0; i < n; ++i) {
    std::size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      std::swap(a[i], a[j]);
    }
  }
  for (std::size_t len = 2; len <= n; len <<= 1) {
    const std::complex<double> w = std::polar(1.0, -2.0 * kPi / static_cast<double>(len));
    for (std::size_t i = 0; i < n; i += len) {
      std::complex<double> wk = 1.0;
      for (std::size_t k = 0; k < len / 2; ++k) {
        const std::complex<double> u = a[i + k], v = a[i + k + len / 2] * wk;
        a[i + k] = u + v;
        a[i + k + len / 2] = u - v;
        wk *= w;
      }
    }
  }
}

// V-10: the generated road's displacement PSD follows its ISO 8608 class,
// Gd(n) = Gd(n0) (n / n0)^-2, octave by octave.
TEST(Track, RoughnessFollowsIso8608) {
  TrackDefinition def = straight_road(2000.0);
  def.surface.roughness = "C";
  const Track track(def);
  EXPECT_NEAR(iso8608_gd0("C"), 256e-6, 1e-12);
  constexpr std::size_t kN = 1 << 15;
  constexpr double kStep = 0.05;  // [m]
  std::vector<std::complex<double>> z(kN);
  double window_power = 0.0;
  for (std::size_t i = 0; i < kN; ++i) {
    const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(i) / kN);  // Hann
    z[i] = w * track.road(100.0 + kStep * static_cast<double>(i), 0.3).z;
    window_power += w * w;
  }
  fft(z);
  const double length = kStep * kN;
  for (double n1 = 0.02; n1 < 2.0; n1 *= 2.0) {
    const double n2 = 2.0 * n1;
    double variance = 0.0;
    for (std::size_t k = 1; k < kN / 2; ++k) {
      const double n = static_cast<double>(k) / length;
      if (n >= n1 && n < n2) {
        variance += 2.0 * std::norm(z[k]) / (kN * window_power);
      }
    }
    const double expected = 256e-6 * 0.01 * (1.0 / n1 - 1.0 / n2);  // integral of Gd over the octave
    EXPECT_NEAR(variance, expected, 0.3 * expected) << "octave from " << n1 << " cycles/m";
  }
}

TEST(Track, SurfaceSurvivesJson) {
  TrackDefinition def = straight_road(200.0, 0.02);
  def.points[2].bank_deg = 4.0;
  def.surface.mu = 0.9;
  def.surface.roughness = "B";
  def.kerbs.push_back({1, 2, "left", 0.8, 0.025});
  def.patches.push_back({2, 3, -2.0, 1.0, 0.5});
  const TrackDefinition back = TrackDefinition::from_json(def.to_json());
  EXPECT_DOUBLE_EQ(back.surface.mu, 0.9);
  EXPECT_EQ(back.surface.roughness, "B");
  ASSERT_EQ(back.kerbs.size(), 1u);
  EXPECT_EQ(back.kerbs[0].side, "left");
  EXPECT_DOUBLE_EQ(back.kerbs[0].height, 0.025);
  ASSERT_EQ(back.patches.size(), 1u);
  EXPECT_DOUBLE_EQ(back.patches[0].mu, 0.5);
  EXPECT_DOUBLE_EQ(back.points[2].bank_deg, 4.0);
  // Same name, same bumps.
  EXPECT_DOUBLE_EQ(Track(back).road(77.0, 0.4).z, Track(def).road(77.0, 0.4).z);

  def.surface.roughness = "Q";
  EXPECT_THROW(def.validate(), std::invalid_argument);
  def.surface.roughness.clear();
  def.kerbs[0].side = "middle";
  EXPECT_THROW(def.validate(), std::invalid_argument);
}

}  // namespace
}  // namespace vsim

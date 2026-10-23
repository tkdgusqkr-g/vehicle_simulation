#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string>

#include "vsim/track/track_mesh.hpp"

namespace vsim {
namespace {

Track ring(double radius, double width) {
  TrackDefinition def;
  def.name = "ring";
  def.closed = true;
  for (int i = 0; i < 16; ++i) {
    const double a = 2.0 * std::numbers::pi * i / 16;
    def.points.push_back({radius * std::cos(a), radius * std::sin(a), 0.0, width, width, 0.0});
  }
  def.sector_points = {8};
  return Track(def);
}

const MeshSection* find(const TrackMesh& mesh, const std::string& name) {
  for (const MeshSection& m : mesh.sections) {
    if (m.name == name) {
      return &m;
    }
  }
  return nullptr;
}

// Signed area of a triangle seen from above (positive: counter-clockwise).
double area_from_above(const MeshSection& m, std::size_t t) {
  const auto p = [&](std::size_t k) {
    const std::uint32_t i = m.indices[3 * t + k];
    return std::array<double, 2>{m.positions[3 * i], m.positions[3 * i + 1]};
  };
  const auto a = p(0), b = p(1), c = p(2);
  return 0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]));
}

TEST(TrackMesh, RoadCoversTheTrackAndFacesUp) {
  const Track track = ring(20.0, 2.0);
  const TrackMesh mesh = make_track_mesh(track);
  const MeshSection* road = find(mesh, "asphalt");
  ASSERT_NE(road, nullptr);
  ASSERT_EQ(road->indices.size() % 3, 0u);
  ASSERT_EQ(road->normals.size(), road->positions.size());
  ASSERT_EQ(road->uvs.size() / 2, road->vertex_count());
  double area = 0.0;
  for (std::size_t t = 0; t < road->indices.size() / 3; ++t) {
    const double a = area_from_above(*road, t);
    ASSERT_GT(a, 0.0) << "triangle " << t << " faces down";
    area += a;
  }
  // Annulus between r = 18 and 22 (the spline is within ~1 % of a circle).
  const double expected = std::numbers::pi * (22.0 * 22.0 - 18.0 * 18.0);
  EXPECT_NEAR(area, expected, 0.02 * expected);
  // Closed: the strip joins up without a duplicate seam row.
  EXPECT_EQ(road->vertex_count(), 2 * static_cast<std::size_t>(std::ceil(track.length() / 0.5)));
  for (const std::uint32_t i : road->indices) {
    ASSERT_LT(i, road->vertex_count());
  }
}

TEST(TrackMesh, HasLinesConesAndGround) {
  const Track track = ring(20.0, 2.0);
  TrackMeshOptions options;
  options.scenery = false;
  const TrackMesh mesh = make_track_mesh(track, options);
  for (const char* name : {"ground", "asphalt", "edge_lines", "start_line", "sector_lines", "cones_blue",
                           "cones_yellow", "cones_orange"}) {
    EXPECT_NE(find(mesh, name), nullptr) << name;
  }
  // Blue cones on the left boundary: inside the counter-clockwise ring.
  const MeshSection* blue = find(mesh, "cones_blue");
  const MeshSection* yellow = find(mesh, "cones_yellow");
  ASSERT_TRUE(blue && yellow);
  EXPECT_EQ(blue->vertex_count(), yellow->vertex_count());
  for (std::size_t i = 0; i < blue->vertex_count(); ++i) {
    ASSERT_LT(std::hypot(blue->positions[3 * i], blue->positions[3 * i + 1]), 18.0);
    ASSERT_GT(std::hypot(yellow->positions[3 * i], yellow->positions[3 * i + 1]), 22.0);
  }
  // A ~125.6 m lap with a pair every 5 m (25 pairs), the pair on the start
  // line orange; 41 vertices per cone.
  const MeshSection* orange = find(mesh, "cones_orange");
  ASSERT_TRUE(orange);
  EXPECT_EQ(orange->vertex_count(), 2u * 41u);
  EXPECT_EQ(blue->vertex_count(), 24u * 41u);
  const MeshSection* ground = find(mesh, "ground");
  ASSERT_TRUE(ground);
  EXPECT_LT(mesh.min[0], -22.0 - 39.0);
  EXPECT_GT(mesh.max[1], 22.0 + 39.0);
  EXPECT_LT(mesh.min[2], 0.0);
  EXPECT_NEAR(mesh.max[2], 0.505, 1e-6);  // big orange cones
}

TEST(TrackMesh, OpenTrackHasStartAndFinishLines) {
  TrackDefinition def;
  def.closed = false;
  def.points = {{0, 0, 0, 3, 3, 0}, {75, 0, 0, 3, 3, 0}};
  const Track track(def);
  TrackMeshOptions options;
  options.cone_spacing = 0.0;
  const TrackMesh mesh = make_track_mesh(track, options);
  EXPECT_EQ(find(mesh, "cones_blue"), nullptr);
  const MeshSection* start = find(mesh, "start_line");
  ASSERT_TRUE(start);
  EXPECT_EQ(start->vertex_count(), 8u);  // two bands of two rows
  const MeshSection* road = find(mesh, "asphalt");
  ASSERT_TRUE(road);
  EXPECT_EQ(road->vertex_count(), 2u * 151u);
  EXPECT_NEAR(mesh.max[0], 75.0 + 40.0, 1e-4);
}

// The skidpad runs over itself: no cone may stand on any part of the road.
TEST(TrackMesh, SelfCrossingCourseKeepsConesOffTheRoad) {
  const Track track(TrackDefinition::from_file(std::string(VSIM_SOURCE_DIR) + "/templates/tracks/skidpad.track.json"));
  const TrackMesh mesh = make_track_mesh(track);
  std::size_t cones = 0;
  for (const char* name : {"cones_blue", "cones_yellow", "cones_orange"}) {
    const MeshSection* m = find(mesh, name);
    ASSERT_TRUE(m) << name;
    ASSERT_EQ(m->vertex_count() % 41, 0u);
    for (std::size_t first = 0; first < m->vertex_count(); first += 41) {
      // Vertex 32 of each cone is the centre of its top.
      const double x = m->positions[3 * (first + 32)], y = m->positions[3 * (first + 32) + 1];
      const CurveProjection p = track.project(x, y);
      const Track::Widths w = track.widths_at(p.s);
      EXPECT_TRUE(p.n > w.left + 0.1 || p.n < -w.right - 0.1)
          << name << " cone at (" << x << ", " << y << ") is on the road (n = " << p.n << ")";
      ++cones;
    }
  }
  EXPECT_GT(cones, 60u);
  // Edge lines are interrupted where another part of the course crosses.
  const MeshSection* lines = find(mesh, "edge_lines");
  ASSERT_TRUE(lines);
  EXPECT_LT(lines->indices.size(), 2 * 6 * static_cast<std::size_t>(std::ceil(track.length() / 0.5)));
}

TEST(TrackMesh, GlbIsValidGltf) {
  const TrackMesh mesh = make_track_mesh(ring(20.0, 2.0));
  const std::string glb = to_glb(mesh);
  ASSERT_GE(glb.size(), 28u);
  const auto u32 = [&](std::size_t offset) {
    std::uint32_t v = 0;
    std::memcpy(&v, glb.data() + offset, 4);
    return v;
  };
  EXPECT_EQ(u32(0), 0x46546C67u);
  EXPECT_EQ(u32(4), 2u);
  EXPECT_EQ(u32(8), glb.size());
  const std::uint32_t json_bytes = u32(12);
  EXPECT_EQ(u32(16), 0x4E4F534Au);
  EXPECT_EQ(json_bytes % 4, 0u);
  const auto doc = nlohmann::json::parse(glb.substr(20, json_bytes));
  const std::uint32_t bin_bytes = u32(20 + json_bytes);
  EXPECT_EQ(u32(24 + json_bytes), 0x004E4942u);
  EXPECT_EQ(28 + json_bytes + bin_bytes, glb.size());
  EXPECT_EQ(doc["buffers"][0]["byteLength"], bin_bytes);
  EXPECT_EQ(doc["meshes"][0]["primitives"].size(), mesh.sections.size());
  EXPECT_EQ(doc["materials"].size(), mesh.sections.size());
  for (const auto& view : doc["bufferViews"]) {
    ASSERT_LE(view["byteOffset"].get<std::size_t>() + view["byteLength"].get<std::size_t>(), bin_bytes);
    ASSERT_EQ(view["byteOffset"].get<std::size_t>() % 4, 0u);
  }
  // y up: the road's position accessor spans no height, and z = -north.
  const auto& road = doc["accessors"][4];  // second section's positions
  EXPECT_NEAR(road["min"][1].get<double>(), 0.0, 1e-6);
  EXPECT_NEAR(road["max"][1].get<double>(), 0.0, 1e-6);
  EXPECT_NEAR(road["max"][2].get<double>(), 22.0, 0.3);
}

}  // namespace
}  // namespace vsim

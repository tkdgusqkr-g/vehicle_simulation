#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstring>
#include <numbers>
#include <string>

#include "test_support.hpp"
#include "vsim/track/track_mesh.hpp"
#include "vsim/visual/gltf.hpp"
#include "vsim/visual/mesh.hpp"
#include "vsim/visual/vehicle_visual.hpp"

namespace vsim {
namespace {

constexpr double kPi = std::numbers::pi;

// Volume enclosed by the triangles (divergence theorem); positive when they
// face outwards.
double signed_volume(const MeshSection& m, std::size_t first_index = 0) {
  double v = 0.0;
  for (std::size_t t = first_index; t + 2 < m.indices.size(); t += 3) {
    const Vec3 a = m.position(m.indices[t]), b = m.position(m.indices[t + 1]), c = m.position(m.indices[t + 2]);
    v += a.dot(b.cross(c)) / 6.0;
  }
  return v;
}

void expect_valid(const MeshSection& m) {
  ASSERT_EQ(m.positions.size(), m.normals.size()) << m.name;
  ASSERT_EQ(m.uvs.size() / 2, m.vertex_count()) << m.name;
  ASSERT_EQ(m.indices.size() % 3, 0u) << m.name;
  for (const std::uint32_t i : m.indices) {
    ASSERT_LT(i, m.vertex_count()) << m.name;
  }
  for (std::size_t i = 0; i < m.vertex_count(); ++i) {
    const double n = std::hypot(m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]);
    ASSERT_NEAR(n, 1.0, 1e-3) << m.name << " vertex " << i;
    ASSERT_TRUE(std::isfinite(m.positions[3 * i]) && std::isfinite(m.positions[3 * i + 2])) << m.name;
  }
}

TEST(Geometry, ClosedShapesFaceOutwards) {
  MeshSection box;
  geometry::box(box, {1, 2, 3}, {0.5, 1.0, 1.5}, 0.4, 0.2);
  EXPECT_NEAR(signed_volume(box), 1.0 * 2.0 * 3.0, 1e-5);

  MeshSection ball;
  geometry::ellipsoid(ball, {0.3, -0.2, 1.0}, {1.0, 2.0, 0.5}, 24, 48);
  EXPECT_NEAR(signed_volume(ball), 4.0 / 3.0 * kPi * 1.0, 0.03 * 4.0 / 3.0 * kPi);

  MeshSection body;
  std::vector<std::vector<Vec3>> rings;
  for (const double x : {-1.0, 0.0, 1.0}) {
    rings.push_back(geometry::superellipse_ring(x, 0.0, 0.5, 0.3, 0.2, 2.0, 64));
  }
  geometry::loft(body, rings, true, true);
  EXPECT_NEAR(signed_volume(body), kPi * 0.3 * 0.2 * 2.0, 0.01);
  // Advancing the other way still faces out (rings reversed accordingly).
  MeshSection back;
  std::vector<std::vector<Vec3>> reversed(rings.rbegin(), rings.rend());
  for (auto& ring : reversed) {
    std::reverse(ring.begin(), ring.end());
  }
  geometry::loft(back, reversed, true, true);
  EXPECT_NEAR(signed_volume(back), kPi * 0.3 * 0.2 * 2.0, 0.01);

  MeshSection disc;  // closed annulus profile
  geometry::lathe(disc, {{0.06, -0.004}, {0.10, -0.004}, {0.10, 0.004}, {0.06, 0.004}, {0.06, -0.004}}, {}, {0, 1, 0}, 64);
  EXPECT_NEAR(signed_volume(disc), kPi * (0.01 - 0.0036) * 0.008, 2e-6);

  MeshSection pipe;
  geometry::tube(pipe, {{0, 0, 0}, {1, 0, 0}, {1, 1, 0.5}}, 0.05, 16);
  EXPECT_GT(signed_volume(pipe), 0.0);

  MeshSection wing;
  geometry::extrude_y(wing, {{0.0, 0.0}, {-0.3, 0.02}, {-0.3, -0.02}}, -0.5, 0.5);  // counter-clockwise in (x, z)
  EXPECT_GT(signed_volume(wing), 0.0);

  MeshSection mirrored;
  geometry::box(mirrored, {0, 1, 0}, {0.1, 0.2, 0.3});
  geometry::mirror_y(mirrored, 0, 0);
  EXPECT_NEAR(signed_volume(mirrored), 0.2 * 0.4 * 0.6, 1e-6);
  EXPECT_LT(mirrored.position(0).y, 0.0);
  for (const MeshSection* m : {&box, &ball, &body, &disc, &pipe, &wing, &mirrored}) {
    expect_valid(*m);
  }
}

TEST(FormulaCar, PartsFollowTheVehicle) {
  const Scenario scenario = test::load_example("drive");
  const VisualModel car = make_formula_car(scenario.params);
  int wheels = 0, uprights = 0, links = 0, bodies = 0, steering = 0;
  for (const VisualPart& part : car.parts) {
    for (const MeshSection& m : part.sections) {
      expect_valid(m);
      EXPECT_FALSE(m.indices.empty()) << part.name << "/" << m.name;
    }
    switch (part.role) {
      case PartRole::Wheel: {
        ++wheels;
        const Vec3 hub = hub_position(scenario.params, part.corner);
        EXPECT_DOUBLE_EQ(part.pivot.x, hub.x);
        EXPECT_DOUBLE_EQ(part.pivot.y, hub.y);
        // The tread faces outwards from the axle (local y).
        const MeshSection& tyre = part.sections.front();
        ASSERT_EQ(tyre.name, "tyre");
        double outward = 0.0;
        for (std::size_t i = 0; i < tyre.vertex_count(); ++i) {
          const Vec3 p = tyre.position(i);
          const double r = std::hypot(p.x, p.z);
          if (r > 0.22) {
            outward += (tyre.normals[3 * i] * p.x + tyre.normals[3 * i + 2] * p.z) / r;
          }
        }
        EXPECT_GT(outward, 0.0) << part.name;
        // Right wheels are mirrored: their outer face (the nut) is on -y.
        double nut_y = 0.0;
        for (const MeshSection& m : part.sections) {
          if (m.name == "nut") {
            nut_y = m.position(0).y;
          }
        }
        EXPECT_EQ(nut_y > 0.0, part.corner % 2 == 0) << part.name;
        break;
      }
      case PartRole::Upright:
        ++uprights;
        break;
      case PartRole::Link: {
        ++links;
        // Inboard ends sit on the car's side of the hub, both on the same side.
        const Vec3 hub = hub_position(scenario.params, part.corner);
        const Vec3 outer = hub + part.outer;
        EXPECT_LT(std::abs(part.pivot.y), std::abs(outer.y)) << part.name;
        EXPECT_GT(part.pivot.y * hub.y, 0.0) << part.name;
        EXPECT_LT((outer - part.pivot).norm(), 0.6) << part.name;
        break;
      }
      case PartRole::Body:
        ++bodies;
        break;
      case PartRole::SteeringWheel:
        ++steering;
        EXPECT_NEAR(part.axis.norm(), 1.0, 1e-9);
        EXPECT_LT(part.axis.x, 0.0);  // faces the driver
        break;
    }
  }
  EXPECT_EQ(wheels, 4);
  EXPECT_EQ(uprights, 4);
  EXPECT_EQ(links, 24);
  EXPECT_EQ(bodies, 1);
  EXPECT_EQ(steering, 1);
  EXPECT_NEAR(car.cg_height, 0.28, 1e-9);

  // FS rule envelopes: rear wing below 1.2 m, within 250 mm of the rear tyres.
  const VisualPart& body = car.parts.front();
  const double xr = -(scenario.params.get_double("vehicle.wheelbase_m") -
                      scenario.params.get_double("vehicle.cg_to_front_axle_m"));
  double top = 0.0, rearmost = 0.0;
  for (const MeshSection& m : body.sections) {
    for (std::size_t i = 0; i < m.vertex_count(); ++i) {
      top = std::max(top, static_cast<double>(m.positions[3 * i + 2]));
      rearmost = std::min(rearmost, static_cast<double>(m.positions[3 * i]));
    }
  }
  EXPECT_LT(top, 1.2);
  EXPECT_GT(rearmost, xr - 0.2286 - 0.25);
}

TEST(FormulaCar, GlbCarriesPartRoles) {
  const Scenario scenario = test::load_example("drive");
  const VisualModel car = make_formula_car(scenario.params);
  const std::string glb = to_glb(car);
  std::uint32_t json_bytes = 0;
  std::memcpy(&json_bytes, glb.data() + 12, 4);
  const auto doc = nlohmann::json::parse(glb.substr(20, json_bytes));
  ASSERT_EQ(doc["nodes"].size(), car.parts.size());
  int wheels = 0;
  for (const auto& node : doc["nodes"]) {
    const std::string role = node["extras"]["role"];
    if (role == "wheel") {
      ++wheels;
      EXPECT_TRUE(node.contains("translation"));
      // glTF frame: y up, so the hub height is the second component: the
      // full model's static hub (unloaded radius minus the static squash).
      EXPECT_NEAR(node["translation"][1].get<double>(), 0.2286 - 0.25 * 280.0 * 9.81 / 100e3, 1e-6);
    }
    if (role == "link") {
      EXPECT_TRUE(node["extras"].contains("outer"));
    }
  }
  EXPECT_EQ(wheels, 4);
  bool clearcoat_named = false;
  for (const auto& material : doc["materials"]) {
    clearcoat_named = clearcoat_named || material["name"] == "paint";
  }
  EXPECT_TRUE(clearcoat_named);
}

// The simulator reads back what it writes (and what the vehicle builder
// exports in the same format): parts, roles, pivots, geometry, winding.
TEST(FormulaCar, GlbReadsBack) {
  const Scenario scenario = test::load_example("drive");
  const VisualModel car = make_formula_car(scenario.params);
  const VisualModel back = read_vehicle_glb(to_glb(car));
  ASSERT_EQ(back.parts.size(), car.parts.size());
  EXPECT_NEAR(back.cg_height, car.cg_height, 1e-9);
  for (std::size_t i = 0; i < car.parts.size(); ++i) {
    const VisualPart& a = car.parts[i];
    const VisualPart& b = back.parts[i];
    SCOPED_TRACE(a.name);
    EXPECT_EQ(b.name, a.name);
    EXPECT_EQ(b.role, a.role);
    EXPECT_EQ(b.corner, a.corner);
    EXPECT_EQ(b.steers, a.steers);
    EXPECT_NEAR((b.pivot - a.pivot).dot(b.pivot - a.pivot), 0.0, 1e-18);
    EXPECT_NEAR((b.outer - a.outer).dot(b.outer - a.outer), 0.0, 1e-18);
    ASSERT_EQ(b.sections.size(), a.sections.size());
    for (std::size_t s = 0; s < a.sections.size(); ++s) {
      const MeshSection& x = a.sections[s];
      const MeshSection& y = b.sections[s];
      EXPECT_EQ(y.name, x.name);
      ASSERT_EQ(y.vertex_count(), x.vertex_count());
      ASSERT_EQ(y.indices, x.indices);
      for (std::size_t v = 0; v < x.positions.size(); ++v) {
        ASSERT_NEAR(y.positions[v], x.positions[v], 1e-5) << x.name;   // float32 in the file
      }
      for (int c = 0; c < 4; ++c) {
        EXPECT_NEAR(y.color[c], x.color[c], 2e-3) << x.name;
      }
      EXPECT_NEAR(y.roughness, x.roughness, 1e-6);
      EXPECT_EQ(y.flags, x.flags);
    }
  }
  EXPECT_THROW(read_vehicle_glb("not a glb"), std::invalid_argument);
}

TEST(TrackScenery, IsDeterministicAndOffTheRoad) {
  const Track track(TrackDefinition::from_file(std::string(VSIM_SOURCE_DIR) + "/templates/tracks/autocross.track.json"));
  const TrackMesh a = make_track_mesh(track);
  const TrackMesh b = make_track_mesh(track);
  ASSERT_EQ(a.sections.size(), b.sections.size());
  std::size_t trees = 0;
  for (std::size_t i = 0; i < a.sections.size(); ++i) {
    expect_valid(a.sections[i]);
    EXPECT_EQ(a.sections[i].positions, b.sections[i].positions) << a.sections[i].name;
    if (a.sections[i].name.rfind("tree_trunk", 0) == 0) {
      // Each trunk starts with its base centre: none on the road.
      const MeshSection& trunks = a.sections[i];
      for (std::size_t v = 0; v < trunks.vertex_count(); v += 14) {
        const Vec3 p = trunks.position(v);
        const CurveProjection proj = track.project(p.x, p.y);
        const Track::Widths w = track.widths_at(proj.s);
        EXPECT_GT(proj.distance, std::max(w.left, w.right) + 5.0);
        ++trees;
      }
    }
  }
  EXPECT_GT(trees, 20u);
  TrackMeshOptions bare;
  bare.scenery = false;
  EXPECT_LT(make_track_mesh(track, bare).sections.size(), a.sections.size());
}

}  // namespace
}  // namespace vsim

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <memory>
#include <cmath>
#include <fstream>
#include <iterator>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>

#include "test_support.hpp"
#include "vsim/link/state_link.hpp"
#include "vsim/sim/simulation.hpp"
#include "vsim/visual/vehicle_visual.hpp"

namespace vsim {
namespace {

// Unique per test run so parallel ctest processes do not share regions.
std::string link_name(const char* test) {
  return std::string("vsim_test_") + test + "_" + std::to_string(vsim_link::now_ns() % 1000000007);
}

TEST(Link, HeaderCopyInTheUnrealPluginIsIdentical) {
  const auto read = [](const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    EXPECT_TRUE(in) << path;
    return std::string(std::istreambuf_iterator<char>(in), {});
  };
  const std::string root = VSIM_SOURCE_DIR;
  EXPECT_EQ(read(root + "/core/include/vsim/link/vsim_link.h"),
            read(root + "/ui/unreal/VsimViewer/Plugins/VsimLink/Source/VsimLink/Public/vsim_link.h"))
      << "copy core/include/vsim/link/vsim_link.h into the plugin";
}

TEST(Link, StateRoundTripsAndViewerInputComesBack) {
  const std::string name = link_name("state");
  StateLink link(name);
  StateLinkReader reader(name);
  ASSERT_TRUE(reader.connect());

  vsim_link::VehicleState state{};
  state.frame = 7;
  state.x = 1.5;
  state.yaw = -0.25;
  state.wheel_omega[vsim_link::kRearRight] = 42.0f;
  std::snprintf(state.info.scenario, sizeof state.info.scenario, "%s", "unit");
  link.publish(state);
  vsim_link::VehicleState copy{};
  ASSERT_TRUE(reader.read(copy));
  EXPECT_EQ(copy.frame, 7u);
  EXPECT_EQ(copy.x, 1.5);
  EXPECT_EQ(copy.yaw, -0.25);
  EXPECT_EQ(copy.wheel_omega[3], 42.0f);
  EXPECT_STREQ(copy.info.scenario, "unit");

  EXPECT_FALSE(link.poll_viewer().keys) << "no keys before the viewer sends any";
  reader.send_keys(vsim_link::kKeyThrottle | vsim_link::kKeyLeft);
  StateLink::ViewerInput input = link.poll_viewer();
  EXPECT_TRUE(input.alive);
  ASSERT_TRUE(input.keys);
  EXPECT_TRUE(input.keys->throttle);
  EXPECT_TRUE(input.keys->left);
  EXPECT_FALSE(input.keys->brake);
  EXPECT_FALSE(input.keys->right);
  EXPECT_GT(input.keys_ns, 0);
  EXPECT_TRUE(input.commands.empty());

  reader.send_command(vsim_link::kCommandReset);
  input = link.poll_viewer();
  ASSERT_EQ(input.commands.size(), 1u);
  EXPECT_EQ(input.commands[0], vsim_link::kCommandReset);
  EXPECT_TRUE(link.poll_viewer().commands.empty()) << "a command is handled once";
  reader.send_command(vsim_link::kCommandReset);
  EXPECT_EQ(link.poll_viewer().commands.size(), 1u) << "the same command again is a new one";
}

TEST(Link, TrackMeshesArriveWithNewGenerations) {
  const std::string name = link_name("track");
  StateLink link(name);
  StateLinkReader reader(name);
  ASSERT_TRUE(reader.connect());
  EXPECT_EQ(reader.track(), nullptr);

  TrackDefinition def;
  def.name = "straight";
  def.closed = false;
  def.points = {{0, 0, 0, 3, 3, 0}, {75, 0, 0, 3, 3, 0}};
  const Track track(def);
  const TrackMesh mesh = make_track_mesh(track);
  EXPECT_EQ(link.publish_track(mesh, track.name(), track.length()), 1u);

  const vsim_link::MeshRegion* region = reader.track();
  ASSERT_NE(region, nullptr);
  EXPECT_EQ(region->generation, 1u);
  EXPECT_EQ(region->kind, vsim_link::kMeshTrack);
  EXPECT_STREQ(region->name, "straight");
  EXPECT_DOUBLE_EQ(region->length, 75.0);
  ASSERT_EQ(region->part_count, 1u);
  ASSERT_EQ(region->section_count, mesh.sections.size());
  const vsim_link::MeshSectionInfo* sections = vsim_link::mesh_sections(*region);
  for (std::size_t i = 0; i < mesh.sections.size(); ++i) {
    const MeshSection& m = mesh.sections[i];
    const vsim_link::MeshSectionInfo& section = sections[i];
    ASSERT_TRUE(vsim_link::section_valid(*region, section));
    EXPECT_EQ(section.name, m.name);
    ASSERT_EQ(section.vertex_count, m.vertex_count());
    ASSERT_EQ(section.index_count, m.indices.size());
    EXPECT_EQ(section.color[2], m.color[2]);
    EXPECT_EQ(section.roughness, m.roughness);
    const float* positions = vsim_link::mesh_array<float>(*region, section.positions);
    const float* normals = vsim_link::mesh_array<float>(*region, section.normals);
    const float* uvs = vsim_link::mesh_array<float>(*region, section.uvs);
    const auto* indices = vsim_link::mesh_array<std::uint32_t>(*region, section.indices);
    EXPECT_TRUE(std::equal(m.positions.begin(), m.positions.end(), positions)) << m.name;
    EXPECT_TRUE(std::equal(m.normals.begin(), m.normals.end(), normals)) << m.name;
    EXPECT_TRUE(std::equal(m.uvs.begin(), m.uvs.end(), uvs)) << m.name;
    EXPECT_TRUE(std::equal(m.indices.begin(), m.indices.end(), indices)) << m.name;
  }

  def.name = "second";
  EXPECT_EQ(link.publish_track(make_track_mesh(Track(def)), def.name, 75.0), 2u);
  region = reader.track();
  ASSERT_NE(region, nullptr);
  EXPECT_EQ(region->generation, 2u);
  EXPECT_STREQ(region->name, "second");

  link.clear_track();
  EXPECT_EQ(reader.track(), nullptr);
}

TEST(Link, CarModelArrivesWithPartRoles) {
  const std::string name = link_name("car");
  StateLink link(name);
  StateLinkReader reader(name);
  ASSERT_TRUE(reader.connect());
  EXPECT_EQ(reader.car(), nullptr);
  const VisualModel car = make_formula_car(test::load_example("drive").params);
  const std::uint64_t generation = link.publish_car(car);
  const vsim_link::MeshRegion* region = reader.car();
  ASSERT_NE(region, nullptr);
  EXPECT_EQ(region->generation, generation);
  EXPECT_EQ(region->kind, vsim_link::kMeshCar);
  EXPECT_FLOAT_EQ(region->cg_height, 0.28f);
  ASSERT_EQ(region->part_count, car.parts.size());
  const vsim_link::MeshPart* parts = vsim_link::mesh_parts(*region);
  const vsim_link::MeshSectionInfo* sections = vsim_link::mesh_sections(*region);
  std::uint32_t next_section = 0;
  for (std::size_t i = 0; i < car.parts.size(); ++i) {
    const VisualPart& expected = car.parts[i];
    const vsim_link::MeshPart& part = parts[i];
    EXPECT_EQ(part.name, expected.name);
    EXPECT_EQ(part.role, static_cast<std::uint32_t>(expected.role));
    EXPECT_EQ(part.corner, expected.corner);
    EXPECT_EQ(part.first_section, next_section);
    EXPECT_EQ(part.section_count, expected.sections.size());
    EXPECT_FLOAT_EQ(part.pivot[2], static_cast<float>(expected.pivot.z));
    EXPECT_EQ((part.flags & vsim_link::kPartSteers) != 0, expected.steers);
    for (std::uint32_t k = 0; k < part.section_count; ++k) {
      const vsim_link::MeshSectionInfo& s = sections[part.first_section + k];
      EXPECT_TRUE(vsim_link::section_valid(*region, s));
      EXPECT_EQ(s.name, expected.sections[k].name);
      EXPECT_EQ(s.metallic, expected.sections[k].metallic);
    }
    next_section += part.section_count;
  }
  EXPECT_EQ(next_section, region->section_count);
  EXPECT_GT(link.publish_track(make_track_mesh(Track(TrackDefinition::from_file(std::string(VSIM_SOURCE_DIR) +
                                                                                     "/templates/tracks/skidpad.track.json"))),
                               "skidpad", 1.0),
            generation);
  EXPECT_NE(reader.car(), nullptr) << "a new track keeps the car";
}

TEST(Link, TakesOverAStaleRegionAndKeepsCounting) {
  const std::string name = link_name("stale");
  auto first = std::make_unique<StateLink>(name);
  TrackDefinition def;
  def.closed = false;
  def.points = {{0, 0, 0, 3, 3, 0}, {20, 0, 0, 3, 3, 0}};
  first->publish_track(make_track_mesh(Track(def)), "a", 20.0);
  first->publish_track(make_track_mesh(Track(def)), "b", 20.0);
  first->region().heartbeat_ns.store(0);  // as if the simulator had crashed
  first->region().publisher_pid = 1;
  // POSIX: the crashed process would not have removed the name.
  StateLink second(name);
  EXPECT_EQ(second.publish_track(make_track_mesh(Track(def)), "c", 20.0), 3u);
  first.reset();
}

TEST(Link, ReadsAreNeverTorn) {
  const std::string name = link_name("torn");
  StateLink link(name);
  StateLinkReader reader(name);
  ASSERT_TRUE(reader.connect());
  std::atomic<bool> done{false};
  std::thread writer([&] {
    vsim_link::VehicleState s{};
    for (std::uint64_t frame = 1; frame <= 200000; ++frame) {
      s.frame = frame;
      s.x = static_cast<double>(frame);
      s.y = -static_cast<double>(frame);
      s.wheel_spin[3] = static_cast<float>(frame % 1000);
      link.publish(s);
    }
    done = true;
  });
  std::uint64_t last = 0, reads = 0;
  vsim_link::VehicleState s{};
  while (!done) {
    if (reader.read(s)) {
      ASSERT_EQ(s.x, static_cast<double>(s.frame));
      ASSERT_EQ(s.y, -static_cast<double>(s.frame));
      ASSERT_EQ(s.wheel_spin[3], static_cast<float>(s.frame % 1000));
      ASSERT_GE(s.frame, last);
      last = s.frame;
      ++reads;
    }
  }
  writer.join();
  EXPECT_GT(reads, 0u);
}

TEST(Link, SnapshotFollowsTheSimulation) {
  Simulation sim(test::load_example("acceleration"));
  LinkSnapshot snapshot;
  snapshot.reset(sim);
  vsim_link::VehicleState s{};
  snapshot.update(sim, s);
  EXPECT_FLOAT_EQ(s.info.wheelbase, 1.53f);
  EXPECT_STREQ(s.info.driver, "maneuver");
  EXPECT_NEAR(s.z, 0.28, 1e-6);
  sim.run_for(1.5);
  snapshot.update(sim, s);
  EXPECT_EQ(s.frame, 2u);
  EXPECT_DOUBLE_EQ(s.sim_time, sim.time());
  EXPECT_DOUBLE_EQ(s.x, test::signal(sim, "veh.chassis.x"));
  EXPECT_DOUBLE_EQ(s.yaw, test::signal(sim, "veh.chassis.yaw"));
  EXPECT_GT(s.speed, 5.0);
  EXPECT_FLOAT_EQ(s.wheel_omega[vsim_link::kRearLeft],
                  static_cast<float>(test::signal(sim, "veh.wheel.rl.omega")));
  EXPECT_FLOAT_EQ(s.wheel_load[vsim_link::kFrontRight], static_cast<float>(test::signal(sim, "veh.wheel.fr.fz")));
  EXPECT_GT(s.soc, 0.5f);
  EXPECT_GT(s.track_s, 5.0f);
  for (const float spin : s.wheel_spin) {
    EXPECT_LE(std::abs(spin), static_cast<float>(std::numbers::pi) + 1e-5f);
  }
  // Accelerating: the nose lifts (negative pitch).
  EXPECT_LT(s.pitch, 0.0);
  // For the viewers' sound and tyre display.
  EXPECT_FLOAT_EQ(s.info.gear_ratio,
                  static_cast<float>(sim.scenario().params.get_double("vehicle.driveline.0.gear_ratio")));
  EXPECT_GT(s.info.gear_ratio, 1.0f);
  EXPECT_FLOAT_EQ(s.tire_temp[vsim_link::kFrontLeft], static_cast<float>(test::signal(sim, "veh.wheel.fl.tire_temp")));
  EXPECT_FLOAT_EQ(s.brake_temp[vsim_link::kRearRight],
                  static_cast<float>(test::signal(sim, "veh.wheel.rr.brake_temp")));
  for (const std::uint8_t surface : s.wheel_surface) {
    EXPECT_EQ(surface, 0u);  // on the road
  }
}

}  // namespace
}  // namespace vsim

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vsim/link/shared_memory.hpp"
#include "vsim/link/vsim_link.h"
#include "vsim/sim/input_hub.hpp"
#include "vsim/sim/signal_bus.hpp"
#include "vsim/track/track_mesh.hpp"
#include "vsim/visual/vehicle_visual.hpp"

namespace vsim {

class Simulation;

// Simulator side of the shared-memory link (vsim_link.h): publishes the
// vehicle state and track meshes, collects what the viewer sends back.
class StateLink {
 public:
  // Creates the state region. Throws if another live simulator publishes
  // under the same name.
  explicit StateLink(std::string name = vsim_link::kStateName);

  const std::string& name() const { return name_; }
  void publish(const vsim_link::VehicleState& state);
  // Puts the mesh in a new region; returns its generation. Viewers switch
  // when the state's track_generation / car_generation changes.
  std::uint64_t publish_track(const TrackMesh& mesh, const std::string& track_name, double length);
  std::uint64_t publish_car(const VisualModel& model);
  void clear_track();
  std::uint64_t track_generation() const { return track_generation_; }
  std::uint64_t car_generation() const { return car_generation_; }

  struct ViewerInput {
    bool alive = false;               // heartbeat within the last second
    std::optional<KeyState> keys;     // if written within kKeyTimeoutNs
    std::int64_t keys_ns = 0;
    std::vector<vsim_link::Command> commands;  // since the last poll
    double latency_ms = 0.0;          // as measured by the viewer
    double frame_ms = 0.0;
  };
  ViewerInput poll_viewer();

  // Direct access for tests and in-process viewers.
  vsim_link::StateRegion& region() const { return *static_cast<vsim_link::StateRegion*>(state_.data()); }

 private:
  std::string name_;
  SharedMemory state_;
  SharedMemory track_, previous_track_;
  SharedMemory car_, previous_car_;
  std::uint64_t generation_ = 0;  // shared by tracks and cars
  std::uint64_t track_generation_ = 0;
  std::uint64_t car_generation_ = 0;
  std::uint32_t last_command_ = 0;
};

// Viewer side, used by tests and tools (renderer plugins carry their own
// copy of this logic on top of vsim_link.h).
class StateLinkReader {
 public:
  explicit StateLinkReader(std::string name = vsim_link::kStateName) : name_(std::move(name)) {}

  // Maps the state region if a live simulator publishes it; reopens after a
  // simulator restart. False while there is none.
  bool connect();
  bool read(vsim_link::VehicleState& state) const;
  // The current track / car mesh regions (mapped on first use per
  // generation); nullptr without one.
  const vsim_link::MeshRegion* track();
  const vsim_link::MeshRegion* car();
  void send_keys(std::uint32_t keys);
  void send_command(vsim_link::Command command);

 private:
  std::string name_;
  SharedMemory state_;
  SharedMemory track_, car_;
  std::uint64_t track_generation_ = 0;
  std::uint64_t car_generation_ = 0;
};

// Fills a VehicleState from a simulation's signals: pose, wheels, energy,
// lap, plus wheel spin angles and a visual body pitch/roll integrated over
// sim time.
class LinkSnapshot {
 public:
  // Call after building or resetting the simulation.
  void reset(const Simulation& sim);
  void update(const Simulation& sim, vsim_link::VehicleState& state);

 private:
  struct Id {
    std::optional<SignalId> id;
    double get(const SignalBus& bus, double fallback = 0.0) const { return id ? bus.get(*id) : fallback; }
  };
  struct Wheel {
    Id steer, omega, load, slip_ratio, slip_angle, hub_offset, camber, tire_temp, brake_temp, surface;
  };

  vsim_link::VehicleInfo info_{};
  Id x_, y_, yaw_, vx_, vy_, yaw_rate_, ax_, ay_, speed_, road_wheel_angle_;
  Id steer_, throttle_, brake_, power_, battery_power_, soc_, battery_voltage_, battery_temp_, ffb_;
  Id target_speed_, track_s_, track_n_, lap_, lap_time_, last_lap_, best_lap_, lap_valid_, last_lap_valid_;
  Id sector_, off_track_, finished_;
  Id roll_, pitch_, heave_, road_z_;  // full model
  std::vector<SignalId> motor_temps_;
  Wheel wheels_[4];
  double roll_per_g_ = 0.0, pitch_per_g_ = 0.0, body_tau_ = 0.05;
  bool tire_thermal_ = false, brake_thermal_ = false;
  double last_time_ = 0.0;
  double ax_filtered_ = 0.0, ay_filtered_ = 0.0;
  double spin_[4] = {};
  std::uint64_t frame_ = 0;
};

}  // namespace vsim

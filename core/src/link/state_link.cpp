#include "vsim/link/state_link.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>
#include <string_view>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include "vsim/models/vehicle/corners.hpp"
#include "vsim/sim/simulation.hpp"

namespace vsim {
namespace {

using vsim_link::MeshPart;
using vsim_link::MeshRegion;
using vsim_link::MeshSectionInfo;
using vsim_link::StateRegion;

constexpr double kGravity = 9.80665;

std::uint32_t process_id() {
#ifdef _WIN32
  return static_cast<std::uint32_t>(_getpid());
#else
  return static_cast<std::uint32_t>(getpid());
#endif
}

template <std::size_t N>
void copy_text(char (&out)[N], std::string_view text) {
  const std::size_t count = std::min(text.size(), N - 1);
  std::memcpy(out, text.data(), count);
  std::memset(out + count, 0, N - count);
}

std::size_t align8(std::size_t n) { return (n + 7) & ~std::size_t{7}; }

struct PartInput {
  std::string name;
  std::uint32_t role = vsim_link::kRoleStatic;
  int corner = -1;
  Vec3 pivot, axis, outer;
  std::uint32_t flags = 0;
  const std::vector<MeshSection>* sections = nullptr;
};

// Writes a mesh region (layout in vsim_link.h) into new shared memory.
SharedMemory write_mesh(const std::string& region_name, std::uint32_t kind, std::uint64_t generation,
                        const std::string& name, double length, double cg_height, const std::vector<PartInput>& parts) {
  std::size_t section_count = 0;
  std::size_t size = 0;
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  for (const PartInput& part : parts) {
    for (const MeshSection& m : *part.sections) {
      ++section_count;
      size += align8(m.positions.size() * 4) + align8(m.normals.size() * 4) + align8(m.uvs.size() * 4) +
              align8(m.indices.size() * 4);
      for (std::size_t i = 0; i < m.positions.size(); i += 3) {
        const double p[3] = {m.positions[i] + part.pivot.x, m.positions[i + 1] + part.pivot.y,
                             m.positions[i + 2] + part.pivot.z};
        for (int a = 0; a < 3; ++a) {
          lo[a] = std::min(lo[a], p[a]);
          hi[a] = std::max(hi[a], p[a]);
        }
      }
    }
  }
  const std::size_t header = align8(sizeof(MeshRegion) + parts.size() * sizeof(MeshPart) +
                                    section_count * sizeof(MeshSectionInfo));
  size += header;
  SharedMemory memory = SharedMemory::create(region_name, size);
  char* base = static_cast<char*>(memory.data());
  std::memset(base, 0, size);

  MeshRegion region{};
  region.magic = vsim_link::kMeshMagic;
  region.version = vsim_link::kVersion;
  region.generation = generation;
  region.size = size;
  region.kind = kind;
  region.part_count = static_cast<std::uint32_t>(parts.size());
  region.section_count = static_cast<std::uint32_t>(section_count);
  copy_text(region.name, name);
  region.length = length;
  for (int a = 0; a < 3; ++a) {
    region.min[a] = section_count > 0 ? lo[a] : 0.0;
    region.max[a] = section_count > 0 ? hi[a] : 0.0;
  }
  region.cg_height = static_cast<float>(cg_height);
  std::memcpy(base, &region, sizeof region);

  std::size_t offset = header;
  const auto put = [&](const void* data, std::size_t bytes) {
    std::memcpy(base + offset, data, bytes);
    const std::size_t at = offset;
    offset += align8(bytes);
    return static_cast<std::uint64_t>(at);
  };
  char* part_table = base + sizeof(MeshRegion);
  char* section_table = part_table + parts.size() * sizeof(MeshPart);
  std::uint32_t section_index = 0;
  for (std::size_t p = 0; p < parts.size(); ++p) {
    const PartInput& in = parts[p];
    MeshPart part{};
    copy_text(part.name, in.name);
    part.role = in.role;
    part.corner = in.corner;
    part.first_section = section_index;
    part.section_count = static_cast<std::uint32_t>(in.sections->size());
    const Vec3* vectors[3] = {&in.pivot, &in.axis, &in.outer};
    float* targets[3] = {part.pivot, part.axis, part.outer};
    for (int v = 0; v < 3; ++v) {
      targets[v][0] = static_cast<float>(vectors[v]->x);
      targets[v][1] = static_cast<float>(vectors[v]->y);
      targets[v][2] = static_cast<float>(vectors[v]->z);
    }
    part.flags = in.flags;
    std::memcpy(part_table + p * sizeof(MeshPart), &part, sizeof part);
    for (const MeshSection& m : *in.sections) {
      MeshSectionInfo info{};
      copy_text(info.name, m.name);
      std::copy(m.color.begin(), m.color.end(), info.color);
      info.metallic = m.metallic;
      info.roughness = m.roughness;
      info.flags = (m.flags & kCastShadow ? vsim_link::kSectionCastShadow : 0u) |
                   (m.flags & kDoubleSided ? vsim_link::kSectionDoubleSided : 0u);
      info.vertex_count = static_cast<std::uint32_t>(m.vertex_count());
      info.index_count = static_cast<std::uint32_t>(m.indices.size());
      info.positions = put(m.positions.data(), m.positions.size() * 4);
      info.normals = put(m.normals.data(), m.normals.size() * 4);
      info.uvs = put(m.uvs.data(), m.uvs.size() * 4);
      info.indices = put(m.indices.data(), m.indices.size() * 4);
      std::memcpy(section_table + section_index * sizeof(MeshSectionInfo), &info, sizeof info);
      ++section_index;
    }
  }
  return memory;
}

}  // namespace

// --- StateLink --------------------------------------------------------------

StateLink::StateLink(std::string name) : name_(std::move(name)) {
  state_ = SharedMemory::create(name_, sizeof(StateRegion));
  StateRegion& r = region();
  const std::int64_t now = vsim_link::now_ns();
  if (state_.existed() && vsim_link::region_valid(r)) {
    if (vsim_link::publisher_alive(r, now) && r.publisher_pid != process_id()) {
      const std::uint32_t pid = r.publisher_pid;
      state_.keep_name();
      state_ = {};
      throw std::runtime_error("another simulator (process " + std::to_string(pid) + ") publishes '" + name_ +
                               "'");
    }
    // Continue the old run's numbering so a viewer that cached its meshes
    // does not take the next ones for them.
    generation_ = std::max(r.track_generation.load(), r.car_generation.load());
  }
  r.magic.store(0, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  r.version = vsim_link::kVersion;
  r.size = sizeof(StateRegion);
  r.publisher_pid = process_id();
  r.sequence.store(0, std::memory_order_relaxed);
  r.heartbeat_ns.store(now, std::memory_order_relaxed);
  r.track_generation.store(0, std::memory_order_relaxed);
  copy_text(r.track_region, "");
  r.car_generation.store(0, std::memory_order_relaxed);
  copy_text(r.car_region, "");
  std::memset(&r.state, 0, sizeof r.state);
  last_command_ = r.viewer.command.load();  // what an earlier run left is not for us
  r.magic.store(vsim_link::kStateMagic, std::memory_order_release);
}

void StateLink::publish(const vsim_link::VehicleState& state) {
  StateRegion& r = region();
  vsim_link::write_state(r, state);
  r.heartbeat_ns.store(state.publish_ns != 0 ? state.publish_ns : vsim_link::now_ns(), std::memory_order_relaxed);
}

std::uint64_t StateLink::publish_track(const TrackMesh& mesh, const std::string& track_name, double length) {
  const std::uint64_t generation = ++generation_;
  const std::string region_name = name_ + "_track_" + std::to_string(generation);
  PartInput part;
  part.name = "track";
  part.sections = &mesh.sections;
  SharedMemory memory = write_mesh(region_name, vsim_link::kMeshTrack, generation, track_name, length, 0.0, {part});
  // Readers may still be opening the previous region; it goes one track later.
  previous_track_ = std::move(track_);
  track_ = std::move(memory);
  StateRegion& r = region();
  copy_text(r.track_region, region_name);
  r.track_generation.store(generation, std::memory_order_release);
  track_generation_ = generation;
  return generation;
}

std::uint64_t StateLink::publish_car(const VisualModel& model) {
  const std::uint64_t generation = ++generation_;
  const std::string region_name = name_ + "_car_" + std::to_string(generation);
  std::vector<PartInput> parts;
  for (const VisualPart& p : model.parts) {
    PartInput in;
    in.name = p.name;
    in.role = static_cast<std::uint32_t>(p.role);
    in.corner = p.corner;
    in.pivot = p.pivot;
    in.axis = p.axis;
    in.outer = p.outer;
    in.flags = p.steers ? vsim_link::kPartSteers : 0u;
    in.sections = &p.sections;
    parts.push_back(std::move(in));
  }
  SharedMemory memory =
      write_mesh(region_name, vsim_link::kMeshCar, generation, model.name, 0.0, model.cg_height, parts);
  previous_car_ = std::move(car_);
  car_ = std::move(memory);
  StateRegion& r = region();
  copy_text(r.car_region, region_name);
  r.car_generation.store(generation, std::memory_order_release);
  car_generation_ = generation;
  return generation;
}

void StateLink::clear_track() {
  StateRegion& r = region();
  r.track_generation.store(0, std::memory_order_release);
  copy_text(r.track_region, "");
  previous_track_ = std::move(track_);
  track_ = {};
  track_generation_ = 0;
}

StateLink::ViewerInput StateLink::poll_viewer() {
  vsim_link::ViewerChannel& v = region().viewer;
  const std::int64_t now = vsim_link::now_ns();
  ViewerInput input;
  input.alive = now - v.heartbeat_ns.load(std::memory_order_relaxed) < vsim_link::kHeartbeatTimeoutNs;
  input.keys_ns = v.keys_ns.load(std::memory_order_acquire);
  if (input.keys_ns != 0 && now - input.keys_ns < vsim_link::kKeyTimeoutNs) {
    const std::uint32_t bits = v.keys.load(std::memory_order_relaxed);
    KeyState keys;
    keys.throttle = (bits & vsim_link::kKeyThrottle) != 0;
    keys.brake = (bits & vsim_link::kKeyBrake) != 0;
    keys.left = (bits & vsim_link::kKeyLeft) != 0;
    keys.right = (bits & vsim_link::kKeyRight) != 0;
    input.keys = keys;
  }
  const std::uint32_t command = v.command.load(std::memory_order_acquire);
  if (command != last_command_) {
    // Only the newest command survives between polls (~1 ms apart).
    last_command_ = command;
    if (const auto c = static_cast<vsim_link::Command>(command & 0xffu); c != vsim_link::kCommandNone) {
      input.commands.push_back(c);
    }
  }
  input.latency_ms = v.latency_us.load(std::memory_order_relaxed) / 1000.0;
  input.frame_ms = v.frame_us.load(std::memory_order_relaxed) / 1000.0;
  return input;
}

// --- StateLinkReader --------------------------------------------------------

bool StateLinkReader::connect() {
  const std::int64_t now = vsim_link::now_ns();
  const auto usable = [&] {
    if (!state_ || state_.size() < sizeof(StateRegion)) {
      return false;
    }
    const auto& r = *static_cast<const StateRegion*>(state_.data());
    return vsim_link::region_valid(r) && vsim_link::publisher_alive(r, now);
  };
  if (usable()) {
    return true;
  }
  state_ = SharedMemory::open(name_);
  track_ = {};
  track_generation_ = 0;
  car_ = {};
  car_generation_ = 0;
  return usable();
}

bool StateLinkReader::read(vsim_link::VehicleState& state) const {
  return state_ && vsim_link::read_state(*static_cast<const StateRegion*>(state_.data()), state);
}

namespace {

// Maps the region a state field names when its generation changes.
const MeshRegion* follow(const std::atomic<std::uint64_t>& generation_field, const char (&region_name)[64],
                         SharedMemory& memory, std::uint64_t& mapped_generation) {
  const std::uint64_t generation = generation_field.load(std::memory_order_acquire);
  if (generation == 0) {
    memory = {};
    mapped_generation = 0;
    return nullptr;
  }
  if (generation != mapped_generation) {
    char name[sizeof region_name + 1] = {};
    std::memcpy(name, region_name, sizeof region_name);
    memory = SharedMemory::open(name);
    mapped_generation = memory ? generation : 0;
  }
  if (!memory || memory.size() < sizeof(MeshRegion)) {
    return nullptr;
  }
  const auto* region = static_cast<const MeshRegion*>(memory.data());
  return vsim_link::mesh_valid(*region, memory.size()) && region->generation == generation ? region : nullptr;
}

}  // namespace

const MeshRegion* StateLinkReader::track() {
  if (!state_) {
    return nullptr;
  }
  const auto& r = *static_cast<const StateRegion*>(state_.data());
  return follow(r.track_generation, r.track_region, track_, track_generation_);
}

const MeshRegion* StateLinkReader::car() {
  if (!state_) {
    return nullptr;
  }
  const auto& r = *static_cast<const StateRegion*>(state_.data());
  return follow(r.car_generation, r.car_region, car_, car_generation_);
}

void StateLinkReader::send_keys(std::uint32_t keys) {
  if (state_) {
    auto& r = *static_cast<StateRegion*>(state_.data());
    vsim_link::send_keys(r, keys);
    r.viewer.heartbeat_ns.store(vsim_link::now_ns(), std::memory_order_relaxed);
  }
}

void StateLinkReader::send_command(vsim_link::Command command) {
  if (state_) {
    vsim_link::send_command(*static_cast<StateRegion*>(state_.data()), command);
  }
}

// --- LinkSnapshot -----------------------------------------------------------

void LinkSnapshot::reset(const Simulation& sim) {
  const SignalBus& bus = sim.bus();
  const auto id = [&](std::string_view name) { return Id{bus.find(name)}; };
  x_ = id("veh.chassis.x");
  y_ = id("veh.chassis.y");
  yaw_ = id("veh.chassis.yaw");
  vx_ = id("veh.chassis.vx");
  vy_ = id("veh.chassis.vy");
  yaw_rate_ = id("veh.chassis.yaw_rate");
  ax_ = id("veh.chassis.ax");
  ay_ = id("veh.chassis.ay");
  speed_ = id("veh.chassis.speed");
  road_wheel_angle_ = id("veh.steer.road_wheel_angle");
  steer_ = id("drv.cmd.steer");
  throttle_ = id("drv.cmd.throttle");
  brake_ = id("drv.cmd.brake");
  power_ = id("pt.power_mech");
  battery_power_ = id("bat.power");
  soc_ = id("bat.soc");
  battery_voltage_ = id("bat.voltage");
  battery_temp_ = id("bat.temp");
  ffb_ = id("veh.ffb.torque_sw");
  target_speed_ = id("drv.target_speed");
  track_s_ = id("track.s");
  track_n_ = id("track.n");
  lap_ = id("track.lap");
  lap_time_ = id("track.lap_time");
  last_lap_ = id("track.last_lap");
  best_lap_ = id("track.best_lap");
  lap_valid_ = id("track.lap_valid");
  last_lap_valid_ = id("track.last_lap_valid");
  sector_ = id("track.sector");
  off_track_ = id("track.off_track");
  finished_ = id("track.finished");
  motor_temps_ = bus.match("pt.motor.*.temp");
  for (std::size_t c = 0; c < kCorners; ++c) {
    const std::string base = "veh.wheel." + std::string(kCornerNames[c]) + ".";
    wheels_[c] = {id(base + "steer"), id(base + "omega"), id(base + "fz"), id(base + "slip_ratio"),
                  id(base + "slip_angle")};
  }

  const ParamTree& p = sim.scenario().params;
  info_ = {};
  copy_text(info_.scenario, sim.scenario().name);
  copy_text(info_.model, p.get_string("vehicle.model", "two_track"));
  copy_text(info_.driver, p.get_string("driver.type", "external"));
  info_.wheelbase = static_cast<float>(p.get_double("vehicle.wheelbase_m"));
  info_.cg_to_front = static_cast<float>(p.get_double("vehicle.cg_to_front_axle_m"));
  info_.cg_height = static_cast<float>(p.get_double("vehicle.cg_height_m", 0.3));
  info_.track_front = static_cast<float>(p.get_double("vehicle.track_front_m", 1.2));
  info_.track_rear = static_cast<float>(p.get_double("vehicle.track_rear_m", 1.2));
  info_.tire_radius = static_cast<float>(p.get_double("vehicle.wheel.radius_m", 0.2));
  info_.tire_width = static_cast<float>(p.get_double("vehicle.tire.width_m", 0.2));
  info_.steering_ratio = static_cast<float>(p.get_double("vehicle.steering.ratio", 1.0));
  info_.steering_lock = static_cast<float>(p.get_double("vehicle.steering.lock_deg", 90.0) * std::numbers::pi / 180.0);
  // Body motion for the picture only, until the suspension model (M5).
  constexpr double kDeg = std::numbers::pi / 180.0;
  roll_per_g_ = p.get_double("vehicle.suspension.roll_gradient_deg_per_g", 1.0) * kDeg;
  pitch_per_g_ = p.get_double("vehicle.suspension.pitch_gradient_deg_per_g", 0.6) * kDeg;
  body_tau_ = std::max(1e-3, p.get_double("vehicle.suspension.load_transfer_tau_s", 0.05));

  last_time_ = sim.time();
  ax_filtered_ = ay_filtered_ = 0.0;
  std::fill(std::begin(spin_), std::end(spin_), 0.0);
}

void LinkSnapshot::update(const Simulation& sim, vsim_link::VehicleState& s) {
  const SignalBus& bus = sim.bus();
  const double t = sim.time();
  const double dt = std::max(0.0, t - last_time_);
  last_time_ = t;

  s.frame = ++frame_;
  s.sim_time = t;
  s.x = x_.get(bus);
  s.y = y_.get(bus);
  s.z = info_.cg_height;
  s.yaw = yaw_.get(bus);
  s.vx = vx_.get(bus);
  s.vy = vy_.get(bus);
  s.yaw_rate = yaw_rate_.get(bus);
  s.ax = ax_.get(bus);
  s.ay = ay_.get(bus);
  s.speed = speed_.get(bus);

  const double blend = 1.0 - std::exp(-dt / body_tau_);
  ax_filtered_ += blend * (s.ax - ax_filtered_);
  ay_filtered_ += blend * (s.ay - ay_filtered_);
  s.roll = roll_per_g_ * ay_filtered_ / kGravity;     // turning left: the left side lifts
  s.pitch = -pitch_per_g_ * ax_filtered_ / kGravity;  // braking: the nose dips

  s.steering_wheel = static_cast<float>(steer_.get(bus));
  s.throttle = static_cast<float>(throttle_.get(bus));
  s.brake = static_cast<float>(brake_.get(bus));
  const double road_wheel = road_wheel_angle_.get(bus);
  const double radius = std::max(0.05, static_cast<double>(info_.tire_radius));
  for (std::size_t c = 0; c < kCorners; ++c) {
    const Wheel& w = wheels_[c];
    const bool front = c < 2;
    // The single-track model has no per-wheel signals: approximate them.
    s.wheel_steer[c] = static_cast<float>(w.steer.get(bus, front ? road_wheel : 0.0));
    const double omega = w.omega.get(bus, s.vx / radius);
    s.wheel_omega[c] = static_cast<float>(omega);
    spin_[c] = std::remainder(spin_[c] + omega * dt, 2.0 * std::numbers::pi);
    s.wheel_spin[c] = static_cast<float>(spin_[c]);
    s.wheel_load[c] = static_cast<float>(w.load.get(bus));
    s.wheel_slip_ratio[c] = static_cast<float>(w.slip_ratio.get(bus));
    s.wheel_slip_angle[c] = static_cast<float>(w.slip_angle.get(bus));
  }

  s.power_kw = static_cast<float>(power_.get(bus) / 1000.0);
  s.battery_power_kw = static_cast<float>(battery_power_.get(bus) / 1000.0);
  s.soc = static_cast<float>(soc_.get(bus));
  s.battery_voltage = static_cast<float>(battery_voltage_.get(bus));
  s.battery_temp = static_cast<float>(battery_temp_.get(bus));
  double hottest = motor_temps_.empty() ? 0.0 : bus.get(motor_temps_.front());
  for (const SignalId id : motor_temps_) {
    hottest = std::max(hottest, bus.get(id));
  }
  s.motor_temp = static_cast<float>(hottest);
  s.ffb_torque = static_cast<float>(ffb_.get(bus));
  s.target_speed = static_cast<float>(target_speed_.get(bus));
  s.track_s = static_cast<float>(track_s_.get(bus));
  s.track_n = static_cast<float>(track_n_.get(bus));
  s.lap = static_cast<std::int32_t>(lap_.get(bus));
  s.lap_time = static_cast<float>(lap_time_.get(bus));
  s.last_lap = static_cast<float>(last_lap_.get(bus));
  s.best_lap = static_cast<float>(best_lap_.get(bus));
  s.lap_valid = lap_valid_.get(bus, 1.0) != 0.0;
  s.last_lap_valid = last_lap_valid_.get(bus) != 0.0;
  s.sector = static_cast<std::int32_t>(sector_.get(bus));
  s.off_track = off_track_.get(bus) != 0.0;
  s.finished = finished_.get(bus) != 0.0;
  s.info = info_;
}

}  // namespace vsim

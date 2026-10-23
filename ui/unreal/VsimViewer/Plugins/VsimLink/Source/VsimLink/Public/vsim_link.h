// vsim_link.h - shared-memory link between the simulator and renderers
// (design doc §16.3). The simulator publishes the vehicle state about every
// millisecond and the track as a mesh; a renderer (Unreal, Unity, a test)
// reads them and sends keys and commands back.
//
// Self-contained (C++17, standard library only) so a renderer plugin can carry
// a copy: ui/unreal/VsimViewer/Plugins/VsimLink/Source/VsimLink/Public/vsim_link.h
// must stay identical to core/include/vsim/link/vsim_link.h (a test checks).
//
// Objects (named shared memory, same user session):
//   Windows  "Local\vsim_state_v2"   CreateFileMapping / OpenFileMapping
//   POSIX    "/vsim_state_v2"        shm_open
// The state region names the current track and car mesh regions
// (track_region, car_region); a new track or car model gets a new region
// and a new generation.
//
// Frame: x east, y north, z up [m], yaw counter-clockwise from +x [rad],
// right-handed. Unreal (left-handed, cm): X = 100 x, Y = -100 y, Z = 100 z,
// yaw_ue = -yaw in degrees; triangles keep their index order.
//
// Times are std::chrono::steady_clock nanoseconds, which is the same clock in
// every process of a machine (QueryPerformanceCounter on Windows,
// CLOCK_MONOTONIC on Linux), so input-to-display latency can be measured
// across the two processes.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace vsim_link {

inline constexpr std::uint32_t kVersion = 2;
inline constexpr std::uint32_t kStateMagic = 0x4B4C5356;  // "VSLK"
inline constexpr std::uint32_t kMeshMagic = 0x48534D56;   // "VMSH"
inline constexpr char kStateName[] = "vsim_state_v2";

enum Corner : int { kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3 };

// Keys a viewer forwards for keyboard driving (driver.type dil or keyboard).
enum Key : std::uint32_t { kKeyThrottle = 1u, kKeyBrake = 2u, kKeyLeft = 4u, kKeyRight = 8u };

enum Command : std::uint32_t { kCommandNone = 0, kCommandReset = 1, kCommandTogglePause = 2 };

// Keys older than this are released by the simulator (viewer closed or hung).
inline constexpr std::int64_t kKeyTimeoutNs = 500'000'000;
// A publisher whose heartbeat is older than this is gone.
inline constexpr std::int64_t kHeartbeatTimeoutNs = 1'000'000'000;

// Vehicle description; constant for a run.
struct VehicleInfo {
  char scenario[64];
  char model[32];
  char driver[16];
  float wheelbase;      // [m]
  float cg_to_front;    // [m] CG to front axle
  float cg_height;      // [m]
  float track_front;    // [m]
  float track_rear;     // [m]
  float tire_radius;    // [m]
  float tire_width;     // [m]
  float steering_ratio; // steering wheel angle / road wheel angle
  float steering_lock;  // [rad] at the steering wheel, one side
  float gear_ratio;     // motor to wheel (first driveline entry), for sound
  float reserved[6];
};

// One snapshot, copied under the sequence lock.
struct VehicleState {
  std::uint64_t frame;       // publish counter
  std::int64_t publish_ns;   // steady clock when published
  std::int64_t input_ns;     // steady clock of the newest driver input applied in this state; 0: none
  double sim_time;           // [s]
  double x, y, z;            // CG position [m]; z: CG height above the road
  double yaw, pitch, roll;   // [rad]; pitch (nose down +) and roll (left side up +) are visual estimates
  double vx, vy, yaw_rate;   // body frame [m/s], [rad/s]
  double ax, ay;             // body frame [m/s^2]
  double speed;              // [m/s]
  float steering_wheel;      // [rad], positive left
  float throttle, brake;     // driver commands 0..1
  float wheel_steer[4];      // road wheel angles [rad], Corner order
  float wheel_omega[4];      // [rad/s]
  float wheel_spin[4];       // integrated spin angle [rad], wrapped to (-pi, pi]
  float wheel_load[4];       // [N]
  float wheel_slip_ratio[4];
  float wheel_slip_angle[4]; // [rad]
  float power_kw;            // motor shaft power, negative while regenerating
  float battery_power_kw;
  float soc;                 // 0..1
  float battery_voltage;     // [V]
  float battery_temp;        // [degC]
  float motor_temp;          // [degC], hottest motor
  float ffb_torque;          // [N m] at the car's steering wheel
  float rtf;                 // real-time factor
  float track_s, track_n;    // [m] along / left of the centerline
  float lap_time, last_lap, best_lap;  // [s]; 0: none yet
  float target_speed;        // [m/s] autonomous driver's target; 0: none
  std::int32_t lap;          // completed laps
  std::int32_t sector;       // current sector, 0-based
  std::uint8_t lap_valid, last_lap_valid, off_track, finished;
  std::uint8_t paused, wheel_connected, reserved8[2];
  float wheel_z[4];          // [m] hub height change from static (suspension travel, up +)
  float wheel_camber[4];     // [rad], top of the wheel outwards +
  float tire_temp[4];        // [degC] tread surface; 0: not modelled
  float brake_temp[4];       // [degC] disc; 0: not modelled
  float road_z;              // [m] road height under the CG
  std::uint8_t wheel_surface[4];  // 0 road, 1 kerb, 2 grass
  float reserved_f[15];
  std::uint64_t track_generation;  // track this state is on; 0: none
  std::uint64_t car_generation;    // car model shown; 0: none
  VehicleInfo info;
};

// Written by the viewer, read by the simulator.
struct ViewerChannel {
  std::atomic<std::uint32_t> keys;          // Key bits
  std::atomic<std::uint32_t> command;       // (count << 8) | Command; the count makes repeats distinct
  std::atomic<std::int64_t> keys_ns;        // steady clock of the last key write
  std::atomic<std::int64_t> heartbeat_ns;   // viewer alive
  std::atomic<std::int32_t> latency_us;     // viewer's input-to-frame latency, for display
  std::atomic<std::int32_t> frame_us;       // viewer frame time
};

struct StateRegion {
  std::atomic<std::uint32_t> magic;         // kStateMagic once initialized
  std::uint32_t version;                    // kVersion
  std::uint32_t size;                       // sizeof(StateRegion)
  std::uint32_t publisher_pid;
  std::atomic<std::uint32_t> sequence;      // odd while the publisher writes `state`
  std::uint32_t reserved;
  std::atomic<std::int64_t> heartbeat_ns;   // publisher alive (steady clock)
  std::atomic<std::uint64_t> track_generation;  // 0: no track
  char track_region[64];                    // name of the region holding that track's mesh
  std::atomic<std::uint64_t> car_generation;    // 0: no car model
  char car_region[64];
  VehicleState state;
  ViewerChannel viewer;
};

// Mesh region (the track with its scenery, or the car), written once before
// the state's generation points to it. Header, then MeshPart[part_count],
// then MeshSectionInfo[section_count], then the arrays, referenced by byte
// offsets from the start of the region: positions and normals 3 floats per
// vertex, uvs 2 floats, indices uint32 (three per triangle, counter-
// clockwise seen from the front). Coordinates: vsim frame, metres; car parts
// in the car frame (origin on the ground below the CG, x forward, y left).
//
// Track: one part (role kRoleStatic). Its "asphalt" section has two vertices
// per row along the track, right edge then left edge, in driving order
// (renderers use it to place cameras).
//
// Car parts move like this (see VehicleState):
//   kRoleBody           whole car pose, then pitch/roll about (0, 0, cg_height)
//   kRoleWheel          at pivot (hub) + wheel_z: steer about z, then spin about y
//   kRoleUpright        at pivot + wheel_z: steer only
//   kRoleSteeringWheel  with the body, turned by steering_wheel about `axis`
//   kRoleLink           a tube along +x from 0 to 1, stretched from `pivot`
//                       (on the body) to hub + wheel_z + `outer` (turned by
//                       the steer angle if kPartSteers)
// Mesh vertices of wheels, uprights and steering wheels are relative to the
// pivot.
enum MeshKind : std::uint32_t { kMeshTrack = 1, kMeshCar = 2 };
enum PartRole : std::uint32_t {
  kRoleStatic = 0, kRoleBody = 1, kRoleWheel = 2, kRoleUpright = 3, kRoleSteeringWheel = 4, kRoleLink = 5
};
enum PartFlags : std::uint32_t { kPartSteers = 1u };
enum SectionFlags : std::uint32_t { kSectionCastShadow = 1u, kSectionDoubleSided = 2u };

struct MeshRegion {
  std::uint32_t magic;          // kMeshMagic
  std::uint32_t version;        // kVersion
  std::uint64_t generation;
  std::uint64_t size;           // bytes of the whole region
  std::uint32_t kind;           // MeshKind
  std::uint32_t part_count;
  std::uint32_t section_count;
  std::uint32_t reserved;
  char name[64];
  double length;                // [m] track length; 0 for the car
  double min[3], max[3];        // bounding box [m]
  float cg_height;              // [m] car: body pitch/roll pivot height
  float reserved_f[3];
};

struct MeshPart {
  char name[32];
  std::uint32_t role;           // PartRole
  std::int32_t corner;          // 0..3 or -1
  std::uint32_t first_section;  // index into the sections
  std::uint32_t section_count;
  float pivot[3];
  float axis[3];
  float outer[3];
  std::uint32_t flags;          // PartFlags
};

struct MeshSectionInfo {
  char name[32];                // material name: asphalt, paint, carbon, tyre, ...
  float color[4];               // sRGB 0..1
  float metallic, roughness;
  std::uint32_t flags;          // SectionFlags
  std::uint32_t vertex_count;
  std::uint32_t index_count;
  std::uint32_t reserved;
  std::uint64_t positions, normals, uvs, indices;
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free && std::atomic<std::int64_t>::is_always_lock_free,
              "the link needs address-free atomics");
static_assert(std::is_trivially_copyable_v<VehicleState> && std::is_trivially_copyable_v<MeshRegion>);
static_assert(sizeof(VehicleInfo) == 176 && sizeof(VehicleState) == 632 && sizeof(StateRegion) == 840,
              "layout change: bump kVersion and the object names");
static_assert(sizeof(MeshRegion) == 176 && sizeof(MeshPart) == 88 && sizeof(MeshSectionInfo) == 104);

inline std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Publisher side: one writer.
inline void write_state(StateRegion& region, const VehicleState& state) {
  const std::uint32_t sequence = region.sequence.load(std::memory_order_relaxed);
  region.sequence.store(sequence + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  std::memcpy(&region.state, &state, sizeof state);
  region.sequence.store(sequence + 2, std::memory_order_release);
}

// Reader side: a consistent copy, or false if the publisher kept writing.
inline bool read_state(const StateRegion& region, VehicleState& out) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    const std::uint32_t before = region.sequence.load(std::memory_order_acquire);
    if (before & 1u) {
      continue;
    }
    std::memcpy(&out, &region.state, sizeof out);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (region.sequence.load(std::memory_order_relaxed) == before) {
      return true;
    }
  }
  return false;
}

inline bool region_valid(const StateRegion& region) {
  return region.magic.load(std::memory_order_acquire) == kStateMagic && region.version == kVersion &&
         region.size == sizeof(StateRegion);
}

inline bool publisher_alive(const StateRegion& region, std::int64_t now) {
  return now - region.heartbeat_ns.load(std::memory_order_relaxed) < kHeartbeatTimeoutNs;
}

// Viewer side.
inline void send_keys(StateRegion& region, std::uint32_t keys) {
  region.viewer.keys.store(keys, std::memory_order_relaxed);
  region.viewer.keys_ns.store(now_ns(), std::memory_order_release);
}

inline void send_command(StateRegion& region, Command command) {
  const std::uint32_t previous = region.viewer.command.load(std::memory_order_relaxed);
  region.viewer.command.store((((previous >> 8) + 1) << 8) | static_cast<std::uint32_t>(command),
                              std::memory_order_release);
}

inline bool mesh_valid(const MeshRegion& region, std::uint64_t mapped_size) {
  return region.magic == kMeshMagic && region.version == kVersion && region.size <= mapped_size &&
         sizeof(MeshRegion) + std::uint64_t(region.part_count) * sizeof(MeshPart) +
                 std::uint64_t(region.section_count) * sizeof(MeshSectionInfo) <=
             region.size;
}

inline const MeshPart* mesh_parts(const MeshRegion& region) {
  return reinterpret_cast<const MeshPart*>(reinterpret_cast<const char*>(&region) + sizeof(MeshRegion));
}

inline const MeshSectionInfo* mesh_sections(const MeshRegion& region) {
  return reinterpret_cast<const MeshSectionInfo*>(reinterpret_cast<const char*>(mesh_parts(region)) +
                                                  region.part_count * sizeof(MeshPart));
}

template <typename T>
inline const T* mesh_array(const MeshRegion& region, std::uint64_t offset) {
  return reinterpret_cast<const T*>(reinterpret_cast<const char*>(&region) + offset);
}

// The arrays of a section lie inside the region.
inline bool section_valid(const MeshRegion& region, const MeshSectionInfo& s) {
  const auto fits = [&](std::uint64_t offset, std::uint64_t bytes) { return offset + bytes <= region.size; };
  return fits(s.positions, 12ull * s.vertex_count) && fits(s.normals, 12ull * s.vertex_count) &&
         fits(s.uvs, 8ull * s.vertex_count) && fits(s.indices, 4ull * s.index_count);
}

}  // namespace vsim_link

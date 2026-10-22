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
//   Windows  "Local\vsim_state_v1"   CreateFileMapping / OpenFileMapping
//   POSIX    "/vsim_state_v1"        shm_open
// The state region names the current track region (track_region); a new
// track gets a new region and a new track_generation.
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

inline constexpr std::uint32_t kVersion = 1;
inline constexpr std::uint32_t kStateMagic = 0x4B4C5356;  // "VSLK"
inline constexpr std::uint32_t kTrackMagic = 0x4B525456;  // "VTRK"
inline constexpr char kStateName[] = "vsim_state_v1";

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
  float reserved[7];
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
  std::uint32_t reserved32;
  std::uint64_t track_generation;  // track this state is on; 0: none
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
  VehicleState state;
  ViewerChannel viewer;
};

// Track mesh region, written once before track_generation points to it.
// Arrays are referenced by byte offsets from the start of the region:
// positions and normals 3 floats per vertex, uvs 2 floats, indices uint32
// (three per triangle, counter-clockwise seen from the front).
struct TrackSection {
  char name[32];               // asphalt, edge_lines, cones_blue, ...
  float color[4];              // sRGB 0..1
  std::uint32_t vertex_count;
  std::uint32_t index_count;
  std::uint64_t positions, normals, uvs, indices;
};

struct TrackRegion {
  std::uint32_t magic;          // kTrackMagic
  std::uint32_t version;        // kVersion
  std::uint64_t generation;
  std::uint64_t size;           // bytes of the whole region
  std::uint32_t section_count;  // TrackSection[section_count] follows this header
  std::uint32_t reserved;
  char name[64];
  double length;                // [m]
  double min[3], max[3];        // bounding box [m]
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free && std::atomic<std::int64_t>::is_always_lock_free,
              "the link needs address-free atomics");
static_assert(std::is_trivially_copyable_v<VehicleState> && std::is_trivially_copyable_v<TrackRegion>);
static_assert(sizeof(VehicleInfo) == 176 && sizeof(VehicleState) == 496 && sizeof(StateRegion) == 632,
              "layout change: bump kVersion and the object names");
static_assert(sizeof(TrackSection) == 88 && sizeof(TrackRegion) == 152);

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

inline const TrackSection* track_sections(const TrackRegion& region) {
  return reinterpret_cast<const TrackSection*>(reinterpret_cast<const char*>(&region) + sizeof(TrackRegion));
}

template <typename T>
inline const T* track_array(const TrackRegion& region, std::uint64_t offset) {
  return reinterpret_cast<const T*>(reinterpret_cast<const char*>(&region) + offset);
}

}  // namespace vsim_link

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "vsim/input/wheel_profile.hpp"
#include "vsim/sim/input_hub.hpp"

namespace vsim {

// Reads a steering wheel through SDL3 on its own thread (~1 kHz), publishes
// the normalized WheelState to the InputHub and renders force feedback from
// the hub's road torque (design doc §9.3). Handles hotplug: a wheel can be
// connected or unplugged while the simulation runs.
class InputService {
 public:
  struct Options {
    std::vector<WheelProfile> profiles;  // matched in order; the G29 default is appended
    bool force_feedback = true;
    double poll_hz = 1000.0;
  };

  struct Status {
    bool connected = false;
    std::string device;
    std::string profile;
    bool force_feedback = false;  // the device accepted a constant-force effect
    std::string ffb_error;        // why not, if it did not
    double ffb_torque = 0.0;      // last device torque [N m], vsim direction
    bool ffb_clipping = false;
    bool ffb_suspended = false;   // wheel held far past the lock; see ForceFeedbackShaper
    bool calibrated = false;      // the profile came from vsim-input calibrate (priority > 0)
  };

  InputService(InputHub& hub, Options options);
  ~InputService();
  InputService(const InputService&) = delete;
  InputService& operator=(const InputService&) = delete;

  // Starts the thread; returns an error if SDL cannot initialize.
  std::optional<std::string> start();
  void stop();
  Status status() const;

 private:
  struct Device;
  void run();
  void open_device(Device& device);
  void close_device(Device& device);

  InputHub& hub_;
  Options options_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  mutable std::mutex mutex_;
  Status status_;
  std::optional<std::string> start_error_;
  std::atomic<bool> started_{false};
};

}  // namespace vsim

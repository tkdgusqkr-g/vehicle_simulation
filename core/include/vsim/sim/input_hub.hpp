#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "vsim/driver/driver_command.hpp"

namespace vsim {

struct KeyState {
  bool throttle = false;
  bool brake = false;
  bool left = false;
  bool right = false;
  std::chrono::steady_clock::time_point stamp{};  // when they were read (latency measurement)
};

// Steering wheel and pedals, already normalized by the device profile.
struct WheelState {
  bool connected = false;
  std::string device;     // device name, for display
  double steer = 0.0;     // steering wheel angle [rad], positive left
  double throttle = 0.0;  // 0..1
  double brake = 0.0;     // 0..1
  double clutch = 0.0;    // 0..1
  bool reset = false;     // buttons mapped by the profile
  bool pause = false;
  std::chrono::steady_clock::time_point stamp{};  // when the device was read
};

// What the force feedback loop should render: the road torque at the car's
// steering wheel and the steering lock for the end stop.
struct ForceFeedbackRequest {
  double torque = 0.0;  // [N m] at the car's steering wheel, positive counter-clockwise
  double lock = 0.0;    // [rad] steering wheel lock
  std::chrono::steady_clock::time_point stamp{};  // wall time of the last update
};

// Thread-safe handoff for inputs that originate outside the sim loop (browser
// keyboard, steering wheel, Python). Driver blocks sample it on their own
// tick, so the value a run uses is pinned to a known sim time and can be
// logged for replay. Force feedback flows the other way: the driver posts the
// road torque and the input thread renders it.
class InputHub {
 public:
  void set_keys(const KeyState& keys) {
    std::lock_guard lock(mutex_);
    keys_ = keys;
  }
  KeyState keys() const {
    std::lock_guard lock(mutex_);
    return keys_;
  }

  void set_command(const DriverCommand& command) {
    std::lock_guard lock(mutex_);
    command_ = command;
  }
  DriverCommand command() const {
    std::lock_guard lock(mutex_);
    return command_;
  }

  void set_wheel(const WheelState& wheel) {
    std::lock_guard lock(mutex_);
    wheel_ = wheel;
  }
  WheelState wheel() const {
    std::lock_guard lock(mutex_);
    return wheel_;
  }

  void set_force_feedback(double torque, double steering_lock) {
    std::lock_guard lock(mutex_);
    ffb_ = {torque, steering_lock, std::chrono::steady_clock::now()};
  }
  ForceFeedbackRequest force_feedback() const {
    std::lock_guard lock(mutex_);
    return ffb_;
  }

 private:
  mutable std::mutex mutex_;
  KeyState keys_;
  DriverCommand command_;
  WheelState wheel_;
  ForceFeedbackRequest ffb_;
};

}  // namespace vsim

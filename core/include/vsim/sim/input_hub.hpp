#pragma once

#include <mutex>

#include "vsim/driver/driver_command.hpp"

namespace vsim {

struct KeyState {
  bool throttle = false;
  bool brake = false;
  bool left = false;
  bool right = false;
};

// Thread-safe handoff for inputs that originate outside the sim loop (browser
// keyboard, Python). Driver blocks sample it on their own tick, so the value a
// run uses is pinned to a known sim time and can be logged for replay.
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

 private:
  mutable std::mutex mutex_;
  KeyState keys_;
  DriverCommand command_;
};

}  // namespace vsim

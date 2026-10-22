#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
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
    // Adds an SDL virtual joystick ("vsim virtual wheel", 6 axes, 16
    // buttons) driven by set_virtual(): tests and UI work without hardware.
    bool virtual_wheel = false;
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

  // Unprocessed device input, for calibration.
  struct RawInput {
    bool connected = false;
    std::string device;
    int vendor = 0;
    int product = 0;
    std::vector<int> axes;           // -32768..32767
    std::vector<std::uint8_t> buttons;
  };

  InputService(InputHub& hub, Options options);
  ~InputService();
  InputService(const InputService&) = delete;
  InputService& operator=(const InputService&) = delete;

  // Starts the thread; returns an error if SDL cannot initialize.
  std::optional<std::string> start();
  void stop();
  Status status() const;
  RawInput raw() const;
  // Profile of the connected device (the G29 default before one connects).
  WheelProfile profile() const;
  // Uses `profile` for the connected device from now on, and for that device
  // after a reconnect (its name is added to the profile's match names).
  void apply_profile(WheelProfile profile);
  // Constant force at a raw device level (-1..1, the sign as SDL sees it,
  // before the profile's ffb.invert) for `seconds`, instead of the road
  // feel: lets calibration find the direction.
  void test_force(double level, double seconds);
  // Virtual wheel input (Options::virtual_wheel); missing entries keep their values.
  void set_virtual(const std::vector<int>& axes, const std::vector<std::uint8_t>& buttons);

 private:
  struct Device;
  void run();
  void open_device(Device& device);
  void open_force_feedback(Device& device, Status& status);
  void close_device(Device& device);

  InputHub& hub_;
  Options options_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  mutable std::mutex mutex_;
  Status status_;
  RawInput raw_;
  WheelProfile active_profile_ = WheelProfile::logitech_g29();
  std::optional<WheelProfile> pending_profile_;
  double test_level_ = 0.0;
  std::chrono::steady_clock::time_point test_until_{};
  std::vector<int> virtual_axes_;
  std::vector<std::uint8_t> virtual_buttons_;
  bool virtual_dirty_ = false;
  std::optional<std::string> start_error_;
  std::atomic<bool> started_{false};
};

}  // namespace vsim

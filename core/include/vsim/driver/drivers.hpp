#pragma once

#include <memory>

#include "vsim/driver/profile.hpp"
#include "vsim/sim/block.hpp"

namespace vsim {

// Base for driver sources: owns the drv.cmd.* outputs (design doc §9.1).
class DriverBlock : public Block {
 public:
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override { return period_; }

 protected:
  virtual void init_driver(const ParamTree& params, Context& ctx) = 0;
  void write_command(SignalBus& bus, const DriverCommand& command) const;

  double period_ = 0.01;
  double max_steer_ = 0.0;  // steering wheel lock [rad]

 private:
  SignalId out_steer_ = 0, out_throttle_ = 0, out_brake_ = 0;
};

// Keyboard (browser or local), turned into smooth pedal and steering inputs
// with rate limits, return-to-centre and a speed-dependent steering limit.
class KeyboardDriver final : public DriverBlock {
 public:
  std::string_view name() const override { return "driver.keyboard"; }
  void step(double t, SignalBus& bus) override;

 private:
  void init_driver(const ParamTree& params, Context& ctx) override;

  InputHub* inputs_ = nullptr;
  SignalId speed_in_ = 0;
  DriverCommand command_;
  double throttle_rise_ = 2.5, throttle_fall_ = 5.0;  // [1/s]
  double brake_rise_ = 4.0, brake_fall_ = 8.0;        // [1/s]
  double steer_rate_ = 0.0, steer_return_ = 0.0;      // [rad/s] at the wheel
  double steer_speed_ref_ = 15.0;                     // [m/s]
  double steer_min_fraction_ = 0.15;
};

// Scripted profiles for steering and pedals, optionally with a PI speed hold
// replacing the pedal profiles.
class ManeuverDriver final : public DriverBlock {
 public:
  std::string_view name() const override { return "driver.maneuver"; }
  void step(double t, SignalBus& bus) override;

 private:
  void init_driver(const ParamTree& params, Context& ctx) override;

  Profile steer_deg_, throttle_, brake_;
  bool speed_hold_ = false;
  double speed_target_ = 0.0;  // [m/s]
  double kp_ = 0.5, ki_ = 0.3;
  double integral_ = 0.0;
  SignalId speed_in_ = 0;
};

// Command set from outside the loop (Python), sampled on the driver tick.
class ExternalDriver final : public DriverBlock {
 public:
  std::string_view name() const override { return "driver.external"; }
  void step(double t, SignalBus& bus) override;

 private:
  void init_driver(const ParamTree& params, Context& ctx) override;

  InputHub* inputs_ = nullptr;
};

// Builds the driver named by driver.type (keyboard | maneuver | external).
std::unique_ptr<DriverBlock> make_driver(const ParamTree& params);

}  // namespace vsim

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "vsim/driver/profile.hpp"
#include "vsim/sim/block.hpp"
#include "vsim/track/track.hpp"

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

// Driver in the loop (design doc §9.3): a steering wheel and pedals when one
// is connected (driver.input: auto | wheel | keyboard), else the keyboard.
// Posts the road torque at the steering wheel for force feedback.
class DilDriver final : public DriverBlock {
 public:
  enum class Source { Auto, Wheel, Keyboard };

  std::string_view name() const override { return "driver.dil"; }
  void step(double t, SignalBus& bus) override;

 private:
  void init_driver(const ParamTree& params, Context& ctx) override;

  InputHub* inputs_ = nullptr;
  Source source_ = Source::Auto;
  KeyboardDriver keyboard_;
  std::optional<SignalId> ffb_in_;
  SignalId wheel_out_ = 0;
};

// Autonomous driver (design doc §9.6): pure pursuit steering on a track line
// with speed-dependent lookahead, PI speed control on a quasi-steady-state
// speed profile (or a constant target speed). Needs a track.
//
//   driver: {type: pursuit, line: racing,          # default: first line of the track, else center
//            lookahead: {min_m: 2.5, gain_s: 0.12, max_m: 12}, steer_rate_deg_per_s: 800,
//            profile: {lateral_g: 1.3, accel_g: 0.9, brake_g: 1.2, max_kph: 100},
//            speed_kph: 40}                         # optional: constant speed instead of the profile
class PursuitDriver final : public DriverBlock {
 public:
  std::string_view name() const override { return "driver.pursuit"; }
  void step(double t, SignalBus& bus) override;

  const SampledCurve& line() const { return line_; }
  const std::vector<double>& speed_profile() const { return target_; }

 private:
  void init_driver(const ParamTree& params, Context& ctx) override;

  SampledCurve line_;
  std::vector<double> target_;  // [m/s] per line sample
  double wheelbase_ = 1.5, cg_to_rear_ = 0.7, steering_ratio_ = 4.5;
  double lookahead_min_ = 2.5, lookahead_gain_ = 0.12, lookahead_max_ = 12.0;
  double preview_ = 0.3;  // [s] look ahead on the speed profile
  double steer_rate_ = 0.0;  // [rad/s] at the steering wheel, like a driver's hands
  DriverCommand command_;
  double kp_ = 0.6, ki_ = 0.4;
  double integral_ = 0.0;
  bool first_ = true;
  double s_ = 0.0;  // position on the line
  SignalId x_in_ = 0, y_in_ = 0, yaw_in_ = 0, speed_in_ = 0;
  SignalId target_out_ = 0, lateral_out_ = 0;
};

// Builds the driver named by driver.type (dil | keyboard | maneuver | pursuit | mpc | external).
std::unique_ptr<DriverBlock> make_driver(const ParamTree& params);

}  // namespace vsim

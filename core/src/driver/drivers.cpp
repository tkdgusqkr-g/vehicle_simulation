#include "vsim/driver/drivers.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "vsim/driver/mpc.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/track/speed_profile.hpp"

namespace vsim {
namespace {

double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

// Moves `current` towards `target`, rising and falling at different rates.
double approach(double current, double target, double rise_rate, double fall_rate, double dt) {
  if (target > current) {
    return std::min(target, current + rise_rate * dt);
  }
  return std::max(target, current - fall_rate * dt);
}

}  // namespace

void DriverBlock::init(const ParamTree& params, Context& ctx) {
  period_ = params.get_double("driver.period_s", 0.01);
  max_steer_ = deg_to_rad(params.get_double("vehicle.steering.lock_deg"));
  SignalBus& bus = ctx.bus();
  out_steer_ = bus.declare("drv.cmd.steer", "rad", "steering wheel angle");
  out_throttle_ = bus.declare("drv.cmd.throttle", "-", "accelerator pedal");
  out_brake_ = bus.declare("drv.cmd.brake", "-", "brake pedal");
  init_driver(params, ctx);
}

void DriverBlock::write_command(SignalBus& bus, const DriverCommand& command) const {
  bus.set(out_steer_, std::clamp(command.steer, -max_steer_, max_steer_));
  bus.set(out_throttle_, std::clamp(command.throttle, 0.0, 1.0));
  bus.set(out_brake_, std::clamp(command.brake, 0.0, 1.0));
}

// --- KeyboardDriver -----------------------------------------------------------

void KeyboardDriver::init_driver(const ParamTree& params, Context& ctx) {
  inputs_ = &ctx.inputs();
  speed_in_ = ctx.bus().declare("veh.chassis.speed", "m/s", "speed over ground");
  throttle_rise_ = params.get_double("driver.throttle_rise_per_s", throttle_rise_);
  throttle_fall_ = params.get_double("driver.throttle_fall_per_s", throttle_fall_);
  brake_rise_ = params.get_double("driver.brake_rise_per_s", brake_rise_);
  brake_fall_ = params.get_double("driver.brake_fall_per_s", brake_fall_);
  steer_rate_ = deg_to_rad(params.get_double("driver.steer_rate_deg_per_s", 360.0));
  steer_return_ = deg_to_rad(params.get_double("driver.steer_return_deg_per_s", 540.0));
  steer_speed_ref_ = params.get_double("driver.steer_speed_ref_mps", steer_speed_ref_);
  steer_min_fraction_ = params.get_double("driver.steer_min_fraction", steer_min_fraction_);
}

void KeyboardDriver::step(double /*t*/, SignalBus& bus) {
  const KeyState keys = inputs_->keys();
  command_.throttle =
      approach(command_.throttle, keys.throttle ? 1.0 : 0.0, throttle_rise_, throttle_fall_, period_);
  command_.brake = approach(command_.brake, keys.brake ? 1.0 : 0.0, brake_rise_, brake_fall_, period_);

  // Less steering lock at speed, as a driver would use.
  const double speed = bus.get(speed_in_) / steer_speed_ref_;
  const double limit =
      max_steer_ * std::max(steer_min_fraction_, 1.0 / (1.0 + speed * speed));
  const double target = (keys.left ? limit : 0.0) - (keys.right ? limit : 0.0);
  const double rate = target == 0.0 ? steer_return_ : steer_rate_;
  command_.steer = std::clamp(approach(command_.steer, target, rate, rate, period_), -limit, limit);

  write_command(bus, command_);
}

// --- ManeuverDriver -----------------------------------------------------------

void ManeuverDriver::init_driver(const ParamTree& params, Context& ctx) {
  steer_deg_ = Profile::from_params(params.subtree("driver.steer"), "driver.steer");
  throttle_ = Profile::from_params(params.subtree("driver.throttle"), "driver.throttle");
  brake_ = Profile::from_params(params.subtree("driver.brake"), "driver.brake");
  speed_hold_ = params.has("driver.speed_hold_kph");
  if (speed_hold_) {
    speed_target_ = params.get_double("driver.speed_hold_kph") / 3.6;
    kp_ = params.get_double("driver.speed_hold_kp", kp_);
    ki_ = params.get_double("driver.speed_hold_ki", ki_);
  }
  speed_in_ = ctx.bus().declare("veh.chassis.speed", "m/s", "speed over ground");
}

void ManeuverDriver::step(double t, SignalBus& bus) {
  DriverCommand command;
  command.steer = deg_to_rad(steer_deg_.value(t));
  if (speed_hold_) {
    // PI on speed error: positive output is throttle, negative is brake.
    const double error = speed_target_ - bus.get(speed_in_);
    const double integral_limit = ki_ > 0.0 ? 1.0 / ki_ : 0.0;
    integral_ = std::clamp(integral_ + error * period_, -integral_limit, integral_limit);
    const double u = kp_ * error + ki_ * integral_;
    command.throttle = std::max(0.0, u);
    command.brake = std::max(0.0, -u);
  } else {
    command.throttle = throttle_.value(t);
    command.brake = brake_.value(t);
  }
  write_command(bus, command);
}

// --- ExternalDriver -----------------------------------------------------------

void ExternalDriver::init_driver(const ParamTree& /*params*/, Context& ctx) {
  inputs_ = &ctx.inputs();
}

void ExternalDriver::step(double /*t*/, SignalBus& bus) { write_command(bus, inputs_->command()); }

// --- DilDriver ----------------------------------------------------------------

void DilDriver::init_driver(const ParamTree& params, Context& ctx) {
  inputs_ = &ctx.inputs();
  const std::string source = params.get_string("driver.input", "auto");
  if (source == "auto") {
    source_ = Source::Auto;
  } else if (source == "wheel") {
    source_ = Source::Wheel;
  } else if (source == "keyboard") {
    source_ = Source::Keyboard;
  } else {
    throw std::invalid_argument("driver.input '" + source + "' (expected auto, wheel or keyboard)");
  }
  keyboard_.init(params, ctx);
  ffb_in_ = ctx.bus().find("veh.ffb.torque_sw");
  wheel_out_ = ctx.bus().declare("drv.input.wheel", "-", "1 while driving with the steering wheel");
}

void DilDriver::step(double t, SignalBus& bus) {
  const WheelState wheel = inputs_->wheel();
  const bool use_wheel =
      source_ == Source::Wheel || (source_ == Source::Auto && wheel.connected);
  if (use_wheel) {
    // A wheel that is required but unplugged gives no input at all.
    DriverCommand command;
    if (wheel.connected) {
      command = {wheel.steer, wheel.throttle, wheel.brake};
    }
    write_command(bus, command);
  } else {
    keyboard_.step(t, bus);
  }
  bus.set(wheel_out_, use_wheel ? 1.0 : 0.0);
  inputs_->set_force_feedback(ffb_in_ ? bus.get(*ffb_in_) : 0.0, max_steer_);
}

// --- PursuitDriver --------------------------------------------------------------

void PursuitDriver::init_driver(const ParamTree& params, Context& ctx) {
  const std::shared_ptr<const Track>& track = ctx.track();
  if (!track) {
    throw std::invalid_argument("driver.type pursuit needs a track (track.file or track.template)");
  }
  const auto& lines = track->definition().lines;
  const std::string line = params.get_string("driver.line", lines.empty() ? "center" : lines.front().name);
  line_ = track->line(line);

  wheelbase_ = params.get_double("vehicle.wheelbase_m");
  cg_to_rear_ = wheelbase_ - params.get_double("vehicle.cg_to_front_axle_m");
  steering_ratio_ = params.get_double("vehicle.steering.ratio");
  lookahead_min_ = params.get_double("driver.lookahead.min_m", lookahead_min_);
  lookahead_gain_ = params.get_double("driver.lookahead.gain_s", lookahead_gain_);
  lookahead_max_ = params.get_double("driver.lookahead.max_m", lookahead_max_);
  preview_ = params.get_double("driver.speed_preview_s", preview_);
  steer_rate_ = deg_to_rad(params.get_double("driver.steer_rate_deg_per_s", 800.0));
  kp_ = params.get_double("driver.speed_kp", kp_);
  ki_ = params.get_double("driver.speed_ki", ki_);
  if (!(lookahead_min_ > 0.0 && lookahead_max_ >= lookahead_min_ && lookahead_gain_ >= 0.0)) {
    throw std::invalid_argument("driver.lookahead: need 0 < min_m <= max_m and gain_s >= 0");
  }

  if (params.has("driver.speed_kph")) {
    target_.assign(line_.samples().size(), params.get_double("driver.speed_kph") / 3.6);
    if (!line_.closed()) {
      target_.back() = 0.0;  // stop at the end of an open line
    }
  } else {
    constexpr double g = 9.81;
    SpeedLimits limits;
    limits.lateral = params.get_double("driver.profile.lateral_g", 1.3) * g;
    limits.accel = params.get_double("driver.profile.accel_g", 0.9) * g;
    limits.brake = params.get_double("driver.profile.brake_g", 1.2) * g;
    limits.top = params.get_double("driver.profile.max_kph", 100.0) / 3.6;
    limits.start = limits.top;
    if (!(limits.lateral > 0.0 && limits.accel > 0.0 && limits.brake > 0.0 && limits.top > 0.0)) {
      throw std::invalid_argument("driver.profile: limits must be positive");
    }
    target_ = quasi_steady_speed(line_, limits);
  }

  SignalBus& bus = ctx.bus();
  x_in_ = bus.declare("veh.chassis.x", "m", "position east");
  y_in_ = bus.declare("veh.chassis.y", "m", "position north");
  yaw_in_ = bus.declare("veh.chassis.yaw", "rad", "heading, counter-clockwise from east");
  speed_in_ = bus.declare("veh.chassis.speed", "m/s", "speed over ground");
  target_out_ = bus.declare("drv.target_speed", "m/s", "speed the driver aims for");
  lateral_out_ = bus.declare("drv.line_offset", "m", "distance from the driving line, positive left");
}

void PursuitDriver::step(double /*t*/, SignalBus& bus) {
  const double yaw = bus.get(yaw_in_);
  const double speed = bus.get(speed_in_);
  // Pure pursuit works from the rear axle.
  const double rx = bus.get(x_in_) - cg_to_rear_ * std::cos(yaw);
  const double ry = bus.get(y_in_) - cg_to_rear_ * std::sin(yaw);
  const CurveProjection here = first_ ? line_.project(rx, ry) : line_.project(rx, ry, s_);
  first_ = false;
  s_ = here.s;

  // At the end of an open line: stop straight ahead.
  if (!line_.closed() && s_ >= line_.length() - 1.0) {
    command_.throttle = 0.0;
    command_.brake = 1.0;
    write_command(bus, command_);
    bus.set(target_out_, 0.0);
    bus.set(lateral_out_, here.n);
    return;
  }

  const double lookahead = std::clamp(lookahead_min_ + lookahead_gain_ * speed, lookahead_min_, lookahead_max_);
  const CurveSample goal = line_.at(s_ + lookahead);
  const double dx = goal.x - rx;
  const double dy = goal.y - ry;
  const double distance = std::max(std::hypot(dx, dy), 0.5);
  const double alpha = std::remainder(std::atan2(dy, dx) - yaw, 2.0 * std::numbers::pi);
  const double road_wheel = std::atan(2.0 * wheelbase_ * std::sin(alpha) / distance);

  // Target speed a little ahead, so braking starts before the corner.
  const auto profile_at = [&](double s) {
    const auto& samples = line_.samples();
    s = line_.wrap(s);
    const auto upper = std::upper_bound(samples.begin(), samples.end(), s,
                                        [](double value, const CurveSample& c) { return value < c.s; });
    const std::size_t i = upper == samples.begin() ? 0 : static_cast<std::size_t>(upper - samples.begin()) - 1;
    return target_[std::min(i, target_.size() - 1)];
  };
  const double target = std::min(profile_at(s_), profile_at(s_ + speed * preview_));
  const double error = target - speed;
  const double integral_limit = ki_ > 0.0 ? 1.0 / ki_ : 0.0;
  integral_ = std::clamp(integral_ + error * period_, -integral_limit, integral_limit);
  const double u = kp_ * error + ki_ * integral_;

  const double steer = road_wheel * steering_ratio_;
  const double max_change = steer_rate_ * period_;
  command_.steer += std::clamp(steer - command_.steer, -max_change, max_change);
  command_.throttle = std::max(0.0, u);
  command_.brake = std::max(0.0, -u);
  write_command(bus, command_);
  bus.set(target_out_, target);
  bus.set(lateral_out_, here.n);
}

// --- Factory ------------------------------------------------------------------

std::unique_ptr<DriverBlock> make_driver(const ParamTree& params) {
  const std::string type = params.get_string("driver.type", "external");
  if (type == "dil") {
    return std::make_unique<DilDriver>();
  }
  if (type == "keyboard") {
    return std::make_unique<KeyboardDriver>();
  }
  if (type == "pursuit") {
    return std::make_unique<PursuitDriver>();
  }
  if (type == "mpc") {
    return std::make_unique<MpcDriver>();
  }
  if (type == "maneuver") {
    return std::make_unique<ManeuverDriver>();
  }
  if (type == "external") {
    return std::make_unique<ExternalDriver>();
  }
  throw std::invalid_argument("driver.type '" + type +
                              "' is not available (expected dil, keyboard, maneuver, pursuit, mpc or external)");
}

}  // namespace vsim

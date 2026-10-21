#include "vsim/driver/drivers.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

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
  max_steer_ = deg_to_rad(params.get_double("vehicle.steering.max_wheel_angle_deg"));
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

// --- Factory ------------------------------------------------------------------

std::unique_ptr<DriverBlock> make_driver(const ParamTree& params) {
  const std::string type = params.get_string("driver.type", "external");
  if (type == "keyboard") {
    return std::make_unique<KeyboardDriver>();
  }
  if (type == "maneuver") {
    return std::make_unique<ManeuverDriver>();
  }
  if (type == "external") {
    return std::make_unique<ExternalDriver>();
  }
  throw std::invalid_argument("driver.type '" + type +
                              "' is not available (expected keyboard, maneuver or external)");
}

}  // namespace vsim

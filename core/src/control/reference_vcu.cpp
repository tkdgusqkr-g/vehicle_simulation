#include "vsim/control/reference_vcu.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vsim {

void ReferenceVcu::init(const ParamTree& params, Context& ctx) {
  SignalBus& bus = ctx.bus();
  period_ = params.get_double("control.period_s", 0.01);
  power_limit_ = params.get_double("control.power_limit_kw", 80.0) * 1e3;
  torque_max_ = params.get_double("vehicle.powertrain.motor_torque_max_nm");
  if (!(power_limit_ > 0.0)) {
    throw std::invalid_argument("control.power_limit_kw must be positive");
  }

  throttle_in_ = bus.declare("drv.cmd.throttle", "-", "accelerator pedal");
  motors_.clear();
  for (const std::string& corner : params.get_string_list("vehicle.powertrain.motors")) {
    const std::string base = "pt.motor." + corner;
    motors_.push_back({bus.declare(base + ".speed", "rad/s", "motor shaft speed"),
                       bus.declare(base + ".torque_cmd", "N*m", "motor torque command")});
  }
}

void ReferenceVcu::step(double /*t*/, SignalBus& bus) {
  const double torque = std::clamp(bus.get(throttle_in_), 0.0, 1.0) * torque_max_;
  double power = 0.0;
  for (const Motor& motor : motors_) {
    power += torque * std::abs(bus.get(motor.speed_in));
  }
  const double scale = power > power_limit_ ? power_limit_ / power : 1.0;
  for (const Motor& motor : motors_) {
    bus.set(motor.torque_cmd_out, torque * scale);
  }
}

}  // namespace vsim

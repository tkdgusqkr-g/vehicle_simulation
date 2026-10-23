#include "vsim/models/powertrain/motor.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace vsim {

MotorParams MotorParams::from_params(const ParamTree& params) {
  const auto get = [&](const char* key, double fallback) {
    return params.get_double(std::string("vehicle.powertrain.motor.") + key, fallback);
  };
  MotorParams p;
  p.torque_max = params.get_double("vehicle.powertrain.motor.torque_max_nm");
  p.power_max = params.get_double("vehicle.powertrain.motor.power_max_kw") * 1e3;
  p.speed_max = get("speed_max_rpm", p.speed_max * 30.0 / std::numbers::pi) * std::numbers::pi / 30.0;
  p.nominal_voltage = get("nominal_voltage_v", p.nominal_voltage);
  p.rotor_inertia = get("rotor_inertia_kgm2", p.rotor_inertia);
  p.torque_time_constant = get("torque_time_constant_s", p.torque_time_constant);
  p.loss_copper = get("losses.copper_w_per_nm2", p.loss_copper);
  p.loss_iron = get("losses.iron_w_per_radps", p.loss_iron);
  p.loss_windage = get("losses.windage_w_per_radps2", p.loss_windage);
  p.loss_constant = get("losses.constant_w", p.loss_constant);
  p.heat_capacity = get("thermal.heat_capacity_j_per_k", p.heat_capacity);
  p.thermal_resistance = get("thermal.resistance_k_per_w", p.thermal_resistance);
  p.coolant_temp = get("thermal.coolant_temp_c", p.coolant_temp);
  p.derate_start = get("thermal.derate_start_c", p.derate_start);
  p.derate_end = get("thermal.derate_end_c", p.derate_end);

  for (std::size_t i = 0; params.has_prefix("vehicle.powertrain.motor.ripple." + std::to_string(i)); ++i) {
    const std::string r = "vehicle.powertrain.motor.ripple." + std::to_string(i) + ".";
    Ripple ripple;
    ripple.order = params.get_double(r + "order");
    ripple.share = params.get_double(r + "amplitude_pct", 0.0) / 100.0;
    ripple.cogging = params.get_double(r + "cogging_nm", 0.0);
    ripple.phase = params.get_double(r + "phase_deg", 0.0) * std::numbers::pi / 180.0;
    if (!(ripple.order > 0.0) || ripple.share < 0.0 || ripple.cogging < 0.0) {
      throw std::invalid_argument(r + "order > 0, amplitude_pct and cogging_nm >= 0");
    }
    p.ripple.push_back(ripple);
  }

  if (!(p.torque_max > 0.0 && p.power_max > 0.0 && p.speed_max > 0.0 && p.nominal_voltage > 0.0)) {
    throw std::invalid_argument("vehicle.powertrain.motor: torque, power, speed and voltage must be positive");
  }
  if (!(p.torque_time_constant > 0.0 && p.heat_capacity > 0.0 && p.thermal_resistance > 0.0)) {
    throw std::invalid_argument("vehicle.powertrain.motor: time constant and thermal values must be positive");
  }
  if (!(p.derate_end > p.derate_start)) {
    throw std::invalid_argument("vehicle.powertrain.motor.thermal: derate_end_c must exceed derate_start_c");
  }
  return p;
}

double MotorParams::derate(double temperature) const {
  return std::clamp((derate_end - temperature) / (derate_end - derate_start), 0.0, 1.0);
}

double MotorParams::envelope(double speed, double dc_voltage, double temperature) const {
  const double w = std::abs(speed);
  // Back-EMF limits power above base speed; less DC voltage, less power.
  const double power = power_max * std::clamp(dc_voltage / nominal_voltage, 0.0, 1.0);
  double torque = w > 1e-9 ? std::min(torque_max, power / w) : torque_max;
  // Fade to zero over the last 5 % below the speed limit.
  torque *= std::clamp((speed_max - w) / (0.05 * speed_max), 0.0, 1.0);
  return torque * derate(temperature);
}

double MotorParams::ripple_torque(double torque, double angle, double speed, double max_frequency) const {
  double extra = 0.0;
  for (const Ripple& r : ripple) {
    const double frequency = r.order * std::abs(speed) / (2.0 * std::numbers::pi);
    const double ratio = frequency / max_frequency;
    const double fade = 1.0 / (1.0 + ratio * ratio * ratio * ratio);
    extra += fade * (r.share * torque + r.cogging) * std::sin(r.order * angle + r.phase);
  }
  return extra;
}

double MotorParams::losses(double torque, double speed) const {
  const double w = std::abs(speed);
  return loss_copper * torque * torque + loss_iron * w + loss_windage * w * w + loss_constant;
}

}  // namespace vsim

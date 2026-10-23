#pragma once

#include <vector>

#include "vsim/sim/params.hpp"

namespace vsim {

// Inverter + motor, Level A (design doc §7.3): torque envelope that shrinks
// with DC voltage and temperature, parametric losses, lumped thermal model.
struct MotorParams {
  double torque_max = 21.0;          // [N m]
  double power_max = 35e3;           // [W] at nominal voltage
  double speed_max = 2094.0;         // [rad/s]
  double nominal_voltage = 600.0;    // [V] the envelope is specified at
  double rotor_inertia = 3e-4;       // [kg m^2]
  double torque_time_constant = 0.002;  // [s]
  // Losses (motor + inverter): copper k_c T^2, iron k_i |w|, windage k_w w^2, constant.
  double loss_copper = 2.0;          // [W / (N m)^2]
  double loss_iron = 0.3;            // [W / (rad/s)]
  double loss_windage = 1e-5;        // [W / (rad/s)^2]
  double loss_constant = 30.0;       // [W]
  double heat_capacity = 3000.0;     // [J/K]
  double thermal_resistance = 0.05;  // [K/W] to coolant
  double coolant_temp = 35.0;        // [degC]
  double derate_start = 100.0;       // [degC] full torque below
  double derate_end = 140.0;         // [degC] zero torque above
  // Torque ripple (design doc §10.4, PH-04): harmonics of the rotor angle, `order` cycles per
  // mechanical revolution (electrical harmonic x pole pairs), a share of the torque plus a
  // load-independent part (cogging).
  struct Ripple {
    double order = 6.0, share = 0.0, cogging = 0.0, phase = 0.0;  // -, -, [N m], [rad]
  };
  std::vector<Ripple> ripple;

  // Reads vehicle.powertrain.motor.*
  static MotorParams from_params(const ParamTree& params);

  // Torque available at |speed| for the given DC voltage and winding temperature.
  double envelope(double speed, double dc_voltage, double temperature) const;
  double derate(double temperature) const;
  double losses(double torque, double speed) const;
  // Ripple on top of `torque` at rotor angle `angle` [rad]. Harmonics above `max_frequency`
  // [Hz] fade out (the plant step cannot carry them; the rotor and wheel inertia filter them).
  double ripple_torque(double torque, double angle, double speed, double max_frequency) const;
};

}  // namespace vsim

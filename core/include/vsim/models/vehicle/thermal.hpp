#pragma once

#include <array>
#include <vector>

#include "vsim/models/vehicle/corners.hpp"
#include "vsim/sim/params.hpp"

namespace vsim {

// Tire temperatures and pressure (design doc §6.4, PH-13). Two lumped
// temperatures per tire: the tread surface, heated by the friction of
// sliding (its share of the slip power) and cooled by the air and the road,
// and the core (carcass and inflation air), heated by rolling hysteresis and
// the brake through the rim. Grip peaks at an optimum surface temperature and
// falls off as a parabola either side; the inflation pressure follows the
// core temperature (ideal gas at constant volume) and grip falls off away
// from its optimum too.
//
//   vehicle.tire.thermal: {optimum_c: 70, window_c: 40, grip_loss: 0.12, min_grip: 0.6,
//     surface_heat_capacity_j_per_k: 500, core_heat_capacity_j_per_k: 3500,
//     surface_core_w_per_k: 35, surface_air_w_per_k: 6, surface_air_w_per_k_per_mps: 2.0,
//     core_air_w_per_k: 2, core_air_w_per_k_per_mps: 0.3, road_contact_w_per_k: 15,
//     slip_heat_share: 0.6, rim_brake_w_per_k: 0.8}
//   vehicle.tire.pressure: {cold_kpa: 80, cold_c: 20, optimum_kpa: 95, grip_loss: 0.5}
// (gauge pressures; grip_loss per (relative pressure error)^2)
struct TireThermalParams {
  bool enabled = false;
  double optimum = 70.0, window = 40.0, grip_loss = 0.12, min_grip = 0.6;
  double surface_capacity = 500.0, core_capacity = 3500.0;
  double surface_core = 35.0;
  double surface_air = 6.0, surface_air_speed = 2.0;
  double core_air = 2.0, core_air_speed = 0.3;
  double road_contact = 15.0;
  double slip_share = 0.6;
  double rim_brake = 0.8;
  // Pressure (gauge) [kPa].
  double cold_pressure = 80.0, cold_temp = 20.0, optimum_pressure = 95.0, pressure_grip_loss = 0.5;

  static TireThermalParams from_params(const ParamTree& params);

  double temperature_grip(double surface_temp) const;
  double pressure(double core_temp) const;  // gauge [kPa]
  double pressure_grip(double pressure_kpa) const;
};

// Brake disc temperature and pad friction (design doc §10.4, PH-17). One
// lumped disc per corner heated by the brake power and cooled by convection
// (growing with speed) and radiation; the pad friction is a factor on the
// nominal brake gain, interpolated from a table over disc temperature (cold
// pads bite less, hot pads fade).
//
//   vehicle.brakes.thermal: {front_heat_capacity_j_per_k: 450, rear_heat_capacity_j_per_k: 350,
//     cooling_w_per_k: 2.5, cooling_w_per_k_per_mps: 0.6, emissivity_area_m2: 0.02,
//     pad_mu_temp_c: [20, 120, 450, 650], pad_mu_factor: [0.85, 1.0, 1.0, 0.7]}
struct BrakeThermalParams {
  bool enabled = false;
  std::array<double, 2> heat_capacity{450.0, 350.0};  // front, rear disc [J/K]
  double cooling = 2.5, cooling_speed = 0.6;           // [W/K], [W/K per m/s]
  double emissive_area = 0.02;                         // emissivity x radiating area [m^2]
  std::vector<double> mu_temp{20.0, 120.0, 450.0, 650.0};
  std::vector<double> mu_factor{0.85, 1.0, 1.0, 0.7};

  static BrakeThermalParams from_params(const ParamTree& params);

  double pad_mu(double disc_temp) const;
  // Heat leaving one disc at a road speed [W].
  double cooling_power(double disc_temp, double ambient, double speed) const;
};

}  // namespace vsim

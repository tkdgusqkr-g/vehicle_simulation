#include "vsim/models/vehicle/thermal.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace vsim {
namespace {

constexpr double kKelvin = 273.15;
constexpr double kAtmosphere = 101.325;   // [kPa]
constexpr double kStefanBoltzmann = 5.670374e-8;

void require_positive(double value, const std::string& key) {
  if (!(value > 0.0)) {
    throw std::invalid_argument(key + " must be positive");
  }
}

void require_non_negative(double value, const std::string& key) {
  if (!(value >= 0.0)) {
    throw std::invalid_argument(key + " must not be negative");
  }
}

}  // namespace

TireThermalParams TireThermalParams::from_params(const ParamTree& params) {
  TireThermalParams p;
  const std::string t = "vehicle.tire.thermal.";
  p.enabled = params.has_prefix("vehicle.tire.thermal");
  p.optimum = params.get_double(t + "optimum_c", p.optimum);
  p.window = params.get_double(t + "window_c", p.window);
  p.grip_loss = params.get_double(t + "grip_loss", p.grip_loss);
  p.min_grip = params.get_double(t + "min_grip", p.min_grip);
  p.surface_capacity = params.get_double(t + "surface_heat_capacity_j_per_k", p.surface_capacity);
  p.core_capacity = params.get_double(t + "core_heat_capacity_j_per_k", p.core_capacity);
  p.surface_core = params.get_double(t + "surface_core_w_per_k", p.surface_core);
  p.surface_air = params.get_double(t + "surface_air_w_per_k", p.surface_air);
  p.surface_air_speed = params.get_double(t + "surface_air_w_per_k_per_mps", p.surface_air_speed);
  p.core_air = params.get_double(t + "core_air_w_per_k", p.core_air);
  p.core_air_speed = params.get_double(t + "core_air_w_per_k_per_mps", p.core_air_speed);
  p.road_contact = params.get_double(t + "road_contact_w_per_k", p.road_contact);
  p.slip_share = params.get_double(t + "slip_heat_share", p.slip_share);
  p.rim_brake = params.get_double(t + "rim_brake_w_per_k", p.rim_brake);
  const std::string pr = "vehicle.tire.pressure.";
  p.cold_pressure = params.get_double(pr + "cold_kpa", p.cold_pressure);
  p.cold_temp = params.get_double(pr + "cold_c", p.cold_temp);
  p.optimum_pressure = params.get_double(pr + "optimum_kpa", p.optimum_pressure);
  p.pressure_grip_loss = params.get_double(pr + "grip_loss", p.pressure_grip_loss);
  if (!p.enabled) {
    return p;
  }
  require_positive(p.window, t + "window_c");
  require_positive(p.surface_capacity, t + "surface_heat_capacity_j_per_k");
  require_positive(p.core_capacity, t + "core_heat_capacity_j_per_k");
  require_positive(p.optimum_pressure, pr + "optimum_kpa");
  for (const double v : {p.grip_loss, p.surface_core, p.surface_air, p.surface_air_speed, p.core_air,
                         p.core_air_speed, p.road_contact, p.rim_brake, p.pressure_grip_loss}) {
    require_non_negative(v, t + "* (heat paths and grip losses)");
  }
  if (!(p.min_grip > 0.0 && p.min_grip <= 1.0) || !(p.slip_share >= 0.0 && p.slip_share <= 1.0)) {
    throw std::invalid_argument(t + "min_grip in (0, 1] and slip_heat_share in [0, 1]");
  }
  if (!(p.cold_pressure + kAtmosphere > 0.0 && p.cold_temp > -kKelvin)) {
    throw std::invalid_argument(pr + "cold_kpa and cold_c out of range");
  }
  return p;
}

double TireThermalParams::temperature_grip(double surface_temp) const {
  const double e = (surface_temp - optimum) / window;
  return std::max(min_grip, 1.0 - grip_loss * e * e);
}

double TireThermalParams::pressure(double core_temp) const {
  return (cold_pressure + kAtmosphere) * (core_temp + kKelvin) / (cold_temp + kKelvin) - kAtmosphere;
}

double TireThermalParams::pressure_grip(double pressure_kpa) const {
  const double e = (pressure_kpa - optimum_pressure) / optimum_pressure;
  return std::max(min_grip, 1.0 - pressure_grip_loss * e * e);
}

BrakeThermalParams BrakeThermalParams::from_params(const ParamTree& params) {
  BrakeThermalParams p;
  const std::string b = "vehicle.brakes.thermal.";
  p.enabled = params.has_prefix("vehicle.brakes.thermal");
  p.heat_capacity[0] = params.get_double(b + "front_heat_capacity_j_per_k", p.heat_capacity[0]);
  p.heat_capacity[1] = params.get_double(b + "rear_heat_capacity_j_per_k", p.heat_capacity[1]);
  p.cooling = params.get_double(b + "cooling_w_per_k", p.cooling);
  p.cooling_speed = params.get_double(b + "cooling_w_per_k_per_mps", p.cooling_speed);
  p.emissive_area = params.get_double(b + "emissivity_area_m2", p.emissive_area);
  if (params.has_prefix(b + "pad_mu_temp_c") || params.has_prefix(b + "pad_mu_factor")) {
    p.mu_temp = params.get_double_list(b + "pad_mu_temp_c");
    p.mu_factor = params.get_double_list(b + "pad_mu_factor");
  }
  if (!p.enabled) {
    return p;
  }
  require_positive(p.heat_capacity[0], b + "front_heat_capacity_j_per_k");
  require_positive(p.heat_capacity[1], b + "rear_heat_capacity_j_per_k");
  require_non_negative(p.cooling, b + "cooling_w_per_k");
  require_non_negative(p.cooling_speed, b + "cooling_w_per_k_per_mps");
  require_non_negative(p.emissive_area, b + "emissivity_area_m2");
  if (p.mu_temp.empty() || p.mu_temp.size() != p.mu_factor.size() ||
      !std::is_sorted(p.mu_temp.begin(), p.mu_temp.end())) {
    throw std::invalid_argument(b + "pad_mu_temp_c (ascending) and pad_mu_factor need the same length");
  }
  for (const double f : p.mu_factor) {
    require_non_negative(f, b + "pad_mu_factor");
  }
  return p;
}

double BrakeThermalParams::pad_mu(double disc_temp) const {
  if (disc_temp <= mu_temp.front()) {
    return mu_factor.front();
  }
  if (disc_temp >= mu_temp.back()) {
    return mu_factor.back();
  }
  const auto it = std::upper_bound(mu_temp.begin(), mu_temp.end(), disc_temp);
  const std::size_t i = static_cast<std::size_t>(it - mu_temp.begin());
  const double f = (disc_temp - mu_temp[i - 1]) / (mu_temp[i] - mu_temp[i - 1]);
  return mu_factor[i - 1] + f * (mu_factor[i] - mu_factor[i - 1]);
}

double BrakeThermalParams::cooling_power(double disc_temp, double ambient, double speed) const {
  const double t = disc_temp + kKelvin, a = ambient + kKelvin;
  return (cooling + cooling_speed * std::abs(speed)) * (disc_temp - ambient) +
         emissive_area * kStefanBoltzmann * (t * t * t * t - a * a * a * a);
}

}  // namespace vsim

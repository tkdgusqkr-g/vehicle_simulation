#include "vsim/models/powertrain/battery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace vsim {

BatteryParams BatteryParams::from_params(const ParamTree& params) {
  const auto get = [&](const std::string& key, double fallback) {
    return params.get_double("vehicle.battery." + key, fallback);
  };
  BatteryParams p;
  p.series = static_cast<int>(params.get_int("vehicle.battery.cells_series"));
  p.parallel = static_cast<int>(params.get_int("vehicle.battery.cells_parallel"));
  p.cell_capacity_ah = params.get_double("vehicle.battery.cell_capacity_ah");
  p.ocv_soc = params.get_double_list("vehicle.battery.ocv_soc");
  p.ocv_voltage = params.get_double_list("vehicle.battery.ocv_v");
  p.r0 = get("r0_mohm", p.r0 * 1e3) * 1e-3;
  for (std::size_t i = 0;; ++i) {
    const std::string key = "rc." + std::to_string(i);
    if (!params.has_prefix("vehicle.battery." + key)) {
      break;
    }
    const double resistance = params.get_double("vehicle.battery." + key + ".r_mohm") * 1e-3;
    const double tau = params.get_double("vehicle.battery." + key + ".tau_s");
    if (!(resistance > 0.0 && tau > 0.0)) {
      throw std::invalid_argument("vehicle.battery.rc: r_mohm and tau_s must be positive");
    }
    p.rc.push_back({resistance, tau / resistance});
  }
  p.extra_resistance = get("extra_resistance_mohm", p.extra_resistance * 1e3) * 1e-3;
  p.cell_voltage_min = get("cell_voltage_min_v", p.cell_voltage_min);
  p.cell_voltage_max = get("cell_voltage_max_v", p.cell_voltage_max);
  p.discharge_current_max = get("discharge_current_max_a", p.discharge_current_max);
  p.charge_current_max = get("charge_current_max_a", p.charge_current_max);
  p.heat_capacity = get("thermal.heat_capacity_j_per_k", p.heat_capacity);
  p.cooling = get("thermal.cooling_w_per_k", p.cooling);
  p.derate_start = get("thermal.derate_start_c", p.derate_start);
  p.cutoff = get("thermal.cutoff_c", p.cutoff);
  p.aux_power = get("aux_power_w", p.aux_power);

  if (p.series <= 0 || p.parallel <= 0 || !(p.cell_capacity_ah > 0.0) || !(p.r0 > 0.0)) {
    throw std::invalid_argument(
        "vehicle.battery: cells_series, cells_parallel, cell_capacity_ah and r0_mohm must be positive");
  }
  if (p.rc.size() > 2) {
    throw std::invalid_argument("vehicle.battery.rc: at most two RC pairs");
  }
  if (p.ocv_soc.size() < 2 || p.ocv_soc.size() != p.ocv_voltage.size() ||
      !std::is_sorted(p.ocv_soc.begin(), p.ocv_soc.end())) {
    throw std::invalid_argument(
        "vehicle.battery: ocv_soc (ascending) and ocv_v need the same length, at least 2");
  }
  if (!(p.cutoff > p.derate_start) || !(p.heat_capacity > 0.0)) {
    throw std::invalid_argument("vehicle.battery.thermal: check heat capacity and derating temperatures");
  }
  return p;
}

double BatteryParams::ocv_cell(double soc) const {
  if (soc <= ocv_soc.front()) {
    return ocv_voltage.front();
  }
  if (soc >= ocv_soc.back()) {
    return ocv_voltage.back();
  }
  const auto upper = std::upper_bound(ocv_soc.begin(), ocv_soc.end(), soc);
  const std::size_t i = static_cast<std::size_t>(upper - ocv_soc.begin());
  const double t = (soc - ocv_soc[i - 1]) / (ocv_soc[i] - ocv_soc[i - 1]);
  return ocv_voltage[i - 1] + t * (ocv_voltage[i] - ocv_voltage[i - 1]);
}

double BatteryParams::pack_resistance() const {
  return series * r0 / parallel + extra_resistance;
}

BatteryOperatingPoint solve_battery_power(double open_voltage, double resistance, double power) {
  // power = (V_oc - R I) I  ->  R I^2 - V_oc I + P = 0, smaller root.
  BatteryOperatingPoint point;
  const double discriminant = open_voltage * open_voltage - 4.0 * resistance * power;
  if (discriminant < 0.0) {
    point.current = open_voltage / (2.0 * resistance);  // maximum power point
    point.limited = true;
  } else {
    point.current = 2.0 * power / (open_voltage + std::sqrt(discriminant));
  }
  point.voltage = open_voltage - resistance * point.current;
  return point;
}

CellExtremes cell_extremes(const BatteryParams& p, const CellSpread& spread, double mean_soc,
                           double rc_voltage, double current) {
  CellExtremes x;
  x.v_min = std::numeric_limits<double>::infinity();
  x.v_max = -x.v_min;
  x.soc_min = x.v_min;
  x.soc_max = x.v_max;
  const double cell_current = current / p.parallel;
  for (std::size_t g = 0; g < spread.capacity.size(); ++g) {
    const double soc = spread.group_soc(g, mean_soc);
    const double r = spread.resistance[g];
    const double v = p.ocv_cell(soc) - r * (rc_voltage + p.r0 * cell_current);
    if (v < x.v_min) {
      x.v_min = v;
      x.weakest = g;
    }
    x.v_max = std::max(x.v_max, v);
    x.soc_min = std::min(x.soc_min, soc);
    x.soc_max = std::max(x.soc_max, soc);
  }
  return x;
}

BatteryLimits battery_limits(const BatteryParams& p, double soc, double rc_voltage,
                             double temperature, const CellSpread* spread) {
  const double thermal = std::clamp((p.cutoff - temperature) / (p.cutoff - p.derate_start), 0.0, 1.0);
  double soc_low = soc, soc_high = soc;
  const double cell_open = p.ocv_cell(soc) - rc_voltage;
  double by_min_voltage = p.parallel * (cell_open - p.cell_voltage_min) / p.r0;
  double by_max_voltage = p.parallel * (p.cell_voltage_max - cell_open) / p.r0;
  if (spread != nullptr && !spread->empty()) {
    // The weakest group sets each limit.
    for (std::size_t g = 0; g < spread->capacity.size(); ++g) {
      const double group_soc = spread->group_soc(g, soc);
      const double r = spread->resistance[g];
      const double open = p.ocv_cell(group_soc) - r * rc_voltage;
      by_min_voltage = std::min(by_min_voltage, p.parallel * (open - p.cell_voltage_min) / (r * p.r0));
      by_max_voltage = std::min(by_max_voltage, p.parallel * (p.cell_voltage_max - open) / (r * p.r0));
      soc_low = std::min(soc_low, group_soc);
      soc_high = std::max(soc_high, group_soc);
    }
  }
  const double empty = std::clamp(soc_low / 0.05, 0.0, 1.0);
  const double full = std::clamp((1.0 - soc_high) / 0.1, 0.0, 1.0);

  BatteryLimits limits;
  limits.discharge_current =
      std::max(0.0, std::min(p.discharge_current_max * thermal * empty, by_min_voltage));
  limits.charge_current =
      std::max(0.0, std::min(p.charge_current_max * thermal * full, by_max_voltage));
  const double open = p.series * cell_open;
  const double resistance = p.pack_resistance();
  limits.discharge_power = limits.discharge_current * (open - resistance * limits.discharge_current);
  limits.charge_power = limits.charge_current * (open + resistance * limits.charge_current);
  return limits;
}

}  // namespace vsim

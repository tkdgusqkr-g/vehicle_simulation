#pragma once

#include <vector>

#include "vsim/sim/params.hpp"

namespace vsim {

// Accumulator as identical cells, series x parallel (design doc §7.2):
// OCV(SoC) - R0 - up to two RC pairs per cell, lumped thermal model and BMS
// current limits.
struct BatteryParams {
  struct RcPair {
    double resistance = 0.0;   // [ohm] per cell
    double capacitance = 0.0;  // [F] per cell
  };

  int series = 140;
  int parallel = 4;
  double cell_capacity_ah = 3.0;
  std::vector<double> ocv_soc;      // ascending
  std::vector<double> ocv_voltage;  // [V] per cell
  double r0 = 0.015;                // [ohm] per cell
  std::vector<RcPair> rc;
  double extra_resistance = 0.03;   // [ohm] busbars, fuses, contactors
  double cell_voltage_min = 3.0;    // [V]
  double cell_voltage_max = 4.2;    // [V]
  double discharge_current_max = 200.0;  // [A] pack
  double charge_current_max = 60.0;      // [A] pack (regen)
  double heat_capacity = 40000.0;   // [J/K] pack
  double cooling = 15.0;            // [W/K] to ambient
  double derate_start = 50.0;       // [degC] full current below
  double cutoff = 60.0;             // [degC] zero current above
  double aux_power = 300.0;         // [W] constant LV and cooling load

  // Reads vehicle.battery.*
  static BatteryParams from_params(const ParamTree& params);

  double ocv_cell(double soc) const;
  double pack_resistance() const;  // ohmic, terminals to cells
};

// Cell-to-cell spread (design doc §7.2, PH-06): each series group of parallel
// cells has its own capacity and resistance factor and starting SoC offset.
// All groups carry the pack current, so a group's SoC and RC voltages follow
// the mean group's exactly (scaled by its factors): no extra states. The BMS
// limits then come from the weakest group (lowest voltage under discharge,
// highest under charge).
//
//   vehicle.battery.spread: {capacity_pct: 1.5, resistance_pct: 6, soc_pct: 0.5,   # 1 sigma, sampled
//                            weak_group: {capacity_pct: -8, resistance_pct: 25, soc_pct: -1}}
// Sampled when realism.variation is real (seeded, one car); weak_group (group 0)
// always applies.
struct CellSpread {
  std::vector<double> capacity;    // factor per series group
  std::vector<double> resistance;  // factor per series group
  std::vector<double> soc_offset;  // at the start
  double soc_start = 0.0;          // mean SoC at the start

  bool empty() const { return capacity.empty(); }
  double group_soc(std::size_t group, double mean_soc) const {
    return soc_start + soc_offset[group] + (mean_soc - soc_start) / capacity[group];
  }
};

struct CellExtremes {
  double v_min = 0.0, v_max = 0.0, soc_min = 0.0, soc_max = 0.0;
  std::size_t weakest = 0;  // the group with the lowest voltage
};

// Cell voltages of every group at pack current `current` [A] (discharge +),
// with `rc_voltage` the mean group's summed RC voltage.
CellExtremes cell_extremes(const BatteryParams& params, const CellSpread& spread, double mean_soc,
                           double rc_voltage, double current);

struct BatteryOperatingPoint {
  double current = 0.0;   // [A], positive = discharge
  double voltage = 0.0;   // [V] at the terminals
  bool limited = false;   // the requested power exceeded what the pack can deliver
};

// Current and terminal voltage for a terminal power draw from a source with
// open-circuit voltage `open_voltage` and series resistance `resistance`.
BatteryOperatingPoint solve_battery_power(double open_voltage, double resistance, double power);

struct BatteryLimits {
  double discharge_current = 0.0;  // [A]
  double charge_current = 0.0;     // [A]
  double discharge_power = 0.0;    // [W] at the terminals
  double charge_power = 0.0;       // [W] at the terminals
};

// BMS limits for the present state: temperature derating, SoC windows and
// cell voltage limits. `rc_voltage` is the summed RC voltage of one cell.
BatteryLimits battery_limits(const BatteryParams& params, double soc, double rc_voltage,
                             double temperature, const CellSpread* spread = nullptr);

}  // namespace vsim

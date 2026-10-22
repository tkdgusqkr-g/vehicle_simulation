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
                             double temperature);

}  // namespace vsim

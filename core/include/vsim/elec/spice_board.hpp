#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "vsim/sim/block.hpp"

namespace vsim {

// A circuit board solved in ngspice alongside the simulation (design doc
// §12.4): a SPICE netlist exported from the schematic (KiCad, Altium), its
// inputs driven by signals and its node voltages and branch currents written
// back as signals. ngspice is loaded at run time (the shared library), so
// the core builds and runs without it.
//
//   board:
//     netlist: boards/apps_conditioning.cir   # relative to the project
//     rate_hz: 1000                            # exchange with the simulation (one block period)
//     max_step_us: 10                          # ngspice's largest time step
//     after: sensors                           # sensors (default) | vcu: where it runs
//     inputs:                                  # sources the netlist names: a signal or a value
//       V_APPS1: sens.apps.ch1
//       V_SUPPLY: {value: 5.0}
//     outputs:                                 # node (voltage to ground) or i(<source>): signal
//       ADC1: board.apps.adc1
//       i(V_SUPPLY): board.apps.supply_current
//     library: ...                             # optional; else VSIM_NGSPICE, then the usual names
//
// Each named source is made an `external` source (its line in the netlist is
// rewritten), analyses and control blocks of the file are dropped and a
// transient analysis is added; `.include` / `.lib` paths become absolute.
// The board runs one period behind: at time t it writes its outputs at t
// (solved from inputs held over the previous period) and takes the inputs
// for the next one. ngspice keeps one circuit per process: one board per
// simulation (several boards go into one top-level netlist).
class SpiceBoard final : public Block {
 public:
  SpiceBoard();
  ~SpiceBoard() override;

  static bool configured(const ParamTree& params);
  // Where it runs: "sensors" or "vcu".
  static std::string after(const ParamTree& params);
  // True if the ngspice shared library loads (`library` or the default names).
  static bool available(const std::string& library = {});

  std::string_view name() const override { return "board"; }
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override;
  void step(double t, SignalBus& bus) override;

  // The netlist as given to ngspice (for tests and error messages).
  const std::string& prepared_netlist() const;

  struct Impl;

 private:
  std::unique_ptr<Impl> impl_;
};

// The netlist prepared for co-simulation: `inputs` made external sources,
// analyses and .control blocks dropped, include paths made absolute against
// `dir`, a transient analysis and .save lines for `saves` added.
std::string prepare_board_netlist(std::string_view text, const std::string& dir,
                                  const std::vector<std::string>& inputs, const std::vector<std::string>& saves,
                                  double max_step, double stop_time);

}  // namespace vsim

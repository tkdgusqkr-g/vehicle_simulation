#pragma once

#include <string_view>

#include "vsim/sim/context.hpp"
#include "vsim/sim/params.hpp"
#include "vsim/sim/signal_bus.hpp"

namespace vsim {

// Discrete-time component (driver, VCU, sensors, plugin adapters).
// Built-in implementations and plugins share this interface (design doc P-8).
class Block {
 public:
  virtual ~Block() = default;

  virtual std::string_view name() const = 0;
  // Declares output signals, resolves input signals and reads parameters.
  // `params` is the whole scenario tree.
  virtual void init(const ParamTree& params, Context& ctx) = 0;
  // Execution period [s]; read after init, must be a multiple of the plant step.
  virtual double period() const = 0;
  // Reads inputs from the bus and writes outputs to it. Outputs are held
  // until the block runs again (zero-order hold).
  virtual void step(double t, SignalBus& bus) = 0;
  // Every tick, right after the plant has read its inputs (a block that
  // stood in for a signal for the plant puts the original value back).
  virtual void after_plant_inputs(SignalBus& /*bus*/) {}
};

}  // namespace vsim

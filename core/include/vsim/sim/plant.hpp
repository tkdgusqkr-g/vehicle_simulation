#pragma once

#include <cstddef>
#include <span>
#include <string_view>

#include "vsim/sim/context.hpp"
#include "vsim/sim/params.hpp"
#include "vsim/sim/signal_bus.hpp"

namespace vsim {

// Continuous-time model integrated as one coupled system (design doc P-3).
class PlantModel {
 public:
  virtual ~PlantModel() = default;

  virtual std::string_view name() const = 0;
  // Declares output signals, resolves input signals and reads parameters.
  virtual void init(const ParamTree& params, Context& ctx) = 0;

  virtual std::size_t state_size() const = 0;
  virtual void initial_state(std::span<double> x) const = 0;

  // Latches inputs from the bus; they stay constant over the next step.
  virtual void read_inputs(const SignalBus& bus) = 0;
  // After read_inputs, with the state at the start of the step [t, t + dt]: a
  // faster discrete subsystem (an inverter's current loop) advances here.
  virtual void begin_step(double /*t*/, std::span<const double> /*x*/, double /*dt*/) {}
  virtual void derivatives(double t, std::span<const double> x, std::span<double> dx) const = 0;
  virtual void write_outputs(double t, std::span<const double> x, SignalBus& bus) const = 0;
};

}  // namespace vsim

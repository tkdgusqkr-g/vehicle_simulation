#pragma once

#include <cstdint>
#include <string_view>

#include "vsim/sim/input_hub.hpp"
#include "vsim/sim/realism.hpp"
#include "vsim/sim/rng.hpp"
#include "vsim/sim/signal_bus.hpp"

namespace vsim {

// Services handed to blocks and plant models during init.
class Context {
 public:
  Context(SignalBus& bus, InputHub& inputs, RealismConfig realism, std::uint64_t master_seed,
          double dt)
      : bus_(bus), inputs_(inputs), realism_(realism), master_seed_(master_seed), dt_(dt) {}

  SignalBus& bus() { return bus_; }
  InputHub& inputs() { return inputs_; }
  const RealismConfig& realism() const { return realism_; }
  std::uint64_t master_seed() const { return master_seed_; }
  double dt() const { return dt_; }  // plant step [s]

  // Independent random stream; use a stable, unique name per source
  // (e.g. "sensors.imu_main.noise").
  Rng make_rng(std::string_view stream) const { return Rng(derive_seed(master_seed_, stream)); }

 private:
  SignalBus& bus_;
  InputHub& inputs_;
  RealismConfig realism_;
  std::uint64_t master_seed_;
  double dt_;
};

}  // namespace vsim

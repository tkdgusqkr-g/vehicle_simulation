#pragma once

#include <span>
#include <vector>

#include "vsim/sim/plant.hpp"

namespace vsim {

// Classic fourth-order Runge-Kutta with fixed step. Inputs latched by the
// plant are held constant across the four stages.
class Rk4Integrator {
 public:
  void step(const PlantModel& plant, double t, double dt, std::span<double> x);

 private:
  std::vector<double> k1_, k2_, k3_, k4_, tmp_;
};

}  // namespace vsim

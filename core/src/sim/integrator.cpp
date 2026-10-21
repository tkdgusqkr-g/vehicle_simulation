#include "vsim/sim/integrator.hpp"

namespace vsim {

void Rk4Integrator::step(const PlantModel& plant, double t, double dt, std::span<double> x) {
  const std::size_t n = x.size();
  if (k1_.size() != n) {
    k1_.assign(n, 0.0);
    k2_.assign(n, 0.0);
    k3_.assign(n, 0.0);
    k4_.assign(n, 0.0);
    tmp_.assign(n, 0.0);
  }
  const double half = 0.5 * dt;

  plant.derivatives(t, x, k1_);
  for (std::size_t i = 0; i < n; ++i) {
    tmp_[i] = x[i] + half * k1_[i];
  }
  plant.derivatives(t + half, tmp_, k2_);
  for (std::size_t i = 0; i < n; ++i) {
    tmp_[i] = x[i] + half * k2_[i];
  }
  plant.derivatives(t + half, tmp_, k3_);
  for (std::size_t i = 0; i < n; ++i) {
    tmp_[i] = x[i] + dt * k3_[i];
  }
  plant.derivatives(t + dt, tmp_, k4_);
  for (std::size_t i = 0; i < n; ++i) {
    x[i] += dt / 6.0 * (k1_[i] + 2.0 * k2_[i] + 2.0 * k3_[i] + k4_[i]);
  }
}

}  // namespace vsim

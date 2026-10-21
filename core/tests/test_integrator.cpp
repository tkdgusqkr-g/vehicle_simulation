#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "vsim/sim/integrator.hpp"

namespace vsim {
namespace {

// x'' = -w^2 x as a first-order system.
class Oscillator final : public PlantModel {
 public:
  explicit Oscillator(double omega) : omega_(omega) {}
  std::string_view name() const override { return "oscillator"; }
  void init(const ParamTree&, Context&) override {}
  std::size_t state_size() const override { return 2; }
  void initial_state(std::span<double> x) const override {
    x[0] = 1.0;
    x[1] = 0.0;
  }
  void read_inputs(const SignalBus&) override {}
  void derivatives(double, std::span<const double> x, std::span<double> dx) const override {
    dx[0] = x[1];
    dx[1] = -omega_ * omega_ * x[0];
  }
  void write_outputs(double, std::span<const double>, SignalBus&) const override {}

 private:
  double omega_;
};

TEST(Rk4Integrator, MatchesAnalyticOscillator) {
  const double omega = 2.0 * std::numbers::pi;  // 1 Hz
  Oscillator plant(omega);
  Rk4Integrator integrator;
  std::vector<double> x(2);
  plant.initial_state(x);
  const double dt = 1e-3;
  for (int i = 0; i < 1000; ++i) {
    integrator.step(plant, i * dt, dt, x);
  }
  // After exactly one period the oscillator is back at its start.
  EXPECT_NEAR(x[0], 1.0, 1e-9);
  EXPECT_NEAR(x[1], 0.0, 1e-8);
}

TEST(Rk4Integrator, FourthOrderConvergence) {
  const double omega = 3.0;
  Oscillator plant(omega);
  const auto error_with = [&](double dt) {
    Rk4Integrator integrator;
    std::vector<double> x(2);
    plant.initial_state(x);
    const int steps = static_cast<int>(std::round(1.0 / dt));
    for (int i = 0; i < steps; ++i) {
      integrator.step(plant, i * dt, dt, x);
    }
    return std::abs(x[0] - std::cos(omega));
  };
  // Halving the step should cut the error by about 2^4.
  const double observed_order = std::log2(error_with(0.02) / error_with(0.01));
  EXPECT_NEAR(observed_order, 4.0, 0.3);
}

}  // namespace
}  // namespace vsim

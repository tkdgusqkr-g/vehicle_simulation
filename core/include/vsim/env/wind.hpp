#pragma once

#include <array>
#include <optional>
#include <string_view>
#include <vector>

#include "vsim/sim/block.hpp"
#include "vsim/sim/rng.hpp"

namespace vsim {

// Wind (design doc §10.2, PH-15): a steady wind, scripted 1-cos gusts and
// turbulence, as the air's velocity over the ground (env.wind.x, env.wind.y,
// world frame). The plant computes its aerodynamics from the car's velocity
// through that air, so a crosswind gives a side force and a yaw moment.
//
//   environment:
//     wind:
//       speed_mps: 6            # mean wind
//       from_deg: 270           # where it blows from, compass (0 north, 90 east)
//       gusts:                  # 1-cos gusts on top: speed rises and falls over duration_s
//         - {at_s: 12, duration_s: 1.5, speed_mps: 10, from_deg: 0}
//       turbulence: {sigma_mps: 1.5, length_m: 20}   # random, per horizontal component
//
// Turbulence is frozen in the air: the car crosses eddies of length_m, so its
// time scale is length_m over the car's airspeed (first-order Gauss-Markov,
// the Dryden spectrum's low-order shape). It follows the scenario seed. With
// `realism.environment: ideal` set explicitly the air is still.
class WindBlock final : public Block {
 public:
  struct Gust {
    double at = 0.0, duration = 1.0, speed = 0.0, from_deg = 0.0;
  };

  static bool configured(const ParamTree& params);

  std::string_view name() const override { return "environment.wind"; }
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override;

  // Wind over the ground at time t without turbulence [m/s], world frame (x east, y north).
  std::array<double, 2> steady(double t) const;

 private:
  double period_ = 0.001;
  bool active_ = true;
  double speed_ = 0.0, from_deg_ = 0.0;
  std::vector<Gust> gusts_;
  double sigma_ = 0.0, length_ = 20.0;
  std::array<double, 2> turbulence_{};
  std::optional<Rng> rng_;
  SignalId x_out_ = 0, y_out_ = 0, speed_out_ = 0, car_vx_ = 0, car_vy_ = 0;
};

}  // namespace vsim

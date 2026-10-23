#include "vsim/env/wind.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

#include "vsim/sim/context.hpp"

namespace vsim {
namespace {

constexpr double kPi = std::numbers::pi;

// Velocity of air blowing from a compass direction (0 north, 90 east), world frame x east, y north.
std::array<double, 2> from_compass(double speed, double from_deg) {
  const double to = (from_deg + 180.0) * kPi / 180.0;
  return {speed * std::sin(to), speed * std::cos(to)};
}

}  // namespace

bool WindBlock::configured(const ParamTree& params) { return params.has_prefix("environment.wind"); }

void WindBlock::init(const ParamTree& params, Context& ctx) {
  const std::string w = "environment.wind.";
  period_ = params.get_double("time.dt", 0.001);
  active_ = !params.has("realism.environment") || ctx.realism().is_real(RealismCategory::Environment);
  speed_ = params.get_double(w + "speed_mps", 0.0);
  from_deg_ = params.get_double(w + "from_deg", 0.0);
  gusts_.clear();
  for (std::size_t i = 0; params.has_prefix(w + "gusts." + std::to_string(i)); ++i) {
    const std::string g = w + "gusts." + std::to_string(i) + ".";
    Gust gust;
    gust.at = params.get_double(g + "at_s");
    gust.duration = params.get_double(g + "duration_s");
    gust.speed = params.get_double(g + "speed_mps");
    gust.from_deg = params.get_double(g + "from_deg", from_deg_);
    if (!(gust.duration > 0.0)) {
      throw std::invalid_argument(g + "duration_s must be positive");
    }
    gusts_.push_back(gust);
  }
  sigma_ = params.get_double(w + "turbulence.sigma_mps", 0.0);
  length_ = params.get_double(w + "turbulence.length_m", length_);
  if (speed_ < 0.0 || sigma_ < 0.0 || !(length_ > 0.0)) {
    throw std::invalid_argument("environment.wind: speed_mps and sigma_mps >= 0, length_m > 0");
  }
  rng_.emplace(ctx.make_rng("environment.wind"));
  turbulence_ = {};

  SignalBus& bus = ctx.bus();
  x_out_ = bus.declare("env.wind.x", "m/s", "wind velocity east (air over the ground)");
  y_out_ = bus.declare("env.wind.y", "m/s", "wind velocity north (air over the ground)");
  speed_out_ = bus.declare("env.wind.speed", "m/s", "wind speed over the ground");
  car_vx_ = bus.declare("veh.chassis.vx", "m/s", "longitudinal velocity (body frame)");
  car_vy_ = bus.declare("veh.chassis.vy", "m/s", "lateral velocity (body frame)");
}

std::array<double, 2> WindBlock::steady(double t) const {
  std::array<double, 2> v = from_compass(speed_, from_deg_);
  for (const Gust& g : gusts_) {
    const double tau = t - g.at;
    if (tau > 0.0 && tau < g.duration) {
      const double s = 0.5 * g.speed * (1.0 - std::cos(2.0 * kPi * tau / g.duration));
      const std::array<double, 2> add = from_compass(s, g.from_deg);
      v[0] += add[0];
      v[1] += add[1];
    }
  }
  return v;
}

void WindBlock::step(double t, SignalBus& bus) {
  std::array<double, 2> v{};
  if (active_) {
    v = steady(t);
    if (sigma_ > 0.0) {
      // The car crosses frozen eddies of length_m at its airspeed.
      const double airspeed = std::hypot(bus.get(car_vx_), bus.get(car_vy_)) + std::hypot(v[0], v[1]);
      const double a = std::exp(-period_ * std::max(airspeed, 1.0) / length_);
      for (double& component : turbulence_) {
        component = a * component + std::sqrt(1.0 - a * a) * sigma_ * rng_->normal();
      }
      v[0] += turbulence_[0];
      v[1] += turbulence_[1];
    }
  }
  bus.set(x_out_, v[0]);
  bus.set(y_out_, v[1]);
  bus.set(speed_out_, std::hypot(v[0], v[1]));
}

}  // namespace vsim

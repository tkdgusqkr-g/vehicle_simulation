#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

#include "vsim/models/tire/tire.hpp"

namespace vsim {
namespace {

double magic_formula(double slip, double b, double c, double d, double e) {
  const double x = b * slip;
  return d * std::sin(c * std::atan(x - e * (x - std::atan(x))));
}

// Slip at the peak of D sin(C atan(B x)); with E != 0 the true peak moves a
// little, which only shifts the normalization of the combined-slip blend.
double peak_slip(double b, double c) { return std::tan(std::numbers::pi / (2.0 * c)) / b; }

}  // namespace

SimpleTire::SimpleTire(const Params& params) : p_(params) {
  if (!(p_.cx > 1.0 && p_.cy > 1.0)) {
    throw std::invalid_argument("tire Cx and Cy must be greater than 1 (curves need a peak)");
  }
  if (!(p_.bx > 0.0 && p_.by > 0.0 && p_.nominal_load > 0.0)) {
    throw std::invalid_argument("tire Bx, By and nominal_load_n must be positive");
  }
  peak_slip_ratio_ = peak_slip(p_.bx, p_.cx);
  peak_slip_angle_ = peak_slip(p_.by, p_.cy);
}

SimpleTire::Params SimpleTire::params_from(const ParamTree& params, std::string_view axle) {
  const auto get = [&](const char* key, double fallback) {
    const std::string shared = std::string("vehicle.tire.simple.") + key;
    const std::string specific =
        "vehicle.tire." + std::string(axle) + ".simple." + std::string(key);
    return params.get_double(specific, params.get_double(shared, fallback));
  };
  Params p;
  p.nominal_load = get("nominal_load_n", p.nominal_load);
  p.mu_x = get("mu_x", p.mu_x);
  p.mu_y = get("mu_y", p.mu_y);
  p.load_sensitivity = get("load_sensitivity", p.load_sensitivity);
  p.bx = get("Bx", p.bx);
  p.cx = get("Cx", p.cx);
  p.ex = get("Ex", p.ex);
  p.by = get("By", p.by);
  p.cy = get("Cy", p.cy);
  p.ey = get("Ey", p.ey);
  p.pneumatic_trail = get("pneumatic_trail_m", p.pneumatic_trail);
  return p;
}

double SimpleTire::friction_scale(double fz) const {
  const double relative = (fz - p_.nominal_load) / p_.nominal_load;
  return std::max(0.3, 1.0 + p_.load_sensitivity * relative);
}

TireForces SimpleTire::evaluate(const TireInput& in) const {
  TireForces out;
  if (in.fz <= 0.0) {
    return out;
  }
  const double scale = friction_scale(in.fz);
  const double dx = p_.mu_x * scale * in.fz;
  const double dy = p_.mu_y * scale * in.fz;

  // Normalized slips; the combined slip magnitude drives both curves and the
  // direction splits the result, giving a friction ellipse.
  const double nx = in.slip_ratio / peak_slip_ratio_;
  const double ny = in.slip_angle / peak_slip_angle_;
  const double combined = std::hypot(nx, ny);
  if (combined < 1e-12) {
    out.fx = p_.bx * p_.cx * dx * in.slip_ratio;
    out.fy = p_.by * p_.cy * dy * in.slip_angle;
  } else {
    out.fx = magic_formula(combined * peak_slip_ratio_, p_.bx, p_.cx, dx, p_.ex) * nx / combined;
    out.fy = magic_formula(combined * peak_slip_angle_, p_.by, p_.cy, dy, p_.ey) * ny / combined;
  }

  // Pneumatic trail falls to zero a little beyond the peak slip angle.
  const double trail = p_.pneumatic_trail * std::max(0.0, 1.0 - std::abs(ny) / 1.2);
  out.mz = -trail * out.fy;
  return out;
}

double SimpleTire::longitudinal_stiffness(double fz) const {
  return p_.bx * p_.cx * p_.mu_x * friction_scale(fz) * fz;
}

double SimpleTire::cornering_stiffness(double fz) const {
  return p_.by * p_.cy * p_.mu_y * friction_scale(fz) * fz;
}

}  // namespace vsim

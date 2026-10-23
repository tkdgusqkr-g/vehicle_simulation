#include "vsim/models/vehicle/suspension.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>
#include <stdexcept>

namespace vsim {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr Vec3 kUp{0.0, 0.0, 1.0};

// Root of f near x0 by Newton steps with a numerical derivative.
double newton(const std::function<double(double)>& f, double x0, const char* what) {
  double x = x0;
  for (int i = 0; i < 60; ++i) {
    const double fx = f(x);
    if (std::abs(fx) < 1e-12) {
      return x;
    }
    constexpr double h = 1e-7;
    const double slope = (f(x + h) - f(x - h)) / (2.0 * h);
    if (!(std::abs(slope) > 1e-14)) {
      break;
    }
    double step = fx / slope;
    step = std::clamp(step, -0.2, 0.2);  // radians: stay on the branch
    x -= step;
    if (std::abs(step) < 1e-13) {
      return x;
    }
  }
  if (std::abs(f(x)) < 1e-9) {
    return x;
  }
  throw std::runtime_error(std::string("suspension linkage does not close (") + what + ")");
}

// Rotation taking unit vector a onto unit vector b (shortest arc).
struct Align {
  Vec3 axis{0.0, 0.0, 1.0};
  double angle = 0.0;
  Align(const Vec3& a, const Vec3& b) {
    const Vec3 c = a.cross(b);
    const double s = c.norm();
    angle = std::atan2(s, a.dot(b));
    if (s > 1e-15) {
      axis = c * (1.0 / s);
    }
  }
  Vec3 operator()(const Vec3& v) const { return rotate_about(v, {}, axis, angle); }
};

Vec3 read_point(const ParamTree& params, const std::string& key, const Vec3& fallback) {
  const std::vector<double> xyz = params.get_double_list(key);
  if (xyz.empty()) {
    return fallback;
  }
  if (xyz.size() != 3) {
    throw std::invalid_argument(key + " needs [x, y, z]");
  }
  return {xyz[0], xyz[1], xyz[2]};
}

}  // namespace

WishboneGeometry WishboneGeometry::from_params(const ParamTree& params, bool front) {
  // The example car's points (templates/projects/example_fs_ev).
  WishboneGeometry g;
  if (front) {
    g.lower_front = {0.13, 0.20, 0.11};
    g.lower_rear = {-0.17, 0.20, 0.11};
    g.lower_outer = {0.0, 0.555, 0.115};
    g.upper_front = {0.10, 0.26, 0.285};
    g.upper_rear = {-0.14, 0.26, 0.285};
    g.upper_outer = {-0.012, 0.535, 0.315};
    g.tie_inner = {0.075, 0.21, 0.165};
    g.tie_outer = {0.075, 0.565, 0.175};
    g.push_outer = {0.0, 0.52, 0.125};
    g.push_inner = {0.0, 0.25, 0.46};
    g.wheel_centre = {0.0, 0.61, 0.222};
  } else {
    g.lower_front = {0.17, 0.22, 0.11};
    g.lower_rear = {-0.13, 0.22, 0.11};
    g.lower_outer = {0.0, 0.535, 0.115};
    g.upper_front = {0.15, 0.27, 0.29};
    g.upper_rear = {-0.12, 0.27, 0.29};
    g.upper_outer = {0.012, 0.515, 0.315};
    g.tie_inner = {-0.09, 0.22, 0.17};
    g.tie_outer = {-0.08, 0.545, 0.18};
    g.push_outer = {0.0, 0.50, 0.125};
    g.push_inner = {0.0, 0.26, 0.44};
    g.wheel_centre = {0.0, 0.59, 0.222};
  }
  const std::string base = std::string("vehicle.suspension.") + (front ? "front" : "rear") + ".";
  const std::string hp = base + "hardpoints.";
  g.lower_front = read_point(params, hp + "lower_front", g.lower_front);
  g.lower_rear = read_point(params, hp + "lower_rear", g.lower_rear);
  g.lower_outer = read_point(params, hp + "lower_outer", g.lower_outer);
  g.upper_front = read_point(params, hp + "upper_front", g.upper_front);
  g.upper_rear = read_point(params, hp + "upper_rear", g.upper_rear);
  g.upper_outer = read_point(params, hp + "upper_outer", g.upper_outer);
  g.tie_inner = read_point(params, hp + "tie_inner", g.tie_inner);
  g.tie_outer = read_point(params, hp + "tie_outer", g.tie_outer);
  g.push_outer = read_point(params, hp + "push_outer", g.push_outer);
  g.push_inner = read_point(params, hp + "push_inner", g.push_inner);
  g.wheel_centre = read_point(params, hp + "wheel_centre", g.wheel_centre);
  if (params.has_prefix(hp + "rocker_pivot")) {
    g.has_rocker = true;
    g.rocker_pivot = read_point(params, hp + "rocker_pivot", {});
    g.rocker_axis = read_point(params, hp + "rocker_axis", {1.0, 0.0, 0.0}).normalized();
    if (!params.has_prefix(hp + "spring_rocker") || !params.has_prefix(hp + "spring_chassis")) {
      throw std::invalid_argument(hp + "rocker_pivot needs spring_rocker and spring_chassis too");
    }
    g.spring_rocker = read_point(params, hp + "spring_rocker", {});
    g.spring_chassis = read_point(params, hp + "spring_chassis", {});
  }
  g.motion_ratio = params.get_double(base + "motion_ratio", 1.0);
  g.static_camber = params.get_double(base + "static_camber_deg", 0.0) * kDeg;
  g.static_toe = params.get_double(base + "static_toe_deg", 0.0) * kDeg;
  g.push_on_upright = params.get_string(base + "push_rod_on", "lower_wishbone") == "upright";
  if (!(g.motion_ratio > 0.0)) {
    throw std::invalid_argument(base + "motion_ratio must be positive");
  }
  if (!(g.wheel_centre.z > 0.05)) {
    throw std::invalid_argument(hp + "wheel_centre must be above the ground (z is the height)");
  }
  return g;
}

WishboneSolver::WishboneSolver(WishboneGeometry geometry) : g_(std::move(geometry)) {
  lower_axis_ = (g_.lower_front - g_.lower_rear).normalized();
  upper_axis_ = (g_.upper_front - g_.upper_rear).normalized();
  kingpin_length_ = (g_.upper_outer - g_.lower_outer).norm();
  tie_length_ = (g_.tie_outer - g_.tie_inner).norm();
  push_length_ = (g_.push_outer - g_.push_inner).norm();
  spring_length_ = (g_.spring_rocker - g_.spring_chassis).norm();
  if (!(kingpin_length_ > 0.01) || !(tie_length_ > 0.01)) {
    throw std::invalid_argument("suspension hardpoints: ball joints or tie rod ends coincide");
  }
  const double toe = -g_.static_toe;  // left wheel: toe-in points it to the right
  const double camber = g_.static_camber;
  spin_axis0_ = {-std::sin(toe) * std::cos(camber), std::cos(toe) * std::cos(camber), -std::sin(camber)};
  tyre_radius_ = g_.wheel_centre.z;
}

CornerPose WishboneSolver::pose(double lower_angle, double rack, Angles& guess) const {
  CornerPose out;
  const Vec3 lbj = rotate_about(g_.lower_outer, g_.lower_rear, lower_axis_, lower_angle);
  const auto upper_at = [&](double a) { return rotate_about(g_.upper_outer, g_.upper_rear, upper_axis_, a); };
  guess.upper = newton([&](double a) { return (upper_at(a) - lbj).norm() - kingpin_length_; }, guess.upper,
                       "upper wishbone");
  const Vec3 ubj = upper_at(guess.upper);

  // Upright: align the kingpin, then turn about it until the tie rod fits.
  const Vec3 u0 = (g_.upper_outer - g_.lower_outer).normalized();
  const Vec3 u = (ubj - lbj).normalized();
  const Align align(u0, u);
  const auto on_upright = [&](const Vec3& p0, double psi) {
    return lbj + rotate_about(align(p0 - g_.lower_outer), {}, u, psi);
  };
  const Vec3 tie_inner = g_.tie_inner + Vec3{0.0, rack, 0.0};
  guess.upright = newton([&](double psi) { return (on_upright(g_.tie_outer, psi) - tie_inner).norm() - tie_length_; },
                         guess.upright, "tie rod");
  const double psi = guess.upright;
  const auto direction = [&](const Vec3& d0) { return rotate_about(align(d0), {}, u, psi); };

  out.lower_ball = lbj;
  out.upper_ball = ubj;
  out.tie_outer = on_upright(g_.tie_outer, psi);
  out.wheel_centre = on_upright(g_.wheel_centre, psi);
  out.spin_axis = direction(spin_axis0_).normalized();
  const Vec3 up_in_plane = (kUp - out.spin_axis * kUp.dot(out.spin_axis)).normalized();
  out.contact_patch = out.wheel_centre - up_in_plane * tyre_radius_;
  out.push_outer = g_.push_on_upright ? on_upright(g_.push_outer, psi)
                                      : rotate_about(g_.push_outer, g_.lower_rear, lower_axis_, lower_angle);
  out.steer = std::atan2(-out.spin_axis.x, out.spin_axis.y);
  out.camber = -std::asin(std::clamp(out.spin_axis.z, -1.0, 1.0));
  out.travel = out.wheel_centre.z - g_.wheel_centre.z;
  if (g_.has_rocker) {
    const auto push_inner_at = [&](double b) {
      return rotate_about(g_.push_inner, g_.rocker_pivot, g_.rocker_axis, b);
    };
    guess.rocker = newton([&](double b) { return (out.push_outer - push_inner_at(b)).norm() - push_length_; },
                          guess.rocker, "rocker");
    const Vec3 spring_end = rotate_about(g_.spring_rocker, g_.rocker_pivot, g_.rocker_axis, guess.rocker);
    out.spring = spring_length_ - (spring_end - g_.spring_chassis).norm();
  } else {
    out.spring = out.travel / g_.motion_ratio;
  }
  return out;
}

CornerPose WishboneSolver::solve(double travel, double rack) const {
  Angles guess;
  // Start the upper arm in proportion to the arm lengths.
  const auto arm = [](const Vec3& p, const Vec3& o, const Vec3& axis) {
    const Vec3 r = p - o;
    return (r - axis * axis.dot(r)).norm();
  };
  const double lower_arm = arm(g_.lower_outer, g_.lower_rear, lower_axis_);
  const double upper_arm = arm(g_.upper_outer, g_.upper_rear, upper_axis_);
  double angle = 0.0;
  Angles scratch;
  const auto rise = [&](double a) {
    scratch = guess;
    scratch.upper = a * lower_arm / upper_arm;
    return pose(a, rack, scratch).travel - travel;
  };
  angle = newton(rise, 0.0, "wheel travel");
  guess.upper = angle * lower_arm / upper_arm;
  return pose(angle, rack, guess);
}

KcTable KcTable::build(const WishboneSolver& solver, double travel_range, std::size_t travel_points, double rack_range,
                       std::size_t rack_points) {
  KcTable t;
  t.travel_range_ = travel_range;
  t.rack_range_ = rack_range;
  t.nt_ = std::max<std::size_t>(travel_points, 2);
  t.nr_ = rack_range > 0.0 ? std::max<std::size_t>(rack_points, 2) : 1;
  t.points_.resize(t.nt_ * t.nr_);
  const CornerPose design = solver.solve(0.0, 0.0);
  constexpr double h = 1e-4;
  for (std::size_t j = 0; j < t.nr_; ++j) {
    const double rack = t.nr_ == 1 ? 0.0 : -rack_range + 2.0 * rack_range * static_cast<double>(j) / (static_cast<double>(t.nr_) - 1.0);
    for (std::size_t i = 0; i < t.nt_; ++i) {
      const double travel = -travel_range + 2.0 * travel_range * static_cast<double>(i) / (static_cast<double>(t.nt_) - 1.0);
      const CornerPose p = solver.solve(travel, rack);
      KcPoint& k = t.points_[j * t.nt_ + i];
      k.steer = p.steer;
      k.camber = p.camber;
      k.cp_x = p.contact_patch.x - design.contact_patch.x;
      k.cp_y = p.contact_patch.y - design.contact_patch.y;
      k.spring = p.spring;
      const CornerPose up = solver.solve(travel + h, rack);
      const CornerPose down = solver.solve(travel - h, rack);
      k.dcp_x = (up.contact_patch.x - down.contact_patch.x) / (2.0 * h);
      k.dcp_y = (up.contact_patch.y - down.contact_patch.y) / (2.0 * h);
      k.dspring = (up.spring - down.spring) / (2.0 * h);
      if (t.nr_ > 1) {
        k.dsteer_drack = (solver.solve(travel, rack + h).steer - solver.solve(travel, rack - h).steer) / (2.0 * h);
      }
      // Steering axis moments of unit tyre forces at the contact patch.
      const Vec3 axis = (p.upper_ball - p.lower_ball).normalized();
      const Vec3 heading = Vec3{p.spin_axis.y, -p.spin_axis.x, 0.0}.normalized();
      const Vec3 lateral = kUp.cross(heading);
      const Vec3 arm = p.contact_patch - p.lower_ball;
      k.arm_x = arm.cross(heading).dot(axis);
      k.arm_y = arm.cross(lateral).dot(axis);
      k.arm_z = arm.cross(kUp).dot(axis);
      k.arm_mz = kUp.dot(axis);
    }
  }
  return t;
}

KcPoint KcTable::at(double travel, double rack) const {
  if (points_.empty()) {
    return {};
  }
  // Travel: linear inside, the end segments extended outside, so values and
  // their travel derivatives (taken from the same segments) stay consistent.
  const double dt = 2.0 * travel_range_ / (static_cast<double>(nt_) - 1.0);
  const double ut = (travel + travel_range_) / dt;
  const auto i = static_cast<std::size_t>(std::clamp(std::floor(ut), 0.0, static_cast<double>(nt_ - 2)));
  const double ft = ut - static_cast<double>(i);
  std::size_t j = 0;
  double fr = 0.0;
  if (nr_ > 1) {
    const double dr = 2.0 * rack_range_ / (static_cast<double>(nr_) - 1.0);
    const double ur = std::clamp((rack + rack_range_) / dr, 0.0, static_cast<double>(nr_ - 1));
    j = static_cast<std::size_t>(std::min(std::floor(ur), static_cast<double>(nr_ - 2)));
    fr = ur - static_cast<double>(j);
  }
  const auto row = [&](std::size_t jj) {
    const KcPoint& a = points_[jj * nt_ + i];
    const KcPoint& b = points_[jj * nt_ + i + 1];
    const auto lerp = [&](double KcPoint::*field) { return a.*field + ft * (b.*field - a.*field); };
    KcPoint k;
    k.steer = lerp(&KcPoint::steer);
    k.camber = lerp(&KcPoint::camber);
    k.cp_x = lerp(&KcPoint::cp_x);
    k.cp_y = lerp(&KcPoint::cp_y);
    k.spring = lerp(&KcPoint::spring);
    k.dcp_x = (b.cp_x - a.cp_x) / dt;
    k.dcp_y = (b.cp_y - a.cp_y) / dt;
    k.dspring = (b.spring - a.spring) / dt;
    k.dsteer_drack = lerp(&KcPoint::dsteer_drack);
    k.arm_x = lerp(&KcPoint::arm_x);
    k.arm_y = lerp(&KcPoint::arm_y);
    k.arm_z = lerp(&KcPoint::arm_z);
    k.arm_mz = lerp(&KcPoint::arm_mz);
    return k;
  };
  if (nr_ == 1) {
    return row(0);
  }
  const KcPoint a = row(j);
  const KcPoint b = row(j + 1);
  KcPoint k;
  const auto mix = [&](double KcPoint::*field) { k.*field = a.*field + fr * (b.*field - a.*field); };
  for (double KcPoint::*field : {&KcPoint::steer, &KcPoint::camber, &KcPoint::cp_x, &KcPoint::cp_y, &KcPoint::dcp_x,
                                 &KcPoint::dcp_y, &KcPoint::spring, &KcPoint::dspring, &KcPoint::dsteer_drack,
                                 &KcPoint::arm_x, &KcPoint::arm_y, &KcPoint::arm_z, &KcPoint::arm_mz}) {
    mix(field);
  }
  return k;
}

KcSummary summarize(const WishboneSolver& solver, double half_track, double wheelbase, bool front) {
  KcSummary s;
  const CornerPose p = solver.solve(0.0, 0.0);
  const Vec3 axis = (p.upper_ball - p.lower_ball).normalized();
  s.kingpin_inclination_deg = std::atan2(-axis.y, axis.z) / kDeg;
  s.caster_deg = std::atan2(-axis.x, axis.z) / kDeg;
  const Vec3 ground = p.lower_ball + axis * (-p.lower_ball.z / axis.z);
  s.scrub_radius_m = p.contact_patch.y - ground.y;
  s.mechanical_trail_m = ground.x - p.contact_patch.x;
  constexpr double h = 1e-4;
  const CornerPose up = solver.solve(h, 0.0);
  const CornerPose down = solver.solve(-h, 0.0);
  const double dcp_y = (up.contact_patch.y - down.contact_patch.y) / (2.0 * h);
  s.roll_centre_height_m = dcp_y * (half_track > 0.0 ? half_track : p.contact_patch.y);
  s.camber_gain_deg_per_m = (up.camber - down.camber) / (2.0 * h) / kDeg;
  s.bump_steer_deg_per_m = (up.steer - down.steer) / (2.0 * h) / kDeg;
  s.motion_ratio = 2.0 * h / (up.spring - down.spring);
  s.anti_geometry = (up.contact_patch.x - down.contact_patch.x) / (2.0 * h);
  if (front && wheelbase > 0.0) {
    // Left turn: the right wheel is outside. Rack where it reaches 10 deg.
    const double target = 10.0 * kDeg;
    const auto outer = [&](double rack) { return -solver.solve(0.0, -rack).steer - target; };
    try {
      double rack = 0.0;
      for (int i = 0; i < 40; ++i) {
        const double f = outer(rack);
        const double slope = (outer(rack + h) - outer(rack - h)) / (2.0 * h);
        if (std::abs(f) < 1e-10 || std::abs(slope) < 1e-12) {
          break;
        }
        rack -= f / slope;
      }
      const double inner = solver.solve(0.0, rack).steer;
      const double track = 2.0 * (half_track > 0.0 ? half_track : p.contact_patch.y);
      const double ideal = std::atan(1.0 / (1.0 / std::tan(target) - track / wheelbase));
      s.ackermann_percent = 100.0 * (inner - target) / (ideal - target);
    } catch (const std::runtime_error&) {
      s.ackermann_percent = 0.0;
    }
  }
  return s;
}

}  // namespace vsim

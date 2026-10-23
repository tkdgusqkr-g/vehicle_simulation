#include "vsim/models/driveline/driveline.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace vsim {
namespace {

// Brakes hold a wheel like static friction: below their capacity they bring
// it to rest with this time constant instead of chattering around zero speed.
constexpr double kBrakeStickTau = 0.005;  // [s]
// Speed difference over which a limited-slip diff reaches full locking torque.
constexpr double kLsdSmoothing = 0.5;  // [rad/s]
// At most a rotor and a wheel per corner.
constexpr std::size_t kMaxDofs = 2 * kCorners;
// The tooth contact damping fades in over this much twist past the free play [rad].
constexpr double kContactFade = 2e-4;

// Twist past the free play (the dead zone) and how far into contact (0..1).
double engaged(double twist, double half_play) {
  if (twist > half_play) {
    return twist - half_play;
  }
  if (twist < -half_play) {
    return twist + half_play;
  }
  return 0.0;
}

std::vector<double> invert(std::vector<double> a, std::size_t n) {
  std::vector<double> inv(n * n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    inv[i * n + i] = 1.0;
  }
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < n; ++row) {
      if (std::abs(a[row * n + col]) > std::abs(a[pivot * n + col])) {
        pivot = row;
      }
    }
    if (std::abs(a[pivot * n + col]) < 1e-12) {
      throw std::invalid_argument("driveline inertia matrix is singular (check inertias)");
    }
    for (std::size_t k = 0; k < n; ++k) {
      std::swap(a[col * n + k], a[pivot * n + k]);
      std::swap(inv[col * n + k], inv[pivot * n + k]);
    }
    const double diag = a[col * n + col];
    for (std::size_t k = 0; k < n; ++k) {
      a[col * n + k] /= diag;
      inv[col * n + k] /= diag;
    }
    for (std::size_t row = 0; row < n; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = a[row * n + col];
      for (std::size_t k = 0; k < n; ++k) {
        a[row * n + k] -= factor * a[col * n + k];
        inv[row * n + k] -= factor * inv[col * n + k];
      }
    }
  }
  return inv;
}

}  // namespace

DrivelineConfig DrivelineConfig::from_params(const ParamTree& params) {
  DrivelineConfig config;
  std::set<std::string> motors;
  std::set<std::size_t> wheels;
  for (std::size_t i = 0;; ++i) {
    const std::string key = "vehicle.driveline." + std::to_string(i);
    if (!params.has_prefix(key)) {
      break;
    }
    const std::string where = "vehicle.driveline[" + std::to_string(i) + "]";
    const ParamTree item = params.subtree(key);

    DrivelineEntry entry;
    entry.motor = item.get_string("motor");
    entry.gear_ratio = item.get_double("gear_ratio");
    entry.efficiency = item.get_double("efficiency", 1.0);
    entry.gear_inertia = item.get_double("gear_inertia_kgm2", 0.0);
    if (item.has("wheel")) {
      entry.wheels = {corner_index(item.get_string("wheel"))};
    } else {
      for (const std::string& name : item.get_string_list("wheels")) {
        entry.wheels.push_back(corner_index(name));
      }
    }

    if (!motors.insert(entry.motor).second) {
      throw std::invalid_argument(where + ": motor '" + entry.motor + "' is listed twice");
    }
    if (!(entry.gear_ratio > 0.0)) {
      throw std::invalid_argument(where + ": gear_ratio must be positive");
    }
    if (!(entry.efficiency > 0.0 && entry.efficiency <= 1.0)) {
      throw std::invalid_argument(where + ": efficiency must be in (0, 1]");
    }
    if (entry.wheels.empty() || entry.wheels.size() > 2) {
      throw std::invalid_argument(where + ": give one 'wheel' or two 'wheels' [left, right]");
    }
    for (const std::size_t wheel : entry.wheels) {
      if (!wheels.insert(wheel).second) {
        throw std::invalid_argument(where + ": wheel '" + std::string(kCornerNames[wheel]) +
                                    "' is already driven by another entry");
      }
    }
    if (entry.wheels.size() == 2) {
      const std::string type = item.get_string("diff.type", "open");
      if (type == "open") {
        entry.diff = DiffType::Open;
      } else if (type == "lsd") {
        entry.diff = DiffType::Lsd;
        entry.lsd_preload = item.get_double("diff.preload_nm", 0.0);
        entry.lsd_ramp = item.get_double("diff.ramp", 0.0);
      } else if (type == "locked") {
        entry.diff = DiffType::Locked;
      } else {
        throw std::invalid_argument(where + ": diff.type '" + type +
                                    "' (expected open, lsd or locked)");
      }
    } else if (item.has_prefix("diff")) {
      throw std::invalid_argument(where + ": a differential needs two wheels");
    }
    if (item.has_prefix("shaft")) {
      if (entry.wheels.size() != 1) {
        throw std::invalid_argument(where + ": shaft compliance is for one-wheel entries");
      }
      entry.shaft_stiffness = item.get_double("shaft.stiffness_nm_per_rad");
      entry.shaft_damping = item.get_double("shaft.damping_nms_per_rad", 0.0);
      entry.backlash = item.get_double("shaft.backlash_deg", 0.0) * 3.14159265358979323846 / 180.0;
      if (!(entry.shaft_stiffness > 0.0) || entry.shaft_damping < 0.0 || entry.backlash < 0.0) {
        throw std::invalid_argument(where + ": shaft needs stiffness_nm_per_rad > 0, damping and backlash >= 0");
      }
    }
    config.entries.push_back(std::move(entry));
  }
  if (config.entries.empty()) {
    throw std::invalid_argument("vehicle.driveline must list at least one motor");
  }
  return config;
}

std::vector<std::string> DrivelineConfig::motor_ids() const {
  std::vector<std::string> ids;
  for (const DrivelineEntry& entry : entries) {
    ids.push_back(entry.motor);
  }
  return ids;
}

void Driveline::init(const DrivelineConfig& config, double wheel_inertia, double rotor_inertia) {
  dofs_ = 0;
  motors_.clear();
  lsds_.clear();
  shafts_.clear();
  std::array<bool, kCorners> assigned{};

  for (std::size_t m = 0; m < config.entries.size(); ++m) {
    const DrivelineEntry& entry = config.entries[m];
    Motor motor{entry.gear_ratio, entry.efficiency, {}};
    const double g = entry.gear_ratio;
    if (entry.wheels.size() == 1 && entry.shaft_stiffness > 0.0) {
      // Rotor and gear on their own coordinate (wheel speed units), the wheel on another.
      const std::size_t rotor = dofs_++;
      const std::size_t wheel = dofs_++;
      wheel_dof_[entry.wheels[0]] = wheel;
      motor.dofs = {{rotor, g}};
      shafts_.push_back({rotor, wheel, entry.wheels[0], entry.shaft_stiffness, entry.shaft_damping,
                         0.5 * entry.backlash});
    } else if (entry.wheels.size() == 1 || entry.diff == DiffType::Locked) {
      const std::size_t dof = dofs_++;
      for (const std::size_t wheel : entry.wheels) {
        wheel_dof_[wheel] = dof;
      }
      motor.dofs = {{dof, g}};
    } else {
      const std::size_t left = dofs_++;
      const std::size_t right = dofs_++;
      wheel_dof_[entry.wheels[0]] = left;
      wheel_dof_[entry.wheels[1]] = right;
      motor.dofs = {{left, 0.5 * g}, {right, 0.5 * g}};
      if (entry.diff == DiffType::Lsd) {
        lsds_.push_back({m, left, right, entry.lsd_preload, entry.lsd_ramp});
      }
    }
    for (const std::size_t wheel : entry.wheels) {
      assigned[wheel] = true;
    }
    motors_.push_back(std::move(motor));
  }
  for (std::size_t wheel = 0; wheel < kCorners; ++wheel) {
    if (!assigned[wheel]) {
      wheel_dof_[wheel] = dofs_++;
    }
  }

  const std::size_t n = dofs_;
  mass_.assign(n * n, 0.0);
  for (std::size_t wheel = 0; wheel < kCorners; ++wheel) {
    const std::size_t dof = wheel_dof_[wheel];
    mass_[dof * n + dof] += wheel_inertia;
  }
  for (std::size_t m = 0; m < motors_.size(); ++m) {
    const double inertia = rotor_inertia + config.entries[m].gear_inertia;
    for (const auto& [a, fa] : motors_[m].dofs) {
      for (const auto& [b, fb] : motors_[m].dofs) {
        mass_[a * n + b] += inertia * fa * fb;
      }
    }
  }
  inv_mass_ = invert(mass_, n);
}

double Driveline::wheel_speed(std::size_t corner, std::span<const double> q) const {
  return q[wheel_dof_[corner]];
}

double Driveline::motor_speed(std::size_t motor, std::span<const double> q) const {
  double speed = 0.0;
  for (const auto& [dof, factor] : motors_[motor].dofs) {
    speed += factor * q[dof];
  }
  return speed;
}

double Driveline::shaft_torque(std::size_t shaft, std::span<const double> q, std::span<const double> twist) const {
  const Shaft& s = shafts_[shaft];
  const double e = engaged(twist[shaft], s.half_play);
  const double contact = std::min(1.0, std::abs(e) / kContactFade);
  return s.stiffness * e + contact * s.damping * (q[s.motor_dof] - q[s.wheel_dof]);
}

double Driveline::spring_energy(std::span<const double> twist) const {
  double energy = 0.0;
  for (std::size_t i = 0; i < shafts_.size() && i < twist.size(); ++i) {
    const double e = engaged(twist[i], shafts_[i].half_play);
    energy += 0.5 * shafts_[i].stiffness * e * e;
  }
  return energy;
}

void Driveline::accelerations(std::span<const double> q, std::span<const double> twist, const Loads& loads,
                              std::span<double> qdot, std::span<double> twist_rate, Losses& losses) const {
  const std::size_t n = dofs_;
  std::array<double, kMaxDofs> torque{};
  for (std::size_t wheel = 0; wheel < kCorners; ++wheel) {
    torque[wheel_dof_[wheel]] += loads.wheel_torque[wheel];
  }

  // Motors through their gears; efficiency loses power in both directions.
  for (std::size_t m = 0; m < motors_.size(); ++m) {
    const Motor& motor = motors_[m];
    const double shaft = loads.motor_torque[m];
    const double speed = motor_speed(m, q);
    const double delivered = shaft * speed >= 0.0 ? shaft * motor.efficiency
                                                   : shaft / motor.efficiency;
    losses.gear += (shaft - delivered) * speed;
    for (const auto& [dof, factor] : motor.dofs) {
      torque[dof] += factor * delivered;
    }
  }

  // Limited-slip friction between the two sides of a differential.
  for (const Lsd& lsd : lsds_) {
    const double slip = q[lsd.left] - q[lsd.right];
    const double input = std::abs(loads.motor_torque[lsd.motor]) * motors_[lsd.motor].gear_ratio;
    const double locking = (lsd.preload + lsd.ramp * input) * std::tanh(slip / kLsdSmoothing);
    torque[lsd.left] -= locking;
    torque[lsd.right] += locking;
    losses.diff += locking * slip;
  }

  // Compliant shafts: the spring through the free play and the tooth damping.
  for (std::size_t i = 0; i < shafts_.size(); ++i) {
    const Shaft& s = shafts_[i];
    const double t = shaft_torque(i, q, twist);
    torque[s.motor_dof] -= t;
    torque[s.wheel_dof] += t;
    const double e = engaged(twist[i], s.half_play);
    const double contact = std::min(1.0, std::abs(e) / kContactFade);
    const double slip = q[s.motor_dof] - q[s.wheel_dof];
    losses.gear += contact * s.damping * slip * slip;
    if (!twist_rate.empty()) {
      twist_rate[i] = slip;
    }
  }

  // Brakes: kinetic friction while slipping, static friction below capacity.
  std::array<double, kMaxDofs> capacity{};
  for (std::size_t wheel = 0; wheel < kCorners; ++wheel) {
    capacity[wheel_dof_[wheel]] += loads.brake_capacity[wheel];
  }
  for (std::size_t dof = 0; dof < n; ++dof) {
    if (capacity[dof] <= 0.0) {
      continue;
    }
    const double hold = -mass_[dof * n + dof] * q[dof] / kBrakeStickTau - torque[dof];
    const double brake = std::clamp(hold, -capacity[dof], capacity[dof]);
    torque[dof] += brake;
    losses.brake -= brake * q[dof];
    for (std::size_t wheel = 0; wheel < kCorners; ++wheel) {  // shared in proportion to capacity
      if (wheel_dof_[wheel] == dof) {
        losses.brake_wheel[wheel] -= brake * q[dof] * loads.brake_capacity[wheel] / capacity[dof];
      }
    }
  }

  for (std::size_t row = 0; row < n; ++row) {
    double sum = 0.0;
    for (std::size_t col = 0; col < n; ++col) {
      sum += inv_mass_[row * n + col] * torque[col];
    }
    qdot[row] = sum;
  }
}

double Driveline::kinetic_energy(std::span<const double> q) const {
  double energy = 0.0;
  for (std::size_t row = 0; row < dofs_; ++row) {
    for (std::size_t col = 0; col < dofs_; ++col) {
      energy += q[row] * mass_[row * dofs_ + col] * q[col];
    }
  }
  return 0.5 * energy;
}

}  // namespace vsim

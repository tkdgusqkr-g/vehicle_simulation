#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include "vsim/models/vehicle/corners.hpp"
#include "vsim/sim/params.hpp"

namespace vsim {

enum class DiffType { Open, Lsd, Locked };

// One motor and what it drives (design doc §7.1).
struct DrivelineEntry {
  std::string motor;  // motor id; signals are pt.motor.<id>.*
  double gear_ratio = 1.0;
  double efficiency = 1.0;
  double gear_inertia = 0.0;  // [kg m^2] at the motor shaft
  // One wheel: rigid connection (e.g. in-wheel motor with planetary gear).
  // Two wheels [left, right]: through a differential.
  std::vector<std::size_t> wheels;
  DiffType diff = DiffType::Open;
  double lsd_preload = 0.0;  // [N m]
  double lsd_ramp = 0.0;     // locking torque per unit of input torque
  // Torsional compliance between the gear and the wheel (one-wheel entries,
  // design doc §7.1, PH-04), at the wheel: stiffness 0 = rigid.
  double shaft_stiffness = 0.0;  // [N m / rad]
  double shaft_damping = 0.0;    // [N m s / rad], while the teeth are in contact
  double backlash = 0.0;         // [rad] total free play
};

// Topology from `vehicle.driveline`, e.g.
//   - {motor: fl, wheel: fl, gear_ratio: 14.0, efficiency: 0.97}
//   - {motor: rear, wheels: [rl, rr], gear_ratio: 4.2, efficiency: 0.96,
//      diff: {type: lsd, preload_nm: 20, ramp: 0.3}}
//   - {motor: fl, wheel: fl, gear_ratio: 14, shaft: {stiffness_nm_per_rad: 6000, damping_nms_per_rad: 4,
//      backlash_deg: 0.6}}   # compliant, with free play (at the wheel)
// Wheels that no entry lists roll freely.
struct DrivelineConfig {
  std::vector<DrivelineEntry> entries;

  static DrivelineConfig from_params(const ParamTree& params);
  std::vector<std::string> motor_ids() const;
};

// Rotational dynamics of wheels, gears, differentials and motor rotors in
// reduced coordinates: independent speeds q, wheel speeds = Aw q and motor
// speeds = Am q. Rigid connections share one coordinate; an open or limited-
// slip differential gives each wheel its own coordinate with the motor at
// their mean; a locked differential ties both wheels to one coordinate. A
// compliant shaft gives the rotor (and gear) its own coordinate, in wheel
// speed units, joined to the wheel by a spring-damper with a dead zone; its
// twist is a state of its own.
class Driveline {
 public:
  struct Loads {
    // Torque on each wheel from the road and rolling resistance [N m],
    // positive in the direction of forward rotation.
    std::array<double, kCorners> wheel_torque{};
    // Brake torque each wheel can resist [N m] (>= 0).
    std::array<double, kCorners> brake_capacity{};
    // Motor shaft torque per motor, in entry order [N m].
    std::span<const double> motor_torque;
  };

  struct Losses {
    double gear = 0.0;   // [W]
    double brake = 0.0;  // [W]
    double diff = 0.0;   // [W], limited-slip friction
    std::array<double, kCorners> brake_wheel{};  // [W] brake power at each wheel
  };

  void init(const DrivelineConfig& config, double wheel_inertia, double rotor_inertia);

  std::size_t dof_count() const { return dofs_; }
  std::size_t motor_count() const { return motors_.size(); }
  std::size_t shaft_count() const { return shafts_.size(); }

  double wheel_speed(std::size_t corner, std::span<const double> q) const;
  double motor_speed(std::size_t motor, std::span<const double> q) const;

  // dq/dt for the given loads and shaft twists; also reports the power dissipated and the
  // twist rates (motor side minus wheel side, at the wheel).
  void accelerations(std::span<const double> q, std::span<const double> twist, const Loads& loads,
                     std::span<double> qdot, std::span<double> twist_rate, Losses& losses) const;
  // Rigid drivelines (no shafts).
  void accelerations(std::span<const double> q, const Loads& loads, std::span<double> qdot,
                     Losses& losses) const {
    accelerations(q, {}, loads, qdot, {}, losses);
  }

  // Torque each shaft carries to its wheel [N m] (spring through the free play, damping).
  double shaft_torque(std::size_t shaft, std::span<const double> q, std::span<const double> twist) const;
  std::size_t shaft_wheel(std::size_t shaft) const { return shafts_[shaft].wheel; }

  // Rotational kinetic energy of all wheels and rotors [J].
  double kinetic_energy(std::span<const double> q) const;
  // Energy in the shaft springs [J].
  double spring_energy(std::span<const double> twist) const;

  // Mass matrix entry (for tests).
  double mass(std::size_t row, std::size_t col) const { return mass_[row * dofs_ + col]; }

 private:
  struct Motor {
    double gear_ratio = 1.0;
    double efficiency = 1.0;
    std::vector<std::pair<std::size_t, double>> dofs;  // (dof, motor speed per dof speed)
  };
  struct Lsd {
    std::size_t motor = 0;
    std::size_t left = 0, right = 0;  // dofs
    double preload = 0.0;
    double ramp = 0.0;
  };
  struct Shaft {
    std::size_t motor_dof = 0, wheel_dof = 0, wheel = 0;
    double stiffness = 0.0, damping = 0.0, half_play = 0.0;
  };

  std::size_t dofs_ = 0;
  std::array<std::size_t, kCorners> wheel_dof_{};
  std::vector<Motor> motors_;
  std::vector<Lsd> lsds_;
  std::vector<Shaft> shafts_;
  std::vector<double> mass_;      // dofs x dofs
  std::vector<double> inv_mass_;  // dofs x dofs
};

}  // namespace vsim

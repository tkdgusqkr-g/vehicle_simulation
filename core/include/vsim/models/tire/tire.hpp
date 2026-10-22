#pragma once

#include <filesystem>
#include <memory>
#include <string_view>

#include "vsim/sim/params.hpp"

namespace vsim {

// Conventions used everywhere in vsim (wheel frame: x forward, y left, z up):
//   slip_ratio  kappa = (omega * R - v_x) / |v_x|; positive when driving.
//   slip_angle  alpha = -atan(v_y / |v_x|) of the contact patch; positive when
//               the wheel points left of its direction of travel, which
//               produces a positive (leftward) lateral force.
//   mz          aligning torque about z; negative for positive alpha (it turns
//               the wheel back towards its direction of travel).
struct TireInput {
  double fz = 0.0;          // normal load [N]
  double slip_ratio = 0.0;  // [-]
  double slip_angle = 0.0;  // [rad]
};

struct TireForces {
  double fx = 0.0;  // [N]
  double fy = 0.0;  // [N]
  double mz = 0.0;  // [N m]
};

class TireModel {
 public:
  virtual ~TireModel() = default;
  virtual TireForces evaluate(const TireInput& input) const = 0;
  // dFx/dkappa and dFy/dalpha at zero slip [N per unit, N/rad].
  virtual double longitudinal_stiffness(double fz) const = 0;
  virtual double cornering_stiffness(double fz) const = 0;
};

// Magic Formula curves per direction with a shared normalized-slip
// combination: at combined slip the force follows a friction ellipse, and
// pure slip reproduces each curve exactly. The aligning torque comes from a
// pneumatic trail that shrinks towards the peak slip angle, so steering goes
// light near the grip limit as on a real car.
class SimpleTire final : public TireModel {
 public:
  struct Params {
    double nominal_load = 700.0;     // [N]
    double mu_x = 1.6;
    double mu_y = 1.6;
    double load_sensitivity = 0.0;   // relative change of mu per +100 % load
    double bx = 12.0, cx = 1.65, ex = 0.3;
    double by = 12.0, cy = 1.4, ey = 0.2;
    double pneumatic_trail = 0.025;  // [m] at zero slip angle
  };

  explicit SimpleTire(const Params& params);
  // Reads vehicle.tire.<axle>.simple.<key>, falling back to vehicle.tire.simple.<key>.
  static Params params_from(const ParamTree& params, std::string_view axle);

  TireForces evaluate(const TireInput& input) const override;
  double longitudinal_stiffness(double fz) const override;
  double cornering_stiffness(double fz) const override;

  const Params& params() const { return p_; }

 private:
  double friction_scale(double fz) const;

  Params p_;
  double peak_slip_ratio_ = 0.0;
  double peak_slip_angle_ = 0.0;
};

// Tire for one axle ("front" or "rear") from vehicle.tire.model: simple | mf.
// File paths are relative to the vehicle project directory.
std::unique_ptr<TireModel> make_tire(const ParamTree& params, std::string_view axle,
                                     const std::filesystem::path& project_dir);

}  // namespace vsim

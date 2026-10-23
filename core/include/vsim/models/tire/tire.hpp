#pragma once

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "vsim/sim/params.hpp"

namespace vsim {

// Conventions used everywhere in vsim (wheel frame: x forward, y left, z up):
//   slip_ratio  kappa = (omega * R - v_x) / |v_x|; positive when driving.
//   slip_angle  alpha = -atan(v_y / |v_x|) of the contact patch; positive when
//               the wheel points left of its direction of travel, which
//               produces a positive (leftward) lateral force.
//   mz          aligning torque about z; negative for positive alpha (it turns
//               the wheel back towards its direction of travel).
//   camber      inclination angle as in ISO-W: rotation about the wheel's x
//               axis, positive when the top of the wheel leans right (-y).
//               A wheel pushes towards the side it leans to (camber thrust).
struct TireInput {
  double fz = 0.0;          // normal load [N]
  double slip_ratio = 0.0;  // [-]
  double slip_angle = 0.0;  // [rad]
  double camber = 0.0;      // [rad]
  double mu_scale = 1.0;    // road friction relative to the tire data (wet, grass, kerb)
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
// light near the grip limit as on a real car. Camber acts as an equivalent
// slip angle (thrust within the friction limit); a lower road friction keeps
// the stiffness and lowers the peak, which then comes at a smaller slip.
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
    double camber_stiffness = 0.8;   // lateral force per Fz per rad of camber at zero slip
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

// Pacejka Magic Formula from a TNO/ADAMS .tir file (MF 5.2 / 6.1 subset):
// pure and combined slip for Fx, Fy and Mz with load and camber dependency
// (FITTYP 61/62 use the 6.1 camber terms, others the 5.2 ones) and the file's
// scaling factors; inflation pressure is taken at nominal and turn slip is
// left out (design doc §6.4). The file's ISO-W convention is converted to the
// vsim convention above (alpha_iso = -alpha); files whose cornering stiffness
// has the opposite sign are detected and flipped, with a note.
class MagicFormulaTire final : public TireModel {
 public:
  static MagicFormulaTire from_file(const std::filesystem::path& file);
  // Coefficients by name (upper case), e.g. {"PDX1", 1.6}. Missing ones are 0
  // except the scaling factors (L*), which default to 1.
  explicit MagicFormulaTire(std::map<std::string, double, std::less<>> coefficients);

  TireForces evaluate(const TireInput& input) const override;
  double longitudinal_stiffness(double fz) const override;
  double cornering_stiffness(double fz) const override;

  double coefficient(std::string_view name) const;
  double nominal_load() const { return fz0_; }
  double unloaded_radius() const { return r0_; }
  const std::vector<std::string>& notes() const { return notes_; }

 private:
  double c(std::string_view name) const;

  std::map<std::string, double, std::less<>> k_;
  std::vector<double> v_;  // the coefficients the formulas use, by index
  double fz0_ = 0.0;
  double r0_ = 0.0;
  double alpha_sign_ = -1.0;  // alpha_iso = alpha_sign * alpha
  bool mf61_ = true;          // camber terms of MF 6.1 (else 5.2)
  std::vector<std::string> notes_;
};

// Reads "KEY = value $ comment" lines of a .tir file; section headers,
// comments and string values (except FITTYP-like numbers) are skipped.
std::map<std::string, double, std::less<>> read_tir_file(const std::filesystem::path& file);

// Tire for one axle ("front" or "rear") from vehicle.tire.model: simple | mf.
// For mf, vehicle.tire.[<axle>.]file names the .tir file, relative to the
// vehicle project directory.
std::unique_ptr<TireModel> make_tire(const ParamTree& params, std::string_view axle,
                                     const std::filesystem::path& project_dir);

}  // namespace vsim

#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "vsim/models/driveline/driveline.hpp"
#include "vsim/models/powertrain/battery.hpp"
#include "vsim/models/powertrain/motor.hpp"
#include "vsim/models/tire/tire.hpp"
#include "vsim/models/vehicle/aero.hpp"
#include "vsim/models/vehicle/corners.hpp"
#include "vsim/models/vehicle/suspension.hpp"
#include "vsim/models/vehicle/thermal.hpp"
#include "vsim/sim/plant.hpp"

namespace vsim {

class Track;

// Two-track vehicle on four tires with transient slip, wheel spin through the
// driveline graph, motors (Level A) and the accumulator equivalent circuit.
// Every power flow is integrated so the energy balance can be checked (V-03).
//
// vehicle.model: two_track (L2, design doc §6.3): planar chassis (x, y, yaw
//   and their rates), quasi-static load transfer through roll stiffness and
//   roll centres, flat road.
// vehicle.model: full (L3, §6.4): 14 DOF. The sprung body also heaves, rolls
//   and pitches, each corner's unsprung mass moves vertically on a
//   spring-damper tire over the track's 3D surface (elevation, bank, kerbs,
//   roughness, friction). Springs, dampers, bump and droop stops and anti-roll
//   bars act along the wheel travel; steering, camber, scrub, jacking forces,
//   motion ratio and the steering-axis moments come from K&C tables solved
//   from the hardpoints. Equations from Kane's method, so the energy balance
//   holds here too.
//
// Inputs:  drv.cmd.steer, drv.cmd.brake, pt.motor.<id>.torque_cmd,
//          (full) rig.<corner>.z / rig.<corner>.vz: road displacement and its
//          rate added under each wheel (four-post rig)
// Outputs: veh.*, pt.*, bat.*, energy.*
class TwoTrackModel final : public PlantModel {
 public:
  // Chassis states; the rest of the state vector is laid out at init().
  enum Chassis : std::size_t { kX, kY, kYaw, kVx, kVy, kYawRate, kAxFiltered, kAyFiltered, kChassisSize };
  // Body states of the full model: sprung CG height, roll (left side up +),
  // pitch (nose down +) and their rates.
  enum Body : std::size_t { kZ, kRoll, kPitch, kVz, kRollRate, kPitchRate, kBodySize };

  // Integrated energies [J], in state order.
  enum Energy : std::size_t {
    kBatteryChemical,  // OCV * I: released by the cells
    kBatteryLoss,      // ohmic and RC resistor losses
    kAux,              // constant auxiliary load (with the electrical network: LV loads and wiring)
    kMotorLoss,        // motor + inverter losses
    kMotorShaft,       // mechanical work at the motor shafts
    kGearLoss,
    kDiffLoss,
    kBrakeLoss,
    kSlipLoss,         // tire slip, including the transient carcass energy
    kRollingLoss,
    kDragLoss,         // drag, and downforce on a moving body (full)
    kDamperLoss,       // suspension dampers (full)
    kTireDampingLoss,  // tire carcass vertical damping (full)
    kEnergyCount
  };

  struct Params {
    double mass = 280.0;
    double yaw_inertia = 110.0;
    double wheelbase = 1.53;
    double cg_to_front = 0.80;
    double cg_height = 0.28;
    double track_front = 1.22;
    double track_rear = 1.18;
    double roll_center_front = 0.03;   // [m] above ground
    double roll_center_rear = 0.05;
    double roll_stiffness_front = 0.5;  // front share of the total roll stiffness
    double load_transfer_tau = 0.05;    // [s] lag on ax, ay for load transfer
    double steering_ratio = 4.5;
    double max_steer_wheel = 1.92;      // [rad] at the steering wheel
    double ackermann = 0.0;             // 0 parallel, 1 full Ackermann
    double mechanical_trail = 0.02;     // [m] caster trail
    double wheel_radius = 0.203;
    double wheel_inertia = 0.3;         // [kg m^2] wheel, tire, hub
    double rolling_resistance = 0.015;
    // Effective rolling radius (wheel_radius at the optimum pressure and static load): grows with
    // the inflation pressure, shrinks with a share of the extra tire deflection (full model).
    double radius_per_kpa = 0.0;        // [m/kPa]
    double radius_deflection_share = 0.0;
    double relaxation_x = 0.10;         // [m]
    double relaxation_y = 0.25;         // [m]
    double low_speed_damping = 0.5;     // damping ratio of the tire modes near standstill
    double low_speed_fade = 3.0;        // [m/s] damping fades out by this speed
    double cla = 3.2;
    double cda = 1.3;
    double aero_front_ratio = 0.45;
    double air_density = 1.2;
    double side_force_area = 1.2;       // [m^2 per rad] side force coefficient x area in yaw (crosswind)
    double side_force_x = -0.25;        // [m] where the side force acts, ahead of the CG +
    double brake_pressure_max = 60.0;   // [bar]
    double brake_gain_front = 30.0;     // [N m / bar] axle total
    double brake_gain_rear = 18.0;
    double ambient_temp = 25.0;         // [degC]

    // Full model.
    std::array<double, kCorners> unsprung_mass{12.0, 12.0, 12.0, 12.0};
    double roll_inertia = 25.0;         // sprung body about its CG [kg m^2]
    double pitch_inertia = 85.0;
    double unloaded_radius = 0.0;       // 0: rolling radius + static deflection
    double tire_vertical_stiffness = 100e3;  // [N/m]
    double tire_vertical_damping = 150.0;    // [N s/m]
    double contact_length = 0.1;        // [m] road averaged over the patch
    struct Axle {
      double spring_rate = 35e3;        // [N/m] at the spring
      double damper_bump = 1800.0;      // [N s/m] at the wheel
      double damper_rebound = 1800.0;
      double damper_knee = 0.1;         // [m/s] above it the slope drops
      double damper_high_speed = 0.5;   // high-speed slope / low-speed slope
      double anti_roll_bar = 120.0;     // [N m/deg] of body roll
      double bump_travel = 0.03;        // [m] wheel travel to the bump stop
      double droop_travel = 0.04;       // [m] to the droop limit
      double bump_stop_rate = 200e3;    // [N/m] at first contact, progressive
    };
    std::array<Axle, 2> axle;

    double cg_to_rear() const { return wheelbase - cg_to_front; }
    double track(std::size_t corner) const { return is_front(corner) ? track_front : track_rear; }
  };

  TwoTrackModel();
  ~TwoTrackModel() override;

  std::string_view name() const override { return full_ ? "full" : "two_track"; }
  bool full() const { return full_; }
  void init(const ParamTree& params, Context& ctx) override;
  std::size_t state_size() const override { return size_; }
  void initial_state(std::span<double> x) const override;
  void read_inputs(const SignalBus& bus) override;
  void derivatives(double t, std::span<const double> x, std::span<double> dx) const override;
  void write_outputs(double t, std::span<const double> x, SignalBus& bus) const override;

  static Params params_from(const ParamTree& params);
  const Params& params() const { return p_; }
  const MotorParams& motor_params() const { return motor_; }
  const BatteryParams& battery_params() const { return battery_; }
  const CellSpread& cell_spread() const { return spread_; }
  const Driveline& driveline() const { return driveline_; }
  const TireThermalParams& tire_thermal() const { return tire_thermal_; }
  const BrakeThermalParams& brake_thermal() const { return brake_thermal_; }
  const AeroMap& aero_map() const { return aero_map_; }

  // State offsets, for tests and tools.
  std::size_t wheel_dof_offset() const { return wheel_offset_; }
  std::size_t energy_offset() const { return energy_offset_; }
  std::size_t battery_offset() const { return battery_offset_; }
  std::size_t body_offset() const { return body_offset_; }          // full model
  std::size_t unsprung_offset() const { return unsprung_offset_; }  // zu[4], then vzu[4]
  // Per corner: tire surface and core temperature, brake disc temperature [degC].
  std::size_t thermal_offset() const { return thermal_offset_; }
  // Rotor angles (one per motor), then shaft twists (one per compliant shaft).
  std::size_t mech_offset() const { return mech_offset_; }
  const KcTable& kc(std::size_t axle) const { return kc_[axle]; }
  // Static figures of the suspension (full model).
  double sprung_mass() const { return sprung_mass_; }
  double sprung_cg_height() const { return h_s_; }
  double static_hub_height(std::size_t corner) const { return hub0_[corner]; }
  double rack_per_steering_rad() const { return rack_per_rad_; }

 private:
  struct Eval;
  struct Signals;

  // Forces and flows for state x; also fills dx unless it is empty.
  void evaluate(std::span<const double> x, Eval& e, std::span<double> dx) const;
  void init_full(const ParamTree& params, Context& ctx);
  double kinetic_energy(std::span<const double> x) const;
  double stored_energy(std::span<const double> x) const;  // potential energy (full)

  Params p_;
  MotorParams motor_;
  BatteryParams battery_;
  CellSpread spread_;
  DrivelineConfig driveline_config_;
  Driveline driveline_;
  std::array<std::unique_ptr<TireModel>, 2> tires_;  // front, rear

  // State layout.
  std::size_t wheel_offset_ = 0;    // driveline coordinates
  std::size_t slip_offset_ = 0;     // per corner: kappa', tan(alpha')
  std::size_t motor_offset_ = 0;    // per motor: torque, temperature
  std::size_t battery_offset_ = 0;  // soc, rc voltages (2), temperature
  std::size_t energy_offset_ = 0;
  std::size_t body_offset_ = 0;
  std::size_t unsprung_offset_ = 0;
  std::size_t thermal_offset_ = 0;
  std::size_t mech_offset_ = 0;     // per motor: rotor angle; per shaft: twist
  double ripple_max_frequency_ = 125.0;  // [Hz] the plant step carries
  std::size_t size_ = 0;
  TireThermalParams tire_thermal_;
  BrakeThermalParams brake_thermal_;
  AeroMap aero_map_;
  double road_temp_ = 25.0;

  // Full model.
  bool full_ = false;
  std::shared_ptr<const Track> track_;
  std::size_t patch_samples_ = 1;
  std::array<KcTable, 2> kc_;
  double rack_per_rad_ = 0.0;             // rack travel per steering wheel radian
  double sprung_mass_ = 0.0, x_s_ = 0.0, h_s_ = 0.0;
  double izz_s_ = 0.0;
  std::array<double, kCorners> free_radius_{};  // unloaded tire radius
  std::array<double, kCorners> hub0_{};   // static hub height above the road
  std::array<double, kCorners> offset0_{};  // static hub height minus body corner height
  std::array<double, kCorners> preload_{};  // spring compression at design [m]
  std::array<double, 25> mass_inverse_{};   // [vx, vy, r, p, q]
  double z0_ = 0.0, roll0_ = 0.0, pitch0_ = 0.0;
  std::array<double, kCorners> zu0_{};
  double stored0_ = 0.0;
  mutable std::array<std::array<double, 3>, kCorners> road_hint_{};
  std::array<double, kCorners> rig_z_{}, rig_vz_{};

  // Initial conditions.
  double x0_ = 0.0, y0_ = 0.0, yaw0_ = 0.0, v0_ = 0.0;
  double soc0_ = 0.9, battery_temp0_ = 25.0, motor_temp0_ = 35.0, tire_temp0_ = 25.0, brake_temp0_ = 25.0;
  double kinetic0_ = 0.0;

  // Inputs, latched once per step.
  double steer_ = 0.0;
  double brake_ = 0.0;
  double wind_x_ = 0.0, wind_y_ = 0.0;  // air over the ground, world frame
  std::vector<double> torque_cmd_;
  // Electrical network (design doc §12.3): battery current and inverter DC
  // voltages come from it instead of the direct battery solve.
  bool network_ = false;
  double network_current_ = 0.0;
  std::vector<double> dc_voltage_, inverter_on_;

  std::unique_ptr<Signals> sig_;
};

}  // namespace vsim

#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include "vsim/sim/plant.hpp"

namespace vsim {

// L1 vehicle model (design doc §6.2): dynamic single-track (bicycle) with
// Magic Formula lateral tires, friction-ellipse coupling, aero, quasi-static
// load transfer and a force-based longitudinal axis (no wheel-spin states;
// those arrive with the L2 model). Below ~1-3 m/s it blends into kinematic
// motion, where slip angles are not defined.
//
// It reads the same vehicle files as the L2 model and uses the lateral curve
// of the simple tire parameters; each driveline entry acts on its axle.
//
// Inputs:  drv.cmd.steer, drv.cmd.brake, pt.motor.<id>.torque_cmd
// Outputs: veh.*, pt.motor.<id>.{torque,speed}, pt.power_mech
class SingleTrackModel final : public PlantModel {
 public:
  enum State : std::size_t { kX, kY, kYaw, kVx, kVy, kYawRate, kAxFiltered, kStateSize };
  static constexpr std::size_t kMaxMotors = 4;

  struct Tire {
    double mu = 1.6;
    double B = 12.0;
    double C = 1.4;
    double E = 0.2;
  };

  struct Params {
    double mass = 280.0;              // [kg], including driver
    double yaw_inertia = 110.0;       // [kg m^2]
    double wheelbase = 1.53;          // [m]
    double cg_to_front = 0.80;        // [m]
    double cg_height = 0.28;          // [m]
    double steering_ratio = 4.5;      // steering wheel angle / road wheel angle
    double max_steer_wheel = 1.92;    // [rad] at the steering wheel
    double tire_radius = 0.203;       // [m]
    Tire tire_front;
    Tire tire_rear;
    double rolling_resistance = 0.015;
    double cla = 3.2;                 // [m^2]
    double cda = 1.3;                 // [m^2]
    double aero_front_ratio = 0.45;   // share of downforce on the front axle
    double air_density = 1.2;         // [kg/m^3]
    double motor_torque_max = 21.0;   // [N m] per motor
    double motor_power_max = 35e3;    // [W] per motor
    double brake_pressure_max = 60.0;     // [bar] at full pedal
    double brake_gain_front = 30.0;       // [N m / bar], front axle total
    double brake_gain_rear = 18.0;        // [N m / bar], rear axle total
    double load_transfer_tau = 0.05;      // [s] lag on ax for load transfer
    double kinematic_speed_low = 1.0;     // [m/s] below: fully kinematic
    double kinematic_speed_high = 3.0;    // [m/s] above: fully dynamic
    double kinematic_tau = 0.05;          // [s] pull towards kinematic motion

    double cg_to_rear() const { return wheelbase - cg_to_front; }
  };

  std::string_view name() const override { return "single_track"; }
  void init(const ParamTree& params, Context& ctx) override;
  std::size_t state_size() const override { return kStateSize; }
  void initial_state(std::span<double> x) const override;
  void read_inputs(const SignalBus& bus) override;
  void derivatives(double t, std::span<const double> x, std::span<double> dx) const override;
  void write_outputs(double t, std::span<const double> x, SignalBus& bus) const override;

  const Params& params() const { return p_; }
  static Params params_from(const ParamTree& params);

 private:
  struct Motor {
    std::string id;
    bool front = true;
    double gear_ratio = 1.0;
    double efficiency = 1.0;
    SignalId torque_cmd_in = 0;
    SignalId torque_out = 0;
    SignalId speed_out = 0;
  };

  struct AxleState {
    double fz = 0.0;     // normal load [N]
    double fx = 0.0;     // longitudinal tire force [N]
    double fy = 0.0;     // lateral tire force [N]
    double alpha = 0.0;  // slip angle [rad]
  };

  struct Eval {
    double delta = 0.0;  // road wheel angle [rad]
    AxleState front;
    AxleState rear;
    double drag = 0.0;
    double downforce = 0.0;
    std::array<double, kMaxMotors> motor_speed{};  // [rad/s], from vehicle speed
    std::array<double, kMaxMotors> motor_torque{};
    double ax = 0.0;  // body-frame acceleration [m/s^2]
    double ay = 0.0;
    double dx[kStateSize] = {};
  };

  Eval evaluate(std::span<const double> x) const;

  Params p_;
  std::vector<Motor> motors_;
  // Latched inputs.
  double steer_ = 0.0;
  double brake_ = 0.0;
  std::array<double, kMaxMotors> torque_cmd_{};
  // Initial pose.
  double x0_ = 0.0, y0_ = 0.0, yaw0_ = 0.0, v0_ = 0.0;

  SignalId in_steer_ = 0, in_brake_ = 0;
  SignalId out_x_ = 0, out_y_ = 0, out_yaw_ = 0, out_vx_ = 0, out_vy_ = 0, out_r_ = 0;
  SignalId out_speed_ = 0, out_ax_ = 0, out_ay_ = 0, out_beta_ = 0, out_delta_ = 0;
  SignalId out_f_fz_ = 0, out_f_fx_ = 0, out_f_fy_ = 0, out_f_alpha_ = 0;
  SignalId out_r_fz_ = 0, out_r_fx_ = 0, out_r_fy_ = 0, out_r_alpha_ = 0;
  SignalId out_drag_ = 0, out_downforce_ = 0, out_power_ = 0;
};

}  // namespace vsim

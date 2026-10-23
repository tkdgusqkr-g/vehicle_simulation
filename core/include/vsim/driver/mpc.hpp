#pragma once

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "vsim/driver/drivers.hpp"
#include "vsim/sim/rng.hpp"

namespace vsim {

// The MPC driver's own idea of the car (design doc §9.5: the driver's
// internal model is not the plant). Dynamic single-track in Frenet
// coordinates along the track centreline, Magic Formula lateral tyres with
// aero load, blending into a kinematic model at walking speed.
//   state  x = [s, n, dpsi, vx, vy, r]   (along, left of the centreline, heading
//                                         relative to it, body velocities, yaw rate)
//   input  u = [delta, ax]                (road-wheel angle, longitudinal acceleration)
struct MpcModel {
  static constexpr std::size_t kStates = 6, kInputs = 2;
  using State = std::array<double, kStates>;
  using Input = std::array<double, kInputs>;
  enum : std::size_t { S, N, Dpsi, Vx, Vy, R };

  // One axle's lateral force: Magic Formula on the axle load, with the friction (peak force
  // over load, load transfer included) changing with the load.
  struct Axle {
    double mu = 1.5, load_ref = 1300.0, sensitivity = 0.0;
    double b = 12.0, c = 1.4;
    double gain = 1.0;         // slip stiffness relative to the tyre data (the driver learns it)
    double peak_slip() const;  // slip angle of the peak force [rad]
    double mu_at(double fz) const;
  };

  double mass = 280.0, yaw_inertia = 110.0, lf = 0.8, lr = 0.73, cg_height = 0.28;
  Axle front, rear;              // what the driver expects of the tyres
  double cla = 3.2, cda = 1.3, air_density = 1.2, aero_front = 0.45;
  double power = 80e3;           // [W] drive power limit (the VCU's)
  double drive_force = 5000.0;   // [N] all motors at full torque
  double brake_front = 0.7;      // front share of the brake force

  // From the vehicle parameters; the axles are fitted to the vehicle's tyre model.
  static MpcModel from_params(const ParamTree& params);

  // Lateral acceleration the car can hold at speed v (downforce included, the first axle to
  // saturate sets it); the driver also takes it as the grip for braking.
  double grip(double vx) const;
  double ax_max(double vx) const;  // traction, motors and power
  // dx/dt at curvature kappa of the centreline.
  State derivative(const State& x, const Input& u, double kappa) const;
};

// Track data the MPC needs along the centreline, resampled on a regular grid.
struct MpcTrack {
  double step = 0.5, length = 0.0;
  bool closed = true;
  std::vector<double> kappa, width_left, width_right, n_ref, psi_ref, v_ref, v_cap;
  double at(const std::vector<double>& v, double s) const;
};

// MPC driver (design doc §9.5). Every mpc_period_s it linearises its model
// along the previous plan (shifted) and solves up to three condensed QPs
// (sequential QP in a trust region): tracking mode follows a line and a speed
// profile and passes the plan through human characteristics (reaction delay
// compensated in the prediction, neuromuscular steering lag, rate limit,
// noise); racing mode (MPCC) maximises progress inside the track, with the
// quasi-steady speed limit as a soft cap. Both keep the plan inside the
// friction circle (as an octagon) and the tyres below their peak slip angle.
// The model's axles are fitted to the vehicle's tyre data, and the driver
// learns how each axle bites (its slip stiffness) while driving.
// Longitudinal acceleration becomes pedals through a nominal inverse map with
// PI on the felt acceleration and on the planned speed. A failed solve uses
// the shifted plan, and pure pursuit after three in a row.
//
//   driver: {type: mpc, mode: track | race, line: racing, horizon_s: 1.5 (race 2), steps: 30,
//            mpc_period_s: 0.05, aggressiveness: 0.95, margin_m: 0.3,
//            weights: {lateral, heading, speed, steer_rate, accel_rate, progress},
//            human: {enabled, delay_s: 0.15, bandwidth_hz: 2.5, steer_rate_deg_per_s: 600, noise_deg: 0.3},
//            model: {grip_scale: 0.95, mu: (fixed, instead of the tyre data), learn: true,
//                    feel_grip: true, front_gain: 1, rear_gain: 1},
//            profile: {lateral_share: 0.8, brake_share: 0.55, max_kph: 110}, speed_profile: file.csv}
class MpcDriver final : public DriverBlock {
 public:
  MpcDriver();
  ~MpcDriver() override;
  std::string_view name() const override { return "driver.mpc"; }
  void step(double t, SignalBus& bus) override;

  // For tests and tools.
  const MpcModel& model() const { return model_; }
  const MpcTrack& reference() const { return ref_; }
  std::int64_t solves() const { return solves_; }
  std::int64_t failures() const { return failures_; }
  std::int64_t fallbacks() const { return fallbacks_; }
  // The current plan per stage: state (6) then input (2).
  std::vector<std::array<double, 8>> planned() const;

  struct Plan;

 private:
  void init_driver(const ParamTree& params, Context& ctx) override;
  bool solve(const MpcModel::State& x0, MpcModel::Input applied);
  // One QP around the guess `ub`: the input changes, or nothing if it failed.
  std::optional<std::vector<MpcModel::Input>> qp_step(const MpcModel::State& x0, MpcModel::Input applied,
                                                      const std::vector<MpcModel::Input>& ub);
  int substeps(double vx, double dt) const;
  MpcModel::State simulate(MpcModel::State x, const MpcModel::Input& u, double dt, int sub) const;
  double pursuit_steer(double x, double y, double yaw, double speed);
  void learn(SignalBus& bus, double vx, double vy, double r);
  void feel_grip(const SignalBus& bus);

  MpcModel model_;
  MpcTrack ref_;
  std::shared_ptr<const Track> track_;
  SampledCurve line_;
  bool race_ = false;
  int steps_ = 30, sqp_iterations_ = 3;
  double dt_mpc_ = 0.05, period_mpc_ = 0.05, aggressiveness_ = 0.85, margin_ = 0.25;
  double half_width_ = 0.7;
  double steering_ratio_ = 4.5, max_delta_ = 0.4, max_delta_rate_ = 1.2, max_jerk_ = 40.0;
  double w_lateral_ = 3.0, w_heading_ = 1.0, w_speed_ = 0.6, w_steer_rate_ = 20.0, w_accel_rate_ = 0.02;
  double w_progress_ = 7.0, w_slack_ = 2000.0;

  // Human characteristics (tracking mode).
  double delay_ = 0.15, bandwidth_ = 2.5, hand_rate_ = 0.0, noise_ = 0.0;
  struct Delayed {
    double delta, ax, v;
  };
  std::deque<Delayed> delay_line_;
  double steer_pos_ = 0.0, steer_vel_ = 0.0, noise_state_ = 0.0;
  std::optional<Rng> rng_;

  // Pedals: nominal inverse map, PI on the felt acceleration and on the planned speed.
  double brake_force_max_ = 3000.0, wheel_radius_ = 0.2;
  double rolling_ = 0.015;
  double kp_ = 0.3, ki_ = 0.4, integral_ = 0.0;
  double ka_p_ = 0.5, ka_i_ = 4.0, accel_integral_ = 0.0;

  std::unique_ptr<Plan> plan_;
  double plan_time_ = -1.0, next_solve_ = 0.0, s_hint_ = 0.0;
  bool have_hint_ = false;
  int failed_in_row_ = 0, last_iterations_ = 0;
  std::int64_t solves_ = 0, failures_ = 0, fallbacks_ = 0;
  DriverCommand command_;
  double pursuit_s_ = 0.0;
  bool pursuit_first_ = true;

  // Learning the car (the steering gain).
  bool learn_ = true, have_last_ = false;
  double last_r_ = 0.0;
  std::array<double, 7> felt_{};
  // Feeling the tire grip (temperature, pressure).
  bool feel_grip_ = true;
  std::array<double, 2> base_mu_{}, felt_grip_{1.0, 1.0};
  double speed_scale_ = 1.0;
  std::array<SignalId, 4> grip_in_{};

  SignalId x_in_ = 0, y_in_ = 0, yaw_in_ = 0, vx_in_ = 0, vy_in_ = 0, r_in_ = 0, ax_in_ = 0, ay_in_ = 0;
  SignalId front_gain_out_ = 0, rear_gain_out_ = 0;
  SignalId status_out_ = 0, iter_out_ = 0, s_out_ = 0, n_out_ = 0, nref_out_ = 0, vref_out_ = 0;
  SignalId ax_out_ = 0, delta_out_ = 0, target_out_ = 0, lateral_out_ = 0, fail_out_ = 0;
  std::array<SignalId, 8> path_x_{}, path_y_{};
};

}  // namespace vsim

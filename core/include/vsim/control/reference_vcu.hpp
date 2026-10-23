#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "vsim/sim/block.hpp"

namespace vsim {

// Reference VCU (design doc §8.1): linear pedal maps, front/rear torque
// split, regenerative braking on the brake pedal, per-wheel slip control
// (traction control when driving, ABS-like when regenerating) and a power
// limit that respects both control.power_limit_kw and the BMS limits.
//
// With sensors on the vehicle (vehicle.sensors, control.use_sensors: true)
// the VCU works from what they measure, as on the car:
//   - accelerator from the two APPS tracks (a `pedal` sensor), with the FSAE
//     plausibility check: tracks more than 10 % apart or out of range for
//     more than 100 ms cut drive torque (T.4.2);
//   - brake from the brake pressure sensor, with the brake/APPS plausibility
//     check: hard braking with more than 25 % accelerator cuts drive torque
//     until the accelerator is below 5 % (EV.4.7);
//   - vehicle speed estimated from the wheel speed sensors and the IMU
//     (complementary filter, yaw-corrected; the slowest wheel when driving,
//     the fastest when braking) for slip control and the regen fade.
// Without them, it reads the driver's pedals and the true speed.
//
// Inputs:  drv.cmd.{throttle,brake} or sens.*, veh.chassis.vx, pt.motor.<id>.speed,
//          bat.limit.{discharge,charge}_power (if the plant has a battery)
// Outputs: pt.motor.<id>.torque_cmd, vcu.{power_limit,regen,throttle,brake,vx_est,
//          apps_implausible,bspc}, vcu.motor.<id>.{slip,tc}
class ReferenceVcu final : public Block {
 public:
  std::string_view name() const override { return "vcu"; }
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override;

 private:
  struct Motor {
    SignalId speed_in = 0;
    SignalId torque_cmd_out = 0;
    double weight = 1.0;        // share of the full drive torque (front/rear split)
    double regen_weight = 1.0;  // share of the full regen torque
    double gear_ratio = 1.0;    // motor speed / wheel speed
    SignalId slip_out = 0;
    SignalId tc_out = 0;
    double reduction = 0.0;     // integral part of the slip control, 0..1
  };

  double period_ = 0.01;
  double torque_max_ = 0.0;      // per motor [N m]
  double power_limit_ = 0.0;     // total [W]
  double efficiency_ = 0.92;     // assumed motor + inverter efficiency for power estimates
  bool regen_ = true;
  double regen_torque_ratio_ = 0.5;  // regen torque at full pedal / torque_max
  double regen_min_speed_ = 5.0 / 3.6;   // [m/s] no regen below
  double regen_full_speed_ = 10.0 / 3.6; // [m/s] full regen above
  double brake_cut_ = 0.05;      // brake pedal above which drive torque is cut
  double wheel_radius_ = 0.2;    // [m]
  bool tc_ = true;
  double tc_target_ = 0.10;      // slip ratio target
  double tc_kp_ = 2.0;           // torque reduction per unit of slip error
  double tc_ki_ = 20.0;          // [1/s]
  double tc_min_speed_ = 1.0;    // [m/s] slip ratio reference floor

  SignalId throttle_in_ = 0;
  SignalId brake_in_ = 0;
  SignalId vx_in_ = 0;

  // Accelerator position sensor: two tracks, their volts at 0 and 100 %.
  struct Apps {
    std::array<SignalId, 2> track{};
    std::array<std::array<double, 2>, 2> volts{};
  };
  std::optional<Apps> apps_;
  double apps_min_v_ = 0.2, apps_max_v_ = 4.8;  // outside: open or short circuit
  double apps_max_deviation_ = 0.10;
  double apps_fault_time_ = 0.10;  // [s] implausible this long -> cut
  double apps_deadband_ = 0.02;
  double apps_bad_since_ = -1.0;   // time the current implausibility started
  bool apps_cut_ = false;
  // Brake pressure sensor and the brake/APPS plausibility check.
  std::optional<SignalId> brake_pressure_in_;
  double brake_pressure_max_ = 100.0;  // [bar] pressure at full pedal
  double bspc_brake_bar_ = 10.0;       // "hard braking"
  double bspc_on_ = 0.25, bspc_off_ = 0.05;
  bool bspc_ = false;
  // Speed estimator: wheel speed sensors (by corner), IMU ax and gz.
  std::array<std::optional<SignalId>, 4> wheel_speed_in_;
  std::optional<SignalId> imu_ax_in_, imu_gz_in_;
  std::array<double, 4> wheel_y_{};  // lateral position of each wheel [m]
  double estimator_tau_ = 0.1;        // [s] correction time constant when the wheels agree
  double estimator_slip_tau_ = 3.0;   // [s] when they don't (wheelspin, lock-up) or under hard acceleration
  double estimator_free_accel_ = 2.0; // [m/s^2] below this the wheels are taken to roll freely
  double vx_est_ = 0.0;
  bool estimator_started_ = false;
  double request_ = 0.0;             // previous request: drive (>0) or brake (<0)
  SignalId throttle_out_ = 0, brake_out_ = 0, vx_est_out_ = 0, apps_out_ = 0, bspc_out_ = 0;

  double estimate_speed(const SignalBus& bus, double truth);

  std::optional<SignalId> discharge_limit_in_;
  std::optional<SignalId> charge_limit_in_;
  SignalId power_limit_out_ = 0;
  SignalId regen_out_ = 0;
  std::vector<Motor> motors_;
};

}  // namespace vsim

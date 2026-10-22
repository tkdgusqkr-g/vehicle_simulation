#pragma once

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
// Vehicle speed for the slip estimate is taken from veh.chassis.vx for now;
// the estimator from wheel speeds and IMU arrives with the sensor chain (M6).
//
// Inputs:  drv.cmd.{throttle,brake}, veh.chassis.vx, pt.motor.<id>.speed,
//          bat.limit.{discharge,charge}_power (if the plant has a battery)
// Outputs: pt.motor.<id>.torque_cmd, vcu.{power_limit,regen}, vcu.motor.<id>.{slip,tc}
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
  std::optional<SignalId> discharge_limit_in_;
  std::optional<SignalId> charge_limit_in_;
  SignalId power_limit_out_ = 0;
  SignalId regen_out_ = 0;
  std::vector<Motor> motors_;
};

}  // namespace vsim

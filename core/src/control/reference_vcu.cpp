#include "vsim/control/reference_vcu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "vsim/models/driveline/driveline.hpp"
#include "vsim/models/powertrain/motor.hpp"
#include "vsim/sim/context.hpp"

namespace vsim {

void ReferenceVcu::init(const ParamTree& params, Context& ctx) {
  SignalBus& bus = ctx.bus();
  period_ = params.get_double("control.period_s", 0.01);
  power_limit_ = params.get_double("control.power_limit_kw", 80.0) * 1e3;
  efficiency_ = params.get_double("control.efficiency_estimate", efficiency_);
  torque_max_ = MotorParams::from_params(params).torque_max;
  regen_ = params.get_bool("control.regen.enabled", regen_);
  regen_torque_ratio_ = params.get_double("control.regen.torque_ratio", regen_torque_ratio_);
  regen_min_speed_ = params.get_double("control.regen.min_speed_kph", regen_min_speed_ * 3.6) / 3.6;
  regen_full_speed_ = params.get_double("control.regen.full_speed_kph", regen_full_speed_ * 3.6) / 3.6;
  brake_cut_ = params.get_double("control.brake_cut", brake_cut_);
  wheel_radius_ = params.get_double("vehicle.wheel.radius_m");
  tc_ = params.get_bool("control.slip_control.enabled", tc_);
  tc_target_ = params.get_double("control.slip_control.target_slip", tc_target_);
  tc_kp_ = params.get_double("control.slip_control.kp", tc_kp_);
  tc_ki_ = params.get_double("control.slip_control.ki_per_s", tc_ki_);
  tc_min_speed_ = params.get_double("control.slip_control.min_speed_kph", tc_min_speed_ * 3.6) / 3.6;
  const double front_share = params.get_double("control.torque_split_front", 0.5);
  const double regen_front_share = params.get_double("control.regen.split_front", 0.5);
  if (!(power_limit_ > 0.0)) {
    throw std::invalid_argument("control.power_limit_kw must be positive");
  }
  if (!(efficiency_ > 0.0 && efficiency_ <= 1.0)) {
    throw std::invalid_argument("control.efficiency_estimate must be in (0, 1]");
  }
  if (!(front_share >= 0.0 && front_share <= 1.0 && regen_front_share >= 0.0 &&
        regen_front_share <= 1.0)) {
    throw std::invalid_argument("control.torque_split_front and control.regen.split_front must be in [0, 1]");
  }
  if (!(tc_target_ > 0.0 && tc_min_speed_ > 0.0 && wheel_radius_ > 0.0)) {
    throw std::invalid_argument("control.slip_control: target_slip and min_speed_kph must be positive");
  }
  if (!(regen_full_speed_ > regen_min_speed_)) {
    throw std::invalid_argument("control.regen: full_speed_kph must exceed min_speed_kph");
  }

  throttle_in_ = bus.declare("drv.cmd.throttle", "-", "accelerator pedal");
  brake_in_ = bus.declare("drv.cmd.brake", "-", "brake pedal");
  vx_in_ = bus.declare("veh.chassis.vx", "m/s", "longitudinal velocity (body frame)");
  discharge_limit_in_ = bus.find("bat.limit.discharge_power");
  charge_limit_in_ = bus.find("bat.limit.charge_power");
  power_limit_out_ = bus.declare("vcu.power_limit", "W", "drive power limit in force");
  regen_out_ = bus.declare("vcu.regen", "-", "regenerative braking fraction");

  // Front/rear split as weights; the axle with the larger share gets full torque.
  const DrivelineConfig driveline = DrivelineConfig::from_params(params);
  bool has_front = false, has_rear = false;
  for (const DrivelineEntry& entry : driveline.entries) {
    (is_front(entry.wheels.front()) ? has_front : has_rear) = true;
  }
  const auto weight = [&](double share, bool front) {
    if (!(has_front && has_rear)) {
      return 1.0;
    }
    return (front ? share : 1.0 - share) / std::max(share, 1.0 - share);
  };
  motors_.clear();
  for (const DrivelineEntry& entry : driveline.entries) {
    const std::string base = "pt.motor." + entry.motor;
    const bool front = is_front(entry.wheels.front());
    const std::string vcu = "vcu.motor." + entry.motor;
    motors_.push_back({bus.declare(base + ".speed", "rad/s", "motor shaft speed"),
                       bus.declare(base + ".torque_cmd", "N*m", "motor torque command"),
                       weight(front_share, front), weight(regen_front_share, front),
                       entry.gear_ratio,
                       bus.declare(vcu + ".slip", "-", "estimated slip ratio of the driven wheel(s)"),
                       bus.declare(vcu + ".tc", "-", "slip control torque factor (1 = no reduction)"),
                       0.0});
  }
}

void ReferenceVcu::step(double /*t*/, SignalBus& bus) {
  const double throttle = std::clamp(bus.get(throttle_in_), 0.0, 1.0);
  const double brake = std::clamp(bus.get(brake_in_), 0.0, 1.0);
  const double vx = bus.get(vx_in_);
  const double infinity = std::numeric_limits<double>::infinity();
  const double discharge_limit = discharge_limit_in_ ? bus.get(*discharge_limit_in_) : infinity;
  const double charge_limit = charge_limit_in_ ? bus.get(*charge_limit_in_) : infinity;

  // Drive unless the brake is pressed (brake has priority, as on the car).
  double request = brake > brake_cut_ ? 0.0 : throttle;
  if (regen_ && brake > 0.0) {
    const double fade = std::clamp((vx - regen_min_speed_) / (regen_full_speed_ - regen_min_speed_), 0.0, 1.0);
    const double regen = brake * regen_torque_ratio_ * fade;
    if (regen > 0.0) {
      request = -regen;
    }
  }

  // Per-motor torque after slip control: PI on the slip ratio beyond the
  // target, in the direction of the torque (wheelspin or regen lock-up).
  const double reference = std::max(std::abs(vx), tc_min_speed_);
  std::vector<double> torque(motors_.size());
  double power = 0.0;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    Motor& motor = motors_[i];
    const double motor_speed = bus.get(motor.speed_in);
    const double slip = (motor_speed / motor.gear_ratio * wheel_radius_ - vx) / reference;
    double factor = 1.0;
    if (tc_ && request != 0.0) {
      const double error = (request > 0.0 ? slip : -slip) - tc_target_;
      motor.reduction = std::clamp(motor.reduction + tc_ki_ * error * period_, 0.0, 1.0);
      factor = std::clamp(1.0 - tc_kp_ * std::max(0.0, error) - motor.reduction, 0.0, 1.0);
    } else {
      motor.reduction = 0.0;
    }
    const double share = request >= 0.0 ? motor.weight : motor.regen_weight;
    torque[i] = request * torque_max_ * share * factor;
    power += torque[i] * std::abs(motor_speed);
    bus.set(motor.slip_out, slip);
    bus.set(motor.tc_out, factor);
  }

  // Estimated battery power, then one scale factor for all motors.
  double limit = 0.0;
  double estimate = 0.0;
  if (request >= 0.0) {
    limit = std::min(power_limit_, discharge_limit);
    estimate = power / efficiency_;
  } else {
    limit = charge_limit;
    estimate = -power * efficiency_;
  }
  const double scale = estimate > limit ? std::max(0.0, limit) / estimate : 1.0;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    bus.set(motors_[i].torque_cmd_out, torque[i] * scale);
  }
  bus.set(power_limit_out_, std::min(power_limit_, discharge_limit));
  bus.set(regen_out_, request < 0.0 ? -request * scale : 0.0);
}

}  // namespace vsim

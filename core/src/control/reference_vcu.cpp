#include "vsim/control/reference_vcu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "vsim/models/driveline/driveline.hpp"
#include "vsim/models/powertrain/motor.hpp"
#include "vsim/models/vehicle/corners.hpp"
#include "vsim/sensors/sensor_system.hpp"
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
  throttle_out_ = bus.declare("vcu.throttle", "-", "accelerator position the VCU acts on");
  brake_out_ = bus.declare("vcu.brake", "-", "brake demand the VCU acts on");
  vx_est_out_ = bus.declare("vcu.vx_est", "m/s", "estimated longitudinal speed");
  apps_out_ = bus.declare("vcu.apps_implausible", "-", "1 while the APPS plausibility check cuts torque");
  bspc_out_ = bus.declare("vcu.bspc", "-", "1 while the brake/APPS plausibility check cuts torque");

  // The car's sensors, found by type (the first of each; wheel speeds by corner).
  apps_.reset();
  brake_pressure_in_.reset();
  wheel_speed_in_ = {};
  imu_ax_in_.reset();
  imu_gz_in_.reset();
  if (params.get_bool("control.use_sensors", true)) {
    const std::vector<SensorSystem::Entry> sensors = SensorSystem::entries(params);
    for (std::size_t i = 0; i < sensors.size(); ++i) {
      const SensorSystem::Entry& e = sensors[i];
      const std::string prefix = "vehicle.sensors." + std::to_string(i);
      const std::string out = "sens." + e.name + ".";
      if (e.type == "pedal" && !apps_ && params.get_string(prefix + ".pedal", "throttle") == "throttle") {
        Apps apps;
        for (std::size_t k = 0; k < 2; ++k) {
          const std::string key = k == 0 ? "ch1" : "ch2";
          apps.track[k] = bus.require(out + key);
          const std::vector<double> volts = params.get_double_list(prefix + "." + key + "_v");
          apps.volts[k] = volts.size() == 2 ? std::array<double, 2>{volts[0], volts[1]}
                                            : (k == 0 ? std::array<double, 2>{0.5, 4.5} : std::array<double, 2>{4.5, 0.5});
          if (apps.volts[k][0] == apps.volts[k][1]) {
            throw std::invalid_argument(prefix + "." + key + "_v: the two voltages must differ");
          }
        }
        apps_ = apps;
      } else if (e.type == "brake_pressure" && !brake_pressure_in_) {
        brake_pressure_in_ = bus.find(out + "pressure");  // absent if the model has no brake pressure
      } else if (e.type == "wheel_speed" && e.corner >= 0 && !wheel_speed_in_[e.corner]) {
        wheel_speed_in_[e.corner] = bus.find(out + "omega");
      } else if ((e.type == "imu" || e.type == "accelerometer") && !imu_ax_in_) {
        imu_ax_in_ = bus.require(out + "ax");
      }
      if ((e.type == "imu" || e.type == "gyro") && !imu_gz_in_) {
        imu_gz_in_ = bus.require(out + "gz");
      }
    }
  }
  apps_min_v_ = params.get_double("control.apps.min_v", apps_min_v_);
  apps_max_v_ = params.get_double("control.apps.max_v", apps_max_v_);
  apps_max_deviation_ = params.get_double("control.apps.max_deviation", apps_max_deviation_);
  apps_fault_time_ = params.get_double("control.apps.fault_time_s", apps_fault_time_);
  apps_deadband_ = params.get_double("control.apps.deadband", apps_deadband_);
  brake_pressure_max_ = params.get_double("vehicle.brakes.max_pressure_bar", brake_pressure_max_);
  bspc_brake_bar_ = params.get_double("control.bspc.brake_bar", bspc_brake_bar_);
  bspc_on_ = params.get_double("control.bspc.apps_on", bspc_on_);
  bspc_off_ = params.get_double("control.bspc.apps_off", bspc_off_);
  estimator_tau_ = params.get_double("control.speed_estimator.tau_s", estimator_tau_);
  estimator_slip_tau_ = params.get_double("control.speed_estimator.slip_tau_s", estimator_slip_tau_);
  estimator_free_accel_ = params.get_double("control.speed_estimator.free_accel_mps2", estimator_free_accel_);
  if (!(apps_deadband_ >= 0.0 && apps_deadband_ < 0.5 && apps_fault_time_ >= 0.0 && estimator_tau_ > 0.0 &&
        estimator_slip_tau_ > 0.0 && brake_pressure_max_ > 0.0 && bspc_off_ < bspc_on_)) {
    throw std::invalid_argument("control: APPS, BSPC or speed estimator settings out of range");
  }
  for (std::size_t c = 0; c < kCorners; ++c) {
    const double track = params.get_double(is_front(c) ? "vehicle.track_front_m" : "vehicle.track_rear_m", 1.2);
    wheel_y_[c] = (is_left(c) ? 0.5 : -0.5) * track;
  }
  apps_bad_since_ = -1.0;
  apps_cut_ = false;
  bspc_ = false;
  vx_est_ = 0.0;
  estimator_started_ = false;
  request_ = 0.0;

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

double ReferenceVcu::estimate_speed(const SignalBus& bus, double truth) {
  // Wheel speeds at the car's centreline (minus the yaw rate's share).
  const double yaw_rate = imu_gz_in_ ? bus.get(*imu_gz_in_) : 0.0;
  double lo = std::numeric_limits<double>::infinity(), hi = -lo, sum = 0.0;
  int count = 0;
  for (std::size_t c = 0; c < kCorners; ++c) {
    if (wheel_speed_in_[c]) {
      const double v = bus.get(*wheel_speed_in_[c]) * wheel_radius_ + yaw_rate * wheel_y_[c];
      lo = std::min(lo, v);
      hi = std::max(hi, v);
      sum += v;
      ++count;
    }
  }
  if (count == 0) {
    return truth;
  }
  // Driving: every wheel may spin, the slowest is closest; braking: the fastest.
  const double reference = request_ > 0.0 ? lo : request_ < 0.0 ? hi : sum / count;
  if (!estimator_started_ || !imu_ax_in_) {
    estimator_started_ = true;
    vx_est_ = reference;
    return vx_est_;
  }
  // Integrate the accelerometer, pull towards the wheels: slowly while they
  // disagree (one or more slipping) or under hard acceleration or braking,
  // when with four driven wheels all of them may slip alike.
  const double ax = bus.get(*imu_ax_in_);
  const double spread = hi - lo;
  const bool wheels_free = spread <= 0.05 * std::max(std::abs(reference), 2.0) && std::abs(ax) <= estimator_free_accel_;
  const double tau = wheels_free ? estimator_tau_ : estimator_slip_tau_;
  vx_est_ += ax * period_ + (reference - vx_est_) * std::min(1.0, period_ / tau);
  return vx_est_;
}

void ReferenceVcu::step(double t, SignalBus& bus) {
  double throttle = std::clamp(bus.get(throttle_in_), 0.0, 1.0);
  double brake = std::clamp(bus.get(brake_in_), 0.0, 1.0);
  if (apps_) {
    // Two tracks to pedal fractions; plausible if in range and within 10 %.
    std::array<double, 2> travel{};
    bool in_range = true;
    for (std::size_t k = 0; k < 2; ++k) {
      const double v = bus.get(apps_->track[k]);
      in_range = in_range && v >= apps_min_v_ && v <= apps_max_v_;
      travel[k] = (v - apps_->volts[k][0]) / (apps_->volts[k][1] - apps_->volts[k][0]);
    }
    const bool plausible = in_range && std::abs(travel[0] - travel[1]) <= apps_max_deviation_;
    if (plausible) {
      apps_bad_since_ = -1.0;
      apps_cut_ = false;
    } else if (apps_bad_since_ < 0.0) {
      apps_bad_since_ = t;
    }
    if (!plausible && t - apps_bad_since_ + 1e-9 >= apps_fault_time_) {
      apps_cut_ = true;
    }
    const double mean = 0.5 * (travel[0] + travel[1]);
    throttle = std::clamp((mean - apps_deadband_) / (1.0 - 2.0 * apps_deadband_), 0.0, 1.0);
  }
  if (brake_pressure_in_) {
    const double pressure = bus.get(*brake_pressure_in_);
    brake = std::clamp(pressure / brake_pressure_max_, 0.0, 1.0);
    // Brake/APPS plausibility: latched until the accelerator is released.
    if (pressure > bspc_brake_bar_ && throttle > bspc_on_) {
      bspc_ = true;
    } else if (throttle < bspc_off_) {
      bspc_ = false;
    }
  }
  const double vx = estimate_speed(bus, bus.get(vx_in_));
  const double infinity = std::numeric_limits<double>::infinity();
  const double discharge_limit = discharge_limit_in_ ? bus.get(*discharge_limit_in_) : infinity;
  const double charge_limit = charge_limit_in_ ? bus.get(*charge_limit_in_) : infinity;

  // Drive unless the brake is pressed (brake has priority, as on the car)
  // or a plausibility check has cut the drive torque.
  double request = brake > brake_cut_ || apps_cut_ || bspc_ ? 0.0 : throttle;
  if (regen_ && brake > brake_cut_) {
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
  bus.set(throttle_out_, throttle);
  bus.set(brake_out_, brake);
  bus.set(vx_est_out_, vx);
  bus.set(apps_out_, apps_cut_ ? 1.0 : 0.0);
  bus.set(bspc_out_, bspc_ ? 1.0 : 0.0);
  request_ = brake > brake_cut_ ? -1.0 : request;
}

}  // namespace vsim

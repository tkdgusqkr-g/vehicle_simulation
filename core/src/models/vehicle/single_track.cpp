#include "vsim/models/vehicle/single_track.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "vsim/models/driveline/driveline.hpp"
#include "vsim/models/powertrain/motor.hpp"
#include "vsim/models/tire/tire.hpp"

namespace vsim {
namespace {

constexpr double kGravity = 9.81;
// Speed scale for smoothing force directions through zero speed [m/s].
constexpr double kDirectionSmoothing = 0.05;

double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

double magic_formula(double slip, double peak, const SingleTrackModel::Tire& tire) {
  const double b = tire.B * slip;
  return peak * std::sin(tire.C * std::atan(b - tire.E * (b - std::atan(b))));
}

SingleTrackModel::Tire read_tire(const ParamTree& params, const std::string& axle) {
  const SimpleTire::Params tire = SimpleTire::params_from(params, axle);
  return {tire.mu_y, tire.by, tire.cy, tire.ey};
}

void require_positive(double value, const char* key) {
  if (!(value > 0.0)) {
    throw std::invalid_argument(std::string(key) + " must be positive");
  }
}

}  // namespace

SingleTrackModel::Params SingleTrackModel::params_from(const ParamTree& params) {
  Params p;
  p.mass = params.get_double("vehicle.mass_kg");
  p.yaw_inertia = params.get_double("vehicle.yaw_inertia_kgm2");
  p.wheelbase = params.get_double("vehicle.wheelbase_m");
  p.cg_to_front = params.get_double("vehicle.cg_to_front_axle_m");
  p.cg_height = params.get_double("vehicle.cg_height_m");
  p.steering_ratio = params.get_double("vehicle.steering.ratio");
  p.max_steer_wheel = deg_to_rad(params.get_double("vehicle.steering.lock_deg"));
  p.tire_radius = params.get_double("vehicle.wheel.radius_m");
  p.tire_front = read_tire(params, "front");
  p.tire_rear = read_tire(params, "rear");
  p.rolling_resistance = params.get_double("vehicle.tire.rolling_resistance", p.rolling_resistance);
  p.cla = params.get_double("vehicle.aero.cla_m2", p.cla);
  p.cda = params.get_double("vehicle.aero.cda_m2", p.cda);
  p.aero_front_ratio = params.get_double("vehicle.aero.front_ratio", p.aero_front_ratio);
  p.air_density = params.get_double("vehicle.aero.air_density_kgm3", p.air_density);
  const MotorParams motor = MotorParams::from_params(params);
  p.motor_torque_max = motor.torque_max;
  p.motor_power_max = motor.power_max;
  p.brake_pressure_max = params.get_double("vehicle.brakes.max_pressure_bar");
  p.brake_gain_front = params.get_double("vehicle.brakes.front_gain_nm_per_bar");
  p.brake_gain_rear = params.get_double("vehicle.brakes.rear_gain_nm_per_bar");
  p.load_transfer_tau =
      params.get_double("vehicle.suspension.load_transfer_tau_s", p.load_transfer_tau);

  require_positive(p.mass, "vehicle.mass_kg");
  require_positive(p.yaw_inertia, "vehicle.yaw_inertia_kgm2");
  require_positive(p.wheelbase, "vehicle.wheelbase_m");
  require_positive(p.steering_ratio, "vehicle.steering.ratio");
  require_positive(p.tire_radius, "vehicle.wheel.radius_m");
  require_positive(p.load_transfer_tau, "vehicle.suspension.load_transfer_tau_s");
  if (!(p.cg_to_front > 0.0 && p.cg_to_front < p.wheelbase)) {
    throw std::invalid_argument("vehicle.cg_to_front_axle_m must lie between 0 and the wheelbase");
  }
  return p;
}

void SingleTrackModel::init(const ParamTree& params, Context& ctx) {
  p_ = params_from(params);
  SignalBus& bus = ctx.bus();

  // Each motor drives distinct wheels, so there are at most four.
  const DrivelineConfig driveline = DrivelineConfig::from_params(params);
  motors_.clear();
  for (const DrivelineEntry& entry : driveline.entries) {
    const bool front = std::all_of(entry.wheels.begin(), entry.wheels.end(),
                                   [](std::size_t wheel) { return is_front(wheel); });
    const bool rear = std::none_of(entry.wheels.begin(), entry.wheels.end(),
                                   [](std::size_t wheel) { return is_front(wheel); });
    if (!front && !rear) {
      throw std::invalid_argument("single_track: motor '" + entry.motor +
                                  "' drives both axles; use the two_track model");
    }
    const std::string base = "pt.motor." + entry.motor;
    motors_.push_back({entry.motor, front, entry.gear_ratio, entry.efficiency,
                       bus.declare(base + ".torque_cmd", "N*m", "motor torque command"),
                       bus.declare(base + ".torque", "N*m", "motor torque delivered"),
                       bus.declare(base + ".speed", "rad/s", "motor shaft speed")});
  }

  in_steer_ = bus.declare("drv.cmd.steer", "rad", "steering wheel angle");
  in_brake_ = bus.declare("drv.cmd.brake", "-", "brake pedal");

  out_x_ = bus.declare("veh.chassis.x", "m", "position east");
  out_y_ = bus.declare("veh.chassis.y", "m", "position north");
  out_yaw_ = bus.declare("veh.chassis.yaw", "rad", "heading, counter-clockwise from east");
  out_vx_ = bus.declare("veh.chassis.vx", "m/s", "longitudinal velocity (body frame)");
  out_vy_ = bus.declare("veh.chassis.vy", "m/s", "lateral velocity (body frame)");
  out_r_ = bus.declare("veh.chassis.yaw_rate", "rad/s", "yaw rate");
  out_speed_ = bus.declare("veh.chassis.speed", "m/s", "speed over ground");
  out_ax_ = bus.declare("veh.chassis.ax", "m/s^2", "longitudinal acceleration (body frame)");
  out_ay_ = bus.declare("veh.chassis.ay", "m/s^2", "lateral acceleration (body frame)");
  out_beta_ = bus.declare("veh.chassis.beta", "rad", "body slip angle");
  out_delta_ = bus.declare("veh.steer.road_wheel_angle", "rad", "front road wheel angle");
  out_f_fz_ = bus.declare("veh.axle.front.fz", "N", "front axle normal load");
  out_f_fx_ = bus.declare("veh.axle.front.fx", "N", "front axle longitudinal tire force");
  out_f_fy_ = bus.declare("veh.axle.front.fy", "N", "front axle lateral tire force");
  out_f_alpha_ = bus.declare("veh.axle.front.slip_angle", "rad", "front slip angle");
  out_r_fz_ = bus.declare("veh.axle.rear.fz", "N", "rear axle normal load");
  out_r_fx_ = bus.declare("veh.axle.rear.fx", "N", "rear axle longitudinal tire force");
  out_r_fy_ = bus.declare("veh.axle.rear.fy", "N", "rear axle lateral tire force");
  out_r_alpha_ = bus.declare("veh.axle.rear.slip_angle", "rad", "rear slip angle");
  out_drag_ = bus.declare("veh.aero.drag", "N", "aerodynamic drag");
  out_downforce_ = bus.declare("veh.aero.downforce", "N", "aerodynamic downforce");
  out_power_ = bus.declare("pt.power_mech", "W", "total motor shaft power");

  x0_ = params.get_double("initial.x_m", 0.0);
  y0_ = params.get_double("initial.y_m", 0.0);
  yaw0_ = deg_to_rad(params.get_double("initial.yaw_deg", 0.0));
  v0_ = params.get_double("initial.speed_mps", 0.0);
}

void SingleTrackModel::initial_state(std::span<double> x) const {
  std::fill(x.begin(), x.end(), 0.0);
  x[kX] = x0_;
  x[kY] = y0_;
  x[kYaw] = yaw0_;
  x[kVx] = v0_;
}

void SingleTrackModel::read_inputs(const SignalBus& bus) {
  steer_ = bus.get(in_steer_);
  brake_ = std::clamp(bus.get(in_brake_), 0.0, 1.0);
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    torque_cmd_[i] = bus.get(motors_[i].torque_cmd_in);
  }
}

SingleTrackModel::Eval SingleTrackModel::evaluate(std::span<const double> x) const {
  const double vx = x[kVx];
  const double vy = x[kVy];
  const double r = x[kYawRate];
  const double yaw = x[kYaw];
  const double lf = p_.cg_to_front;
  const double lr = p_.cg_to_rear();
  const double m = p_.mass;
  const double direction = std::tanh(vx / kDirectionSmoothing);

  Eval e;
  e.delta = std::clamp(steer_, -p_.max_steer_wheel, p_.max_steer_wheel) / p_.steering_ratio;

  // Aero and normal loads; load transfer uses lagged ax to avoid an algebraic loop.
  const double dynamic_pressure = 0.5 * p_.air_density * (vx * vx + vy * vy);
  e.downforce = p_.cla * dynamic_pressure;
  e.drag = p_.cda * dynamic_pressure * direction;
  const double transfer = m * x[kAxFiltered] * p_.cg_height / p_.wheelbase;
  e.front.fz = std::max(0.0, m * kGravity * lr / p_.wheelbase +
                                 e.downforce * p_.aero_front_ratio - transfer);
  e.rear.fz = std::max(0.0, m * kGravity * lf / p_.wheelbase +
                                e.downforce * (1.0 - p_.aero_front_ratio) + transfer);

  // Motors: torque envelope min(T_max, P_max / omega).
  double drive_front = 0.0;
  double drive_rear = 0.0;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const Motor& motor = motors_[i];
    e.motor_speed[i] = std::abs(vx) / p_.tire_radius * motor.gear_ratio;
    const double torque_available =
        e.motor_speed[i] > 1e-6
            ? std::min(p_.motor_torque_max, p_.motor_power_max / e.motor_speed[i])
            : p_.motor_torque_max;
    e.motor_torque[i] = std::clamp(torque_cmd_[i], -torque_available, torque_available);
    const double force = e.motor_torque[i] * motor.gear_ratio * motor.efficiency / p_.tire_radius;
    (motor.front ? drive_front : drive_rear) += force;
  }

  // Longitudinal tire forces, limited by friction.
  const double pressure = brake_ * p_.brake_pressure_max;
  const auto longitudinal = [&](AxleState& axle, double drive, double brake_gain, double mu) {
    const double brake = pressure * brake_gain / p_.tire_radius * direction;
    const double rolling = p_.rolling_resistance * axle.fz * direction;
    const double limit = mu * axle.fz;
    axle.fx = std::clamp(drive - brake - rolling, -limit, limit);
  };
  longitudinal(e.front, drive_front, p_.brake_gain_front, p_.tire_front.mu);
  longitudinal(e.rear, drive_rear, p_.brake_gain_rear, p_.tire_rear.mu);

  // Lateral tire forces with friction-ellipse reduction.
  const double vx_slip = std::max(vx, p_.kinematic_speed_low);
  e.front.alpha = e.delta - std::atan2(vy + lf * r, vx_slip);
  e.rear.alpha = -std::atan2(vy - lr * r, vx_slip);
  const auto lateral = [](AxleState& axle, const Tire& tire) {
    const double limit = tire.mu * axle.fz;
    if (limit <= 0.0) {
      axle.fy = 0.0;
      return;
    }
    const double used = axle.fx / limit;
    const double peak = limit * std::sqrt(std::max(0.0, 1.0 - used * used));
    axle.fy = magic_formula(axle.alpha, peak, tire);
  };
  lateral(e.front, p_.tire_front);
  lateral(e.rear, p_.tire_rear);

  // Below kinematic_speed_high, lateral motion is pulled towards kinematic
  // rolling. Slip angles are meaningless near standstill, so their lateral
  // forces must not feed the longitudinal equation there either (a turned
  // wheel at rest would otherwise push the car backwards).
  const double weight = std::clamp((vx - p_.kinematic_speed_low) /
                                       (p_.kinematic_speed_high - p_.kinematic_speed_low),
                                   0.0, 1.0);

  // Dynamic single-track equations (body frame).
  const double cos_d = std::cos(e.delta);
  const double sin_d = std::sin(e.delta);
  const double fx_body = e.front.fx * cos_d - weight * e.front.fy * sin_d + e.rear.fx - e.drag;
  const double fy_front = e.front.fx * sin_d + e.front.fy * cos_d;
  const double fy_body = fy_front + e.rear.fy;
  const double dvx = fx_body / m + vy * r;
  const double dvy_dynamic = fy_body / m - vx * r;
  const double dr_dynamic = (lf * fy_front - lr * e.rear.fy) / p_.yaw_inertia;

  const double r_kinematic = vx * std::tan(e.delta) / p_.wheelbase;
  const double vy_kinematic = r_kinematic * lr;
  const double dvy = weight * dvy_dynamic + (1.0 - weight) * (vy_kinematic - vy) / p_.kinematic_tau;
  const double dr = weight * dr_dynamic + (1.0 - weight) * (r_kinematic - r) / p_.kinematic_tau;

  e.ax = dvx - vy * r;
  e.ay = dvy + vx * r;
  e.dx[kX] = vx * std::cos(yaw) - vy * std::sin(yaw);
  e.dx[kY] = vx * std::sin(yaw) + vy * std::cos(yaw);
  e.dx[kYaw] = r;
  e.dx[kVx] = dvx;
  e.dx[kVy] = dvy;
  e.dx[kYawRate] = dr;
  e.dx[kAxFiltered] = (e.ax - x[kAxFiltered]) / p_.load_transfer_tau;
  return e;
}

void SingleTrackModel::derivatives(double /*t*/, std::span<const double> x,
                                   std::span<double> dx) const {
  const Eval e = evaluate(x);
  std::copy(std::begin(e.dx), std::end(e.dx), dx.begin());
}

void SingleTrackModel::write_outputs(double /*t*/, std::span<const double> x,
                                     SignalBus& bus) const {
  const Eval e = evaluate(x);
  const double speed = std::hypot(x[kVx], x[kVy]);
  bus.set(out_x_, x[kX]);
  bus.set(out_y_, x[kY]);
  bus.set(out_yaw_, x[kYaw]);
  bus.set(out_vx_, x[kVx]);
  bus.set(out_vy_, x[kVy]);
  bus.set(out_r_, x[kYawRate]);
  bus.set(out_speed_, speed);
  bus.set(out_ax_, e.ax);
  bus.set(out_ay_, e.ay);
  bus.set(out_beta_, speed > 0.5 ? std::atan2(x[kVy], x[kVx]) : 0.0);
  bus.set(out_delta_, e.delta);
  bus.set(out_f_fz_, e.front.fz);
  bus.set(out_f_fx_, e.front.fx);
  bus.set(out_f_fy_, e.front.fy);
  bus.set(out_f_alpha_, e.front.alpha);
  bus.set(out_r_fz_, e.rear.fz);
  bus.set(out_r_fx_, e.rear.fx);
  bus.set(out_r_fy_, e.rear.fy);
  bus.set(out_r_alpha_, e.rear.alpha);
  bus.set(out_drag_, e.drag);
  bus.set(out_downforce_, e.downforce);
  double power = 0.0;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    bus.set(motors_[i].torque_out, e.motor_torque[i]);
    bus.set(motors_[i].speed_out, e.motor_speed[i]);
    power += e.motor_torque[i] * e.motor_speed[i];
  }
  bus.set(out_power_, power);
}

}  // namespace vsim

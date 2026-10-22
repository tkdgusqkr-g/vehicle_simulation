#include "vsim/models/vehicle/two_track.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "vsim/sim/context.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

constexpr double kGravity = 9.81;
// Rolling resistance fades in over this wheel speed so it never pushes [rad/s].
constexpr double kRollingSmoothing = 0.5;
// Bounds of the transient slip states; beyond them the tire slides anyway.
constexpr double kMaxSlipRatio = 10.0;
constexpr double kMaxTanSlipAngle = 10.0;

double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

void require_positive(double value, const char* key) {
  if (!(value > 0.0)) {
    throw std::invalid_argument(std::string(key) + " must be positive");
  }
}

// Relaxation-length slip state: sigma * ds/dt + |v| * s = slip velocity.
double slip_rate(double state, double slip_velocity, double speed, double sigma, double bound) {
  const double rate = (slip_velocity - std::abs(speed) * state) / sigma;
  if ((state >= bound && rate > 0.0) || (state <= -bound && rate < 0.0)) {
    return 0.0;
  }
  return rate;
}

}  // namespace

struct TwoTrackModel::Eval {
  double delta = 0.0;  // road wheel angle of the steering input [rad]
  std::array<double, kCorners> steer{};
  std::array<double, kCorners> fz{}, fx{}, fy{}, mz{};
  std::array<double, kCorners> slip_ratio{}, slip_angle{}, omega{};
  double downforce = 0.0;
  double drag = 0.0;
  double ax = 0.0, ay = 0.0;
  double ffb = 0.0;
  double brake_pressure = 0.0;

  std::array<double, kCorners> motor_torque{}, motor_speed{}, motor_avail{};
  std::array<double, kCorners> motor_loss{}, motor_power{};
  double power_mech = 0.0;
  double power_elec = 0.0;  // at the battery terminals, including aux

  double ocv = 0.0;  // pack
  BatteryOperatingPoint battery;
  BatteryLimits limits;
};

struct TwoTrackModel::Signals {
  struct Wheel {
    SignalId fz, fx, fy, mz, slip_ratio, slip_angle, omega, steer;
  };
  struct Axle {
    SignalId fz, fx, fy, slip_angle;
  };
  struct Motor {
    SignalId torque_cmd, torque, speed, avail, temp, loss, power;
  };

  SignalId steer, brake;
  SignalId x, y, yaw, vx, vy, r, speed, ax, ay, beta, delta;
  std::array<Wheel, kCorners> wheel;
  std::array<Axle, 2> axle;
  SignalId drag, downforce, brake_pressure, ffb;
  std::vector<Motor> motor;
  SignalId power_mech, power_elec;
  SignalId soc, voltage, current, power, ocv, battery_temp, overloaded;
  SignalId limit_discharge_power, limit_charge_power, limit_discharge_current, limit_charge_current;
  std::array<SignalId, kEnergyCount> energy;
  SignalId kinetic, rc_stored, residual;
};

TwoTrackModel::TwoTrackModel() = default;
TwoTrackModel::~TwoTrackModel() = default;

TwoTrackModel::Params TwoTrackModel::params_from(const ParamTree& params) {
  Params p;
  p.mass = params.get_double("vehicle.mass_kg");
  p.yaw_inertia = params.get_double("vehicle.yaw_inertia_kgm2");
  p.wheelbase = params.get_double("vehicle.wheelbase_m");
  p.cg_to_front = params.get_double("vehicle.cg_to_front_axle_m");
  p.cg_height = params.get_double("vehicle.cg_height_m");
  p.track_front = params.get_double("vehicle.track_front_m");
  p.track_rear = params.get_double("vehicle.track_rear_m");
  p.roll_center_front = params.get_double("vehicle.suspension.roll_center_front_m", p.roll_center_front);
  p.roll_center_rear = params.get_double("vehicle.suspension.roll_center_rear_m", p.roll_center_rear);
  p.roll_stiffness_front =
      params.get_double("vehicle.suspension.roll_stiffness_front_ratio", p.roll_stiffness_front);
  p.load_transfer_tau = params.get_double("vehicle.suspension.load_transfer_tau_s", p.load_transfer_tau);
  p.steering_ratio = params.get_double("vehicle.steering.ratio");
  p.max_steer_wheel = deg_to_rad(params.get_double("vehicle.steering.lock_deg"));
  p.ackermann = params.get_double("vehicle.steering.ackermann", p.ackermann);
  p.mechanical_trail = params.get_double("vehicle.steering.mechanical_trail_m", p.mechanical_trail);
  p.wheel_radius = params.get_double("vehicle.wheel.radius_m");
  p.wheel_inertia = params.get_double("vehicle.wheel.inertia_kgm2");
  p.rolling_resistance = params.get_double("vehicle.tire.rolling_resistance", p.rolling_resistance);
  p.relaxation_x = params.get_double("vehicle.tire.relaxation_length_x_m", p.relaxation_x);
  p.relaxation_y = params.get_double("vehicle.tire.relaxation_length_y_m", p.relaxation_y);
  p.low_speed_damping = params.get_double("vehicle.tire.low_speed_damping_ratio", p.low_speed_damping);
  p.low_speed_fade = params.get_double("vehicle.tire.low_speed_fade_mps", p.low_speed_fade);
  p.cla = params.get_double("vehicle.aero.cla_m2", p.cla);
  p.cda = params.get_double("vehicle.aero.cda_m2", p.cda);
  p.aero_front_ratio = params.get_double("vehicle.aero.front_ratio", p.aero_front_ratio);
  p.air_density = params.get_double("vehicle.aero.air_density_kgm3", p.air_density);
  p.brake_pressure_max = params.get_double("vehicle.brakes.max_pressure_bar");
  p.brake_gain_front = params.get_double("vehicle.brakes.front_gain_nm_per_bar");
  p.brake_gain_rear = params.get_double("vehicle.brakes.rear_gain_nm_per_bar");
  p.ambient_temp = params.get_double("environment.ambient_temp_c", p.ambient_temp);

  require_positive(p.mass, "vehicle.mass_kg");
  require_positive(p.yaw_inertia, "vehicle.yaw_inertia_kgm2");
  require_positive(p.wheelbase, "vehicle.wheelbase_m");
  require_positive(p.track_front, "vehicle.track_front_m");
  require_positive(p.track_rear, "vehicle.track_rear_m");
  require_positive(p.load_transfer_tau, "vehicle.suspension.load_transfer_tau_s");
  require_positive(p.steering_ratio, "vehicle.steering.ratio");
  require_positive(p.wheel_radius, "vehicle.wheel.radius_m");
  require_positive(p.wheel_inertia, "vehicle.wheel.inertia_kgm2");
  require_positive(p.relaxation_x, "vehicle.tire.relaxation_length_x_m");
  require_positive(p.relaxation_y, "vehicle.tire.relaxation_length_y_m");
  require_positive(p.low_speed_fade, "vehicle.tire.low_speed_fade_mps");
  if (!(p.cg_to_front > 0.0 && p.cg_to_front < p.wheelbase)) {
    throw std::invalid_argument("vehicle.cg_to_front_axle_m must lie between 0 and the wheelbase");
  }
  if (!(p.roll_stiffness_front >= 0.0 && p.roll_stiffness_front <= 1.0)) {
    throw std::invalid_argument("vehicle.suspension.roll_stiffness_front_ratio must be in [0, 1]");
  }
  if (!(p.ackermann >= 0.0 && p.ackermann <= 1.0)) {
    throw std::invalid_argument("vehicle.steering.ackermann must be in [0, 1]");
  }
  return p;
}

void TwoTrackModel::init(const ParamTree& params, Context& ctx) {
  p_ = params_from(params);
  motor_ = MotorParams::from_params(params);
  battery_ = BatteryParams::from_params(params);
  driveline_config_ = DrivelineConfig::from_params(params);
  driveline_.init(driveline_config_, p_.wheel_inertia, motor_.rotor_inertia);
  const std::filesystem::path project_dir = path_from_utf8(params.get_string("project.dir", "."));
  tires_[0] = make_tire(params, "front", project_dir);
  tires_[1] = make_tire(params, "rear", project_dir);

  const std::size_t motors = driveline_.motor_count();
  wheel_offset_ = kChassisSize;
  slip_offset_ = wheel_offset_ + driveline_.dof_count();
  motor_offset_ = slip_offset_ + 2 * kCorners;
  battery_offset_ = motor_offset_ + 2 * motors;
  energy_offset_ = battery_offset_ + 4;
  size_ = energy_offset_ + kEnergyCount;
  torque_cmd_.assign(motors, 0.0);

  x0_ = params.get_double("initial.x_m", 0.0);
  y0_ = params.get_double("initial.y_m", 0.0);
  yaw0_ = deg_to_rad(params.get_double("initial.yaw_deg", 0.0));
  v0_ = params.get_double("initial.speed_mps", 0.0);
  soc0_ = params.get_double("initial.soc", soc0_);
  battery_temp0_ = params.get_double("initial.battery_temp_c", p_.ambient_temp);
  motor_temp0_ = params.get_double("initial.motor_temp_c", motor_.coolant_temp);
  if (!(soc0_ >= 0.0 && soc0_ <= 1.0)) {
    throw std::invalid_argument("initial.soc must be in [0, 1]");
  }
  std::vector<double> initial(size_);
  initial_state(initial);
  kinetic0_ = kinetic_energy(initial);

  SignalBus& bus = ctx.bus();
  sig_ = std::make_unique<Signals>();
  Signals& s = *sig_;
  s.steer = bus.declare("drv.cmd.steer", "rad", "steering wheel angle");
  s.brake = bus.declare("drv.cmd.brake", "-", "brake pedal");

  s.x = bus.declare("veh.chassis.x", "m", "position east");
  s.y = bus.declare("veh.chassis.y", "m", "position north");
  s.yaw = bus.declare("veh.chassis.yaw", "rad", "heading, counter-clockwise from east");
  s.vx = bus.declare("veh.chassis.vx", "m/s", "longitudinal velocity (body frame)");
  s.vy = bus.declare("veh.chassis.vy", "m/s", "lateral velocity (body frame)");
  s.r = bus.declare("veh.chassis.yaw_rate", "rad/s", "yaw rate");
  s.speed = bus.declare("veh.chassis.speed", "m/s", "speed over ground");
  s.ax = bus.declare("veh.chassis.ax", "m/s^2", "longitudinal acceleration (body frame)");
  s.ay = bus.declare("veh.chassis.ay", "m/s^2", "lateral acceleration (body frame)");
  s.beta = bus.declare("veh.chassis.beta", "rad", "body slip angle");
  s.delta = bus.declare("veh.steer.road_wheel_angle", "rad", "road wheel angle of the steering input");
  for (std::size_t c = 0; c < kCorners; ++c) {
    const std::string base = "veh.wheel." + std::string(kCornerNames[c]) + ".";
    s.wheel[c] = {bus.declare(base + "fz", "N", "normal load"),
                  bus.declare(base + "fx", "N", "longitudinal tire force (wheel frame)"),
                  bus.declare(base + "fy", "N", "lateral tire force (wheel frame)"),
                  bus.declare(base + "mz", "N*m", "aligning torque"),
                  bus.declare(base + "slip_ratio", "-", "transient slip ratio"),
                  bus.declare(base + "slip_angle", "rad", "transient slip angle"),
                  bus.declare(base + "omega", "rad/s", "wheel speed"),
                  bus.declare(base + "steer", "rad", "road wheel angle")};
  }
  for (std::size_t a = 0; a < 2; ++a) {
    const std::string base = std::string("veh.axle.") + (a == 0 ? "front." : "rear.");
    s.axle[a] = {bus.declare(base + "fz", "N", "axle normal load"),
                 bus.declare(base + "fx", "N", "axle longitudinal tire force"),
                 bus.declare(base + "fy", "N", "axle lateral tire force"),
                 bus.declare(base + "slip_angle", "rad", "mean slip angle")};
  }
  s.drag = bus.declare("veh.aero.drag", "N", "aerodynamic drag");
  s.downforce = bus.declare("veh.aero.downforce", "N", "aerodynamic downforce");
  s.brake_pressure = bus.declare("veh.brake.pressure", "bar", "brake line pressure");
  s.ffb = bus.declare("veh.ffb.torque_sw", "N*m",
                      "road torque at the steering wheel, positive counter-clockwise");
  s.motor.clear();
  for (const std::string& id : driveline_config_.motor_ids()) {
    const std::string base = "pt.motor." + id + ".";
    s.motor.push_back({bus.declare(base + "torque_cmd", "N*m", "motor torque command"),
                       bus.declare(base + "torque", "N*m", "motor torque delivered"),
                       bus.declare(base + "speed", "rad/s", "motor shaft speed"),
                       bus.declare(base + "torque_avail", "N*m", "torque envelope at this speed"),
                       bus.declare(base + "temp", "degC", "winding temperature"),
                       bus.declare(base + "loss", "W", "motor and inverter losses"),
                       bus.declare(base + "power_elec", "W", "DC power drawn")});
  }
  s.power_mech = bus.declare("pt.power_mech", "W", "total motor shaft power");
  s.power_elec = bus.declare("pt.power_elec", "W", "battery terminal power, including aux");
  s.soc = bus.declare("bat.soc", "-", "state of charge");
  s.voltage = bus.declare("bat.voltage", "V", "terminal voltage");
  s.current = bus.declare("bat.current", "A", "current, positive discharging");
  s.power = bus.declare("bat.power", "W", "terminal power, positive discharging");
  s.ocv = bus.declare("bat.ocv", "V", "open-circuit voltage");
  s.battery_temp = bus.declare("bat.temp", "degC", "cell temperature");
  s.overloaded = bus.declare("bat.overloaded", "-", "1 when demand exceeds the pack's maximum power");
  s.limit_discharge_power = bus.declare("bat.limit.discharge_power", "W", "BMS discharge power limit");
  s.limit_charge_power = bus.declare("bat.limit.charge_power", "W", "BMS charge power limit");
  s.limit_discharge_current = bus.declare("bat.limit.discharge_current", "A", "BMS discharge current limit");
  s.limit_charge_current = bus.declare("bat.limit.charge_current", "A", "BMS charge current limit");
  constexpr std::array<const char*, kEnergyCount> kEnergyNames = {
      "battery_chemical", "battery_loss", "aux", "motor_loss", "motor_shaft", "gear_loss",
      "diff_loss", "brake_loss", "slip_loss", "rolling_loss", "drag_loss"};
  for (std::size_t i = 0; i < kEnergyCount; ++i) {
    s.energy[i] = bus.declare(std::string("energy.") + kEnergyNames[i], "J", "integrated energy");
  }
  s.kinetic = bus.declare("energy.kinetic", "J", "chassis and rotating kinetic energy");
  s.rc_stored = bus.declare("energy.battery_rc_stored", "J", "energy in the cell RC elements");
  s.residual = bus.declare("energy.residual", "J",
                           "chemical energy minus losses and stored energy gain (should stay ~0)");
}

void TwoTrackModel::initial_state(std::span<double> x) const {
  std::fill(x.begin(), x.end(), 0.0);
  x[kX] = x0_;
  x[kY] = y0_;
  x[kYaw] = yaw0_;
  x[kVx] = v0_;
  for (std::size_t dof = 0; dof < driveline_.dof_count(); ++dof) {
    x[wheel_offset_ + dof] = v0_ / p_.wheel_radius;
  }
  for (std::size_t m = 0; m < driveline_.motor_count(); ++m) {
    x[motor_offset_ + 2 * m + 1] = motor_temp0_;
  }
  x[battery_offset_] = soc0_;
  x[battery_offset_ + 3] = battery_temp0_;
}

void TwoTrackModel::read_inputs(const SignalBus& bus) {
  steer_ = bus.get(sig_->steer);
  brake_ = std::clamp(bus.get(sig_->brake), 0.0, 1.0);
  for (std::size_t m = 0; m < torque_cmd_.size(); ++m) {
    torque_cmd_[m] = bus.get(sig_->motor[m].torque_cmd);
  }
}

void TwoTrackModel::evaluate(std::span<const double> x, Eval& e, std::span<double> dx) const {
  const double vx = x[kVx];
  const double vy = x[kVy];
  const double r = x[kYawRate];
  const double yaw = x[kYaw];
  const double m = p_.mass;
  const double lf = p_.cg_to_front;
  const double lr = p_.cg_to_rear();
  const double wheelbase = p_.wheelbase;
  const double radius = p_.wheel_radius;
  const std::span<const double> q = x.subspan(wheel_offset_, driveline_.dof_count());

  // Steering with partial Ackermann.
  e.delta = std::clamp(steer_, -p_.max_steer_wheel, p_.max_steer_wheel) / p_.steering_ratio;
  const double tan_delta = std::tan(e.delta);
  const double half_front = 0.5 * p_.track_front;
  const double ackermann_left = std::atan2(wheelbase * tan_delta, wheelbase - tan_delta * half_front);
  const double ackermann_right = std::atan2(wheelbase * tan_delta, wheelbase + tan_delta * half_front);
  e.steer = {e.delta + p_.ackermann * (ackermann_left - e.delta),
             e.delta + p_.ackermann * (ackermann_right - e.delta), 0.0, 0.0};

  // Aero.
  const double speed_sq = vx * vx + vy * vy;
  const double speed = std::sqrt(speed_sq);
  const double dynamic_pressure = 0.5 * p_.air_density * speed_sq;
  e.downforce = p_.cla * dynamic_pressure;
  e.drag = p_.cda * dynamic_pressure;

  // Normal loads: static + aero + quasi-static transfer from lagged ax, ay.
  const double ax_lag = x[kAxFiltered];
  const double ay_lag = x[kAyFiltered];
  const double h = p_.cg_height;
  const double long_transfer = m * ax_lag * h / wheelbase;
  const double roll_axis = p_.roll_center_front + (p_.roll_center_rear - p_.roll_center_front) * lf / wheelbase;
  const double lat_front = m * ay_lag *
                           (lr / wheelbase * p_.roll_center_front + (h - roll_axis) * p_.roll_stiffness_front) /
                           p_.track_front;
  const double lat_rear = m * ay_lag *
                          (lf / wheelbase * p_.roll_center_rear + (h - roll_axis) * (1.0 - p_.roll_stiffness_front)) /
                          p_.track_rear;
  for (std::size_t c = 0; c < kCorners; ++c) {
    const bool front = is_front(c);
    const double axle_static = m * kGravity * (front ? lr : lf) / wheelbase;
    const double axle_aero = e.downforce * (front ? p_.aero_front_ratio : 1.0 - p_.aero_front_ratio);
    const double lateral = (front ? lat_front : lat_rear) * (is_left(c) ? -1.0 : 1.0);
    const double longitudinal = 0.5 * long_transfer * (front ? -1.0 : 1.0);
    e.fz[c] = std::max(0.0, 0.5 * (axle_static + axle_aero) + longitudinal + lateral);
  }

  // Battery open-circuit state and motor envelopes.
  const double soc = x[battery_offset_];
  const double rc1 = x[battery_offset_ + 1];
  const double rc2 = x[battery_offset_ + 2];
  const double battery_temp = x[battery_offset_ + 3];
  const double cell_ocv = battery_.ocv_cell(soc);
  e.ocv = battery_.series * cell_ocv;
  const double open_voltage = battery_.series * (cell_ocv - rc1 - rc2);
  const double resistance = battery_.pack_resistance();
  const std::size_t motors = driveline_.motor_count();

  const auto motor_demand = [&](double dc_voltage) {
    double total = battery_.aux_power;
    for (std::size_t i = 0; i < motors; ++i) {
      const double temp = x[motor_offset_ + 2 * i + 1];
      e.motor_speed[i] = driveline_.motor_speed(i, q);
      e.motor_avail[i] = motor_.envelope(e.motor_speed[i], dc_voltage, temp);
      e.motor_torque[i] = std::clamp(x[motor_offset_ + 2 * i], -e.motor_avail[i], e.motor_avail[i]);
      e.motor_loss[i] = motor_.losses(e.motor_torque[i], e.motor_speed[i]);
      e.motor_power[i] = e.motor_torque[i] * e.motor_speed[i] + e.motor_loss[i];
      total += e.motor_power[i];
    }
    return total;
  };
  // The DC voltage sags with current; one fixed-point pass is enough because
  // it only matters where the envelope is voltage limited.
  const BatteryOperatingPoint first = solve_battery_power(open_voltage, resistance, motor_demand(open_voltage));
  e.power_elec = motor_demand(first.voltage);
  e.battery = solve_battery_power(open_voltage, resistance, e.power_elec);
  e.limits = battery_limits(battery_, soc, rc1 + rc2, battery_temp);
  e.power_mech = 0.0;
  for (std::size_t i = 0; i < motors; ++i) {
    e.power_mech += e.motor_torque[i] * e.motor_speed[i];
  }

  // Tires with transient slip.
  e.brake_pressure = brake_ * p_.brake_pressure_max;
  Driveline::Loads loads;
  loads.motor_torque = std::span<const double>(e.motor_torque.data(), motors);
  double fx_body = 0.0, fy_body = 0.0, yaw_moment = 0.0;
  double slip_power = 0.0, rolling_power = 0.0;
  e.ffb = 0.0;
  for (std::size_t c = 0; c < kCorners; ++c) {
    const bool front = is_front(c);
    const double px = front ? lf : -lr;
    const double py = (is_left(c) ? 0.5 : -0.5) * p_.track(c);
    const double cos_s = std::cos(e.steer[c]);
    const double sin_s = std::sin(e.steer[c]);
    const double vx_body = vx - r * py;
    const double vy_body = vy + r * px;
    const double vx_wheel = cos_s * vx_body + sin_s * vy_body;
    const double vy_wheel = -sin_s * vx_body + cos_s * vy_body;
    e.omega[c] = driveline_.wheel_speed(c, q);
    const double slip_velocity = e.omega[c] * radius - vx_wheel;

    const double kappa = x[slip_offset_ + 2 * c];
    const double tan_alpha = x[slip_offset_ + 2 * c + 1];
    if (!dx.empty()) {
      dx[slip_offset_ + 2 * c] = slip_rate(kappa, slip_velocity, vx_wheel, p_.relaxation_x, kMaxSlipRatio);
      dx[slip_offset_ + 2 * c + 1] =
          slip_rate(tan_alpha, -vy_wheel, vx_wheel, p_.relaxation_y, kMaxTanSlipAngle);
    }

    // Near standstill the relaxation states alone are undamped springs; add
    // slip proportional to the slip velocity (fading out with speed) so the
    // tire modes settle instead of ringing.
    const TireModel& tire = *tires_[front ? 0 : 1];
    double kappa_used = kappa;
    double tan_alpha_used = tan_alpha;
    const double fade = std::max(0.0, 1.0 - std::abs(vx_wheel) / p_.low_speed_fade);
    if (fade > 0.0 && e.fz[c] > 0.0) {
      const double zeta = p_.low_speed_damping;
      const double c_kappa = tire.longitudinal_stiffness(e.fz[c]);
      const double c_alpha = tire.cornering_stiffness(e.fz[c]);
      const double wheel_mass = p_.wheel_inertia / (radius * radius);
      kappa_used += fade * 2.0 * zeta * std::sqrt(wheel_mass / (p_.relaxation_x * c_kappa)) * slip_velocity;
      tan_alpha_used -= fade * 2.0 * zeta * std::sqrt(0.25 * m / (p_.relaxation_y * c_alpha)) * vy_wheel;
    }
    const TireForces f = tire.evaluate({e.fz[c], kappa_used, std::atan(tan_alpha_used)});
    e.fx[c] = f.fx;
    e.fy[c] = f.fy;
    e.mz[c] = f.mz;
    e.slip_ratio[c] = kappa;
    e.slip_angle[c] = std::atan(tan_alpha);

    const double fx_b = cos_s * f.fx - sin_s * f.fy;
    const double fy_b = sin_s * f.fx + cos_s * f.fy;
    fx_body += fx_b;
    fy_body += fy_b;
    yaw_moment += px * fy_b - py * fx_b + f.mz;
    slip_power += f.fx * slip_velocity - f.fy * vy_wheel - f.mz * r;

    const double rolling = p_.rolling_resistance * e.fz[c] * radius * std::tanh(e.omega[c] / kRollingSmoothing);
    rolling_power += rolling * e.omega[c];
    loads.wheel_torque[c] = -f.fx * radius - rolling;
    loads.brake_capacity[c] = 0.5 * e.brake_pressure * (front ? p_.brake_gain_front : p_.brake_gain_rear);
    if (front) {
      e.ffb += (f.mz - p_.mechanical_trail * f.fy) / p_.steering_ratio;
    }
  }

  // Aero drag along the velocity.
  const double drag_scale = speed > 0.0 ? e.drag / speed : 0.0;
  fx_body -= drag_scale * vx;
  fy_body -= drag_scale * vy;
  e.ax = fx_body / m;
  e.ay = fy_body / m;
  if (dx.empty()) {
    return;
  }

  dx[kX] = vx * std::cos(yaw) - vy * std::sin(yaw);
  dx[kY] = vx * std::sin(yaw) + vy * std::cos(yaw);
  dx[kYaw] = r;
  dx[kVx] = e.ax + vy * r;
  dx[kVy] = e.ay - vx * r;
  dx[kYawRate] = yaw_moment / p_.yaw_inertia;
  dx[kAxFiltered] = (e.ax - ax_lag) / p_.load_transfer_tau;
  dx[kAyFiltered] = (e.ay - ay_lag) / p_.load_transfer_tau;

  Driveline::Losses losses;
  driveline_.accelerations(q, loads, dx.subspan(wheel_offset_, driveline_.dof_count()), losses);

  for (std::size_t i = 0; i < motors; ++i) {
    const double torque = x[motor_offset_ + 2 * i];
    const double temp = x[motor_offset_ + 2 * i + 1];
    const double target = std::clamp(torque_cmd_[i], -e.motor_avail[i], e.motor_avail[i]);
    dx[motor_offset_ + 2 * i] = (target - torque) / motor_.torque_time_constant;
    dx[motor_offset_ + 2 * i + 1] =
        (e.motor_loss[i] - (temp - motor_.coolant_temp) / motor_.thermal_resistance) / motor_.heat_capacity;
  }

  // Battery: charge, RC polarisation, heat.
  const double current = e.battery.current;
  const double cell_current = current / battery_.parallel;
  const double cells = static_cast<double>(battery_.series * battery_.parallel);
  dx[battery_offset_] = -current / (3600.0 * battery_.cell_capacity_ah * battery_.parallel);
  double rc_loss = 0.0;
  const std::array<double, 2> rc = {rc1, rc2};
  for (std::size_t k = 0; k < 2; ++k) {
    if (k < battery_.rc.size()) {
      const BatteryParams::RcPair& pair = battery_.rc[k];
      dx[battery_offset_ + 1 + k] = cell_current / pair.capacitance - rc[k] / (pair.resistance * pair.capacitance);
      rc_loss += cells * rc[k] * rc[k] / pair.resistance;
    }
  }
  const double battery_loss = current * current * resistance + rc_loss;
  dx[battery_offset_ + 3] =
      (battery_loss - battery_.cooling * (battery_temp - p_.ambient_temp)) / battery_.heat_capacity;

  double motor_loss = 0.0;
  for (std::size_t i = 0; i < motors; ++i) {
    motor_loss += e.motor_loss[i];
  }
  const std::span<double> energy = dx.subspan(energy_offset_, kEnergyCount);
  energy[kBatteryChemical] = e.ocv * current;
  energy[kBatteryLoss] = battery_loss;
  energy[kAux] = battery_.aux_power;
  energy[kMotorLoss] = motor_loss;
  energy[kMotorShaft] = e.power_mech;
  energy[kGearLoss] = losses.gear;
  energy[kDiffLoss] = losses.diff;
  energy[kBrakeLoss] = losses.brake;
  energy[kSlipLoss] = slip_power;
  energy[kRollingLoss] = rolling_power;
  energy[kDragLoss] = e.drag * speed;
}

double TwoTrackModel::kinetic_energy(std::span<const double> x) const {
  return 0.5 * p_.mass * (x[kVx] * x[kVx] + x[kVy] * x[kVy]) +
         0.5 * p_.yaw_inertia * x[kYawRate] * x[kYawRate] +
         driveline_.kinetic_energy(x.subspan(wheel_offset_, driveline_.dof_count()));
}

void TwoTrackModel::derivatives(double /*t*/, std::span<const double> x, std::span<double> dx) const {
  Eval e;
  evaluate(x, e, dx);
}

void TwoTrackModel::write_outputs(double /*t*/, std::span<const double> x, SignalBus& bus) const {
  Eval e;
  evaluate(x, e, {});
  const Signals& s = *sig_;
  const double speed = std::hypot(x[kVx], x[kVy]);
  bus.set(s.x, x[kX]);
  bus.set(s.y, x[kY]);
  bus.set(s.yaw, x[kYaw]);
  bus.set(s.vx, x[kVx]);
  bus.set(s.vy, x[kVy]);
  bus.set(s.r, x[kYawRate]);
  bus.set(s.speed, speed);
  bus.set(s.ax, e.ax);
  bus.set(s.ay, e.ay);
  bus.set(s.beta, speed > 0.5 ? std::atan2(x[kVy], x[kVx]) : 0.0);
  bus.set(s.delta, e.delta);

  std::array<double, 2> axle_fz{}, axle_fx{}, axle_fy{}, axle_alpha{};
  for (std::size_t c = 0; c < kCorners; ++c) {
    const Signals::Wheel& w = s.wheel[c];
    bus.set(w.fz, e.fz[c]);
    bus.set(w.fx, e.fx[c]);
    bus.set(w.fy, e.fy[c]);
    bus.set(w.mz, e.mz[c]);
    bus.set(w.slip_ratio, e.slip_ratio[c]);
    bus.set(w.slip_angle, e.slip_angle[c]);
    bus.set(w.omega, e.omega[c]);
    bus.set(w.steer, e.steer[c]);
    const std::size_t a = is_front(c) ? 0 : 1;
    axle_fz[a] += e.fz[c];
    axle_fx[a] += e.fx[c];
    axle_fy[a] += e.fy[c];
    axle_alpha[a] += 0.5 * e.slip_angle[c];
  }
  for (std::size_t a = 0; a < 2; ++a) {
    bus.set(s.axle[a].fz, axle_fz[a]);
    bus.set(s.axle[a].fx, axle_fx[a]);
    bus.set(s.axle[a].fy, axle_fy[a]);
    bus.set(s.axle[a].slip_angle, axle_alpha[a]);
  }
  bus.set(s.drag, e.drag);
  bus.set(s.downforce, e.downforce);
  bus.set(s.brake_pressure, e.brake_pressure);
  bus.set(s.ffb, e.ffb);

  for (std::size_t i = 0; i < s.motor.size(); ++i) {
    const Signals::Motor& motor = s.motor[i];
    bus.set(motor.torque, e.motor_torque[i]);
    bus.set(motor.speed, e.motor_speed[i]);
    bus.set(motor.avail, e.motor_avail[i]);
    bus.set(motor.temp, x[motor_offset_ + 2 * i + 1]);
    bus.set(motor.loss, e.motor_loss[i]);
    bus.set(motor.power, e.motor_power[i]);
  }
  bus.set(s.power_mech, e.power_mech);
  bus.set(s.power_elec, e.power_elec);

  bus.set(s.soc, x[battery_offset_]);
  bus.set(s.voltage, e.battery.voltage);
  bus.set(s.current, e.battery.current);
  bus.set(s.power, e.battery.voltage * e.battery.current);
  bus.set(s.ocv, e.ocv);
  bus.set(s.battery_temp, x[battery_offset_ + 3]);
  bus.set(s.overloaded, e.battery.limited ? 1.0 : 0.0);
  bus.set(s.limit_discharge_power, e.limits.discharge_power);
  bus.set(s.limit_charge_power, e.limits.charge_power);
  bus.set(s.limit_discharge_current, e.limits.discharge_current);
  bus.set(s.limit_charge_current, e.limits.charge_current);

  // Energy bookkeeping (V-03).
  const std::span<const double> energy = x.subspan(energy_offset_, kEnergyCount);
  for (std::size_t i = 0; i < kEnergyCount; ++i) {
    bus.set(s.energy[i], energy[i]);
  }
  const double kinetic_now = kinetic_energy(x);
  const double kinetic_gain = kinetic_now - kinetic0_;
  double rc_stored = 0.0;
  const double cells = static_cast<double>(battery_.series * battery_.parallel);
  for (std::size_t k = 0; k < battery_.rc.size(); ++k) {
    const double v = x[battery_offset_ + 1 + k];
    rc_stored += cells * 0.5 * battery_.rc[k].capacitance * v * v;
  }
  bus.set(s.kinetic, kinetic_now);
  bus.set(s.rc_stored, rc_stored);
  const double sinks = energy[kBatteryLoss] + energy[kAux] + energy[kMotorLoss] + energy[kGearLoss] +
                       energy[kDiffLoss] + energy[kBrakeLoss] + energy[kSlipLoss] + energy[kRollingLoss] +
                       energy[kDragLoss];
  bus.set(s.residual, energy[kBatteryChemical] - sinks - kinetic_gain - rc_stored);
}

}  // namespace vsim

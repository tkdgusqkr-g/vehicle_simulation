#include "vsim/models/vehicle/two_track.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "vsim/elec/network.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/track/track.hpp"
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

// Progressive bump stop: k e (1 + e / kBumpProgression) and its energy.
constexpr double kBumpProgression = 0.005;  // [m]
double bump_force(double rate, double e) { return e > 0.0 ? rate * e * (1.0 + e / kBumpProgression) : 0.0; }
double bump_energy(double rate, double e) {
  return e > 0.0 ? rate * (0.5 * e * e + e * e * e / (3.0 * kBumpProgression)) : 0.0;
}
// Droop limit: a stiff linear stop.
constexpr double kDroopStopRate = 2e6;  // [N/m]

// Digressive damper at the wheel: bump / rebound slopes up to the knee, a
// lower slope above it.
double damper_force(const TwoTrackModel::Params::Axle& a, double v) {
  const double c = v >= 0.0 ? a.damper_bump : a.damper_rebound;
  const double speed = std::abs(v);
  const double f = speed <= a.damper_knee ? c * speed : c * (a.damper_knee + a.damper_high_speed * (speed - a.damper_knee));
  return v >= 0.0 ? f : -f;
}

// Solves the 5x5 system m u = b with m given by its inverse (row major).
std::array<double, 5> apply(const std::array<double, 25>& inverse, const std::array<double, 5>& b) {
  std::array<double, 5> u{};
  for (std::size_t i = 0; i < 5; ++i) {
    for (std::size_t j = 0; j < 5; ++j) {
      u[i] += inverse[5 * i + j] * b[j];
    }
  }
  return u;
}

std::array<double, 25> invert5(std::array<double, 25> a) {
  std::array<double, 25> inv{};
  for (std::size_t i = 0; i < 5; ++i) {
    inv[6 * i] = 1.0;
  }
  for (std::size_t c = 0; c < 5; ++c) {
    std::size_t pivot = c;
    for (std::size_t r = c + 1; r < 5; ++r) {
      if (std::abs(a[5 * r + c]) > std::abs(a[5 * pivot + c])) {
        pivot = r;
      }
    }
    for (std::size_t k = 0; k < 5; ++k) {
      std::swap(a[5 * c + k], a[5 * pivot + k]);
      std::swap(inv[5 * c + k], inv[5 * pivot + k]);
    }
    const double d = a[5 * c + c];
    if (std::abs(d) < 1e-12) {
      throw std::invalid_argument("vehicle mass matrix is singular (check masses and inertias)");
    }
    for (std::size_t k = 0; k < 5; ++k) {
      a[5 * c + k] /= d;
      inv[5 * c + k] /= d;
    }
    for (std::size_t r = 0; r < 5; ++r) {
      if (r != c) {
        const double f = a[5 * r + c];
        for (std::size_t k = 0; k < 5; ++k) {
          a[5 * r + k] -= f * a[5 * c + k];
          inv[5 * r + k] -= f * inv[5 * c + k];
        }
      }
    }
  }
  return inv;
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
  double airspeed = 0.0, aero_yaw = 0.0;      // through the air; flow angle (air from the left +)
  double aero_fx = 0.0, aero_fy = 0.0, aero_moment = 0.0;  // body frame, about the CG
  double aero_side = 0.0;                     // the crosswind part of aero_fy
  double ax = 0.0, ay = 0.0;
  double yaw_accel = 0.0;  // [rad/s^2]
  double ffb = 0.0;
  double brake_pressure = 0.0;
  // Thermal (PH-13, PH-17).
  std::array<double, kCorners> tire_grip{1.0, 1.0, 1.0, 1.0}, tire_pressure{}, brake_mu{1.0, 1.0, 1.0, 1.0};
  std::array<double, kCorners> slip_heat{}, rolling_heat{};

  std::array<double, kCorners> motor_torque{}, motor_speed{}, motor_avail{};
  std::array<double, kCorners> motor_loss{}, motor_power{};
  double power_mech = 0.0;
  double power_elec = 0.0;  // at the battery terminals, including aux
  double motor_power_sum = 0.0;  // DC power of the motors
  double open_voltage = 0.0;     // pack, behind its resistance

  double ocv = 0.0;  // pack
  BatteryOperatingPoint battery;
  BatteryLimits limits;

  // Full model.
  double rack = 0.0;
  std::array<double, kCorners> travel{}, travel_rate{}, camber{}, mu{1.0, 1.0, 1.0, 1.0};
  std::array<double, kCorners> road_z{}, deflection{}, spring_force{}, damper_force{}, arb_force{};
  std::array<double, kCorners> wheel_z{}, kingpin_moment{}, spring_compression{};
  std::array<int, kCorners> surface{};
  double az = 0.0;  // vertical acceleration of the sprung CG
  double roll_accel = 0.0, pitch_accel = 0.0;
};

struct TwoTrackModel::Signals {
  struct Wheel {
    SignalId fz, fx, fy, mz, slip_ratio, slip_angle, omega, steer;
    SignalId tire_temp, tire_core_temp, tire_pressure, tire_grip, brake_temp, brake_mu;
  };
  struct Axle {
    SignalId fz, fx, fy, slip_angle;
  };
  struct Motor {
    SignalId torque_cmd, torque, speed, avail, temp, loss, power;
  };

  struct Corner3 {
    SignalId z, travel, camber, spring, damper, deflection, road_z, mu, surface, rig_z, rig_vz, hub_offset;
  };

  SignalId steer, brake;
  SignalId x, y, yaw, vx, vy, r, speed, ax, ay, beta, delta, yaw_accel;
  std::array<Wheel, kCorners> wheel;
  // Full model.
  SignalId z, roll, pitch, vz, roll_rate, pitch_rate, roll_accel, pitch_accel, az, rack, potential, road_z, heave;
  std::array<Corner3, kCorners> corner3;
  std::array<Axle, 2> axle;
  SignalId drag, downforce, brake_pressure, brake_balance, ffb;
  SignalId wind_x, wind_y, airspeed, aero_yaw, aero_side, aero_moment;
  std::vector<Motor> motor;
  SignalId power_mech, power_elec;
  SignalId soc, voltage, current, power, ocv, open_voltage, battery_temp, overloaded;
  SignalId cell_v_min, cell_v_max, soc_min, soc_max, weak_group;
  // Electrical network inputs (when it carries the accumulator).
  SignalId elec_current = 0;
  std::vector<SignalId> elec_dc, elec_enabled;
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
  p.side_force_area = params.get_double("vehicle.aero.side_force_area_m2_per_rad", p.side_force_area);
  p.side_force_x = params.get_double("vehicle.aero.side_force_x_m", p.side_force_x);
  p.brake_pressure_max = params.get_double("vehicle.brakes.max_pressure_bar");
  p.brake_gain_front = params.get_double("vehicle.brakes.front_gain_nm_per_bar");
  p.brake_gain_rear = params.get_double("vehicle.brakes.rear_gain_nm_per_bar");
  p.ambient_temp = params.get_double("environment.ambient_temp_c", p.ambient_temp);

  // Full model (read always, checked when used).
  for (std::size_t c = 0; c < kCorners; ++c) {
    p.unsprung_mass[c] = params.get_double("vehicle.unsprung_mass_kg." + std::string(kCornerNames[c]),
                                           params.get_double("vehicle.unsprung_mass_kg", p.unsprung_mass[c]));
  }
  p.roll_inertia = params.get_double("vehicle.roll_inertia_kgm2", p.roll_inertia);
  p.pitch_inertia = params.get_double("vehicle.pitch_inertia_kgm2", p.pitch_inertia);
  p.unloaded_radius = params.get_double("vehicle.wheel.unloaded_radius_m", 0.0);
  p.tire_vertical_stiffness =
      1000.0 * params.get_double("vehicle.tire.vertical_stiffness_n_per_mm", p.tire_vertical_stiffness / 1000.0);
  p.tire_vertical_damping = params.get_double("vehicle.tire.vertical_damping_n_s_per_m", p.tire_vertical_damping);
  p.contact_length = params.get_double("vehicle.tire.contact_length_m", p.contact_length);
  for (std::size_t a = 0; a < 2; ++a) {
    const std::string base = std::string("vehicle.suspension.") + (a == 0 ? "front." : "rear.");
    Params::Axle& x = p.axle[a];
    x.spring_rate = 1000.0 * params.get_double(base + "spring_rate_n_per_mm", x.spring_rate / 1000.0);
    x.damper_bump = params.get_double(base + "damper_n_s_per_m", x.damper_bump);
    x.damper_rebound = params.get_double(base + "damper_rebound_n_s_per_m", x.damper_bump);
    x.damper_knee = params.get_double(base + "damper_knee_mps", x.damper_knee);
    x.damper_high_speed = params.get_double(base + "damper_high_speed_ratio", x.damper_high_speed);
    x.anti_roll_bar = params.get_double(base + "anti_roll_bar_nm_per_deg", x.anti_roll_bar);
    x.bump_travel = params.get_double(base + "bump_travel_m", x.bump_travel);
    x.droop_travel = params.get_double(base + "droop_travel_m", x.droop_travel);
    x.bump_stop_rate = 1000.0 * params.get_double(base + "bump_stop_n_per_mm", x.bump_stop_rate / 1000.0);
  }

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
  full_ = params.get_string("vehicle.model", "two_track") == "full";
  motor_ = MotorParams::from_params(params);
  battery_ = BatteryParams::from_params(params);
  driveline_config_ = DrivelineConfig::from_params(params);
  driveline_.init(driveline_config_, p_.wheel_inertia, motor_.rotor_inertia);
  const std::filesystem::path project_dir = path_from_utf8(params.get_string("project.dir", "."));
  tires_[0] = make_tire(params, "front", project_dir);
  tires_[1] = make_tire(params, "rear", project_dir);
  tire_thermal_ = TireThermalParams::from_params(params);
  brake_thermal_ = BrakeThermalParams::from_params(params);
  road_temp_ = params.get_double("environment.road_temp_c", p_.ambient_temp);

  const std::size_t motors = driveline_.motor_count();
  wheel_offset_ = kChassisSize;
  slip_offset_ = wheel_offset_ + driveline_.dof_count();
  motor_offset_ = slip_offset_ + 2 * kCorners;
  battery_offset_ = motor_offset_ + 2 * motors;
  energy_offset_ = battery_offset_ + 4;
  body_offset_ = energy_offset_ + kEnergyCount;
  unsprung_offset_ = body_offset_ + kBodySize;
  thermal_offset_ = full_ ? unsprung_offset_ + 2 * kCorners : body_offset_;
  size_ = thermal_offset_ + 3 * kCorners;
  torque_cmd_.assign(motors, 0.0);
  network_ = ElectricalNetwork::powers_plant(params);
  dc_voltage_.assign(motors, 0.0);
  inverter_on_.assign(motors, 0.0);

  x0_ = params.get_double("initial.x_m", 0.0);
  y0_ = params.get_double("initial.y_m", 0.0);
  yaw0_ = deg_to_rad(params.get_double("initial.yaw_deg", 0.0));
  v0_ = params.get_double("initial.speed_mps", 0.0);
  soc0_ = params.get_double("initial.soc", soc0_);
  battery_temp0_ = params.get_double("initial.battery_temp_c", p_.ambient_temp);
  motor_temp0_ = params.get_double("initial.motor_temp_c", motor_.coolant_temp);
  // Tires start warm (at their optimum) unless the scenario says otherwise; brakes at ambient.
  tire_temp0_ = params.get_double("initial.tire_temp_c", tire_thermal_.enabled ? tire_thermal_.optimum : p_.ambient_temp);
  brake_temp0_ = params.get_double("initial.brake_temp_c", p_.ambient_temp);
  if (!(soc0_ >= 0.0 && soc0_ <= 1.0)) {
    throw std::invalid_argument("initial.soc must be in [0, 1]");
  }
  // Cell spread: sampled per series group when variation is real, plus a known weak group.
  spread_ = {};
  if (params.has_prefix("vehicle.battery.spread")) {
    const std::string b = "vehicle.battery.spread.";
    const double sc = params.get_double(b + "capacity_pct", 0.0) / 100.0;
    const double sr = params.get_double(b + "resistance_pct", 0.0) / 100.0;
    const double ss = params.get_double(b + "soc_pct", 0.0) / 100.0;
    if (sc < 0.0 || sr < 0.0 || ss < 0.0) {
      throw std::invalid_argument("vehicle.battery.spread: capacity_pct, resistance_pct, soc_pct >= 0");
    }
    const auto groups = static_cast<std::size_t>(battery_.series);
    spread_.capacity.assign(groups, 1.0);
    spread_.resistance.assign(groups, 1.0);
    spread_.soc_offset.assign(groups, 0.0);
    spread_.soc_start = soc0_;
    if (ctx.realism().is_real(RealismCategory::Variation)) {
      Rng rng = ctx.make_rng("battery.cells");
      for (std::size_t g = 0; g < groups; ++g) {
        spread_.capacity[g] = std::clamp(1.0 + sc * rng.normal(), 0.5, 1.5);
        spread_.resistance[g] = std::clamp(1.0 + sr * rng.normal(), 0.3, 3.0);
        spread_.soc_offset[g] = ss * rng.normal();
      }
    }
    spread_.capacity[0] = std::max(0.2, spread_.capacity[0] + params.get_double(b + "weak_group.capacity_pct", 0.0) / 100.0);
    spread_.resistance[0] = std::max(0.2, spread_.resistance[0] + params.get_double(b + "weak_group.resistance_pct", 0.0) / 100.0);
    spread_.soc_offset[0] += params.get_double(b + "weak_group.soc_pct", 0.0) / 100.0;
  }
  if (full_) {
    init_full(params, ctx);
  }
  std::vector<double> initial(size_);
  initial_state(initial);
  kinetic0_ = kinetic_energy(initial);
  stored0_ = stored_energy(initial);

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
  s.yaw_accel = bus.declare("veh.chassis.yaw_accel", "rad/s^2", "yaw acceleration");
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
                  bus.declare(base + "steer", "rad", "road wheel angle"),
                  bus.declare(base + "tire_temp", "degC", "tread surface temperature"),
                  bus.declare(base + "tire_core_temp", "degC", "carcass and inflation air temperature"),
                  bus.declare(base + "tire_pressure", "kPa", "inflation pressure (gauge)"),
                  bus.declare(base + "tire_grip", "-", "grip factor from temperature and pressure"),
                  bus.declare(base + "brake_temp", "degC", "brake disc temperature"),
                  bus.declare(base + "brake_mu", "-", "pad friction relative to nominal")};
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
  s.wind_x = bus.declare("env.wind.x", "m/s", "wind velocity east (air over the ground)");
  s.wind_y = bus.declare("env.wind.y", "m/s", "wind velocity north (air over the ground)");
  s.airspeed = bus.declare("veh.aero.airspeed", "m/s", "speed through the air");
  s.aero_yaw = bus.declare("veh.aero.yaw_angle", "rad", "flow angle, air from the left +");
  s.aero_side = bus.declare("veh.aero.side_force", "N", "aerodynamic side force (body frame, left +)");
  s.aero_moment = bus.declare("veh.aero.yaw_moment", "N*m", "aerodynamic yaw moment about the CG");
  s.brake_pressure = bus.declare("veh.brake.pressure", "bar", "brake line pressure");
  s.brake_balance = bus.declare("veh.brake.balance", "-", "front share of the brake torque capacity");
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
  s.open_voltage = bus.declare("bat.open_voltage", "V", "pack voltage behind its resistance (OCV less RC)");
  if (network_) {
    s.elec_current = bus.declare("elec.bat.current", "A", "accumulator current from the electrical network");
    s.elec_dc.clear();
    s.elec_enabled.clear();
    for (const std::string& id : driveline_config_.motor_ids()) {
      s.elec_dc.push_back(bus.declare("elec.motor." + id + ".dc_voltage", "V", "inverter DC-link voltage"));
      s.elec_enabled.push_back(bus.declare("elec.motor." + id + ".enabled", "-", "1 while the inverter can drive"));
    }
  }
  s.battery_temp = bus.declare("bat.temp", "degC", "cell temperature");
  s.overloaded = bus.declare("bat.overloaded", "-", "1 when demand exceeds the pack's maximum power");
  s.cell_v_min = bus.declare("bat.cell_v_min", "V", "lowest cell group voltage");
  s.cell_v_max = bus.declare("bat.cell_v_max", "V", "highest cell group voltage");
  s.soc_min = bus.declare("bat.soc_min", "-", "lowest cell group state of charge");
  s.soc_max = bus.declare("bat.soc_max", "-", "highest cell group state of charge");
  s.weak_group = bus.declare("bat.weak_group", "-", "series group with the lowest voltage (0-based)");
  s.limit_discharge_power = bus.declare("bat.limit.discharge_power", "W", "BMS discharge power limit");
  s.limit_charge_power = bus.declare("bat.limit.charge_power", "W", "BMS charge power limit");
  s.limit_discharge_current = bus.declare("bat.limit.discharge_current", "A", "BMS discharge current limit");
  s.limit_charge_current = bus.declare("bat.limit.charge_current", "A", "BMS charge current limit");
  constexpr std::array<const char*, kEnergyCount> kEnergyNames = {
      "battery_chemical", "battery_loss", "aux", "motor_loss", "motor_shaft", "gear_loss",
      "diff_loss", "brake_loss", "slip_loss", "rolling_loss", "drag_loss", "damper_loss",
      "tire_damping_loss"};
  for (std::size_t i = 0; i < kEnergyCount; ++i) {
    s.energy[i] = bus.declare(std::string("energy.") + kEnergyNames[i], "J", "integrated energy");
  }
  s.kinetic = bus.declare("energy.kinetic", "J", "chassis and rotating kinetic energy");
  s.rc_stored = bus.declare("energy.battery_rc_stored", "J", "energy in the cell RC elements");
  s.residual = bus.declare("energy.residual", "J",
                           "chemical energy minus losses and stored energy gain (should stay ~0)");
  if (full_) {
    s.z = bus.declare("veh.chassis.z", "m", "sprung CG height");
    s.roll = bus.declare("veh.chassis.roll", "rad", "body roll, left side up");
    s.pitch = bus.declare("veh.chassis.pitch", "rad", "body pitch, nose down");
    s.vz = bus.declare("veh.chassis.vz", "m/s", "sprung CG vertical velocity");
    s.roll_rate = bus.declare("veh.chassis.roll_rate", "rad/s", "body roll rate");
    s.pitch_rate = bus.declare("veh.chassis.pitch_rate", "rad/s", "body pitch rate");
    s.roll_accel = bus.declare("veh.chassis.roll_accel", "rad/s^2", "body roll acceleration");
    s.pitch_accel = bus.declare("veh.chassis.pitch_accel", "rad/s^2", "body pitch acceleration");
    s.az = bus.declare("veh.chassis.az", "m/s^2", "sprung CG vertical acceleration");
    s.rack = bus.declare("veh.steer.rack", "m", "steering rack travel, left +");
    s.potential = bus.declare("energy.potential", "J", "gravity, spring and tire energy gained since the start");
    s.road_z = bus.declare("veh.chassis.road_z", "m", "road height under the car (mean of the wheels)");
    s.heave = bus.declare("veh.chassis.heave", "m", "sprung CG height above the road minus its static height");
    for (std::size_t c = 0; c < kCorners; ++c) {
      const std::string name(kCornerNames[c]);
      const std::string base = "veh.wheel." + name + ".";
      s.corner3[c] = {bus.declare(base + "z", "m", "hub height"),
                      bus.declare(base + "travel", "m", "suspension travel from static, bump +"),
                      bus.declare(base + "camber", "rad", "camber, top outwards +"),
                      bus.declare(base + "spring_force", "N", "spring and stops at the wheel"),
                      bus.declare(base + "damper_force", "N", "damper at the wheel"),
                      bus.declare(base + "deflection", "m", "tire vertical deflection"),
                      bus.declare(base + "road_z", "m", "road height under the tire"),
                      bus.declare(base + "mu", "-", "road friction relative to the tire data"),
                      bus.declare(base + "surface", "-", "0 road, 1 kerb, 2 grass"),
                      bus.declare("rig." + name + ".z", "m", "four-post rig: road displacement under the wheel"),
                      bus.declare("rig." + name + ".vz", "m/s", "four-post rig: its rate"),
                      bus.declare(base + "hub_offset", "m", "hub height above the car's road level minus its static height")};
    }
  }
}

void TwoTrackModel::init_full(const ParamTree& params, Context& ctx) {
  track_ = ctx.track();
  patch_samples_ = track_ && track_->has_relief() ? 3 : 1;
  const Params& p = p_;
  double unsprung = 0.0, unsprung_x = 0.0;
  std::array<double, kCorners> x{}, y{};
  for (std::size_t c = 0; c < kCorners; ++c) {
    require_positive(p.unsprung_mass[c], "vehicle.unsprung_mass_kg");
    x[c] = is_front(c) ? p.cg_to_front : -p.cg_to_rear();
    y[c] = (is_left(c) ? 0.5 : -0.5) * p.track(c);
    unsprung += p.unsprung_mass[c];
    unsprung_x += p.unsprung_mass[c] * x[c];
  }
  sprung_mass_ = p.mass - unsprung;
  if (!(sprung_mass_ > 0.2 * p.mass)) {
    throw std::invalid_argument("vehicle.unsprung_mass_kg leaves too little sprung mass");
  }
  require_positive(p.roll_inertia, "vehicle.roll_inertia_kgm2");
  require_positive(p.pitch_inertia, "vehicle.pitch_inertia_kgm2");
  require_positive(p.tire_vertical_stiffness, "vehicle.tire.vertical_stiffness_n_per_mm");
  require_positive(p.contact_length, "vehicle.tire.contact_length_m");
  x_s_ = -unsprung_x / sprung_mass_;

  // Static loads with the car level on a flat road.
  const double g = kGravity;
  std::array<double, kCorners> fz0{};
  double hubs_moment = 0.0;
  for (std::size_t c = 0; c < kCorners; ++c) {
    fz0[c] = 0.5 * p.mass * g * (is_front(c) ? p.cg_to_rear() : p.cg_to_front) / p.wheelbase;
    const double unloaded = p.unloaded_radius > 0.0 ? p.unloaded_radius
                                                    : p.wheel_radius + fz0[c] / p.tire_vertical_stiffness;
    free_radius_[c] = unloaded;
    hub0_[c] = unloaded - fz0[c] / p.tire_vertical_stiffness;
    hubs_moment += p.unsprung_mass[c] * hub0_[c];
  }
  h_s_ = (p.mass * p.cg_height - hubs_moment) / sprung_mass_;
  if (!(h_s_ > 0.0)) {
    throw std::invalid_argument("vehicle.cg_height_m is below the hubs: no room for the sprung mass");
  }
  izz_s_ = p.yaw_inertia - sprung_mass_ * x_s_ * x_s_;
  for (std::size_t c = 0; c < kCorners; ++c) {
    izz_s_ -= p.unsprung_mass[c] * (x[c] * x[c] + y[c] * y[c]);
  }
  require_positive(izz_s_, "vehicle.yaw_inertia_kgm2 minus the unsprung masses' share");

  // K&C tables; the rack follows the steering ratio at the centre.
  for (std::size_t a = 0; a < 2; ++a) {
    const WishboneSolver solver(WishboneGeometry::from_params(params, a == 0));
    const double travel = std::max(0.06, 1.5 * std::max(p.axle[a].bump_travel, p.axle[a].droop_travel));
    if (a == 0) {
      constexpr double h = 1e-4;
      const double slope = (solver.solve(0.0, h).steer - solver.solve(0.0, -h).steer) / (2.0 * h);
      if (!(std::abs(slope) > 1e-6)) {
        throw std::invalid_argument("front tie rods do not steer the wheels (check the hardpoints)");
      }
      rack_per_rad_ = 1.0 / (p.steering_ratio * slope);
      const double rack_max = 1.15 * std::abs(rack_per_rad_) * p.max_steer_wheel;
      kc_[a] = KcTable::build(solver, travel, 49, rack_max, 25);
    } else {
      kc_[a] = KcTable::build(solver, travel, 49, 0.0, 1);
    }
  }

  // Spring preloads carry the static sprung loads at zero travel.
  for (std::size_t c = 0; c < kCorners; ++c) {
    const std::size_t a = is_front(c) ? 0 : 1;
    const KcPoint k = kc_[a].at(0.0, 0.0);
    const double sprung_load = fz0[c] - p.unsprung_mass[c] * g;
    require_positive(p.axle[a].spring_rate, "vehicle.suspension.<axle>.spring_rate_n_per_mm");
    preload_[c] = sprung_load / (p.axle[a].spring_rate * k.dspring) - k.spring;
    offset0_[c] = hub0_[c] - h_s_;
  }

  // Kane's mass matrix for [vx, vy, r, p, q].
  const double m = p.mass, ms = sprung_mass_, h = h_s_;
  const std::array<double, 25> mass = {m,   0.0,      0.0,          0.0,          ms * h,
                                       0.0, m,        0.0,          -ms * h,      0.0,
                                       0.0, 0.0,      p.yaw_inertia, -ms * h * x_s_, 0.0,
                                       0.0, -ms * h,  -ms * h * x_s_, p.roll_inertia + ms * h * h, 0.0,
                                       ms * h, 0.0,   0.0,          0.0,          p.pitch_inertia + ms * h * h};
  mass_inverse_ = invert5(mass);

  // Start on the road under the start pose: wheels at their static
  // deflection, the body fitted to them.
  const double c0 = std::cos(yaw0_), s0 = std::sin(yaw0_);
  std::array<double, kCorners> road{};
  // The wheels follow the car's branch where a course crosses itself
  // (skidpad): start their searches from the car's place near the start.
  double car_s = 0.0;
  if (track_) {
    const CurveProjection near_start = track_->project(x0_, y0_, track_->start_s(), 30.0);
    const CurveProjection anywhere = track_->project(x0_, y0_);
    car_s = near_start.distance <= anywhere.distance + 0.5 ? near_start.s : anywhere.s;
  }
  for (std::size_t c = 0; c < kCorners; ++c) {
    road_hint_[c].fill(0.0);
    if (track_) {
      const double wx = x0_ + c0 * x[c] - s0 * y[c];
      const double wy = y0_ + s0 * x[c] + c0 * y[c];
      double hint = track_->project(wx, wy, car_s, 5.0).s;
      road[c] = track_->road(wx, wy, &hint).z;
      road_hint_[c].fill(hint);
    }
    zu0_[c] = road[c] + hub0_[c];
  }
  // Least squares: zu - offset0 = Z - pitch (x - x_s) + roll y.
  double a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0, b1 = 0, b2 = 0, b3 = 0;
  for (std::size_t c = 0; c < kCorners; ++c) {
    const double r = zu0_[c] - offset0_[c];
    const double u = -(x[c] - x_s_), v = y[c];
    a11 += 1.0; a12 += u; a13 += v; a22 += u * u; a23 += u * v; a33 += v * v;
    b1 += r; b2 += u * r; b3 += v * r;
  }
  // 3x3 solve (Cramer).
  const double det = a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
  z0_ = (b1 * (a22 * a33 - a23 * a23) - a12 * (b2 * a33 - a23 * b3) + a13 * (b2 * a23 - a22 * b3)) / det;
  pitch0_ = (a11 * (b2 * a33 - a23 * b3) - b1 * (a12 * a33 - a23 * a13) + a13 * (a12 * b3 - b2 * a13)) / det;
  roll0_ = (a11 * (a22 * b3 - b2 * a23) - a12 * (a12 * b3 - b2 * a13) + b1 * (a12 * a23 - a22 * a13)) / det;
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
  for (std::size_t c = 0; c < kCorners; ++c) {
    x[thermal_offset_ + 3 * c] = x[thermal_offset_ + 3 * c + 1] = tire_temp0_;
    x[thermal_offset_ + 3 * c + 2] = brake_temp0_;
  }
  if (full_) {
    x[body_offset_ + kZ] = z0_;
    x[body_offset_ + kRoll] = roll0_;
    x[body_offset_ + kPitch] = pitch0_;
    for (std::size_t c = 0; c < kCorners; ++c) {
      x[unsprung_offset_ + c] = zu0_[c];
    }
  }
}

void TwoTrackModel::read_inputs(const SignalBus& bus) {
  steer_ = bus.get(sig_->steer);
  wind_x_ = bus.get(sig_->wind_x);
  wind_y_ = bus.get(sig_->wind_y);
  brake_ = std::clamp(bus.get(sig_->brake), 0.0, 1.0);
  for (std::size_t m = 0; m < torque_cmd_.size(); ++m) {
    torque_cmd_[m] = bus.get(sig_->motor[m].torque_cmd);
  }
  if (network_) {
    network_current_ = bus.get(sig_->elec_current);
    for (std::size_t m = 0; m < torque_cmd_.size(); ++m) {
      dc_voltage_[m] = bus.get(sig_->elec_dc[m]);
      inverter_on_[m] = bus.get(sig_->elec_enabled[m]);
    }
  }
  if (full_) {
    for (std::size_t c = 0; c < kCorners; ++c) {
      rig_z_[c] = bus.get(sig_->corner3[c].rig_z);
      rig_vz_[c] = bus.get(sig_->corner3[c].rig_vz);
    }
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
  const double g = kGravity;

  // Body states of the full model.
  const double zs = full_ ? x[body_offset_ + kZ] : 0.0;
  const double roll = full_ ? x[body_offset_ + kRoll] : 0.0;
  const double pitch = full_ ? x[body_offset_ + kPitch] : 0.0;
  const double vz = full_ ? x[body_offset_ + kVz] : 0.0;
  const double roll_rate = full_ ? x[body_offset_ + kRollRate] : 0.0;
  const double pitch_rate = full_ ? x[body_offset_ + kPitchRate] : 0.0;

  // Steering: partial Ackermann (L2) or the rack through the K&C tables.
  const double steer_wheel = std::clamp(steer_, -p_.max_steer_wheel, p_.max_steer_wheel);
  e.delta = steer_wheel / p_.steering_ratio;
  if (!full_) {
    const double tan_delta = std::tan(e.delta);
    const double half_front = 0.5 * p_.track_front;
    const double ackermann_left = std::atan2(wheelbase * tan_delta, wheelbase - tan_delta * half_front);
    const double ackermann_right = std::atan2(wheelbase * tan_delta, wheelbase + tan_delta * half_front);
    e.steer = {e.delta + p_.ackermann * (ackermann_left - e.delta),
               e.delta + p_.ackermann * (ackermann_right - e.delta), 0.0, 0.0};
  }
  e.rack = steer_wheel * rack_per_rad_;

  // Aero from the velocity through the air (the wind moves the air over the ground): drag along
  // the flow, downforce, and in a crosswind a side force behind or ahead of the CG.
  const double speed_sq = vx * vx + vy * vy;
  const double speed = std::sqrt(speed_sq);
  {
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double ua = vx - (cy * wind_x_ + sy * wind_y_);
    const double va = vy - (-sy * wind_x_ + cy * wind_y_);
    const double air_sq = ua * ua + va * va;
    e.airspeed = std::sqrt(air_sq);
    const double dynamic_pressure = 0.5 * p_.air_density * air_sq;
    e.downforce = p_.cla * dynamic_pressure;
    e.drag = p_.cda * dynamic_pressure;
    const double along = e.airspeed > 0.0 ? e.drag / e.airspeed : 0.0;
    e.aero_yaw = e.airspeed > 1e-6 ? std::atan2(va, ua) : 0.0;
    e.aero_side = -dynamic_pressure * p_.side_force_area * std::sin(e.aero_yaw);
    e.aero_fx = -along * ua;
    e.aero_fy = -along * va + e.aero_side;
    e.aero_moment = p_.side_force_x * e.aero_side;
  }

  // Contact patches: position in the body frame (from the CG), velocity, and
  // what each one stands on.
  std::array<double, kCorners> px{}, py{}, vcx{}, vcy{};
  std::array<KcPoint, kCorners> kc{};
  std::array<std::array<std::array<double, 2>, 3>, kCorners> grad{};
  // The patch averages the road over three points along it; on a smooth,
  // flat track one is enough.
  const std::size_t patch_samples = patch_samples_;
  const double weight = 1.0 / static_cast<double>(patch_samples);
  const auto patch_offset = [&](std::size_t k) {
    return patch_samples == 1 ? 0.0 : (static_cast<double>(k) - 1.0) * 0.5 * p_.contact_length;
  };
  if (!full_) {
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
      const double axle_static = m * g * (front ? lr : lf) / wheelbase;
      const double axle_aero = e.downforce * (front ? p_.aero_front_ratio : 1.0 - p_.aero_front_ratio);
      const double lateral = (front ? lat_front : lat_rear) * (is_left(c) ? -1.0 : 1.0);
      const double longitudinal = 0.5 * long_transfer * (front ? -1.0 : 1.0);
      e.fz[c] = std::max(0.0, 0.5 * (axle_static + axle_aero) + longitudinal + lateral);
      px[c] = front ? lf : -lr;
      py[c] = (is_left(c) ? 0.5 : -0.5) * p_.track(c);
      vcx[c] = vx - r * py[c];
      vcy[c] = vy + r * px[c];
    }
  } else {
    const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
    for (std::size_t c = 0; c < kCorners; ++c) {
      const bool front = is_front(c);
      const double side = is_left(c) ? 1.0 : -1.0;
      const double xc = front ? lf : -lr;
      const double yc = side * 0.5 * p_.track(c);
      const double xb = xc - x_s_;
      const double zu = x[unsprung_offset_ + c];
      const double vzu = x[unsprung_offset_ + kCorners + c];
      e.wheel_z[c] = zu;
      e.travel[c] = zu - (zs - pitch * xb + roll * yc) - offset0_[c];
      e.travel_rate[c] = vzu - (vz - pitch_rate * xb + roll_rate * yc);
      kc[c] = kc_[front ? 0 : 1].at(e.travel[c], side * e.rack);
      e.steer[c] = side * kc[c].steer;
      e.camber[c] = kc[c].camber;
      px[c] = xc + kc[c].cp_x;
      py[c] = yc + side * kc[c].cp_y;
      vcx[c] = vx - r * py[c] + kc[c].dcp_x * e.travel_rate[c];
      vcy[c] = vy + r * px[c] + side * kc[c].dcp_y * e.travel_rate[c];

      // The road averaged over the contact patch (three samples along the
      // body's x axis), and its rate seen by the moving patch.
      double z_road = rig_z_[c], z_rate = rig_vz_[c], mu = 0.0;
      int surface = 0;
      for (std::size_t k = 0; k < patch_samples; ++k) {
        const double ox = patch_offset(k);
        double gx = 0.0, gy = 0.0, z = 0.0, mu_k = 1.0;
        if (track_) {
          const double bx = px[c] + ox;
          const double wx = x[kX] + cos_yaw * bx - sin_yaw * py[c];
          const double wy = x[kY] + sin_yaw * bx + cos_yaw * py[c];
          const RoadPoint rp = track_->road(wx, wy, &road_hint_[c][k]);
          z = rp.z;
          mu_k = rp.mu;
          gx = cos_yaw * rp.dzdx + sin_yaw * rp.dzdy;
          gy = -sin_yaw * rp.dzdx + cos_yaw * rp.dzdy;
          surface = std::max(surface, static_cast<int>(rp.kind));
        }
        grad[c][k] = {gx, gy};
        z_road += z * weight;
        z_rate += (gx * vcx[c] + gy * (vcy[c] + r * ox)) * weight;
        mu += mu_k * weight;
      }
      e.road_z[c] = z_road;
      e.mu[c] = mu;
      e.surface[c] = surface;
      e.deflection[c] = z_road + free_radius_[c] - zu;
      const double deflection_rate = z_rate - vzu;
      e.fz[c] = e.deflection[c] > 0.0
                    ? std::max(0.0, p_.tire_vertical_stiffness * e.deflection[c] +
                                        p_.tire_vertical_damping * deflection_rate)
                    : 0.0;
    }
    // Springs, stops, dampers, anti-roll bars along the wheel travel.
    for (std::size_t c = 0; c < kCorners; ++c) {
      const std::size_t a = is_front(c) ? 0 : 1;
      const Params::Axle& ax = p_.axle[a];
      const double s = e.travel[c];
      e.spring_compression[c] = preload_[c] + kc[c].spring;
      e.spring_force[c] = ax.spring_rate * e.spring_compression[c] * kc[c].dspring +
                          bump_force(ax.bump_stop_rate, s - ax.bump_travel) -
                          (s < -ax.droop_travel ? kDroopStopRate * (-ax.droop_travel - s) : 0.0);
      e.damper_force[c] = damper_force(ax, e.travel_rate[c]);
      const std::size_t other = c ^ 1u;
      const double arb = ax.anti_roll_bar * 180.0 / std::numbers::pi / (p_.track(c) * p_.track(c));
      e.arb_force[c] = arb * (s - e.travel[other]);
    }
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
    double total = network_ ? 0.0 : battery_.aux_power;
    for (std::size_t i = 0; i < motors; ++i) {
      const double temp = x[motor_offset_ + 2 * i + 1];
      e.motor_speed[i] = driveline_.motor_speed(i, q);
      if (network_ && inverter_on_[i] < 0.5) {
        // Inverter off (DC link low, logic supply lost): no torque, no DC power.
        e.motor_avail[i] = e.motor_torque[i] = e.motor_loss[i] = e.motor_power[i] = 0.0;
        continue;
      }
      e.motor_avail[i] = motor_.envelope(e.motor_speed[i], network_ ? dc_voltage_[i] : dc_voltage, temp);
      e.motor_torque[i] = std::clamp(x[motor_offset_ + 2 * i], -e.motor_avail[i], e.motor_avail[i]);
      e.motor_loss[i] = motor_.losses(e.motor_torque[i], e.motor_speed[i]);
      e.motor_power[i] = e.motor_torque[i] * e.motor_speed[i] + e.motor_loss[i];
      total += e.motor_power[i];
    }
    return total;
  };
  // The DC voltage sags with current; one fixed-point pass is enough because
  // it only matters where the envelope is voltage limited.
  e.open_voltage = open_voltage;
  if (network_) {
    // The electrical network sets the current (DC links, LV loads, contactors).
    e.motor_power_sum = motor_demand(0.0);
    e.battery.current = network_current_;
    e.battery.voltage = open_voltage - resistance * network_current_;
    e.battery.limited = false;
    e.power_elec = e.battery.voltage * e.battery.current;
  } else {
    const BatteryOperatingPoint first =
        solve_battery_power(open_voltage, resistance, motor_demand(open_voltage));
    e.power_elec = motor_demand(first.voltage);
    e.battery = solve_battery_power(open_voltage, resistance, e.power_elec);
    e.motor_power_sum = e.power_elec - battery_.aux_power;
  }
  e.limits = battery_limits(battery_, soc, rc1 + rc2, battery_temp, spread_.empty() ? nullptr : &spread_);
  e.power_mech = 0.0;
  for (std::size_t i = 0; i < motors; ++i) {
    e.power_mech += e.motor_torque[i] * e.motor_speed[i];
  }

  // Tire grip from temperature and pressure, pad friction from disc temperature.
  for (std::size_t c = 0; c < kCorners; ++c) {
    const double surface = x[thermal_offset_ + 3 * c], core = x[thermal_offset_ + 3 * c + 1];
    e.tire_pressure[c] = tire_thermal_.pressure(core);
    if (tire_thermal_.enabled) {
      e.tire_grip[c] = tire_thermal_.temperature_grip(surface) * tire_thermal_.pressure_grip(e.tire_pressure[c]);
    }
    if (brake_thermal_.enabled) {
      e.brake_mu[c] = brake_thermal_.pad_mu(x[thermal_offset_ + 3 * c + 2]);
    }
  }

  // Tires with transient slip.
  e.brake_pressure = brake_ * p_.brake_pressure_max;
  Driveline::Loads loads;
  loads.motor_torque = std::span<const double>(e.motor_torque.data(), motors);
  double fx_body = 0.0, fy_body = 0.0, yaw_moment = 0.0;
  double slip_power = 0.0, rolling_power = 0.0;
  std::array<double, kCorners> jack{};  // generalized tire force along each wheel's travel
  e.ffb = 0.0;
  for (std::size_t c = 0; c < kCorners; ++c) {
    const bool front = is_front(c);
    const double cos_s = std::cos(e.steer[c]);
    const double sin_s = std::sin(e.steer[c]);
    const double vx_wheel = cos_s * vcx[c] + sin_s * vcy[c];
    const double vy_wheel = -sin_s * vcx[c] + cos_s * vcy[c];
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
    // Camber as the tire sees it (ISO: top leaning right +): the left wheel's
    // outward lean is to the left. Camber thrust builds up as the tire rolls,
    // so it fades out towards standstill like the slip damping fades in (a
    // parked car with toe and camber does not creep).
    const double inclination = (is_left(c) ? -1.0 : 1.0) * e.camber[c] * (1.0 - fade);
    const TireForces f =
        tire.evaluate({e.fz[c], kappa_used, std::atan(tan_alpha_used), inclination, e.mu[c] * e.tire_grip[c]});
    e.fx[c] = f.fx;
    e.fy[c] = f.fy;
    e.mz[c] = f.mz;
    e.slip_ratio[c] = kappa;
    e.slip_angle[c] = std::atan(tan_alpha);

    const double fx_b = cos_s * f.fx - sin_s * f.fy;
    const double fy_b = sin_s * f.fx + cos_s * f.fy;
    fx_body += fx_b;
    fy_body += fy_b;
    yaw_moment += px[c] * fy_b - py[c] * fx_b + f.mz;
    slip_power += f.fx * slip_velocity - f.fy * vy_wheel - f.mz * r;
    e.slip_heat[c] = std::max(0.0, f.fx * slip_velocity - f.fy * vy_wheel);

    const double rolling = p_.rolling_resistance * e.fz[c] * radius * std::tanh(e.omega[c] / kRollingSmoothing);
    rolling_power += rolling * e.omega[c];
    e.rolling_heat[c] = std::max(0.0, rolling * e.omega[c]);
    loads.wheel_torque[c] = -f.fx * radius - rolling;
    loads.brake_capacity[c] =
        0.5 * e.brake_pressure * (front ? p_.brake_gain_front : p_.brake_gain_rear) * e.brake_mu[c];
    if (!full_) {
      if (front) {
        e.ffb += (f.mz - p_.mechanical_trail * f.fy) / p_.steering_ratio;
      }
      continue;
    }
    // Full model: the road's slope tilts the normal force (applied at the
    // patch samples), and both act along the travel through the scrub.
    const double side = is_left(c) ? 1.0 : -1.0;
    double gx_force = 0.0, gy_force = 0.0;
    for (std::size_t k = 0; k < patch_samples; ++k) {
      const double ox = patch_offset(k);
      const double fgx = -e.fz[c] * weight * grad[c][k][0];
      const double fgy = -e.fz[c] * weight * grad[c][k][1];
      gx_force += fgx;
      gy_force += fgy;
      yaw_moment += (px[c] + ox) * fgy - py[c] * fgx;
    }
    fx_body += gx_force;
    fy_body += gy_force;
    jack[c] = (fx_b + gx_force) * kc[c].dcp_x + (fy_b + gy_force) * side * kc[c].dcp_y;
    // Steering-axis moment (towards steer-left); the right corner's table is
    // the mirror image of the left one.
    const KcPoint& k = kc[c];
    e.kingpin_moment[c] = side * (k.arm_x * f.fx + k.arm_z * e.fz[c]) + k.arm_y * f.fy + k.arm_mz * f.mz;
    if (front) {
      e.ffb += e.kingpin_moment[c] * k.dsteer_drack * rack_per_rad_;
    }
  }

  fx_body += e.aero_fx;
  fy_body += e.aero_fy;
  yaw_moment += e.aero_moment;

  if (!full_) {
    e.ax = fx_body / m;
    e.ay = fy_body / m;
    e.yaw_accel = yaw_moment / p_.yaw_inertia;
    if (dx.empty()) {
      return;
    }
    const double ax_lag = x[kAxFiltered];
    const double ay_lag = x[kAyFiltered];
    dx[kX] = vx * std::cos(yaw) - vy * std::sin(yaw);
    dx[kY] = vx * std::sin(yaw) + vy * std::cos(yaw);
    dx[kYaw] = r;
    dx[kVx] = e.ax + vy * r;
    dx[kVy] = e.ay - vx * r;
    dx[kYawRate] = yaw_moment / p_.yaw_inertia;
    dx[kAxFiltered] = (e.ax - ax_lag) / p_.load_transfer_tau;
    dx[kAyFiltered] = (e.ay - ay_lag) / p_.load_transfer_tau;
  } else {
    // Generalized forces (Kane): body heave, roll, pitch; unsprung heave.
    const double ms = sprung_mass_, h = h_s_;
    const double df_front = e.downforce * p_.aero_front_ratio;
    const double df_rear = e.downforce - df_front;
    const double xb_front = lf - x_s_, xb_rear = -lr - x_s_;
    double q_z = -ms * g - e.downforce;
    double q_roll = ms * g * h * roll;
    double q_pitch = ms * g * h * pitch + xb_front * df_front + xb_rear * df_rear;
    std::array<double, kCorners> q_unsprung{};
    for (std::size_t c = 0; c < kCorners; ++c) {
      const double xc = is_front(c) ? lf : -lr;
      const double yc = (is_left(c) ? 0.5 : -0.5) * p_.track(c);
      const double body = e.spring_force[c] + e.damper_force[c] + e.arb_force[c] - jack[c];
      q_z += body;
      q_roll += yc * body;
      q_pitch -= (xc - x_s_) * body;
      q_unsprung[c] = e.fz[c] + jack[c] - e.spring_force[c] - e.damper_force[c] - e.arb_force[c] -
                      p_.unsprung_mass[c] * g;
    }
    const double ixx = p_.roll_inertia, iyy = p_.pitch_inertia;
    const std::array<double, 5> b = {
        fx_body + m * r * vy - ms * h * r * roll_rate,
        fy_body - m * r * vx - ms * h * r * pitch_rate,
        yaw_moment - ms * x_s_ * h * r * pitch_rate - (iyy - ixx) * roll_rate * pitch_rate,
        q_roll + ms * h * r * vx + ms * h * h * r * pitch_rate + (iyy - izz_s_) * pitch_rate * r,
        q_pitch + ms * h * r * vy - ms * h * h * r * roll_rate + ms * h * r * r * x_s_ + (izz_s_ - ixx) * r * roll_rate};
    const std::array<double, 5> u = apply(mass_inverse_, b);
    e.ax = u[0] - r * vy;
    e.ay = u[1] + r * vx;
    e.az = q_z / ms;
    e.yaw_accel = u[2];
    e.roll_accel = u[3];
    e.pitch_accel = u[4];
    if (dx.empty()) {
      return;
    }
    dx[kX] = vx * std::cos(yaw) - vy * std::sin(yaw);
    dx[kY] = vx * std::sin(yaw) + vy * std::cos(yaw);
    dx[kYaw] = r;
    dx[kVx] = u[0];
    dx[kVy] = u[1];
    dx[kYawRate] = u[2];
    dx[kAxFiltered] = 0.0;
    dx[kAyFiltered] = 0.0;
    dx[body_offset_ + kZ] = vz;
    dx[body_offset_ + kRoll] = roll_rate;
    dx[body_offset_ + kPitch] = pitch_rate;
    dx[body_offset_ + kVz] = e.az;
    dx[body_offset_ + kRollRate] = u[3];
    dx[body_offset_ + kPitchRate] = u[4];
    for (std::size_t c = 0; c < kCorners; ++c) {
      dx[unsprung_offset_ + c] = x[unsprung_offset_ + kCorners + c];
      dx[unsprung_offset_ + kCorners + c] = q_unsprung[c] / p_.unsprung_mass[c];
    }
  }

  Driveline::Losses losses;
  driveline_.accelerations(q, loads, dx.subspan(wheel_offset_, driveline_.dof_count()), losses);

  // Tire and brake temperatures.
  for (std::size_t c = 0; c < kCorners; ++c) {
    const std::size_t o = thermal_offset_ + 3 * c;
    const double surface = x[o], core = x[o + 1], disc = x[o + 2];
    const double rim = tire_thermal_.enabled && brake_thermal_.enabled ? tire_thermal_.rim_brake * (disc - core) : 0.0;
    if (tire_thermal_.enabled) {
      const TireThermalParams& t = tire_thermal_;
      const double to_core = t.surface_core * (surface - core);
      const double to_air = (t.surface_air + t.surface_air_speed * speed) * (surface - p_.ambient_temp);
      const double to_road = e.fz[c] > 0.0 ? t.road_contact * (surface - road_temp_) : 0.0;
      dx[o] = (t.slip_share * e.slip_heat[c] - to_core - to_air - to_road) / t.surface_capacity;
      const double core_air = (t.core_air + t.core_air_speed * speed) * (core - p_.ambient_temp);
      dx[o + 1] = (e.rolling_heat[c] + to_core + rim - core_air) / t.core_capacity;
    } else {
      dx[o] = dx[o + 1] = 0.0;
    }
    if (brake_thermal_.enabled) {
      const double capacity = brake_thermal_.heat_capacity[is_front(c) ? 0 : 1];
      const double cooling = brake_thermal_.cooling_power(disc, p_.ambient_temp, speed);
      dx[o + 2] = (losses.brake_wheel[c] - cooling - rim) / capacity;
    } else {
      dx[o + 2] = 0.0;
    }
  }

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
  // With the electrical network: what leaves the terminals but not into the
  // motors (LV loads, wiring, DC-link charge).
  energy[kAux] = network_ ? e.power_elec - e.motor_power_sum : battery_.aux_power;
  energy[kMotorLoss] = motor_loss;
  energy[kMotorShaft] = e.power_mech;
  energy[kGearLoss] = losses.gear;
  energy[kDiffLoss] = losses.diff;
  energy[kBrakeLoss] = losses.brake;
  energy[kSlipLoss] = slip_power;
  energy[kRollingLoss] = rolling_power;
  // What the air takes from the car (negative when the wind pushes it).
  energy[kDragLoss] = -(e.aero_fx * vx + e.aero_fy * vy + e.aero_moment * r);
  energy[kDamperLoss] = 0.0;
  energy[kTireDampingLoss] = 0.0;
  if (full_) {
    const double df_front = e.downforce * p_.aero_front_ratio;
    energy[kDragLoss] += df_front * (vz - pitch_rate * (lf - x_s_)) +
                         (e.downforce - df_front) * (vz - pitch_rate * (-lr - x_s_));
    for (std::size_t c = 0; c < kCorners; ++c) {
      energy[kDamperLoss] += e.damper_force[c] * e.travel_rate[c];
      // Tire carcass: what the force takes beyond the spring's share.
      double z_rate = rig_vz_[c];
      for (std::size_t k = 0; k < patch_samples; ++k) {
        z_rate += (grad[c][k][0] * vcx[c] + grad[c][k][1] * (vcy[c] + r * patch_offset(k))) * weight;
      }
      const double deflection_rate = z_rate - x[unsprung_offset_ + kCorners + c];
      energy[kTireDampingLoss] +=
          (e.fz[c] - p_.tire_vertical_stiffness * std::max(0.0, e.deflection[c])) * deflection_rate;
    }
  }
}

double TwoTrackModel::kinetic_energy(std::span<const double> x) const {
  double kinetic = 0.5 * p_.mass * (x[kVx] * x[kVx] + x[kVy] * x[kVy]) +
                   0.5 * p_.yaw_inertia * x[kYawRate] * x[kYawRate] +
                   driveline_.kinetic_energy(x.subspan(wheel_offset_, driveline_.dof_count()));
  if (full_) {
    // Kane's kinetic energy: the sprung CG moves with roll and pitch about
    // the ground below it.
    const double ms = sprung_mass_, h = h_s_;
    const double p = x[body_offset_ + kRollRate], q = x[body_offset_ + kPitchRate], vz = x[body_offset_ + kVz];
    kinetic += ms * h * (x[kVx] * q - x[kVy] * p) + 0.5 * (p_.roll_inertia + ms * h * h) * p * p +
               0.5 * (p_.pitch_inertia + ms * h * h) * q * q - ms * h * x_s_ * p * x[kYawRate] + 0.5 * ms * vz * vz;
    for (std::size_t c = 0; c < kCorners; ++c) {
      const double v = x[unsprung_offset_ + kCorners + c];
      kinetic += 0.5 * p_.unsprung_mass[c] * v * v;
    }
  }
  return kinetic;
}

double TwoTrackModel::stored_energy(std::span<const double> x) const {
  if (!full_) {
    return 0.0;
  }
  Eval e;
  evaluate(x, e, {});
  const double g = kGravity;
  const double roll = x[body_offset_ + kRoll], pitch = x[body_offset_ + kPitch];
  double energy = sprung_mass_ * g * x[body_offset_ + kZ] - 0.5 * sprung_mass_ * g * h_s_ * (roll * roll + pitch * pitch);
  for (std::size_t c = 0; c < kCorners; ++c) {
    const Params::Axle& a = p_.axle[is_front(c) ? 0 : 1];
    const double s = e.travel[c];
    const double droop = std::max(0.0, -a.droop_travel - s);
    energy += p_.unsprung_mass[c] * g * x[unsprung_offset_ + c] +
              0.5 * a.spring_rate * e.spring_compression[c] * e.spring_compression[c] +
              bump_energy(a.bump_stop_rate, s - a.bump_travel) + 0.5 * kDroopStopRate * droop * droop;
    const double deflection = std::max(0.0, e.deflection[c]);
    energy += 0.5 * p_.tire_vertical_stiffness * deflection * deflection;
    if (is_left(c)) {
      const double arb = a.anti_roll_bar * 180.0 / std::numbers::pi / (p_.track(c) * p_.track(c));
      const double twist = s - e.travel[c + 1];
      energy += 0.5 * arb * twist * twist;
    }
  }
  return energy;
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
  bus.set(s.yaw_accel, e.yaw_accel);
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
    bus.set(w.tire_temp, x[thermal_offset_ + 3 * c]);
    bus.set(w.tire_core_temp, x[thermal_offset_ + 3 * c + 1]);
    bus.set(w.tire_pressure, e.tire_pressure[c]);
    bus.set(w.tire_grip, e.tire_grip[c]);
    bus.set(w.brake_temp, x[thermal_offset_ + 3 * c + 2]);
    bus.set(w.brake_mu, e.brake_mu[c]);
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
  bus.set(s.airspeed, e.airspeed);
  bus.set(s.aero_yaw, e.aero_yaw);
  bus.set(s.aero_side, e.aero_side);
  bus.set(s.aero_moment, e.aero_moment);
  bus.set(s.brake_pressure, e.brake_pressure);
  {
    const double gf = p_.brake_gain_front * 0.5 * (e.brake_mu[0] + e.brake_mu[1]);
    const double gr = p_.brake_gain_rear * 0.5 * (e.brake_mu[2] + e.brake_mu[3]);
    bus.set(s.brake_balance, gf + gr > 0.0 ? gf / (gf + gr) : 0.0);
  }
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
  bus.set(s.open_voltage, e.open_voltage);
  bus.set(s.battery_temp, x[battery_offset_ + 3]);
  bus.set(s.overloaded, e.battery.limited ? 1.0 : 0.0);
  {
    const double soc = x[battery_offset_], rc = x[battery_offset_ + 1] + x[battery_offset_ + 2];
    if (spread_.empty()) {
      const double v = battery_.ocv_cell(soc) - rc - battery_.r0 * e.battery.current / battery_.parallel;
      bus.set(s.cell_v_min, v);
      bus.set(s.cell_v_max, v);
      bus.set(s.soc_min, soc);
      bus.set(s.soc_max, soc);
      bus.set(s.weak_group, 0.0);
    } else {
      const CellExtremes c = cell_extremes(battery_, spread_, soc, rc, e.battery.current);
      bus.set(s.cell_v_min, c.v_min);
      bus.set(s.cell_v_max, c.v_max);
      bus.set(s.soc_min, c.soc_min);
      bus.set(s.soc_max, c.soc_max);
      bus.set(s.weak_group, static_cast<double>(c.weakest));
    }
  }
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
                       energy[kDragLoss] + energy[kDamperLoss] + energy[kTireDampingLoss];
  const double potential_gain = full_ ? stored_energy(x) - stored0_ : 0.0;
  bus.set(s.residual, energy[kBatteryChemical] - sinks - kinetic_gain - rc_stored - potential_gain);

  if (full_) {
    bus.set(s.z, x[body_offset_ + kZ]);
    bus.set(s.roll, x[body_offset_ + kRoll]);
    bus.set(s.pitch, x[body_offset_ + kPitch]);
    bus.set(s.vz, x[body_offset_ + kVz]);
    bus.set(s.roll_rate, x[body_offset_ + kRollRate]);
    bus.set(s.pitch_rate, x[body_offset_ + kPitchRate]);
    bus.set(s.roll_accel, e.roll_accel);
    bus.set(s.pitch_accel, e.pitch_accel);
    bus.set(s.az, e.az);
    bus.set(s.rack, e.rack);
    bus.set(s.potential, potential_gain);
    const double road = 0.25 * (e.road_z[0] + e.road_z[1] + e.road_z[2] + e.road_z[3]);
    bus.set(s.road_z, road);
    bus.set(s.heave, x[body_offset_ + kZ] - road - h_s_);
    for (std::size_t c = 0; c < kCorners; ++c) {
      bus.set(s.corner3[c].hub_offset, e.wheel_z[c] - road - hub0_[c]);
      const Signals::Corner3& w = s.corner3[c];
      bus.set(w.z, e.wheel_z[c]);
      bus.set(w.travel, e.travel[c]);
      bus.set(w.camber, e.camber[c]);
      bus.set(w.spring, e.spring_force[c] + e.arb_force[c]);
      bus.set(w.damper, e.damper_force[c]);
      bus.set(w.deflection, e.deflection[c]);
      bus.set(w.road_z, e.road_z[c]);
      bus.set(w.mu, e.mu[c]);
      bus.set(w.surface, static_cast<double>(e.surface[c]));
    }
  }
}

}  // namespace vsim

#include "vsim/driver/mpc.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "vsim/driver/qp.hpp"
#include "vsim/models/tire/tire.hpp"
#include "vsim/models/vehicle/corners.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

constexpr double kG = 9.81;
constexpr double kPi = std::numbers::pi;
constexpr double kInf = std::numeric_limits<double>::infinity();

double deg_to_rad(double deg) { return deg * kPi / 180.0; }

// Largest change of one SQP iteration: road-wheel angle [rad], acceleration [m/s^2].
constexpr double kTrustDelta = 0.06, kTrustAccel = 4.0;
// Damping ratio of the neuromuscular steering filter.
constexpr double kHandDamping = 0.7;
// How fast the driver's feel for the tire grip follows the tires [s].
constexpr double kFeelTime = 1.0;
// Learning the axles: signal filter and averaging time [s].
constexpr double kLearnFilter = 0.05, kLearnTime = 6.0;
// Largest correction of the pedal for the acceleration the driver feels [m/s^2].
constexpr double kAccelCorrection = 4.0;
// The friction circle as an octagon: |ax|, |ay| <= g and |ax| + |ay| <= 1.25 g.
constexpr double kFrictionOctagon = 1.25;

}  // namespace

// --- Model ----------------------------------------------------------------------------------
namespace {

// Lateral force of one axle at its load with the lateral load transfer of steady cornering
// (outer wheel + inner wheel), from the vehicle's own tyre model.
double axle_force(const TireModel& tire, double load, double transfer, double slip) {
  TireInput in;
  in.slip_angle = slip;
  in.fz = std::max(0.0, 0.5 * load + transfer);
  double f = tire.evaluate(in).fy;
  in.fz = std::max(0.0, 0.5 * load - transfer);
  return f + tire.evaluate(in).fy;
}

struct AxlePeak {
  double force = 0.0, slip = 0.0;
};

AxlePeak axle_peak(const TireModel& tire, double load, double transfer) {
  AxlePeak p;
  for (double slip = 0.0025; slip <= 0.5; slip += 0.0025) {
    const double f = axle_force(tire, load, transfer, slip);
    if (f > p.force) {
      p = {f, slip};
    }
  }
  return p;
}

}  // namespace

double MpcModel::Axle::peak_slip() const { return std::tan(kPi / (2.0 * c)) / b; }

double MpcModel::Axle::mu_at(double fz) const {
  return mu * std::max(0.3, 1.0 + sensitivity * (fz / load_ref - 1.0));
}

MpcModel MpcModel::from_params(const ParamTree& params) {
  MpcModel m;
  m.mass = params.get_double("vehicle.mass_kg");
  m.yaw_inertia = params.get_double("vehicle.yaw_inertia_kgm2");
  m.lf = params.get_double("vehicle.cg_to_front_axle_m");
  m.lr = params.get_double("vehicle.wheelbase_m") - m.lf;
  m.cg_height = params.get_double("vehicle.cg_height_m", m.cg_height);
  m.cla = params.get_double("vehicle.aero.cla_m2", m.cla);
  m.cda = params.get_double("vehicle.aero.cda_m2", m.cda);
  m.air_density = params.get_double("vehicle.aero.air_density_kgm3", m.air_density);
  m.aero_front = params.get_double("vehicle.aero.front_ratio", m.aero_front);
  m.power = params.get_double("control.power_limit_kw", m.power * 1e-3) * 1e3;
  const double bf = params.get_double("vehicle.brakes.front_gain_nm_per_bar", 16.0);
  const double br = params.get_double("vehicle.brakes.rear_gain_nm_per_bar", 5.0);
  m.brake_front = bf + br > 0.0 ? bf / (bf + br) : m.brake_front;
  const double torque = params.get_double("vehicle.powertrain.motor.torque_max_nm", 20.0);
  double drive = 0.0;
  for (std::size_t i = 0; params.has_prefix("vehicle.driveline." + std::to_string(i)); ++i) {
    const std::string p = "vehicle.driveline." + std::to_string(i);
    drive += torque * params.get_double(p + ".gear_ratio", 1.0) * params.get_double(p + ".efficiency", 1.0);
  }
  if (drive > 0.0) {
    m.drive_force = drive / params.get_double("vehicle.wheel.radius_m");
  }
  if (!(m.mass > 0.0 && m.yaw_inertia > 0.0 && m.lf > 0.0 && m.lr > 0.0)) {
    throw std::invalid_argument("driver.mpc: vehicle mass, inertia and axle positions must be positive");
  }

  // The axles as the driver knows them: each axle's lateral force curve from the car's tyres
  // at its share of the load, with the lateral load transfer of hard cornering (roll centres
  // and roll stiffness split as in the two-track model), fitted with a Magic Formula. Load
  // transfer is what makes one axle give up first: the balance of the car.
  const std::filesystem::path dir = path_from_utf8(params.get_string("project.dir", "."));
  const std::unique_ptr<TireModel> tires[2] = {make_tire(params, "front", dir), make_tire(params, "rear", dir)};
  const double wheelbase = m.lf + m.lr;
  const double rc_front = params.get_double("vehicle.suspension.roll_center_front_m", 0.03);
  const double rc_rear = params.get_double("vehicle.suspension.roll_center_rear_m", 0.05);
  const double roll_front = params.get_double("vehicle.suspension.roll_stiffness_front_ratio", 0.5);
  const double track[2] = {params.get_double("vehicle.track_front_m"), params.get_double("vehicle.track_rear_m")};
  const double roll_axis = rc_front + (rc_rear - rc_front) * m.lf / wheelbase;
  const double speed = 15.0;  // typical cornering speed for the downforce share
  const double aero = 0.5 * m.air_density * m.cla * speed * speed;
  const double load[2] = {m.mass * kG * m.lr / wheelbase + m.aero_front * aero,
                          m.mass * kG * m.lf / wheelbase + (1.0 - m.aero_front) * aero};
  const auto transfer = [&](int axle, double ay, double scale) {
    const double share = axle == 0 ? m.lr / wheelbase * rc_front + (m.cg_height - roll_axis) * roll_front
                                   : m.lf / wheelbase * rc_rear + (m.cg_height - roll_axis) * (1.0 - roll_front);
    return scale * m.mass * ay * share / track[axle];
  };
  double ay = 1.5 * kG;
  for (int iteration = 0; iteration < 4; ++iteration) {  // the lateral acceleration the car can hold
    const double front = axle_peak(*tires[0], load[0], transfer(0, ay, 1.0)).force;
    const double rear = axle_peak(*tires[1], load[1], transfer(1, ay, 1.0)).force;
    ay = 0.9 * std::min(front * wheelbase / (m.mass * m.lr), rear * wheelbase / (m.mass * m.lf));
  }
  const double scale = params.get_double("driver.model.grip_scale", 0.95);
  if (!(scale > 0.0 && scale <= 1.5)) {
    throw std::invalid_argument("driver.model.grip_scale must be in (0, 1.5]");
  }
  for (int a = 0; a < 2; ++a) {
    Axle& axle = a == 0 ? m.front : m.rear;
    const AxlePeak peak = axle_peak(*tires[a], load[a], transfer(a, ay, 1.0));
    const AxlePeak heavy = axle_peak(*tires[a], 1.25 * load[a], transfer(a, ay, 1.25));
    if (!(peak.force > 0.0)) {
      throw std::invalid_argument("driver.mpc: the tyre model gives no lateral force");
    }
    axle.load_ref = load[a];
    axle.mu = scale * peak.force / load[a];
    axle.sensitivity = (heavy.force / (1.25 * load[a]) / (peak.force / load[a]) - 1.0) / 0.25;
    // Shape: B C D is the cornering stiffness, the peak at B alpha = tan(pi / 2C).
    const double stiffness = axle_force(*tires[a], load[a], transfer(a, ay, 1.0), 0.002) / 0.002;
    const double k = stiffness / peak.force * peak.slip;
    double lo = 1.05, hi = 2.5;
    for (int i = 0; i < 60; ++i) {
      const double c = 0.5 * (lo + hi);
      (std::tan(kPi / (2.0 * c)) > k / c ? lo : hi) = c;
    }
    axle.c = 0.5 * (lo + hi);
    axle.b = std::tan(kPi / (2.0 * axle.c)) / peak.slip;
  }
  if (params.has("driver.model.mu")) {  // a fixed friction instead of the tyre data
    m.front.mu = m.rear.mu = params.get_double("driver.model.mu");
    m.front.sensitivity = m.rear.sensitivity = 0.0;
  }
  if (!(m.front.mu > 0.0 && m.rear.mu > 0.0)) {
    throw std::invalid_argument("driver.model.mu must be positive");
  }
  return m;
}

double MpcModel::grip(double vx) const {
  // Steady cornering: the axle that saturates first sets the limit.
  const double wheelbase = lf + lr;
  const double aero = 0.5 * air_density * cla * vx * vx;
  const double fzf = mass * kG * lr / wheelbase + aero_front * aero;
  const double fzr = mass * kG * lf / wheelbase + (1.0 - aero_front) * aero;
  return std::min(front.mu_at(fzf) * fzf * wheelbase / (mass * lr), rear.mu_at(fzr) * fzr * wheelbase / (mass * lf));
}

double MpcModel::ax_max(double vx) const {
  return std::min({grip(vx), drive_force / mass, power / (mass * std::max(vx, 1.0))});
}

MpcModel::State MpcModel::derivative(const State& x, const Input& u, double kappa) const {
  const double n = x[N], dpsi = x[Dpsi], vx = x[Vx], vy = x[Vy], r = x[R];
  const double delta = u[0], ax = u[1];
  const double wheelbase = lf + lr;
  State d{};
  const double ds = (vx * std::cos(dpsi) - vy * std::sin(dpsi)) / std::max(1.0 - n * kappa, 0.2);
  d[S] = ds;
  d[N] = vx * std::sin(dpsi) + vy * std::cos(dpsi);
  d[Dpsi] = r - kappa * ds;

  // Dynamic single track, slip angles with a speed floor.
  const double ve = std::max(vx, 3.0);
  const double aero = 0.5 * air_density * cla * vx * vx;
  const double transfer = mass * ax * cg_height / wheelbase;
  const double fzf = std::max(0.0, mass * kG * lr / wheelbase + aero_front * aero - transfer);
  const double fzr = std::max(0.0, mass * kG * lf / wheelbase + (1.0 - aero_front) * aero + transfer);
  const double af = front.gain * (delta - std::atan2(vy + lf * r, ve));
  const double ar = rear.gain * -std::atan2(vy - lr * r, ve);
  // Combined slip: what braking or driving takes from each axle's grip circle. Braking splits
  // by the brake balance; drive follows the axle loads (traction control on every wheel).
  const double fx = mass * ax;
  const double fxf = fx * (ax >= 0.0 ? fzf / std::max(fzf + fzr, 1.0) : brake_front), fxr = fx - fxf;
  const double gf = front.mu_at(fzf) * fzf, gr = rear.mu_at(fzr) * fzr;
  const double cap_f = std::sqrt(std::max(gf * gf - fxf * fxf, 0.01 * gf * gf));
  const double cap_r = std::sqrt(std::max(gr * gr - fxr * fxr, 0.01 * gr * gr));
  // The force stays at its peak past the peak slip: the driver does not plan on the falling
  // side of the curve (nor see more lock as a way to lose grip).
  const double pf = front.peak_slip(), pr = rear.peak_slip();
  const double fyf = cap_f * std::sin(front.c * std::atan(front.b * std::clamp(af, -pf, pf)));
  const double fyr = cap_r * std::sin(rear.c * std::atan(rear.b * std::clamp(ar, -pr, pr)));
  double dvy = (fyf * std::cos(delta) + fyr) / mass - vx * r;
  double dr = (lf * fyf * std::cos(delta) - lr * fyr) / yaw_inertia;
  // Walking speed: relax towards the kinematic model (no slip, rolls where it points).
  const double w = std::clamp((vx - 1.0) / 2.0, 0.0, 1.0);
  if (w < 1.0) {
    const double beta = std::atan(lr * std::tan(delta) / wheelbase);
    const double vk = std::max(vx, 0.0);
    const double r_kin = vk * std::cos(beta) * std::tan(delta) / wheelbase;
    const double vy_kin = vk * std::sin(beta);
    dvy = w * dvy + (1.0 - w) * (vy_kin - vy) / 0.1;
    dr = w * dr + (1.0 - w) * (r_kin - r) / 0.1;
  }
  d[Vx] = ax + vy * r - fyf * std::sin(delta) / mass;
  d[Vy] = dvy;
  d[R] = dr;
  return d;
}

double MpcTrack::at(const std::vector<double>& v, double s) const {
  if (v.empty()) {
    return 0.0;
  }
  const auto n = static_cast<double>(v.size());
  double i = s / step;
  if (closed) {
    i = std::fmod(i, n);
    if (i < 0.0) {
      i += n;
    }
  } else {
    i = std::clamp(i, 0.0, n - 1.0);
  }
  const auto k = static_cast<std::size_t>(i);
  const std::size_t k1 = closed ? (k + 1) % v.size() : std::min(k + 1, v.size() - 1);
  const double t = i - static_cast<double>(k);
  return v[k] + t * (v[k1] - v[k]);
}

// --- Driver ---------------------------------------------------------------------------------
struct MpcDriver::Plan {
  double origin = 0.0;  // simulation time u[0] starts to apply
  double dt = 0.05;
  std::vector<MpcModel::Input> u;
  std::vector<MpcModel::State> x;  // u.size() + 1
  MpcModel::Input input_at(double t) const {
    const double k = std::clamp((t - origin) / dt, 0.0, static_cast<double>(u.size() - 1));
    const auto i = static_cast<std::size_t>(k);
    const std::size_t j = std::min(i + 1, u.size() - 1);
    const double f = k - static_cast<double>(i);
    return {u[i][0] + f * (u[j][0] - u[i][0]), u[i][1] + f * (u[j][1] - u[i][1])};
  }
  double speed_at(double t) const {
    const double k = std::clamp((t - origin) / dt, 0.0, static_cast<double>(x.size() - 1));
    const auto i = static_cast<std::size_t>(k);
    const std::size_t j = std::min(i + 1, x.size() - 1);
    return x[i][MpcModel::Vx] + (k - static_cast<double>(i)) * (x[j][MpcModel::Vx] - x[i][MpcModel::Vx]);
  }
};

MpcDriver::MpcDriver() = default;
MpcDriver::~MpcDriver() = default;

namespace {

// Quasi-steady speed along a driven line with the driver's grip expectation
// (downforce included, longitudinal grip from the friction circle).
std::vector<double> qss_speed(const MpcModel& m, const std::vector<double>& kappa, const std::vector<double>& ds,
                              double share, double brake_share, double top, bool closed) {
  const std::size_t n = kappa.size();
  std::vector<double> v(n);
  for (std::size_t i = 0; i < n; ++i) {  // the fastest speed the curve allows (bisection)
    double lo = 0.0, hi = top;
    if (hi * hi * std::abs(kappa[i]) > share * m.grip(hi)) {
      for (int k = 0; k < 40; ++k) {
        const double mid = 0.5 * (lo + hi);
        (mid * mid * std::abs(kappa[i]) <= share * m.grip(mid) ? lo : hi) = mid;
      }
    } else {
      lo = top;
    }
    v[i] = lo;
  }
  const auto avail = [&](double speed, double k, bool accelerating) {
    const double grip = share * m.grip(speed);
    const double lateral = speed * speed * std::abs(k);
    const double a = std::sqrt(std::max(0.0, grip * grip - lateral * lateral));
    return accelerating ? std::min(a, share * m.ax_max(speed)) : a * brake_share / share;
  };
  for (int pass = 0; pass < (closed ? 2 : 1); ++pass) {
    for (std::size_t i = 0; i + 1 < n + (closed ? 1 : 0); ++i) {
      const std::size_t a = i % n, b = (i + 1) % n;
      v[b] = std::min(v[b], std::sqrt(v[a] * v[a] + 2.0 * avail(v[a], kappa[a], true) * ds[a]));
    }
    for (std::size_t i = n - 1 + (closed ? 1 : 0); i-- > 0;) {
      const std::size_t a = i % n, b = (i + 1) % n;
      v[a] = std::min(v[a], std::sqrt(v[b] * v[b] + 2.0 * avail(v[b], kappa[b], false) * ds[a]));
    }
  }
  return v;
}

}  // namespace

void MpcDriver::init_driver(const ParamTree& params, Context& ctx) {
  track_ = ctx.track();
  if (!track_) {
    throw std::invalid_argument("driver.type mpc needs a track (track.file or track.template)");
  }
  model_ = MpcModel::from_params(params);
  const std::string mode = params.get_string("driver.mode", "track");
  if (mode != "track" && mode != "race") {
    throw std::invalid_argument("driver.mode '" + mode + "' (expected track or race)");
  }
  race_ = mode == "race";
  const double horizon = params.get_double("driver.horizon_s", race_ ? 2.0 : 1.5);
  steps_ = static_cast<int>(params.get_int("driver.steps", 30));
  period_mpc_ = params.get_double("driver.mpc_period_s", 0.05);
  if (steps_ < 5 || steps_ > 80 || !(horizon > 0.2) || !(period_mpc_ >= period_)) {
    throw std::invalid_argument("driver: steps 5..80, horizon_s > 0.2 and mpc_period_s >= period_s");
  }
  dt_mpc_ = horizon / steps_;
  aggressiveness_ = params.get_double("driver.aggressiveness", race_ ? 0.97 : 0.95);
  margin_ = params.get_double("driver.margin_m", race_ ? 0.15 : 0.3);
  half_width_ = 0.5 * params.get_double("vehicle.track_front_m", 1.22) + 0.5 * params.get_double("vehicle.tire.width_m", 0.19);
  steering_ratio_ = params.get_double("vehicle.steering.ratio");
  max_delta_ = deg_to_rad(params.get_double("vehicle.steering.lock_deg")) / steering_ratio_;
  max_delta_rate_ = deg_to_rad(params.get_double("driver.road_wheel_rate_deg_per_s", 90.0));
  max_jerk_ = params.get_double("driver.max_jerk", 60.0);
  w_lateral_ = params.get_double("driver.weights.lateral", race_ ? 0.3 : w_lateral_);
  w_heading_ = params.get_double("driver.weights.heading", race_ ? 0.2 : w_heading_);
  w_speed_ = params.get_double("driver.weights.speed", w_speed_);
  w_steer_rate_ = params.get_double("driver.weights.steer_rate", w_steer_rate_);
  w_accel_rate_ = params.get_double("driver.weights.accel_rate", w_accel_rate_);
  w_progress_ = params.get_double("driver.weights.progress", w_progress_);
  if (!(aggressiveness_ > 0.0 && aggressiveness_ <= 1.2) || margin_ < 0.0) {
    throw std::invalid_argument("driver: aggressiveness in (0, 1.2] and margin_m >= 0");
  }

  // Human characteristics: on in tracking mode, off when racing.
  const bool human = params.get_bool("driver.human.enabled", !race_);
  delay_ = human ? params.get_double("driver.human.delay_s", 0.15) : 0.0;
  bandwidth_ = human ? params.get_double("driver.human.bandwidth_hz", 2.5) : 0.0;
  hand_rate_ = deg_to_rad(params.get_double("driver.human.steer_rate_deg_per_s", human ? 600.0 : 1500.0));
  noise_ = human ? deg_to_rad(params.get_double("driver.human.noise_deg", 0.3)) : 0.0;
  if (delay_ < 0.0 || delay_ > 0.5 || bandwidth_ < 0.0) {
    throw std::invalid_argument("driver.human: delay_s in [0, 0.5], bandwidth_hz >= 0");
  }
  rng_.emplace(ctx.make_rng("driver.noise"));

  // Pedal map: what full throttle and full brake give.
  wheel_radius_ = params.get_double("vehicle.wheel.radius_m");
  const double brake_torque = params.get_double("vehicle.brakes.max_pressure_bar", 30.0) *
                              (params.get_double("vehicle.brakes.front_gain_nm_per_bar", 16.0) +
                               params.get_double("vehicle.brakes.rear_gain_nm_per_bar", 5.0));
  brake_force_max_ = brake_torque / wheel_radius_ +
                     params.get_double("control.regen.torque_ratio", 0.0) * model_.drive_force;
  rolling_ = params.get_double("vehicle.tire.rolling_resistance", rolling_);
  kp_ = params.get_double("driver.speed_kp", kp_);
  ki_ = params.get_double("driver.speed_ki", ki_);
  ka_p_ = params.get_double("driver.accel_kp", ka_p_);
  ka_i_ = params.get_double("driver.accel_ki", ka_i_);
  learn_ = params.get_bool("driver.model.learn", true);
  feel_grip_ = params.get_bool("driver.model.feel_grip", true);
  base_mu_ = {model_.front.mu, model_.rear.mu};
  felt_grip_ = {1.0, 1.0};
  speed_scale_ = 1.0;
  model_.front.gain = params.get_double("driver.model.front_gain", 1.0);
  model_.rear.gain = params.get_double("driver.model.rear_gain", 1.0);
  if (!(model_.front.gain > 0.0 && model_.rear.gain > 0.0)) {
    throw std::invalid_argument("driver.model: front_gain and rear_gain must be positive");
  }
  if (kp_ < 0.0 || ki_ < 0.0 || ka_p_ < 0.0 || !(ka_i_ > 0.0)) {
    throw std::invalid_argument("driver: speed_kp, speed_ki, accel_kp >= 0 and accel_ki > 0");
  }

  // Reference along the centreline: curvature, widths, the line and its speeds.
  const auto& lines = track_->definition().lines;
  std::string line = params.get_string("driver.line", lines.empty() ? "center" : lines.front().name);
  line_ = track_->line(line);
  const TrackDefinition::Line* def = nullptr;
  for (const auto& l : lines) {
    if (l.name == line) {
      def = &l;
    }
  }
  ref_.closed = track_->closed();
  ref_.length = track_->length();
  const auto n = static_cast<std::size_t>(std::max(4.0, std::round(ref_.length / ref_.step)));
  ref_.step = ref_.length / static_cast<double>(ref_.closed ? n : n - 1);
  ref_.kappa.resize(n);
  ref_.width_left.resize(n);
  ref_.width_right.resize(n);
  ref_.n_ref.resize(n);
  std::vector<double> px(n), py(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double s = static_cast<double>(i) * ref_.step;
    const CurveSample c = track_->at(s);
    const Track::Widths w = track_->widths_at(s);
    ref_.kappa[i] = c.curvature;
    ref_.width_left[i] = w.left;
    ref_.width_right[i] = w.right;
    ref_.n_ref[i] = def != nullptr ? track_->line_offset(*def, s) : 0.0;
    px[i] = c.x - ref_.n_ref[i] * std::sin(c.heading);
    py[i] = c.y + ref_.n_ref[i] * std::cos(c.heading);
  }
  // The driven line: heading relative to the centreline, curvature and step length.
  ref_.psi_ref.resize(n);
  std::vector<double> kline(n), dsl(n);
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t a = ref_.closed ? (i + n - 1) % n : (i == 0 ? 0 : i - 1);
    const std::size_t b = ref_.closed ? (i + 1) % n : std::min(i + 1, n - 1);
    const double dn = ref_.n_ref[b] - ref_.n_ref[a];
    const double span = ref_.step * static_cast<double>(b >= a ? b - a : b + n - a);
    ref_.psi_ref[i] = span > 0.0 ? std::atan(dn / span) : 0.0;
    // Three-point curvature of the line points.
    const double ax = px[a], ay = py[a], bx = px[i], by = py[i], cx = px[b], cy = py[b];
    const double cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx);
    const double l1 = std::hypot(bx - ax, by - ay), l2 = std::hypot(cx - bx, cy - by), l3 = std::hypot(cx - ax, cy - ay);
    kline[i] = l1 * l2 * l3 > 1e-9 ? 2.0 * cross / (l1 * l2 * l3) : ref_.kappa[i];
    dsl[i] = std::hypot(px[b] - px[i], py[b] - py[i]);
    if (b == i) {
      dsl[i] = ref_.step;
    }
  }
  const double top = params.get_double("driver.profile.max_kph", 110.0) / 3.6;
  ref_.v_cap = qss_speed(model_, kline, dsl, aggressiveness_, aggressiveness_, top, ref_.closed);
  // Tracking: a driver who corners at `lateral_share` of the grip they expect and brakes
  // earlier and more gently (`brake_share`), keeping the rest (up to `aggressiveness`) to
  // catch up when the car is off the plan.
  const double lateral_share = params.get_double("driver.profile.lateral_share", race_ ? aggressiveness_ : 0.8);
  const double brake_share = params.get_double("driver.profile.brake_share", race_ ? aggressiveness_ : 0.55);
  if (!(lateral_share > 0.0 && lateral_share <= aggressiveness_ && brake_share > 0.0 && brake_share <= aggressiveness_)) {
    throw std::invalid_argument("driver.profile: lateral_share and brake_share in (0, aggressiveness]");
  }
  ref_.v_ref = qss_speed(model_, kline, dsl, lateral_share, brake_share, top, ref_.closed);
  if (!ref_.closed) {  // stop at the end of an open course
    ref_.v_ref.back() = ref_.v_cap.back() = 0.0;
    for (std::size_t i = n - 1; i > 0; --i) {
      const double brake = model_.grip(ref_.v_ref[i]) * 0.8;
      ref_.v_ref[i - 1] = std::min(ref_.v_ref[i - 1], std::sqrt(ref_.v_ref[i] * ref_.v_ref[i] + 2.0 * brake * dsl[i - 1]));
      ref_.v_cap[i - 1] = std::min(ref_.v_cap[i - 1], std::sqrt(ref_.v_cap[i] * ref_.v_cap[i] + 2.0 * brake * dsl[i - 1]));
    }
  }
  // A recorded speed profile (s along the centreline, m/s) replaces the computed one.
  if (params.has("driver.speed_profile")) {
    std::filesystem::path file = path_from_utf8(params.get_string("driver.speed_profile"));
    if (file.is_relative()) {
      file = path_from_utf8(params.get_string("project.dir", ".")) / file;
    }
    std::ifstream in(file);
    if (!in) {
      throw std::invalid_argument("driver.speed_profile: cannot read '" + path_to_utf8(file) + "'");
    }
    std::vector<std::pair<double, double>> pts;
    for (std::string row; std::getline(in, row);) {
      std::replace(row.begin(), row.end(), ',', ' ');
      std::istringstream r(row);
      double s = 0.0, v = 0.0;
      if (r >> s >> v) {
        pts.emplace_back(s, v);
      }
    }
    if (pts.size() < 2) {
      throw std::invalid_argument("driver.speed_profile: need rows 's_m, v_mps'");
    }
    std::sort(pts.begin(), pts.end());
    for (std::size_t i = 0; i < n; ++i) {
      const double s = static_cast<double>(i) * ref_.step;
      const auto it = std::lower_bound(pts.begin(), pts.end(), std::pair{s, -kInf});
      if (it == pts.begin()) {
        ref_.v_ref[i] = it->second;
      } else if (it == pts.end()) {
        ref_.v_ref[i] = pts.back().second;
      } else {
        const auto& [s0, v0] = *(it - 1);
        const auto& [s1, v1] = *it;
        ref_.v_ref[i] = v0 + (v1 - v0) * (s - s0) / std::max(s1 - s0, 1e-9);
      }
    }
  }

  SignalBus& bus = ctx.bus();
  x_in_ = bus.declare("veh.chassis.x", "m", "position east");
  y_in_ = bus.declare("veh.chassis.y", "m", "position north");
  yaw_in_ = bus.declare("veh.chassis.yaw", "rad", "heading, counter-clockwise from east");
  vx_in_ = bus.declare("veh.chassis.vx", "m/s", "longitudinal velocity (body frame)");
  vy_in_ = bus.declare("veh.chassis.vy", "m/s", "lateral velocity (body frame)");
  r_in_ = bus.declare("veh.chassis.yaw_rate", "rad/s", "yaw rate");
  ax_in_ = bus.declare("veh.chassis.ax", "m/s^2", "longitudinal acceleration (body frame)");
  ay_in_ = bus.declare("veh.chassis.ay", "m/s^2", "lateral acceleration (body frame)");
  for (std::size_t c = 0; c < grip_in_.size(); ++c) {
    grip_in_[c] = bus.declare("veh.wheel." + std::string(kCornerNames[c]) + ".tire_grip", "-",
                              "grip factor from temperature and pressure");
  }
  front_gain_out_ = bus.declare("drv.mpc.front_gain", "-", "front axle slip stiffness the driver has learnt (1: the tyre data)");
  rear_gain_out_ = bus.declare("drv.mpc.rear_gain", "-", "rear axle slip stiffness the driver has learnt (1: the tyre data)");
  status_out_ = bus.declare("drv.mpc.status", "-", "0 solved, 1 previous plan, 2 pure pursuit fallback");
  iter_out_ = bus.declare("drv.mpc.iterations", "-", "QP iterations of the last solve");
  fail_out_ = bus.declare("drv.mpc.failures", "-", "solves that failed so far");
  s_out_ = bus.declare("drv.mpc.s", "m", "position along the centreline the driver sees");
  n_out_ = bus.declare("drv.mpc.n", "m", "lateral position, positive left of the centreline");
  nref_out_ = bus.declare("drv.mpc.n_ref", "m", "the line the driver aims at");
  vref_out_ = bus.declare("drv.mpc.v_ref", "m/s", race_ ? "speed limit guide (quasi-steady)" : "reference speed");
  ax_out_ = bus.declare("drv.mpc.ax_cmd", "m/s^2", "planned longitudinal acceleration");
  delta_out_ = bus.declare("drv.mpc.delta_cmd", "rad", "planned road-wheel angle");
  target_out_ = bus.declare("drv.target_speed", "m/s", "speed the driver aims for");
  lateral_out_ = bus.declare("drv.line_offset", "m", "distance from the driving line, positive left");
  for (std::size_t i = 0; i < path_x_.size(); ++i) {
    path_x_[i] = bus.declare("drv.mpc.path." + std::to_string(i) + ".x", "m", "planned path point (world)");
    path_y_[i] = bus.declare("drv.mpc.path." + std::to_string(i) + ".y", "m", "planned path point (world)");
  }
}

int MpcDriver::substeps(double vx, double dt) const {
  // The tyre modes are fast at low speed: RK4 needs sub-steps there.
  const double stiffness = std::max(model_.front.mu * model_.front.b * model_.front.c,
                                    model_.rear.mu * model_.rear.b * model_.rear.c) * kG;  // per kg
  return std::clamp(static_cast<int>(std::ceil(dt * stiffness / std::max(vx, 3.0) / 2.0)), 1, 8);
}

MpcModel::State MpcDriver::simulate(MpcModel::State x, const MpcModel::Input& u, double dt, int sub) const {
  if (sub <= 0) {
    sub = substeps(x[MpcModel::Vx], dt);
  }
  const double h = dt / sub;
  for (int i = 0; i < sub; ++i) {
    const auto f = [&](const MpcModel::State& s) { return model_.derivative(s, u, ref_.at(ref_.kappa, s[0])); };
    const MpcModel::State k1 = f(x);
    MpcModel::State t{};
    for (std::size_t j = 0; j < 6; ++j) t[j] = x[j] + 0.5 * h * k1[j];
    const MpcModel::State k2 = f(t);
    for (std::size_t j = 0; j < 6; ++j) t[j] = x[j] + 0.5 * h * k2[j];
    const MpcModel::State k3 = f(t);
    for (std::size_t j = 0; j < 6; ++j) t[j] = x[j] + h * k3[j];
    const MpcModel::State k4 = f(t);
    for (std::size_t j = 0; j < 6; ++j) x[j] += h / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
  }
  return x;
}

bool MpcDriver::solve(const MpcModel::State& x0, MpcModel::Input applied) {
  const std::size_t N = static_cast<std::size_t>(steps_);
  const double dt = dt_mpc_;

  // Initial guess: the previous plan moved to the new origin, else steer along the line.
  std::vector<MpcModel::Input> ub(N);
  for (std::size_t k = 0; k < N; ++k) {
    if (plan_) {
      ub[k] = plan_->input_at(plan_time_ + static_cast<double>(k) * dt);
    } else {
      const double kappa = ref_.at(ref_.kappa, x0[0] + static_cast<double>(k) * dt * std::max(x0[3], 1.0));
      ub[k] = {std::atan((model_.lf + model_.lr) * kappa), 0.0};
    }
  }
  // Sequential QP: re-linearise while the steps are still large (one when it has converged).
  last_iterations_ = 0;
  bool solved = false;
  for (int iteration = 0; iteration < sqp_iterations_; ++iteration) {
    const std::optional<std::vector<MpcModel::Input>> step = qp_step(x0, applied, ub);
    if (!step) {
      break;
    }
    solved = true;
    double largest = 0.0;
    for (std::size_t k = 0; k < N; ++k) {
      ub[k][0] = std::clamp(ub[k][0] + (*step)[k][0], -max_delta_, max_delta_);
      ub[k][1] += (*step)[k][1];
      largest = std::max({largest, std::abs((*step)[k][0]) / kTrustDelta, std::abs((*step)[k][1]) / kTrustAccel});
    }
    if (largest < 0.5) {
      break;
    }
  }
  if (!solved) {
    return false;
  }
  auto plan = std::make_unique<Plan>();
  plan->dt = dt;
  plan->u = ub;
  plan->x.resize(N + 1);
  plan->x[0] = x0;
  for (std::size_t k = 0; k < N; ++k) {
    plan->x[k + 1] = simulate(plan->x[k], plan->u[k], dt, substeps(plan->x[k][MpcModel::Vx], dt));
  }
  plan_ = std::move(plan);
  return true;
}

std::optional<std::vector<MpcModel::Input>> MpcDriver::qp_step(const MpcModel::State& x0, MpcModel::Input applied,
                                                               const std::vector<MpcModel::Input>& ub) {
  const std::size_t N = static_cast<std::size_t>(steps_);
  const double dt = dt_mpc_;
  constexpr std::size_t nx = MpcModel::kStates;

  std::vector<MpcModel::State> xb(N + 1);
  xb[0] = x0;
  for (std::size_t k = 0; k < N; ++k) {
    xb[k + 1] = simulate(xb[k], ub[k], dt, substeps(xb[k][MpcModel::Vx], dt));
  }

  // Linearisation along the guess and condensing: dx_k = G_k dz, z per stage [d delta, d ax, slack].
  const std::size_t nz = 3 * N;
  std::vector<std::vector<double>> G(N + 1, std::vector<double>(nx * nz, 0.0));
  for (std::size_t k = 0; k < N; ++k) {
    double A[nx][nx], B[nx][2];
    const int sub = substeps(xb[k][MpcModel::Vx], dt);  // the same integration for every perturbation
    for (std::size_t j = 0; j < nx; ++j) {
      MpcModel::State xp = xb[k], xm = xb[k];
      const double e = 1e-5 * std::max(1.0, std::abs(xb[k][j]));
      xp[j] += e;
      xm[j] -= e;
      const MpcModel::State fp = simulate(xp, ub[k], dt, sub), fm = simulate(xm, ub[k], dt, sub);
      for (std::size_t i = 0; i < nx; ++i) A[i][j] = (fp[i] - fm[i]) / (2.0 * e);
    }
    for (std::size_t j = 0; j < 2; ++j) {
      MpcModel::Input up = ub[k], um = ub[k];
      const double e = j == 0 ? 1e-5 : 1e-4;
      up[j] += e;
      um[j] -= e;
      const MpcModel::State fp = simulate(xb[k], up, dt, sub), fm = simulate(xb[k], um, dt, sub);
      for (std::size_t i = 0; i < nx; ++i) B[i][j] = (fp[i] - fm[i]) / (2.0 * e);
    }
    const std::size_t cols = 3 * k;  // columns that can be nonzero in G_k
    for (std::size_t i = 0; i < nx; ++i) {
      double* out = &G[k + 1][i * nz];
      for (std::size_t c = 0; c < cols; ++c) {
        double v = 0.0;
        for (std::size_t j = 0; j < nx; ++j) v += A[i][j] * G[k][j * nz + c];
        out[c] = v;
      }
      out[3 * k] = B[i][0];
      out[3 * k + 1] = B[i][1];
    }
  }

  // Rows per stage k = 1..N: track left, track right, friction (lateral +-, four diagonals),
  // speed cap (racing); per input k = 0..N-1: steering rate +-, jerk +-, front and rear slip
  // angle +-.
  const std::size_t rows = N * (race_ ? 9 : 8) + 8 * N;
  QuadraticProgram qp;
  qp.resize(nz, rows);
  const auto row_of = [&](std::size_t k, std::size_t state) { return &G[k][state * nz]; };
  // Quadratic penalty w (e0 + c dz)^2 on a state component.
  const auto penalise = [&](std::size_t k, std::size_t state, double e0, double w) {
    const double* c = row_of(k, state);
    const std::size_t cols = 3 * k;
    for (std::size_t i = 0; i < cols; ++i) {
      if (c[i] == 0.0) continue;
      qp.g[i] += 2.0 * w * e0 * c[i];
      for (std::size_t j = 0; j < cols; ++j) qp.H[i * nz + j] += 2.0 * w * c[i] * c[j];
    }
  };
  for (std::size_t k = 1; k <= N; ++k) {
    const double s = xb[k][MpcModel::S];
    const double terminal = k == N ? 3.0 : 1.0;
    penalise(k, MpcModel::N, xb[k][MpcModel::N] - ref_.at(ref_.n_ref, s), w_lateral_ * terminal);
    penalise(k, MpcModel::Dpsi, xb[k][MpcModel::Dpsi] - ref_.at(ref_.psi_ref, s), w_heading_ * terminal);
    if (!race_) {
      penalise(k, MpcModel::Vx, xb[k][MpcModel::Vx] - speed_scale_ * ref_.at(ref_.v_ref, s), w_speed_ * terminal);
    }
  }
  if (race_) {  // progress: maximise s at the end of the horizon
    const double* c = row_of(N, MpcModel::S);
    for (std::size_t i = 0; i < nz; ++i) qp.g[i] -= w_progress_ * c[i];
  }
  // Input rates (from the input being applied now) and slack costs.
  for (std::size_t k = 0; k < N; ++k) {
    for (std::size_t j = 0; j < 2; ++j) {
      const double w = j == 0 ? w_steer_rate_ : w_accel_rate_;
      const double prev = k == 0 ? applied[j] : ub[k - 1][j];
      const double e0 = ub[k][j] - prev;
      const std::size_t a = 3 * k + j;
      qp.H[a * nz + a] += 2.0 * w;
      qp.g[a] += 2.0 * w * e0;
      if (k > 0) {
        const std::size_t b = 3 * (k - 1) + j;
        qp.H[b * nz + b] += 2.0 * w;
        qp.H[a * nz + b] -= 2.0 * w;
        qp.H[b * nz + a] -= 2.0 * w;
        qp.g[b] -= 2.0 * w * e0;
      }
    }
    const std::size_t sl = 3 * k + 2;
    qp.H[sl * nz + sl] += 2.0 * w_slack_;
    qp.g[sl] += w_slack_;
    qp.lb[sl] = 0.0;
  }
  for (std::size_t i = 0; i < nz; ++i) qp.H[i * nz + i] += 1e-6;

  std::size_t row = 0;
  const auto set_row = [&](const double* c, std::size_t cols, double scale, std::size_t slack, double slack_coef,
                           double rhs) {
    for (std::size_t i = 0; i < cols; ++i) qp.A[row * nz + i] = scale * c[i];
    qp.A[row * nz + slack] -= slack_coef;
    qp.b[row] = rhs;
    ++row;
  };
  std::vector<double> scratch(nz, 0.0);
  for (std::size_t k = 1; k <= N; ++k) {
    const double s = xb[k][MpcModel::S];
    const double n = xb[k][MpcModel::N];
    const double room_l = ref_.at(ref_.width_left, s) - half_width_ - margin_;
    const double room_r = ref_.at(ref_.width_right, s) - half_width_ - margin_;
    const std::size_t slack = 3 * (k - 1) + 2;
    set_row(row_of(k, MpcModel::N), 3 * k, 1.0, slack, 1.0, room_l - n);
    set_row(row_of(k, MpcModel::N), 3 * k, -1.0, slack, 1.0, room_r + n);
    // Friction: the lateral acceleration and the octagon |ax| + |ay| <= 1.25 g around the
    // friction circle, both linear in the step. (A tangent of the circle at the guess would
    // let the solver "brake its way" out of a saturated corner; bounds on ax worked out at the
    // guess let it plan a corner it cannot make.)
    const double vx = xb[k][MpcModel::Vx], r = xb[k][MpcModel::R];
    const double grip = aggressiveness_ * model_.grip(vx);
    const double ay = vx * r, ax = ub[k - 1][1];
    const double* gv = row_of(k, MpcModel::Vx);
    const double* gr = row_of(k, MpcModel::R);
    for (std::size_t i = 0; i < 3 * k; ++i) {
      scratch[i] = r * gv[i] + vx * gr[i];
    }
    set_row(scratch.data(), 3 * k, 1.0, slack, 0.2 * grip, grip - ay);
    set_row(scratch.data(), 3 * k, -1.0, slack, 0.2 * grip, grip + ay);
    for (const double sx : {1.0, -1.0}) {
      for (const double sy : {1.0, -1.0}) {
        qp.A[row * nz + 3 * (k - 1) + 1] = sx;  // the row below adds to it
        for (std::size_t i = 0; i < 3 * k; ++i) qp.A[row * nz + i] += sy * scratch[i];
        qp.A[row * nz + slack] -= 0.2 * grip;
        qp.b[row] = kFrictionOctagon * grip - sx * ax - sy * ay;
        ++row;
      }
    }
    if (race_) {
      const double cap = speed_scale_ * ref_.at(ref_.v_cap, s);
      set_row(row_of(k, MpcModel::Vx), 3 * k, 1.0, slack, std::max(cap, 1.0) * 0.1, cap - xb[k][MpcModel::Vx]);
    }
  }
  for (std::size_t k = 0; k < N; ++k) {
    for (std::size_t j = 0; j < 2; ++j) {
      const double limit = (j == 0 ? max_delta_rate_ : max_jerk_) * dt;
      const double prev = k == 0 ? applied[j] : ub[k - 1][j];
      const double e0 = ub[k][j] - prev;
      for (const double sign : {1.0, -1.0}) {
        qp.A[row * nz + 3 * k + j] = sign;
        if (k > 0) qp.A[row * nz + 3 * (k - 1) + j] = -sign;
        qp.b[row] = limit - sign * e0 + (k == 0 ? std::max(0.0, sign * e0 - limit) : 0.0);  // never infeasible at k = 0
        ++row;
      }
    }
    // Input bounds: the steering lock; braking and drive within the driver's grip, drive also
    // within what the motors give. (The gentler braking of the tracking driver is in the speed
    // profile: the bound leaves room to catch up.)
    qp.lb[3 * k] = -max_delta_ - ub[k][0];
    qp.ub[3 * k] = max_delta_ - ub[k][0];
    const double vx = xb[k][MpcModel::Vx];
    const double grip = aggressiveness_ * model_.grip(vx);
    qp.lb[3 * k + 1] = -grip - ub[k][1];
    qp.ub[3 * k + 1] = std::min(grip, model_.ax_max(vx)) - ub[k][1];
    // Trust region: the linearisation only holds near the guess (tyres saturate). A guess
    // outside the bounds steps towards them as far as the region allows.
    for (std::size_t j = 0; j < 2; ++j) {
      const double trust = j == 0 ? kTrustDelta : kTrustAccel;
      double& lo = qp.lb[3 * k + j];
      double& hi = qp.ub[3 * k + j];
      lo = std::clamp(lo, -trust, trust);
      hi = std::clamp(hi, -trust, trust);
    }

    // Slip angles within the peak of the tyre curve: past it more lock only scrubs (and the
    // driver's model, flat there, could not tell the solver so).
    const MpcModel::State& xs = xb[k];
    const double ve = std::max(xs[MpcModel::Vx], 3.0);
    for (const bool front : {true, false}) {
      const MpcModel::Axle& axle = front ? model_.front : model_.rear;
      const double peak = axle.peak_slip();
      const double arm = front ? model_.lf : -model_.lr;
      const double q = (xs[MpcModel::Vy] + arm * xs[MpcModel::R]) / ve;
      const double slip = axle.gain * ((front ? ub[k][0] : 0.0) - std::atan(q));
      const double dq = axle.gain / (1.0 + q * q);
      const double dvx = xs[MpcModel::Vx] > 3.0 ? -q / ve : 0.0;
      const double* gvx = row_of(k, MpcModel::Vx);
      const double* gvy = row_of(k, MpcModel::Vy);
      const double* gr = row_of(k, MpcModel::R);
      for (std::size_t i = 0; i < 3 * k; ++i) {
        scratch[i] = -dq * (gvy[i] / ve + arm * gr[i] / ve + dvx * gvx[i]);
      }
      for (const double sign : {1.0, -1.0}) {
        for (std::size_t i = 0; i < 3 * k; ++i) qp.A[row * nz + i] = sign * scratch[i];
        if (front) {
          qp.A[row * nz + 3 * k] = sign * axle.gain;
        }
        qp.A[row * nz + 3 * k + 2] = -0.2 * peak;
        qp.b[row] = peak - sign * slip;
        ++row;
      }
    }
  }

  const QpResult result = solve_qp(qp);
  last_iterations_ += result.iterations;
  bool ok = result.converged || (result.primal_residual < 1e-4 && result.dual_residual < 1e-3);
  for (const double v : result.z) {
    ok = ok && std::isfinite(v);
  }
  if (!ok) {
    return std::nullopt;
  }
  std::vector<MpcModel::Input> step(N);
  for (std::size_t k = 0; k < N; ++k) {
    step[k] = {result.z[3 * k], result.z[3 * k + 1]};
  }
  return step;
}

void MpcDriver::learn(SignalBus& bus, double vx, double vy, double r) {
  // How each axle bites, as the driver feels it: the axle forces from the measured lateral
  // and yaw acceleration, the slip angle the driver's tyre curve needs for each, and the slip
  // the car's motion and the steering give. The ratio scales the axle's slip in the model; it
  // absorbs what the single track leaves out (Ackermann, compliance and roll steer, toe,
  // camber, the tyres' own stiffness) and follows the car as it changes. All signals pass
  // the same low-pass filter.
  const double delta = command_.steer / steering_ratio_;
  const double rdot = have_last_ ? (r - last_r_) / period_ : 0.0;
  last_r_ = r;
  const std::array<double, 7> raw = {vx, vy, r, delta, bus.get(ax_in_), bus.get(ay_in_), rdot};
  const double a = have_last_ ? period_ / (kLearnFilter + period_) : 1.0;
  have_last_ = true;
  for (std::size_t i = 0; i < raw.size(); ++i) felt_[i] += a * (raw[i] - felt_[i]);
  const auto [fvx, fvy, fr, fdelta, fax, fay, frdot] = felt_;
  bus.set(front_gain_out_, model_.front.gain);
  bus.set(rear_gain_out_, model_.rear.gain);
  if (!learn_ || fvx < 6.0 || std::abs(fay) < 2.0) {
    return;
  }
  const double wheelbase = model_.lf + model_.lr;
  const double aero = 0.5 * model_.air_density * model_.cla * fvx * fvx;
  const double transfer = model_.mass * fax * model_.cg_height / wheelbase;
  for (const bool front : {true, false}) {
    MpcModel::Axle& axle = front ? model_.front : model_.rear;
    const double load = model_.mass * kG * (front ? model_.lr : model_.lf) / wheelbase +
                        (front ? model_.aero_front : 1.0 - model_.aero_front) * aero + (front ? -transfer : transfer);
    const double force = (model_.mass * fay * (front ? model_.lr : model_.lf) +
                          (front ? 1.0 : -1.0) * model_.yaw_inertia * frdot) / wheelbase;
    const double ratio = force / std::max(axle.mu_at(load) * load, 1.0);
    const double slip = front ? fdelta - std::atan((fvy + model_.lf * fr) / fvx) : -std::atan((fvy - model_.lr * fr) / fvx);
    if (std::abs(ratio) > 0.75 || std::abs(slip) < 0.01) {  // near the peak the slip is ill-defined
      continue;
    }
    const double needed = std::tan(std::asin(ratio) / axle.c) / axle.b;
    const double gain = needed / slip;
    if (gain > 0.4 && gain < 2.0) {
      axle.gain += period_ / kLearnTime * (gain - axle.gain);
    }
  }
}

void MpcDriver::feel_grip(const SignalBus& bus) {
  // The grip the tires have now (temperature, pressure) as the driver feels it through the
  // car: it scales what the driver expects of each axle and the speeds it aims for (cornering
  // speed goes with the square root of the grip).
  if (!feel_grip_) {
    return;
  }
  const double a = period_ / (kFeelTime + period_);
  for (std::size_t axle = 0; axle < 2; ++axle) {
    double grip = 0.0;
    for (std::size_t side = 0; side < 2; ++side) {
      const double g = bus.get(grip_in_[2 * axle + side]);
      grip += 0.5 * (g > 0.0 ? g : 1.0);  // 0: the plant has no tire temperatures
    }
    felt_grip_[axle] += a * (grip - felt_grip_[axle]);
  }
  model_.front.mu = base_mu_[0] * felt_grip_[0];
  model_.rear.mu = base_mu_[1] * felt_grip_[1];
  speed_scale_ = std::sqrt(std::min(felt_grip_[0], felt_grip_[1]));
}

double MpcDriver::pursuit_steer(double x, double y, double yaw, double speed) {
  const double wheelbase = model_.lf + model_.lr;
  const double rx = x - model_.lr * std::cos(yaw), ry = y - model_.lr * std::sin(yaw);
  const CurveProjection here = pursuit_first_ ? line_.project(rx, ry) : line_.project(rx, ry, pursuit_s_);
  pursuit_first_ = false;
  pursuit_s_ = here.s;
  const double lookahead = std::clamp(2.5 + 0.12 * speed, 2.5, 12.0);
  const CurveSample goal = line_.at(pursuit_s_ + lookahead);
  const double dx = goal.x - rx, dy = goal.y - ry;
  const double alpha = std::remainder(std::atan2(dy, dx) - yaw, 2.0 * kPi);
  return std::atan(2.0 * wheelbase * std::sin(alpha) / std::max(std::hypot(dx, dy), 0.5));
}

void MpcDriver::step(double t, SignalBus& bus) {
  const double x = bus.get(x_in_), y = bus.get(y_in_), yaw = bus.get(yaw_in_);
  const double vx = bus.get(vx_in_), vy = bus.get(vy_in_), r = bus.get(r_in_);
  const CurveProjection here = have_hint_ ? track_->project(x, y, s_hint_) : track_->project(x, y);
  have_hint_ = true;
  s_hint_ = here.s;
  const MpcModel::State state = {here.s, here.n, std::remainder(yaw - here.heading, 2.0 * kPi), vx, vy, r};
  learn(bus, vx, vy, r);
  feel_grip(bus);
  bus.set(s_out_, here.s);
  bus.set(n_out_, here.n);
  bus.set(nref_out_, ref_.at(ref_.n_ref, here.s));
  bus.set(lateral_out_, here.n - ref_.at(ref_.n_ref, here.s));

  // End of an open course: stop.
  if (!ref_.closed && here.s > ref_.length - 1.0) {
    command_.throttle = 0.0;
    command_.brake = 1.0;
    write_command(bus, command_);
    bus.set(target_out_, 0.0);
    return;
  }

  // Inputs already on their way to the hands (reaction delay).
  const auto delay_steps = static_cast<std::size_t>(std::lround(delay_ / period_));
  if (t + 1e-9 >= next_solve_) {
    // Where the car will be when this plan starts to act.
    MpcModel::State x0 = state;
    for (const Delayed& d : delay_line_) {
      x0 = simulate(x0, {d.delta, d.ax}, period_, 0);
    }
    const MpcModel::Input applied = delay_line_.empty()
                                        ? MpcModel::Input{command_.steer / steering_ratio_, 0.0}
                                        : MpcModel::Input{delay_line_.back().delta, delay_line_.back().ax};
    plan_time_ = t + delay_;
    ++solves_;
    if (solve(x0, applied)) {
      plan_->origin = plan_time_;
      failed_in_row_ = 0;
      bus.set(status_out_, 0.0);
    } else {
      ++failures_;
      ++failed_in_row_;
      if (failed_in_row_ > 3) {
        plan_.reset();
      }
      bus.set(status_out_, plan_ ? 1.0 : 2.0);
      bus.set(fail_out_, static_cast<double>(failures_));
    }
    bus.set(iter_out_, last_iterations_);
    next_solve_ = std::max(next_solve_ + period_mpc_, t + 0.5 * period_mpc_);
    if (plan_) {
      for (std::size_t i = 0; i < path_x_.size(); ++i) {
        const std::size_t k = (i + 1) * (plan_->x.size() - 1) / path_x_.size();
        const MpcModel::State& p = plan_->x[k];
        const CurveSample c = track_->at(p[MpcModel::S]);
        bus.set(path_x_[i], c.x - p[MpcModel::N] * std::sin(c.heading));
        bus.set(path_y_[i], c.y + p[MpcModel::N] * std::cos(c.heading));
      }
    }
  }

  // What the plan wants at the moment the hands will act on it.
  Delayed now{};
  if (plan_) {
    // The hands lag the intent (neuromuscular filter): the driver steers ahead by its group delay.
    const double lead = bandwidth_ > 0.0 ? 2.0 * kHandDamping / (2.0 * kPi * bandwidth_) : 0.0;
    const MpcModel::Input u = plan_->input_at(t + delay_);
    now = {plan_->input_at(t + delay_ + lead)[0], u[1], plan_->speed_at(t + delay_)};
  } else {
    ++fallbacks_;
    const double v_ref = speed_scale_ * ref_.at(ref_.v_ref, here.s + vx * 0.3);
    now = {pursuit_steer(x, y, yaw, vx), std::clamp((v_ref - vx) * 2.0, -8.0, 8.0), v_ref};
  }
  delay_line_.push_back(now);
  Delayed act = now;
  while (delay_line_.size() > delay_steps) {
    act = delay_line_.front();
    delay_line_.pop_front();
  }
  if (delay_steps == 0) {
    delay_line_.clear();
  }

  // Steering: neuromuscular lag (second order), hand rate limit, noise.
  const double target = act.delta * steering_ratio_;
  if (bandwidth_ > 0.0) {
    const double w = 2.0 * kPi * bandwidth_;
    steer_vel_ += period_ * (w * w * (target - steer_pos_) - 2.0 * kHandDamping * w * steer_vel_);
    steer_pos_ += period_ * steer_vel_;
  } else {
    steer_pos_ = target;
  }
  if (noise_ > 0.0) {
    const double a = std::exp(-period_ / 0.5);
    noise_state_ = a * noise_state_ + std::sqrt(1.0 - a * a) * noise_ * rng_->normal();
  }
  const double max_change = hand_rate_ * period_;
  command_.steer += std::clamp(steer_pos_ + noise_state_ - command_.steer, -max_change, max_change);

  // Pedals: force for the planned acceleration (drag and rolling included), corrected by what
  // the driver feels (PI on the acceleration: the car brakes less than expected, press harder)
  // and PI on the planned speed.
  const double ax_error = act.ax - bus.get(ax_in_);
  accel_integral_ = std::clamp(accel_integral_ + ax_error * period_, -kAccelCorrection / ka_i_, kAccelCorrection / ka_i_);
  const double correction = std::clamp(ka_p_ * ax_error + ka_i_ * accel_integral_, -kAccelCorrection, kAccelCorrection);
  const double drag = 0.5 * model_.air_density * model_.cda * vx * vx + rolling_ * model_.mass * kG;
  const double force = model_.mass * (act.ax + correction) + (vx > 0.5 ? drag : 0.0);
  const double ff = force >= 0.0 ? force / model_.drive_force : force / brake_force_max_;
  const double error = act.v - vx;
  const double limit = ki_ > 0.0 ? 0.5 / ki_ : 0.0;
  integral_ = std::clamp(integral_ + error * period_, -limit, limit);
  const double u = ff + kp_ * error + ki_ * integral_;
  command_.throttle = std::clamp(u, 0.0, 1.0);
  command_.brake = std::clamp(-u, 0.0, 1.0);
  write_command(bus, command_);
  bus.set(target_out_, act.v);
  bus.set(vref_out_, speed_scale_ * ref_.at(race_ ? ref_.v_cap : ref_.v_ref, here.s));
  bus.set(ax_out_, act.ax);
  bus.set(delta_out_, act.delta);
}

std::vector<std::array<double, 8>> MpcDriver::planned() const {
  std::vector<std::array<double, 8>> out;
  if (plan_) {
    for (std::size_t k = 0; k < plan_->u.size(); ++k) {
      const auto& x = plan_->x[k];
      out.push_back({x[0], x[1], x[2], x[3], x[4], x[5], plan_->u[k][0], plan_->u[k][1]});
    }
  }
  return out;
}

}  // namespace vsim

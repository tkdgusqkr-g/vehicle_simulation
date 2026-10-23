#include "vsim/sensors/sensor_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <numbers>
#include <optional>
#include <stdexcept>

#include "vsim/models/vehicle/corners.hpp"
#include "vsim/sensors/sensor_chain.hpp"
#include "vsim/sim/context.hpp"

namespace vsim {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;
constexpr double kG = 9.81;

// An input signal that may be missing (then a fallback value).
struct Input {
  std::optional<SignalId> id;
  double get(const SignalBus& bus, double fallback = 0.0) const { return id ? bus.get(*id) : fallback; }
};

struct Mat3 {
  std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};
  // Rotation from roll (x), pitch (y), yaw (z), applied in that order to
  // vectors of the rotated frame: R = Rz(yaw) Ry(pitch) Rx(roll).
  static Mat3 from_euler(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    Mat3 r;
    r.m = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
           sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
           -sp,     cp * sr,                cp * cr};
    return r;
  }
  std::array<double, 3> times(const std::array<double, 3>& v) const {
    return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
  }
  // Transpose times v: a vector of the outer frame in the rotated frame.
  std::array<double, 3> transpose_times(const std::array<double, 3>& v) const {
    return {m[0] * v[0] + m[3] * v[1] + m[6] * v[2], m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
            m[2] * v[0] + m[5] * v[1] + m[8] * v[2]};
  }
};

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

std::array<double, 3> read_vec(const ParamTree& params, const std::string& key, std::array<double, 3> fallback) {
  const std::vector<double> v = params.get_double_list(key);
  if (v.empty()) {
    return fallback;
  }
  if (v.size() != 3) {
    throw std::invalid_argument(key + " needs three values");
  }
  return {v[0], v[1], v[2]};
}

}  // namespace

// One sensor: continuous bookkeeping every block step, a sample at its rate.
class SensorSystem::Sensor {
 public:
  struct Setup {
    const ParamTree& params;
    Context& ctx;
    std::string prefix;  // vehicle.sensors.<i>
    std::string name;
    double dt = 0.001;
    bool real = false, variation = false;
  };

  Sensor(const Setup& s, double default_rate) {
    const double rate = s.params.get_double(s.prefix + ".rate_hz", default_rate);
    if (!(rate > 0.0)) {
      throw std::invalid_argument(s.prefix + ".rate_hz must be positive");
    }
    // Samples land on plant steps: faster sensors sample every step.
    interval_ = std::max(s.dt, std::round(1.0 / rate / s.dt) * s.dt);
  }
  virtual ~Sensor() = default;

  void tick(double t, double dt, SignalBus& bus) {
    update(t, dt, bus);
    if (t + 1e-9 >= next_) {
      sample(t, bus);
      while (next_ <= t + 1e-9) {
        next_ += interval_;
      }
    }
  }

 protected:
  virtual void update(double /*t*/, double /*dt*/, const SignalBus& /*bus*/) {}
  virtual void sample(double t, SignalBus& bus) = 0;

  // A channel with the scenario's overrides of the defaults, its own random
  // streams and its faults.
  ChannelModel channel(const Setup& s, const std::string& key, const ChannelParams& defaults,
                       double supply = 0.0) const {
    const ChannelParams p = ChannelParams::from_params(s.params, s.prefix + "." + key, defaults);
    ChannelModel model(p, 1.0 / interval_, s.ctx.make_rng("sensors." + s.name + "." + key),
                       s.ctx.make_rng("variation.sensors." + s.name + "." + key), s.real, s.variation);
    for (const ChannelFault& f : faults_from_params(s.params, s.prefix, key, supply)) {
      model.add_fault(f);
    }
    return model;
  }
  SignalId output(const Setup& s, const std::string& key, const char* unit, const char* what) const {
    return s.ctx.bus().declare("sens." + s.name + "." + key, unit, what);
  }

  double interval_ = 0.001;
  double next_ = 0.0;
};

namespace {

using Setup = SensorSystem::Sensor::Setup;

// --- IMU, accelerometer, gyro ------------------------------------------------------------
class Imu final : public SensorSystem::Sensor {
 public:
  Imu(const Setup& s, bool accel, bool gyro) : Sensor(s, 500.0), accel_(accel), gyro_(gyro) {
    const ParamTree& p = s.params;
    const SignalBus& bus = s.ctx.bus();
    position_ = read_vec(p, s.prefix + ".position", {0.0, 0.0, p.get_double("vehicle.cg_height_m", 0.3)});
    std::array<double, 3> orient = read_vec(p, s.prefix + ".orientation_deg", {0.0, 0.0, 0.0});
    if (s.variation) {
      // Mounting misalignment of this unit (bracket, board, chip).
      Rng rng = s.ctx.make_rng("variation.sensors." + s.name + ".mount");
      const double sigma = p.get_double(s.prefix + ".misalignment_deg", 0.5);
      for (double& a : orient) {
        a += sigma * rng.normal();
      }
    }
    mount_ = Mat3::from_euler(orient[0] * kDeg, orient[1] * kDeg, orient[2] * kDeg);
    anti_alias_ = p.get_double(s.prefix + ".anti_alias_hz", 0.0);

    // A MEMS automotive IMU (datasheet-like defaults).
    ChannelParams a;
    a.noise_density = 150e-6 * kG;   // 150 ug/sqrt(Hz)
    a.bias_instability = 40e-6 * kG;
    a.bias = 20e-3 * kG;             // 20 mg zero-g offset spread
    a.scale_error = 0.003;
    a.bandwidth = 200.0;
    a.resolution = 16.0 * kG / 32768.0;
    a.min = -16.0 * kG;
    a.max = 16.0 * kG;
    ChannelParams g;
    g.noise_density = 0.005 * kDeg;  // 0.005 deg/s/sqrt(Hz)
    g.bias_instability = 3.0 / 3600.0 * kDeg;
    g.bias = 0.3 * kDeg;
    g.scale_error = 0.003;
    g.bandwidth = 200.0;
    g.resolution = 500.0 * kDeg / 32768.0;
    g.min = -500.0 * kDeg;
    g.max = 500.0 * kDeg;
    for (int i = 0; i < 3; ++i) {
      const char axis = "xyz"[i];
      if (accel_) {
        accel_ch_.push_back(channel(s, std::string("a") + axis, ChannelParams::from_params(p, s.prefix + ".accel", a)));
        accel_out_[i] = output(s, std::string("a") + axis, "m/s^2", "specific force at the mount, sensor axes");
      }
      if (gyro_) {
        gyro_ch_.push_back(channel(s, std::string("g") + axis, ChannelParams::from_params(p, s.prefix + ".gyro", g)));
        gyro_out_[i] = output(s, std::string("g") + axis, "rad/s", "angular rate, sensor axes");
      }
    }
    const auto in = [&](const char* name) { return Input{bus.find(name)}; };
    ax_ = in("veh.chassis.ax");
    ay_ = in("veh.chassis.ay");
    az_ = in("veh.chassis.az");
    roll_ = in("veh.chassis.roll");
    pitch_ = in("veh.chassis.pitch");
    p_ = in("veh.chassis.roll_rate");
    q_ = in("veh.chassis.pitch_rate");
    r_ = in("veh.chassis.yaw_rate");
    dp_ = in("veh.chassis.roll_accel");
    dq_ = in("veh.chassis.pitch_accel");
    dr_ = in("veh.chassis.yaw_accel");
    // Motor-order vibration picked up at the mount (above the plant's
    // bandwidth, so made here): amplitude [m/s^2] along a sensor axis.
    for (std::size_t i = 0;; ++i) {
      const std::string base = s.prefix + ".vibration." + std::to_string(i);
      if (!p.has_prefix(base)) {
        break;
      }
      Vibration v;
      v.order = p.get_double(base + ".order");
      v.amplitude = p.get_double(base + ".amplitude");
      const std::string axis = p.get_string(base + ".axis", "z");
      v.axis = axis == "x" ? 0 : axis == "y" ? 1 : 2;
      // The named motor, or the first one.
      const std::string motor = p.get_string(base + ".motor", "");
      v.speed = Input{bus.find("pt.motor." + motor + ".speed")};
      if (!v.speed.id) {
        // Another driveline (scenario override): the first motor.
        const std::vector<SignalId> motors = bus.match("pt.motor.*.speed");
        if (!motor.empty() && motors.empty()) {
          throw std::invalid_argument(base + ".motor: no motor '" + motor + "' on this vehicle");
        }
        v.speed = Input{motors.empty() ? std::nullopt : std::optional<SignalId>(motors.front())};
      }
      vibration_.push_back(v);
    }
    real_ = s.real;
  }

 protected:
  void update(double /*t*/, double dt, const SignalBus& bus) override {
    for (Vibration& v : vibration_) {
      v.frequency = std::abs(v.speed.get(bus)) * v.order / (2.0 * kPi);
      v.phase = std::fmod(v.phase + 2.0 * kPi * v.frequency * dt, 2.0 * kPi);
    }
  }

  void sample(double t, SignalBus& bus) override {
    // Body attitude and angular velocity in the car's level (yawing) frame:
    // roll and pitch rates are Euler rates, yaw rate about the vertical.
    const double roll = roll_.get(bus), pitch = pitch_.get(bus);
    const double p = p_.get(bus), q = q_.get(bus), r = r_.get(bus);
    const std::array<double, 3> w = {p * std::cos(pitch), q, r - p * std::sin(pitch)};
    const std::array<double, 3> dw = {dp_.get(bus), dq_.get(bus), dr_.get(bus)};
    const Mat3 body = Mat3::from_euler(roll, pitch, 0.0);
    // Acceleration of the mount: the frame origin's, body heave, and the
    // rotation about the origin (tangential and centripetal).
    const std::array<double, 3> arm = body.times(position_);
    const std::array<double, 3> tangential = cross(dw, arm);
    const std::array<double, 3> centripetal = cross(w, cross(w, arm));
    const std::array<double, 3> a = {ax_.get(bus) + tangential[0] + centripetal[0],
                                     ay_.get(bus) + tangential[1] + centripetal[1],
                                     az_.get(bus) + tangential[2] + centripetal[2] + kG};  // specific force: minus gravity
    // Into the body (rolled and pitched), then the sensor's mounting.
    const std::array<double, 3> f = mount_.transpose_times(body.transpose_times(a));
    const std::array<double, 3> rate = mount_.transpose_times(body.transpose_times(w));
    std::array<double, 3> vib{};
    if (real_) {
      for (const Vibration& v : vibration_) {
        // An analog prefilter before sampling (second order) attenuates what
        // would otherwise alias to low frequencies.
        double gain = 1.0, lag = 0.0;
        if (anti_alias_ > 0.0) {
          const double x = v.frequency / anti_alias_;
          gain = 1.0 / (1.0 + x * x);
          lag = 2.0 * std::atan(x);
        }
        vib[v.axis] += v.amplitude * gain * std::sin(v.phase - lag);
      }
    }
    for (std::size_t i = 0; i < 3; ++i) {
      if (accel_) {
        bus.set(accel_out_[i], accel_ch_[i].sample(t, f[i] + vib[i]));
      }
      if (gyro_) {
        bus.set(gyro_out_[i], gyro_ch_[i].sample(t, rate[i]));
      }
    }
  }

 private:
  struct Vibration {
    double order = 1.0, amplitude = 0.0;
    int axis = 2;
    Input speed;
    double frequency = 0.0, phase = 0.0;
  };
  bool accel_, gyro_, real_ = false;
  std::array<double, 3> position_{};
  Mat3 mount_;
  double anti_alias_ = 0.0;
  std::vector<ChannelModel> accel_ch_, gyro_ch_;
  std::array<SignalId, 3> accel_out_{}, gyro_out_{};
  Input ax_, ay_, az_, roll_, pitch_, p_, q_, r_, dp_, dq_, dr_;
  std::vector<Vibration> vibration_;
};

// Corner from the sensor entry (wheel: fl) or its position.
std::size_t corner_of(const ParamTree& params, const std::string& prefix) {
  const std::string wheel = params.get_string(prefix + ".wheel", "");
  if (!wheel.empty()) {
    return corner_index(wheel);
  }
  if (!params.has_prefix(prefix + ".position")) {
    throw std::invalid_argument(prefix + ": needs `wheel` (fl, fr, rl, rr) or a position");
  }
  const std::array<double, 3> pos = read_vec(params, prefix + ".position", {0.0, 0.0, 0.0});
  return (pos[0] >= 0.0 ? 0 : 2) + (pos[1] >= 0.0 ? 0 : 1);
}
std::size_t corner_of(const Setup& s) { return corner_of(s.params, s.prefix); }

// --- Wheel speed: toothed wheel and pulse timing ------------------------------------------
class WheelSpeed final : public SensorSystem::Sensor {
 public:
  explicit WheelSpeed(const Setup& s) : Sensor(s, 200.0) {
    const ParamTree& p = s.params;
    const std::size_t corner = corner_of(s);
    omega_ = Input{s.ctx.bus().find("veh.wheel." + std::string(kCornerNames[corner]) + ".omega")};
    teeth_ = static_cast<int>(p.get_int(s.prefix + ".teeth", 48));
    timeout_ = p.get_double(s.prefix + ".timeout_s", 0.1);
    resolution_ = p.get_double(s.prefix + ".timer_resolution_s", 1e-6);
    directional_ = p.get_bool(s.prefix + ".directional", true);
    if (teeth_ < 1 || !(timeout_ > 0.0)) {
      throw std::invalid_argument(s.prefix + ": teeth and timeout_s must be positive");
    }
    // Tooth spacing errors of this ring (manufacturing).
    tooth_error_.assign(static_cast<std::size_t>(teeth_), 0.0);
    if (s.variation) {
      Rng rng = s.ctx.make_rng("variation.sensors." + s.name + ".teeth");
      const double sigma = p.get_double(s.prefix + ".tooth_error_deg", 0.05) * kDeg;
      for (double& e : tooth_error_) {
        e = sigma * rng.normal();
      }
    }
    real_ = s.real;
    omega_out_ = output(s, "omega", "rad/s", "wheel speed from pulse timing");
    pulses_out_ = output(s, "pulses", "-", "pulse count");
  }

 protected:
  void update(double t, double dt, const SignalBus& bus) override {
    const double omega = omega_.get(bus);
    truth_ = omega;
    const double angle0 = angle_, angle1 = angle_ + omega * dt;
    angle_ = angle1;
    if (!real_ || omega == 0.0) {
      return;
    }
    // Teeth whose edge lies between the two angles (edges at k * pitch + error).
    const double pitch = 2.0 * kPi / teeth_;
    const double lo = std::min(angle0, angle1), hi = std::max(angle0, angle1);
    for (auto k = static_cast<long long>(std::floor(lo / pitch)) - 1; k <= static_cast<long long>(std::floor(hi / pitch)) + 1;
         ++k) {
      const auto tooth = static_cast<std::size_t>(((k % teeth_) + teeth_) % teeth_);
      const double edge = static_cast<double>(k) * pitch + tooth_error_[tooth];
      if (edge <= lo || edge > hi) {
        continue;
      }
      double when = t - dt + (edge - angle0) / (angle1 - angle0) * dt;
      when = std::round(when / resolution_) * resolution_;  // capture timer
      edges_.push_back({when, omega > 0.0 ? 1 : -1});
      ++pulses_;
    }
    while (edges_.size() > 64) {
      edges_.pop_front();
    }
  }

  void sample(double t, SignalBus& bus) override {
    double omega = truth_;
    if (real_) {
      omega = 0.0;
      // Period method over the edges since the last sample (at least two),
      // nothing if the last edge is older than the timeout.
      if (edges_.size() >= 2 && t - edges_.back().t <= timeout_) {
        std::size_t first = edges_.size() - 2;
        while (first > 0 && edges_[first - 1].t > last_sample_) {
          --first;
        }
        const double span = edges_.back().t - edges_[first].t;
        const auto count = static_cast<double>(edges_.size() - 1 - first);
        if (span > 0.0) {
          omega = count * 2.0 * kPi / teeth_ / span;
          if (directional_ && edges_.back().direction < 0) {
            omega = -omega;
          }
        }
      }
    }
    last_sample_ = t;
    bus.set(omega_out_, omega);
    bus.set(pulses_out_, static_cast<double>(pulses_));
  }

 private:
  struct Edge {
    double t;
    int direction;
  };
  Input omega_;
  int teeth_ = 48;
  double timeout_ = 0.1, resolution_ = 1e-6;
  bool directional_ = true, real_ = false;
  std::vector<double> tooth_error_;
  double angle_ = 0.0, truth_ = 0.0, last_sample_ = -1.0;
  std::deque<Edge> edges_;
  long long pulses_ = 0;
  SignalId omega_out_ = 0, pulses_out_ = 0;
};

// The bus signal a sensor type measures by default ("" if none or several).
std::string default_source(const std::string& type, const ParamTree& params, const std::string& prefix) {
  if (type == "wheel_speed" || type == "suspension_travel") {
    const std::string corner(kCornerNames[corner_of(params, prefix)]);
    return "veh.wheel." + corner + (type == "wheel_speed" ? ".omega" : ".travel");
  }
  if (type == "brake_pressure") {
    return "veh.brake.pressure";
  }
  if (type == "current" || type == "voltage") {
    return "bat." + type;
  }
  if (type == "steering_angle") {
    return "drv.cmd.steer";
  }
  return "";
}

// --- One value from a bus signal through a channel ----------------------------------------
class SignalSensor final : public SensorSystem::Sensor {
 public:
  SignalSensor(const Setup& s, double rate, const std::string& signal, const std::string& key, const char* unit,
               const char* what, ChannelParams defaults, double scale = 1.0)
      : Sensor(s, rate), scale_(scale) {
    const std::string source = s.params.get_string(s.prefix + ".signal", signal);
    if (source.empty()) {
      throw std::invalid_argument(s.prefix + ": this sensor type needs `signal` (the bus signal it measures)");
    }
    input_ = Input{s.ctx.bus().find(source)};
    if (!input_.id) {
      throw std::invalid_argument(s.prefix + ": the vehicle model has no signal '" + source + "' to measure");
    }
    channel_.emplace_back(channel(s, key, defaults));
    out_ = output(s, key, unit, what);
  }

 protected:
  void sample(double t, SignalBus& bus) override {
    bus.set(out_, channel_.front().sample(t, scale_ * input_.get(bus)));
  }

 private:
  Input input_;
  double scale_ = 1.0;
  std::vector<ChannelModel> channel_;
  SignalId out_ = 0;
};

// --- Pedal position: two tracks with different transfer functions (FSAE APPS) -------------
class Pedal final : public SensorSystem::Sensor {
 public:
  explicit Pedal(const Setup& s) : Sensor(s, 1000.0) {
    const ParamTree& p = s.params;
    const std::string pedal = p.get_string(s.prefix + ".pedal", "throttle");
    pedal_ = Input{s.ctx.bus().find("drv.cmd." + pedal)};
    ChannelParams v;
    v.noise_density = 0.2e-3;  // [V/sqrt(Hz)] wiper and supply noise
    v.bias = 0.02;
    v.scale_error = 0.01;
    v.resolution = 5.0 / 4096.0;  // 12-bit ADC on 5 V
    v.min = 0.0;
    v.max = 5.0;
    for (int i = 0; i < 2; ++i) {
      const std::string key = i == 0 ? "ch1" : "ch2";
      const std::vector<double> map = p.get_double_list(s.prefix + "." + key + "_v");
      map_[i] = map.size() == 2 ? std::array<double, 2>{map[0], map[1]}
                                : (i == 0 ? std::array<double, 2>{0.5, 4.5} : std::array<double, 2>{4.5, 0.5});
      channels_.push_back(channel(s, key, v, 5.0));
      out_[i] = output(s, key, "V", "pedal track voltage");
    }
  }

 protected:
  void sample(double t, SignalBus& bus) override {
    const double travel = std::clamp(pedal_.get(bus), 0.0, 1.0);
    for (std::size_t i = 0; i < 2; ++i) {
      bus.set(out_[i], channels_[i].sample(t, map_[i][0] + (map_[i][1] - map_[i][0]) * travel));
    }
  }

 private:
  Input pedal_;
  std::array<std::array<double, 2>, 2> map_{};
  std::vector<ChannelModel> channels_;
  std::array<SignalId, 2> out_{};
};

// --- GNSS: antenna position, speed, course; delay, noise, outages -------------------------
class Gnss final : public SensorSystem::Sensor {
 public:
  explicit Gnss(const Setup& s) : Sensor(s, 10.0) {
    const ParamTree& p = s.params;
    const SignalBus& bus = s.ctx.bus();
    antenna_ = read_vec(p, s.prefix + ".position", {0.0, 0.0, 1.0});
    const double rate = 1.0 / interval_;
    const double sigma = p.get_double(s.prefix + ".position_sigma_m", 0.02);  // RTK-fixed
    ChannelParams pos;
    pos.noise_density = sigma / std::sqrt(rate);
    pos.latency = p.get_double(s.prefix + ".latency_s", 0.05);
    pos.bias_instability = p.get_double(s.prefix + ".drift_m", 0.0);
    pos.bias_tau = 30.0;
    ChannelParams speed = pos;
    speed.noise_density = p.get_double(s.prefix + ".speed_sigma_mps", 0.05) / std::sqrt(rate);
    speed.bias_instability = 0.0;
    ChannelParams course = speed;
    course.noise_density = p.get_double(s.prefix + ".heading_sigma_deg", 0.5) * kDeg / std::sqrt(rate);
    x_ = channel(s, "x", pos);
    y_ = channel(s, "y", pos);
    v_ = channel(s, "speed", speed);
    h_ = channel(s, "heading", course);
    out_ = {output(s, "x", "m", "antenna east"), output(s, "y", "m", "antenna north"),
            output(s, "speed", "m/s", "speed over ground"), output(s, "heading", "rad", "course over ground"),
            output(s, "valid", "-", "1 with a fix, 0 during an outage")};
    in_ = {Input{bus.find("veh.chassis.x")}, Input{bus.find("veh.chassis.y")}, Input{bus.find("veh.chassis.yaw")},
           Input{bus.find("veh.chassis.vx")}, Input{bus.find("veh.chassis.vy")}};
    // Outages (trees, stands, bridges): exponential gaps and durations.
    outage_rate_ = p.get_double(s.prefix + ".outages_per_min", 0.0) / 60.0;
    outage_duration_ = p.get_double(s.prefix + ".outage_s", 2.0);
    outage_rng_.emplace(s.ctx.make_rng("sensors." + s.name + ".outage"));
    real_ = s.real;
    if (real_ && outage_rate_ > 0.0) {
      next_outage_ = -std::log(1.0 - outage_rng_->uniform()) / outage_rate_;
    }
  }

 protected:
  void sample(double t, SignalBus& bus) override {
    const double yaw = in_[2].get(bus), c = std::cos(yaw), sn = std::sin(yaw);
    const double x = in_[0].get(bus) + c * antenna_[0] - sn * antenna_[1];
    const double y = in_[1].get(bus) + sn * antenna_[0] + c * antenna_[1];
    const double vx = in_[3].get(bus), vy = in_[4].get(bus);
    const double speed = std::hypot(vx, vy);
    const double course = speed > 0.5 ? yaw + std::atan2(vy, vx) : yaw;
    bool valid = true;
    if (real_ && outage_rate_ > 0.0) {
      if (t >= next_outage_) {
        outage_until_ = t + outage_duration_ * -std::log(1.0 - outage_rng_->uniform());
        next_outage_ = outage_until_ - std::log(1.0 - outage_rng_->uniform()) / outage_rate_;
      }
      valid = t >= outage_until_;
    }
    const double mx = x_->sample(t, x), my = y_->sample(t, y), mv = v_->sample(t, speed);
    const double mh = h_->sample(t, std::remainder(course, 2.0 * kPi));
    if (valid) {
      bus.set(out_[0], mx);
      bus.set(out_[1], my);
      bus.set(out_[2], mv);
      bus.set(out_[3], std::remainder(mh, 2.0 * kPi));
    }
    bus.set(out_[4], valid ? 1.0 : 0.0);
  }

 private:
  std::array<double, 3> antenna_{};
  std::optional<ChannelModel> x_, y_, v_, h_;  // no default constructor
  std::array<SignalId, 5> out_{};
  std::array<Input, 5> in_;
  double outage_rate_ = 0.0, outage_duration_ = 2.0, next_outage_ = 1e300, outage_until_ = -1.0;
  std::optional<Rng> outage_rng_;
  bool real_ = false;
};

}  // namespace

SensorSystem::SensorSystem() = default;
SensorSystem::~SensorSystem() = default;

bool SensorSystem::configured(const ParamTree& params) { return params.has_prefix("vehicle.sensors.0"); }

std::vector<SensorSystem::Entry> SensorSystem::entries(const ParamTree& params) {
  std::vector<Entry> out;
  for (std::size_t i = 0;; ++i) {
    const std::string prefix = "vehicle.sensors." + std::to_string(i);
    if (!params.has_prefix(prefix)) {
      break;
    }
    Entry e;
    e.type = params.get_string(prefix + ".type");
    e.name = params.get_string(prefix + ".name", e.type + "_" + std::to_string(i + 1));
    for (const Entry& other : out) {
      if (other.name == e.name) {
        throw std::invalid_argument(prefix + ": sensor name '" + e.name + "' is used twice");
      }
    }
    if (e.type == "wheel_speed" || e.type == "suspension_travel") {
      e.corner = static_cast<int>(corner_of(params, prefix));
    }
    out.push_back(e);
  }
  return out;
}

void SensorSystem::init(const ParamTree& params, Context& ctx) {
  period_ = ctx.dt();
  sensors_.clear();
  const bool real = ctx.realism().is_real(RealismCategory::Sensors);
  const bool variation = ctx.realism().is_real(RealismCategory::Variation);
  const bool full = params.get_string("vehicle.model", "two_track") == "full";
  const std::vector<Entry> list = entries(params);
  for (std::size_t i = 0; i < list.size(); ++i) {
    const std::string prefix = "vehicle.sensors." + std::to_string(i);
    const std::string& type = list[i].type;
    // A sensor of a quantity a simpler model level does not compute (no
    // battery on L1, no damper travel on L2) is left out there; the full
    // model computes them all, so there a missing signal is a typo.
    const std::string source = params.get_string(prefix + ".signal", default_source(type, params, prefix));
    if (!source.empty() && !ctx.bus().find(source) && !full) {
      continue;
    }
    const Setup s{params, ctx, prefix, list[i].name, ctx.dt(), real, variation};
    std::unique_ptr<Sensor> sensor;
    if (type == "imu") {
      sensor = std::make_unique<Imu>(s, true, true);
    } else if (type == "accelerometer") {
      sensor = std::make_unique<Imu>(s, true, false);
    } else if (type == "gyro") {
      sensor = std::make_unique<Imu>(s, false, true);
    } else if (type == "wheel_speed") {
      sensor = std::make_unique<WheelSpeed>(s);
    } else if (type == "pedal") {
      sensor = std::make_unique<Pedal>(s);
    } else if (type == "gnss") {
      sensor = std::make_unique<Gnss>(s);
    } else if (type == "steering_angle") {
      ChannelParams c;
      c.resolution = 0.1 * kDeg;
      c.noise_density = 0.01 * kDeg;
      c.bias = 1.0 * kDeg;  // zero offset of this unit
      c.latency = 0.005;
      sensor = std::make_unique<SignalSensor>(s, 100.0, "drv.cmd.steer", "angle", "rad", "steering wheel angle", c);
    } else if (type == "brake_pressure") {
      ChannelParams c;
      c.noise_density = 0.0015;  // [bar/sqrt(Hz)]: about 0.05 bar rms at 1 kHz
      c.bias = 0.25;             // 0.25 % of a 100 bar range
      c.scale_error = 0.01;
      c.bandwidth = 100.0;
      c.resolution = 100.0 / 4096.0;
      c.min = 0.0;
      c.max = 100.0;
      sensor = std::make_unique<SignalSensor>(s, 1000.0, "veh.brake.pressure", "pressure", "bar",
                                              "brake line pressure", c);
    } else if (type == "current") {
      ChannelParams c;
      c.noise_density = 0.02;  // [A/sqrt(Hz)] Hall sensor
      c.bias = 0.3;
      c.scale_error = 0.005;
      c.bandwidth = 1000.0;
      c.resolution = 0.05;
      sensor = std::make_unique<SignalSensor>(s, 1000.0, "bat.current", "value", "A", "measured current", c);
    } else if (type == "voltage") {
      ChannelParams c;
      c.noise_density = 0.01;
      c.bias = 0.2;
      c.scale_error = 0.003;
      c.bandwidth = 1000.0;
      c.resolution = 0.1;
      sensor = std::make_unique<SignalSensor>(s, 1000.0, "bat.voltage", "value", "V", "measured voltage", c);
    } else if (type == "temperature") {
      ChannelParams c;
      c.noise_density = 0.05;
      c.bias = 0.5;
      c.resolution = 0.1;
      const double tau = params.get_double(prefix + ".time_constant_s", 2.0);  // thermal lag of the probe
      c.bandwidth = 1.0 / (2.0 * kPi * tau);
      sensor = std::make_unique<SignalSensor>(s, 10.0, "", "value", "degC", "measured temperature", c);
    } else if (type == "suspension_travel") {
      ChannelParams c;
      c.noise_density = 0.005e-3;  // [m/sqrt(Hz)] linear potentiometer
      c.bias = 0.5e-3;
      c.resolution = 75e-3 / 4096.0;
      sensor = std::make_unique<SignalSensor>(s, 500.0, source, "travel", "m", "damper travel at the wheel, bump +", c);
    } else if (type == "signal") {
      sensor = std::make_unique<SignalSensor>(s, 100.0, "", "value", "", "measured signal", ChannelParams{});
    } else {
      throw std::invalid_argument(prefix + ".type '" + type +
                                  "' (expected imu, accelerometer, gyro, wheel_speed, pedal, gnss, steering_angle, "
                                  "brake_pressure, current, voltage, temperature, suspension_travel or signal)");
    }
    sensors_.push_back(std::move(sensor));
  }
}

void SensorSystem::step(double t, SignalBus& bus) {
  for (const auto& sensor : sensors_) {
    sensor->tick(t, period_, bus);
  }
}

}  // namespace vsim

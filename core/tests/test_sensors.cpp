#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <tuple>
#include <vector>

#include "test_support.hpp"
#include "vsim/sensors/sensor_chain.hpp"
#include "vsim/sensors/sensor_system.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/input_hub.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

constexpr double kPi = std::numbers::pi;
constexpr double kG = 9.81;

std::vector<double> column(const Simulation& sim, const std::string& name) {
  const auto& names = sim.recorder().names();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    throw std::out_of_range("not logged: " + name);
  }
  return sim.recorder().columns()[static_cast<std::size_t>(it - names.begin())];
}

// Value of a logged signal at time t (the last row at or before t).
double at(const Simulation& sim, const std::string& name, double t) {
  const std::vector<double>& times = sim.recorder().times();
  const auto row = static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), t + 1e-9) - times.begin());
  return column(sim, name)[row == 0 ? 0 : row - 1];
}

// Overlapping Allan deviation of rate samples y (interval tau0) at m * tau0.
double allan_deviation(const std::vector<double>& y, double tau0, std::size_t m) {
  std::vector<double> x(y.size() + 1, 0.0);  // integrated
  for (std::size_t i = 0; i < y.size(); ++i) {
    x[i + 1] = x[i] + y[i] * tau0;
  }
  double sum = 0.0;
  const std::size_t n = x.size() - 2 * m;
  for (std::size_t k = 0; k < n; ++k) {
    const double d = x[k + 2 * m] - 2.0 * x[k + m] + x[k];
    sum += d * d;
  }
  const double tau = static_cast<double>(m) * tau0;
  return std::sqrt(sum / (2.0 * tau * tau * static_cast<double>(n)));
}

// Amplitude of the f [Hz] component of samples taken every dt.
double amplitude_at(const std::vector<double>& v, double dt, double f) {
  double c = 0.0, s = 0.0, mean = 0.0;
  for (const double x : v) {
    mean += x;
  }
  mean /= static_cast<double>(v.size());
  for (std::size_t i = 0; i < v.size(); ++i) {
    const double phase = 2.0 * kPi * f * static_cast<double>(i) * dt;
    c += (v[i] - mean) * std::cos(phase);
    s += (v[i] - mean) * std::sin(phase);
  }
  return 2.0 * std::hypot(c, s) / static_cast<double>(v.size());
}

ChannelModel make_channel(const ChannelParams& p, double rate, bool real, bool variation, std::uint64_t seed = 1) {
  return ChannelModel(p, rate, Rng(seed), Rng(seed + 1000), real, variation);
}

// --- One channel --------------------------------------------------------------------------

TEST(SensorChannel, IdealPassesTheTruth) {
  ChannelParams p;
  p.noise_density = 1.0;
  p.bias = 1.0;
  p.scale_error = 0.1;
  p.latency = 0.01;
  p.resolution = 0.5;
  p.min = -1.0;
  p.max = 1.0;
  ChannelModel channel = make_channel(p, 100.0, false, false);
  for (int i = 0; i < 100; ++i) {
    const double truth = 3.0 * std::sin(0.37 * i);
    EXPECT_EQ(channel.sample(0.01 * i, truth), truth);
  }
}

// V-11: the Allan deviation of a simulated channel shows the datasheet's
// white noise (N / sqrt(tau)), bias instability floor (0.664 B) and rate
// random walk (K sqrt(tau / 3)).
TEST(SensorChannel, AllanDeviationMatchesTheDatasheet) {
  const double rate = 10.0, tau0 = 1.0 / rate;
  const auto record = [&](const ChannelParams& p, std::size_t n) {
    ChannelModel channel = make_channel(p, rate, true, false, 7);
    std::vector<double> y(n);
    for (std::size_t i = 0; i < n; ++i) {
      y[i] = channel.sample(static_cast<double>(i) * tau0, 0.0);
    }
    return y;
  };
  ChannelParams p;
  p.noise_density = 2e-4;
  p.bias_instability = 1e-3;
  p.bias_tau = 10.0;
  const std::vector<double> y = record(p, 200'000);  // 5.5 h
  EXPECT_NEAR(allan_deviation(y, tau0, 1), p.noise_density / std::sqrt(tau0), 0.05 * p.noise_density / std::sqrt(tau0));
  double floor = 0.0;
  for (const std::size_t m : {50, 100, 150, 200, 300, 500, 1000}) {
    floor = std::max(floor, allan_deviation(y, tau0, m));
  }
  EXPECT_NEAR(floor, 0.664 * p.bias_instability, 0.15 * 0.664 * p.bias_instability);

  ChannelParams walk;
  walk.random_walk = 1e-3;
  const std::vector<double> w = record(walk, 200'000);
  for (const std::size_t m : {200, 1000}) {
    const double tau = static_cast<double>(m) * tau0;
    EXPECT_NEAR(allan_deviation(w, tau0, m), walk.random_walk * std::sqrt(tau / 3.0),
                0.2 * walk.random_walk * std::sqrt(tau / 3.0))
        << "tau " << tau;
  }
}

TEST(SensorChannel, BandwidthLatencyQuantizationAndRange) {
  ChannelParams p;
  p.bandwidth = 10.0;
  ChannelModel lowpass = make_channel(p, 1000.0, true, false);
  lowpass.sample(0.0, 0.0);
  double y = 0.0;
  const auto tau_samples = static_cast<int>(std::lround(1000.0 / (2.0 * kPi * p.bandwidth)));
  for (int i = 1; i <= tau_samples; ++i) {
    y = lowpass.sample(0.001 * i, 1.0);
  }
  EXPECT_NEAR(y, 1.0 - std::exp(-1.0), 0.02);  // first order: 63 % after one time constant

  ChannelParams d;
  d.latency = 0.003;
  d.resolution = 0.5;
  ChannelModel delayed = make_channel(d, 1000.0, true, false);
  for (int i = 0; i < 20; ++i) {
    const double out = delayed.sample(0.001 * i, 0.3 * i);
    EXPECT_DOUBLE_EQ(out, std::round(0.3 * std::max(0, i - 3) / 0.5) * 0.5) << i;
  }

  ChannelParams r;
  r.min = -1.0;
  r.max = 2.0;
  ChannelModel ranged = make_channel(r, 1000.0, true, false);
  EXPECT_EQ(ranged.sample(0.0, 5.0), 2.0);
  EXPECT_EQ(ranged.sample(0.001, -5.0), -1.0);
}

TEST(SensorChannel, FaultsOverrideTheOutput) {
  const ParamTree params = ParamTree::from_yaml_string(R"(
s:
  faults:
    - {channel: ch1, type: stuck, at_s: 0.5, until_s: 0.7}
    - {channel: ch1, type: short, at_s: 0.8, until_s: 0.9}
    - {channel: ch2, type: open, at_s: 0.1}
    - {type: offset, value: 0.25, at_s: 0.95}
)");
  ChannelModel ch1 = make_channel({}, 100.0, false, false);
  for (const ChannelFault& f : faults_from_params(params, "s", "ch1", 5.0)) {
    ch1.add_fault(f);
  }
  EXPECT_EQ(faults_from_params(params, "s", "ch2", 5.0).size(), 2u);  // its own and the one for every channel
  double held = 0.0;
  for (int i = 0; i < 100; ++i) {
    const double t = 0.01 * i, truth = static_cast<double>(i);
    const double out = ch1.sample(t, truth);
    if (i == 49) {
      held = out;
    }
    if (t < 0.5 - 1e-9 || (t >= 0.7 - 1e-9 && t < 0.8 - 1e-9) || (t >= 0.9 - 1e-9 && t < 0.95 - 1e-9)) {
      EXPECT_EQ(out, truth) << t;
    } else if (t < 0.7 - 1e-9) {
      EXPECT_EQ(out, held) << t;  // stuck at the last good value
    } else if (t < 0.9 - 1e-9) {
      EXPECT_EQ(out, 5.0) << t;  // shorted to the supply
    } else {
      EXPECT_EQ(out, truth + 0.25) << t;
    }
  }
  EXPECT_THROW(faults_from_params(ParamTree::from_yaml_string("s: {faults: [{type: melt}]}"), "s", "x", 5.0),
               std::invalid_argument);
}

TEST(SensorChannel, VariationIsFixedPerUnit) {
  ChannelParams p;
  p.bias = 0.1;
  p.scale_error = 0.01;
  ChannelModel a = make_channel(p, 100.0, false, true, 1);
  ChannelModel b = make_channel(p, 100.0, false, true, 2);
  ChannelModel a_again = make_channel(p, 100.0, false, true, 1);
  const double a0 = a.sample(0.0, 0.0), a1 = a.sample(0.01, 1.0);
  EXPECT_NE(a0, 0.0);
  EXPECT_EQ(a.sample(0.02, 0.0), a0);  // the same error every sample
  EXPECT_NE(b.sample(0.0, 0.0), a0);   // another unit, another error
  EXPECT_EQ(a_again.sample(0.0, 0.0), a0);
  EXPECT_NEAR(a1 - a0, 1.0, 0.05);  // scale error of a few percent at most
}

// Transducer stage: temperature coefficients (linear in the temperature
// change, scale part proportional to the reading), terminal-based
// nonlinearity (zero at 0 and full scale, odd) and hysteresis (play band).
TEST(SensorChannel, TemperatureNonlinearityAndHysteresis) {
  ChannelParams p;
  p.bias_tempco = 0.01;
  p.scale_tempco = 1e-3;
  ChannelModel drift = make_channel(p, 100.0, false, true);
  double temperature = 25.0;
  drift.set_temperature_source(&temperature);
  const auto read = [&](double t, double truth) {
    temperature = t;
    return drift.sample(0.0, truth);
  };
  const double base = read(25.0, 2.0), warm = read(65.0, 2.0), hot = read(105.0, 2.0);
  EXPECT_GT(std::abs(warm - base), 1e-3);
  EXPECT_NEAR(hot - base, 2.0 * (warm - base), 1e-12);
  EXPECT_NE(read(65.0, 0.0) - read(25.0, 0.0), warm - base);  // the scale part follows the reading
  ChannelModel ideal = make_channel(p, 100.0, false, false);
  ideal.set_temperature_source(&temperature);
  EXPECT_EQ(ideal.sample(0.0, 2.0), 2.0);  // per-unit coefficients only with realism.variation

  ChannelParams n;
  n.nonlinearity = 0.01;
  n.min = -10.0;
  n.max = 10.0;
  ChannelModel curve = make_channel(n, 100.0, false, true, 3);
  EXPECT_DOUBLE_EQ(curve.sample(0.0, 0.0), 0.0);
  EXPECT_NEAR(curve.sample(0.0, 10.0), 10.0, 1e-12);
  EXPECT_NEAR(curve.sample(0.0, -10.0), -10.0, 1e-12);
  const double u = 10.0 / std::sqrt(3.0);
  const double peak = curve.sample(0.0, u) - u;
  EXPECT_GT(std::abs(peak), 1e-3);
  EXPECT_LT(std::abs(peak), 0.5);  // a few sigma of 1 % of 10
  EXPECT_NEAR(curve.sample(0.0, -u) + u, -peak, 1e-12);
  for (const double x : {1.0, 3.0, 8.0}) {
    EXPECT_LT(std::abs(curve.sample(0.0, x) - x), std::abs(peak)) << x;
  }

  ChannelParams h;
  h.hysteresis = 0.2;
  ChannelModel play = make_channel(h, 100.0, true, false);
  EXPECT_DOUBLE_EQ(play.sample(0.0, 0.0), 0.0);
  EXPECT_NEAR(play.sample(0.01, 1.0), 0.9, 1e-12);   // rising: lags by half the band
  EXPECT_NEAR(play.sample(0.02, 0.85), 0.9, 1e-12);  // turned round: held inside the band
  EXPECT_NEAR(play.sample(0.03, 0.5), 0.6, 1e-12);   // falling: leads by half the band
}

// --- Sensors on a hand-made bus -----------------------------------------------------------

// The sensor block alone: the plant signals it reads are set directly.
class Rig {
 public:
  explicit Rig(const std::string& sensors_yaml, bool real = false, bool variation = false, std::uint64_t seed = 1) {
    params_ = ParamTree::from_yaml_string("vehicle:\n  sensors:\n" + sensors_yaml);
    params_.set("realism.sensors", real ? "real" : "ideal");
    params_.set("realism.variation", variation ? "real" : "ideal");
    params_.set("vehicle.model", "full");  // every signal a sensor names must exist
    for (const char* name :
         {"veh.chassis.x", "veh.chassis.y", "veh.chassis.yaw", "veh.chassis.vx", "veh.chassis.vy",
          "veh.chassis.ax", "veh.chassis.ay", "veh.chassis.az", "veh.chassis.roll", "veh.chassis.pitch",
          "veh.chassis.roll_rate", "veh.chassis.pitch_rate", "veh.chassis.yaw_rate", "veh.chassis.roll_accel",
          "veh.chassis.pitch_accel", "veh.chassis.yaw_accel", "veh.wheel.fl.omega", "pt.motor.m.speed",
          "drv.cmd.throttle", "drv.cmd.steer", "veh.brake.pressure"}) {
      bus_.declare(name, "", "");
    }
    ctx_ = std::make_unique<Context>(bus_, inputs_, RealismConfig::from_params(params_), seed, 0.001);
    sensors_.init(params_, *ctx_);
  }

  void set(const std::string& name, double value) { bus_.set(bus_.require(name), value); }
  double get(const std::string& name) const { return bus_.get(bus_.require(name)); }
  double time() const { return static_cast<double>(tick_) * 0.001; }
  void step() {
    sensors_.step(time(), bus_);
    ++tick_;
  }
  void run_for(double seconds) {
    for (auto n = std::lround(seconds / 0.001); n > 0; --n) {
      step();
    }
  }

 private:
  ParamTree params_;
  SignalBus bus_;
  InputHub inputs_;
  std::unique_ptr<Context> ctx_;
  SensorSystem sensors_;
  long long tick_ = 0;
};

// PH-01: the accelerometer measures specific force in its own axes, so a
// car standing on a slope or a bank reads part of gravity sideways.
TEST(Sensors, ImuReadsGravityOnASlope) {
  Rig rig("    - {name: imu, type: imu, position: [0, 0, 0.3]}\n");
  rig.set("veh.chassis.pitch", -0.1);  // nose up
  rig.step();
  EXPECT_NEAR(rig.get("sens.imu.ax"), kG * std::sin(0.1), 1e-9);
  EXPECT_NEAR(rig.get("sens.imu.az"), kG * std::cos(0.1), 1e-9);
  rig.set("veh.chassis.pitch", 0.0);
  rig.set("veh.chassis.roll", 0.1);  // left side up
  rig.run_for(0.002);
  EXPECT_NEAR(rig.get("sens.imu.ax"), 0.0, 1e-9);
  EXPECT_NEAR(rig.get("sens.imu.ay"), kG * std::sin(0.1), 1e-9);
  // Yawing on the bank: the gyro sees it partly about its y axis.
  rig.set("veh.chassis.yaw_rate", 0.5);
  rig.run_for(0.002);
  EXPECT_NEAR(rig.get("sens.imu.gy"), 0.5 * std::sin(0.1), 1e-9);
  EXPECT_NEAR(rig.get("sens.imu.gz"), 0.5 * std::cos(0.1), 1e-9);
}

TEST(Sensors, ImuLeverArmAndMounting) {
  Rig rig("    - {name: nose, type: accelerometer, position: [1.0, 0.5, 0.25]}\n"
          "    - {name: turned, type: imu, position: [0, 0, 0.25], orientation_deg: [0, 0, 90]}\n");
  rig.set("veh.chassis.yaw_rate", 1.0);
  rig.set("veh.chassis.yaw_accel", 2.0);
  rig.set("veh.chassis.ax", 3.0);
  rig.step();
  // Centripetal -r^2 (x, y) plus tangential (dr/dt) z x (x, y).
  EXPECT_NEAR(rig.get("sens.nose.ax"), 3.0 - 1.0 - 2.0 * 0.5, 1e-9);
  EXPECT_NEAR(rig.get("sens.nose.ay"), -0.5 + 2.0 * 1.0, 1e-9);
  EXPECT_NEAR(rig.get("sens.nose.az"), kG, 1e-9);
  // Turned 90 deg left: its x axis points left, its y axis backwards.
  EXPECT_NEAR(rig.get("sens.turned.ax"), 0.0, 1e-9);
  EXPECT_NEAR(rig.get("sens.turned.ay"), -3.0, 1e-9);
  EXPECT_NEAR(rig.get("sens.turned.gz"), 1.0, 1e-9);
}

// Per-unit cross-axis coupling and gyro g-sensitivity; the sensor warms
// towards the signal it is mounted near, behind its thermal lag.
TEST(Sensors, ImuCrossAxisGSensitivityAndTemperature) {
  const auto reading = [](const std::string& extra, double ax, const char* channel) {
    Rig rig("    - {name: imu, type: imu, position: [0, 0, 0.3], misalignment_deg: 0" + extra + "}\n", false, true);
    rig.set("veh.chassis.ax", ax);
    rig.step();
    return rig.get(std::string("sens.imu.") + channel);
  };
  const std::string quiet = ", accel: {cross_axis: 0}, gyro: {cross_axis: 0, g_sensitivity_dps_per_g: 0}";
  EXPECT_DOUBLE_EQ(reading(quiet, 5.0, "ay"), reading(quiet, 0.0, "ay"));
  EXPECT_DOUBLE_EQ(reading(quiet, 5.0, "gz"), reading(quiet, 0.0, "gz"));
  const std::string coupled = ", accel: {cross_axis: 0.02}, gyro: {g_sensitivity_dps_per_g: 0.1}";
  EXPECT_GT(std::abs(reading(coupled, 5.0, "ay") - reading(coupled, 0.0, "ay")), 1e-3);
  EXPECT_GT(std::abs(reading(coupled, 5.0, "gz") - reading(coupled, 0.0, "gz")), 1e-6);

  // Mounted near something that heats from 25 to 85 degC, one second lag.
  const std::string near = "    - {name: imu, type: imu, position: [0, 0, 0.3], temperature_signal: drv.cmd.throttle,"
                           " thermal_tau_s: 1.0}\n";
  Rig warming(near, false, true), still(near, false, true);
  for (Rig* rig : {&warming, &still}) {
    rig->set("drv.cmd.throttle", 25.0);
    rig->step();
  }
  warming.set("drv.cmd.throttle", 85.0);
  warming.run_for(1.0);
  still.run_for(1.0);
  const double after_tau = warming.get("sens.imu.az") - still.get("sens.imu.az");
  warming.run_for(5.0);
  still.run_for(5.0);
  const double settled = warming.get("sens.imu.az") - still.get("sens.imu.az");
  EXPECT_GT(std::abs(settled), 1e-4);
  EXPECT_NEAR(after_tau / settled, 1.0 - std::exp(-1.0), 0.01);
}

// PH-03: wheel speed from a toothed ring is accurate when the teeth pass
// quickly and drops out when they pass slower than the timeout.
TEST(Sensors, WheelSpeedFromPulsesDropsOutAtLowSpeed) {
  const std::string ws = "    - {name: ws, type: wheel_speed, wheel: fl, teeth: 36, rate_hz: 200}\n";
  Rig ideal(ws);
  ideal.set("veh.wheel.fl.omega", 0.5);
  ideal.run_for(0.1);
  EXPECT_EQ(ideal.get("sens.ws.omega"), 0.5);

  Rig rig(ws, true);
  rig.set("veh.wheel.fl.omega", 50.0);
  rig.run_for(1.0);
  EXPECT_NEAR(rig.get("sens.ws.omega"), 50.0, 0.001 * 50.0);
  EXPECT_NEAR(rig.get("sens.ws.pulses"), 50.0 * 0.995 * 36 / (2.0 * kPi), 1.0);  // up to the last sample

  // Teeth every 2 pi / 36 / 0.5 = 0.35 s, timeout 0.1 s: mostly no reading.
  rig.set("veh.wheel.fl.omega", 0.5);
  int zero = 0, total = 0;
  for (int i = 0; i < 2000; ++i) {
    rig.step();
    if (i > 500 && i % 5 == 0) {
      ++total;
      zero += rig.get("sens.ws.omega") == 0.0 ? 1 : 0;
    }
  }
  EXPECT_GT(zero, total / 2);
  EXPECT_LT(zero, total);  // right after a tooth it still reads
}

// PH-11: motor-order vibration above half the sample rate shows up at a low
// alias frequency, unless an analog prefilter removes it before sampling.
TEST(Sensors, VibrationAliasesWithoutAPrefilter) {
  const auto alias_amplitude = [](const std::string& prefilter) {
    Rig rig("    - {name: imu, type: accelerometer, rate_hz: 500" + prefilter +
                ", vibration: [{motor: m, order: 1, amplitude: 1.0, axis: z}],"
                " accel: {noise_density: 0, bias_instability: 0, bandwidth_hz: 0, resolution: 0}}\n",
            true);
    rig.set("pt.motor.m.speed", 2.0 * kPi * 503.0);  // 503 Hz, aliases to 3 Hz
    std::vector<double> az;
    for (int i = 0; i < 4000; ++i) {
      rig.step();
      if (i % 2 == 0) {
        az.push_back(rig.get("sens.imu.az"));
      }
    }
    return amplitude_at(az, 0.002, 3.0);
  };
  const double raw = alias_amplitude("");
  const double filtered = alias_amplitude(", anti_alias_hz: 100");
  EXPECT_NEAR(raw, 1.0, 0.05);
  EXPECT_NEAR(filtered, 1.0 / (1.0 + 5.03 * 5.03), 0.01);
}

TEST(Sensors, AreDeterministicPerSeed) {
  const std::string yaml = "    - {name: imu, type: imu}\n    - {name: ws, type: wheel_speed, wheel: fl}\n";
  const auto run = [&](std::uint64_t seed) {
    Rig rig(yaml, true, true, seed);
    rig.set("veh.wheel.fl.omega", 30.0);
    std::vector<double> out;
    for (int i = 0; i < 500; ++i) {
      rig.step();
      out.push_back(rig.get("sens.imu.ax"));
      out.push_back(rig.get("sens.ws.omega"));
    }
    return out;
  };
  EXPECT_EQ(run(3), run(3));
  EXPECT_NE(run(3), run(4));
}

TEST(Sensors, RejectBadDefinitions) {
  EXPECT_THROW(Rig("    - {name: a, type: imu}\n    - {name: a, type: gyro}\n"), std::invalid_argument);
  EXPECT_THROW(Rig("    - {type: lidar}\n"), std::invalid_argument);
  EXPECT_THROW(Rig("    - {type: signal}\n"), std::invalid_argument);
  EXPECT_THROW(Rig("    - {type: temperature, signal: no.such.signal}\n"), std::invalid_argument);
  EXPECT_THROW(Rig("    - {type: imu, rate_hz: 0}\n"), std::invalid_argument);
  EXPECT_THROW(Rig("    - {type: imu, accel: {noise_density: -1}}\n"), std::invalid_argument);
}

// --- The VCU on the example car's sensors ------------------------------------------------

// FSAE T.4.2: an APPS track that fails (open circuit) cuts drive torque once
// the implausibility has lasted 100 ms.
TEST(Sensors, AppsOpenCircuitCutsTorqueAfter100ms) {
  Simulation sim(load_example("acceleration", {{"vehicle.sensors.5.faults.0.channel", "ch2"},
                                               {"vehicle.sensors.5.faults.0.type", "open"},
                                               {"vehicle.sensors.5.faults.0.at_s", "1.0"},
                                               {"logging.signals", ""},
                                               {"logging.signals.0", "vcu.*"},
                                               {"logging.signals.1", "pt.motor.*"},
                                               {"logging.signals.2", "sens.apps.*"}}));
  ASSERT_EQ(sim.scenario().params.get_string("vehicle.sensors.5.name"), "apps");
  sim.run_for(1.5);
  EXPECT_EQ(at(sim, "sens.apps.ch2", 1.001), 0.0);
  EXPECT_EQ(at(sim, "vcu.apps_implausible", 1.095), 0.0);
  EXPECT_GT(at(sim, "pt.motor.fl.torque_cmd", 1.095), 5.0);  // still driving
  EXPECT_EQ(at(sim, "vcu.apps_implausible", 1.105), 1.0);
  for (const char* id : {"fl", "fr", "rl", "rr"}) {
    EXPECT_EQ(at(sim, std::string("pt.motor.") + id + ".torque_cmd", 1.105), 0.0) << id;
    EXPECT_LT(std::abs(signal(sim, std::string("pt.motor.") + id + ".torque")), 1.0) << id;
  }
}

// Tracks more than 10 % apart (a slipped sensor) are implausible too.
TEST(Sensors, AppsDisagreementCutsTorque) {
  Simulation sim(load_example("acceleration", {{"driver.throttle.value", "0.5"},
                                               {"vehicle.sensors.5.faults.0.channel", "ch1"},
                                               {"vehicle.sensors.5.faults.0.type", "offset"},
                                               {"vehicle.sensors.5.faults.0.value", "0.6"},
                                               {"vehicle.sensors.5.faults.0.at_s", "0.5"}}));
  sim.run_for(0.55);
  EXPECT_EQ(signal(sim, "vcu.apps_implausible"), 0.0);
  sim.run_for(0.1);
  EXPECT_EQ(signal(sim, "vcu.apps_implausible"), 1.0);
  EXPECT_EQ(signal(sim, "pt.motor.rl.torque_cmd"), 0.0);
}

// FSAE EV.4.7: braking hard with the accelerator pressed cuts drive torque,
// and the cut holds after the brake is released until the accelerator is.
TEST(Sensors, BrakePlausibilityLatchesUntilPedalRelease) {
  Simulation sim(load_example("acceleration", {{"driver.throttle.type", "step"},
                                               {"driver.throttle.before", "0.5"},
                                               {"driver.throttle.at_s", "3.0"},
                                               {"driver.throttle.value", "0"},
                                               {"driver.brake.type", "sine"},
                                               {"driver.brake.amplitude", "0.5"},
                                               {"driver.brake.freq_hz", "0.5"},
                                               {"driver.brake.start_s", "1.0"},
                                               {"logging.signals", ""},
                                               {"logging.signals.0", "vcu.*"},
                                               {"logging.signals.1", "pt.motor.*"},
                                               {"logging.signals.2", "drv.*"}}));
  sim.run_for(3.5);
  EXPECT_EQ(at(sim, "vcu.bspc", 0.9), 0.0);
  EXPECT_GT(at(sim, "pt.motor.fl.torque_cmd", 0.9), 0.0);
  EXPECT_EQ(at(sim, "vcu.bspc", 1.5), 1.0);
  // Brake released (2-3 s), accelerator still at 50 %: no torque.
  EXPECT_EQ(at(sim, "drv.cmd.brake", 2.5), 0.0);
  EXPECT_EQ(at(sim, "vcu.bspc", 2.5), 1.0);
  EXPECT_EQ(at(sim, "pt.motor.fl.torque_cmd", 2.5), 0.0);
  EXPECT_EQ(at(sim, "vcu.bspc", 3.1), 0.0);
}

// The VCU's speed estimate from noisy wheel speed pulses and the IMU stays
// close to the truth through a 4WD launch and steady cornering.
TEST(Sensors, SpeedEstimateTracksTheTruth) {
  for (const auto& [scenario, duration, bound] :
       {std::tuple{"acceleration", 4.0, 1.0}, std::tuple{"skidpad_auto", 12.0, 0.4}}) {
    Simulation sim(load_example(scenario, {{"realism.sensors", "real"},
                                           {"realism.variation", "real"},
                                           {"logging.signals", ""},
                                           {"logging.signals.0", "veh.chassis.*"},
                                           {"logging.signals.1", "vcu.*"}}));
    sim.run_for(duration);
    const std::vector<double>& times = sim.recorder().times();
    const std::vector<double> est = column(sim, "vcu.vx_est");
    const std::vector<double> vx = column(sim, "veh.chassis.vx");
    double worst = 0.0;
    for (std::size_t i = 0; i < times.size(); ++i) {
      if (times[i] > 0.3) {
        worst = std::max(worst, std::abs(est[i] - vx[i]));
      }
    }
    EXPECT_LT(worst, bound) << scenario;
    EXPECT_GT(signal(sim, "veh.chassis.vx"), 5.0) << scenario;
  }
}

// The sensors measure what a simpler model level computes and are left out
// where it computes nothing (no brake pressure or battery on L1).
TEST(Sensors, SimplerModelsKeepTheSensorsTheyCanFeed) {
  Simulation sim(load_example("straight_accel", {{"vehicle.model", "single_track"},
                                                 {"logging.signals", ""},
                                                 {"logging.signals.0", "sens.*"}}));
  sim.run_for(1.0);
  EXPECT_TRUE(sim.bus().find("sens.imu.ax").has_value());
  EXPECT_TRUE(sim.bus().find("sens.apps.ch1").has_value());
  EXPECT_FALSE(sim.bus().find("sens.bse.pressure").has_value());
  EXPECT_FALSE(sim.bus().find("sens.hv_current.value").has_value());
  EXPECT_GT(signal(sim, "veh.chassis.vx"), 5.0);
}

}  // namespace
}  // namespace vsim

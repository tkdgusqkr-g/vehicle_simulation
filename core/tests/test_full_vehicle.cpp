#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include "test_support.hpp"
#include "vsim/models/vehicle/aero.hpp"
#include "vsim/models/vehicle/two_track.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/integrator.hpp"
#include "vsim/track/track.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

// A track file in a fresh temp directory, removed afterwards.
class TrackFile {
 public:
  explicit TrackFile(const TrackDefinition& def) {
    dir_ = std::filesystem::temp_directory_path() / ("vsim_full_" + std::to_string(std::random_device{}()));
    std::filesystem::create_directories(dir_);
    path_ = dir_ / "track.track.json";
    std::ofstream(path_) << def.to_json();
  }
  ~TrackFile() {
    std::error_code ignored;
    std::filesystem::remove_all(dir_, ignored);
  }
  std::string path() const { return path_.string(); }

 private:
  std::filesystem::path dir_, path_;
};

// Straight open track along +x, 400 m, rising by `grade`, banked by `bank_deg`.
TrackDefinition straight(double grade = 0.0, double bank_deg = 0.0) {
  TrackDefinition def;
  def.name = "straight";
  def.closed = false;
  for (int i = 0; i <= 8; ++i) {
    TrackDefinition::Point p;
    p.x = 50.0 * i;
    p.z = grade * p.x;
    p.w_left = p.w_right = 3.0;
    p.bank_deg = bank_deg;
    def.points.push_back(p);
  }
  return def;
}

double max_abs(const std::vector<double>& v) {
  double m = 0.0;
  for (const double x : v) {
    m = std::max(m, std::abs(x));
  }
  return m;
}

std::vector<double> column(const Simulation& sim, const std::string& name) {
  const auto& names = sim.recorder().names();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    throw std::out_of_range("not logged: " + name);
  }
  return sim.recorder().columns()[static_cast<std::size_t>(it - names.begin())];
}

double stddev(const std::vector<double>& v, std::size_t from = 0) {
  double mean = 0.0;
  for (std::size_t i = from; i < v.size(); ++i) {
    mean += v[i];
  }
  mean /= static_cast<double>(v.size() - from);
  double var = 0.0;
  for (std::size_t i = from; i < v.size(); ++i) {
    var += (v[i] - mean) * (v[i] - mean);
  }
  return std::sqrt(var / static_cast<double>(v.size() - from));
}

TEST(Full, IsTheExampleCarsModel) {
  Simulation sim(load_example("drive"));
  EXPECT_EQ(sim.scenario().params.get_string("vehicle.model"), "full");
  EXPECT_NO_THROW(sim.bus().require("veh.chassis.roll"));
  EXPECT_NO_THROW(sim.bus().require("veh.wheel.fl.travel"));
  EXPECT_NO_THROW(sim.bus().require("rig.rr.z"));
}

TEST(Full, RestsInEquilibrium) {
  Simulation sim(load_example("straight_accel", {{"driver.throttle.value", "0"}}));
  const double z0 = signal(sim, "veh.chassis.z");
  sim.run_for(3.0);
  EXPECT_NEAR(signal(sim, "veh.chassis.x"), 0.0, 1e-9);
  EXPECT_NEAR(signal(sim, "veh.chassis.vx"), 0.0, 1e-9);
  EXPECT_NEAR(signal(sim, "veh.chassis.z"), z0, 1e-9);
  EXPECT_NEAR(signal(sim, "veh.chassis.roll"), 0.0, 1e-12);
  EXPECT_NEAR(signal(sim, "veh.chassis.pitch"), 0.0, 1e-12);
  for (const char* c : {"fl", "fr", "rl", "rr"}) {
    EXPECT_NEAR(signal(sim, std::string("veh.wheel.") + c + ".travel"), 0.0, 1e-9) << c;
  }
  // The static loads add up to the weight.
  double fz = 0.0;
  for (const char* c : {"fl", "fr", "rl", "rr"}) {
    fz += signal(sim, std::string("veh.wheel.") + c + ".fz");
  }
  EXPECT_NEAR(fz, sim.scenario().params.get_double("vehicle.mass_kg") * 9.81, 1e-6);
}

// V-03 for the full model: Kane's equations keep the books exact, with the
// body moving on its springs and the tires over rough grass.
TEST(Full, EnergyIsConserved) {
  for (const char* scenario : {"sine_steer", "straight_accel"}) {
    Simulation sim(load_example(scenario));
    sim.run_for(12.0);
    const double chemical = signal(sim, "energy.battery_chemical");
    ASSERT_GT(chemical, 1e4) << scenario;
    EXPECT_LT(std::abs(signal(sim, "energy.residual")), 1e-6 * chemical) << scenario;
    EXPECT_GT(signal(sim, "energy.damper_loss"), 0.0) << scenario;
  }
}

TEST(Full, BodyMovesTheRightWay) {
  // Squat under power, dive under braking.
  Simulation accel(load_example("straight_accel"));
  accel.run_for(1.0);
  EXPECT_LT(signal(accel, "veh.chassis.pitch"), -0.1 * kDeg);
  Simulation brake(load_example("straight_accel", {{"initial.speed_mps", "20"},
                                                   {"driver.throttle.value", "0"},
                                                   {"driver.brake.type", "const"},
                                                   {"driver.brake.value", "0.6"}}));
  brake.run_for(0.5);
  EXPECT_GT(signal(brake, "veh.chassis.pitch"), 0.1 * kDeg);
  // Turning left lifts the left side's load and rolls the body out (left up).
  Simulation turn(load_example("sine_steer", {{"driver.steer.type", "const"}, {"driver.steer.value", "30"}}));
  turn.run_for(8.0);
  EXPECT_GT(signal(turn, "veh.chassis.ay"), 5.0);
  EXPECT_GT(signal(turn, "veh.chassis.roll"), 0.3 * kDeg);
  EXPECT_LT(signal(turn, "veh.wheel.fl.fz"), signal(turn, "veh.wheel.fr.fz"));
  EXPECT_GT(signal(turn, "veh.wheel.fr.travel"), 0.0);  // outside wheel in bump
}

// The full and the planar model agree on steady cornering in the linear
// range (the full one adds camber, compliance-free K&C and real load transfer).
TEST(Full, SteadyCorneringNearTheTwoTrackModel) {
  const auto yaw_rate = [](const char* model) {
    Simulation sim(load_example("sine_steer", {{"vehicle.model", model},
                                               {"driver.steer.type", "const"},
                                               {"driver.steer.value", "20"}}));
    sim.run_for(12.0);
    return signal(sim, "veh.chassis.yaw_rate");
  };
  const double l2 = yaw_rate("two_track");
  const double l3 = yaw_rate("full");
  EXPECT_GT(l3, 0.1);
  EXPECT_NEAR(l3, l2, 0.1 * l2);
}

// Rolling back down a 5 % grade from rest (the normal force tilts with the road).
TEST(Full, RollsDownHill) {
  const TrackFile file(straight(0.05));
  Simulation sim(load_example("straight_accel", {{"track.file", file.path()},
                                                 {"initial.x_m", "100"},
                                                 {"initial.y_m", "0"},
                                                 {"driver.throttle.value", "0"},
                                                 {"vehicle.tire.rolling_resistance", "0"},
                                                 {"vehicle.aero.cda_m2", "0"}}));
  sim.run_for(3.0);
  // Effective mass: the wheels' and rotors' spin inertia rolls along.
  const ParamTree& p = sim.scenario().params;
  const double m = p.get_double("vehicle.mass_kg");
  const double radius = p.get_double("vehicle.wheel.radius_m");
  const double ratio = p.get_double("vehicle.driveline.0.gear_ratio");
  const double spin = p.get_double("vehicle.wheel.inertia_kgm2") +
                      p.get_double("vehicle.powertrain.motor.rotor_inertia_kgm2") * ratio * ratio;
  const double expected = -9.81 * 0.05 * m / (m + 4.0 * spin / (radius * radius));
  EXPECT_NEAR(signal(sim, "veh.chassis.vx") / 3.0, expected, 0.03 * std::abs(expected));
  EXPECT_NEAR(signal(sim, "veh.chassis.pitch"), -std::atan(0.05), 0.2 * kDeg);  // body follows the slope
}

// Parked on a 5 deg bank: the body leans with the road, the tires hold it.
TEST(Full, BodyFollowsTheBank) {
  const TrackFile file(straight(0.0, 5.0));
  Simulation sim(load_example("straight_accel", {{"track.file", file.path()},
                                                 {"initial.x_m", "100"},
                                                 {"initial.y_m", "0"},
                                                 {"driver.throttle.value", "0"}}));
  sim.run_for(3.0);
  EXPECT_NEAR(signal(sim, "veh.chassis.roll"), 5.0 * kDeg, 0.2 * kDeg);
  EXPECT_LT(std::abs(signal(sim, "veh.chassis.vy")), 1e-3);
  EXPECT_LT(signal(sim, "veh.wheel.fl.fz"), signal(sim, "veh.wheel.fr.fz"));  // the low side carries more
}

// A kerb under the left wheels hammers them; the right ones run smooth.
TEST(Full, KerbHitsTheWheelsOnIt) {
  TrackDefinition def = straight();
  def.kerbs.push_back({2, 5, "left", 1.0, 0.03});
  const TrackFile file(def);
  // Left wheels 0.4 m onto the kerb: centre 3.0 + 0.4 - 0.61 left of the centreline.
  Simulation sim(load_example("sine_steer", {{"track.file", file.path()},
                                             {"driver.steer.type", "const"},
                                             {"driver.steer.value", "0"},
                                             {"initial.x_m", "105"},
                                             {"initial.y_m", "2.79"},
                                             {"initial.speed_mps", "11.1"},
                                             {"logging.rate_hz", "1000"}}));
  sim.run_for(4.0);  // open loop: the car slowly drifts off the kerb later
  const std::vector<double> surface = column(sim, "veh.wheel.fl.surface");
  EXPECT_EQ(*std::min_element(surface.begin(), surface.end()), 1.0);  // on the kerb all along
  EXPECT_EQ(*std::max_element(surface.begin(), surface.end()), 1.0);
  const std::vector<double> left = column(sim, "veh.wheel.fl.fz");
  const std::vector<double> right = column(sim, "veh.wheel.fr.fz");
  EXPECT_GT(stddev(left, 500), 5.0 * stddev(right, 500));
  EXPECT_GT(max_abs(column(sim, "veh.wheel.fl.travel")), 0.004);
  const std::vector<double> right_surface = column(sim, "veh.wheel.fr.surface");
  EXPECT_EQ(*std::max_element(right_surface.begin(), right_surface.end()), 0.0);  // on the road
}

// The road texture reaches the steering wheel: a class B road shakes the
// force feedback, a smooth one does not.
TEST(Full, RoughRoadShakesTheSteering) {
  const auto ffb_shake = [](const std::string& roughness) {
    TrackDefinition def = straight();
    def.surface.roughness = roughness;
    const TrackFile file(def);
    Simulation sim(load_example("sine_steer", {{"track.file", file.path()},
                                               {"driver.steer.type", "const"},
                                               {"driver.steer.value", "0"},
                                               {"initial.x_m", "20"},
                                               {"initial.y_m", "0"},
                                               {"initial.speed_mps", "15"},
                                               {"driver.speed_hold_kph", "54"},
                                               {"logging.rate_hz", "1000"}}));
    sim.run_for(6.0);
    // Shake: the torque's change from one millisecond to the next.
    const std::vector<double> ffb = column(sim, "veh.ffb.torque_sw");
    std::vector<double> diff;
    for (std::size_t i = 2001; i < ffb.size(); ++i) {
      diff.push_back(ffb[i] - ffb[i - 1]);
    }
    return stddev(diff);
  };
  const double rough = ffb_shake("B"), smooth = ffb_shake("");
  EXPECT_GT(rough, 1e-3);  // [N m per ms]
  EXPECT_GT(rough, 5.0 * smooth);
}

// V-09: on a four-post rig, a symmetric car shaken in phase on all wheels
// heaves like a quarter car; body and wheel amplitudes match the analytic
// two-mass response from body resonance through wheel hop.
TEST(Full, FourPostRigMatchesTheQuarterCar) {
  Scenario scenario = load_example("drive", {{"vehicle.cg_to_front_axle_m", "0.765"},
                                             {"vehicle.track_rear_m", "1.22"},
                                             {"vehicle.suspension.rear.spring_rate_n_per_mm", "35"},
                                             {"vehicle.suspension.rear.damper_n_s_per_m", "1500"},
                                             {"vehicle.suspension.front.damper_n_s_per_m", "1500"},
                                             {"vehicle.suspension.rear.damper_rebound_n_s_per_m", "1500"},
                                             {"vehicle.suspension.front.damper_rebound_n_s_per_m", "1500"},
                                             {"vehicle.suspension.front.damper_knee_mps", "100"},
                                             {"vehicle.suspension.rear.damper_knee_mps", "100"}});
  // Same corners front and rear: copy the front hardpoints to the rear.
  const ParamTree front = scenario.params.subtree("vehicle.suspension.front");
  for (const auto& [key, value] : front.entries()) {
    if (key.rfind("hardpoints.", 0) == 0 || key.rfind("static_", 0) == 0) {
      scenario.params.set("vehicle.suspension.rear." + key, value);
    }
  }
  const ParamTree& params = scenario.params;
  SignalBus bus;
  InputHub inputs;
  Context ctx(bus, inputs, RealismConfig::from_params(params), 1, 0.001);
  TwoTrackModel model;
  model.init(params, ctx);
  ASSERT_TRUE(model.full());
  std::vector<double> x(model.state_size());
  model.initial_state(x);
  model.write_outputs(0.0, x, bus);

  const double ms = model.sprung_mass() / 4.0;
  const double mu = params.get_double("vehicle.unsprung_mass_kg.fl");
  const double k = params.get_double("vehicle.suspension.front.spring_rate_n_per_mm") * 1000.0;  // motion ratio 1
  const double c = 1500.0;
  const double kt = params.get_double("vehicle.tire.vertical_stiffness_n_per_mm") * 1000.0;
  const double ct = params.get_double("vehicle.tire.vertical_damping_n_s_per_m");
  const double amplitude = 0.002;
  const double z_body0 = x[model.body_offset() + TwoTrackModel::kZ];
  const double z_wheel0 = x[model.unsprung_offset()];

  std::vector<SignalId> rig_z, rig_vz;
  for (const char* corner : {"fl", "fr", "rl", "rr"}) {
    rig_z.push_back(bus.require(std::string("rig.") + corner + ".z"));
    rig_vz.push_back(bus.require(std::string("rig.") + corner + ".vz"));
  }
  Rk4Integrator integrator;
  for (const double f : {1.0, 2.0, 3.5, 6.0, 10.0, 15.0, 22.0}) {
    model.initial_state(x);
    const double w = 2.0 * kPi * f;
    const double dt = 0.0005;
    const int settle = static_cast<int>(std::ceil(3.0 / dt)), cycles = static_cast<int>(std::ceil(10.0 * f));
    const int measure = static_cast<int>(std::lround(cycles / f / dt));
    std::complex<double> body{}, wheel{};
    for (int i = 0; i < settle + measure; ++i) {
      const double t = i * dt;
      for (std::size_t corner = 0; corner < 4; ++corner) {
        bus.set(rig_z[corner], amplitude * std::sin(w * (t + 0.5 * dt)));
        bus.set(rig_vz[corner], amplitude * w * std::cos(w * (t + 0.5 * dt)));
      }
      model.read_inputs(bus);
      integrator.step(model, t, dt, x);
      if (i >= settle) {
        const std::complex<double> phasor = std::exp(std::complex<double>(0.0, -w * (t + dt)));
        body += (x[model.body_offset() + TwoTrackModel::kZ] - z_body0) * phasor;
        wheel += (x[model.unsprung_offset()] - z_wheel0) * phasor;
      }
    }
    // Fourier coefficient of sin: 2 / N * sum(x e^{-iwt}) = amplitude * (-i) * H.
    const double measured_body = 2.0 * std::abs(body) / measure / amplitude;
    const double measured_wheel = 2.0 * std::abs(wheel) / measure / amplitude;

    // Analytic quarter car.
    const std::complex<double> jw(0.0, w);
    const std::complex<double> a11 = -w * w * ms + jw * c + k, a12 = -(jw * c + k);
    const std::complex<double> a22 = -w * w * mu + jw * (c + ct) + k + kt, b2 = kt + jw * ct;
    const std::complex<double> det = a11 * a22 - a12 * a12;
    const double analytic_body = std::abs(-a12 * b2 / det);
    const double analytic_wheel = std::abs(a11 * b2 / det);
    EXPECT_NEAR(measured_body, analytic_body, 0.03 * analytic_body + 0.002) << f << " Hz";
    EXPECT_NEAR(measured_wheel, analytic_wheel, 0.03 * analytic_wheel + 0.002) << f << " Hz";
  }
}


// Ride height aero map (§6.4): downforce compresses the springs, the map
// gives more downforce closer to the road; diving under braking moves the
// balance forward until the front stalls.
TEST(Full, AeroFollowsTheRideHeightMap) {
  Simulation sim(load_example("straight_accel", {{"initial.speed_mps", "28"}, {"driver.throttle.value", "0.25"}}));
  const ParamTree& params = sim.scenario().params;
  const AeroMap map = AeroMap::from_params(params);
  ASSERT_TRUE(map.enabled);
  sim.run_for(1.5);
  const double front = signal(sim, "veh.aero.ride_height_front"), rear = signal(sim, "veh.aero.ride_height_rear");
  EXPECT_LT(front, -0.01);
  EXPECT_LT(rear, -0.01);
  const AeroMap::Sample at = map.at(front, rear);
  EXPECT_GT(at.cla_factor, 1.05);
  const double airspeed = signal(sim, "veh.aero.airspeed");
  const double q = 0.5 * params.get_double("vehicle.aero.air_density_kgm3") * airspeed * airspeed;
  EXPECT_NEAR(signal(sim, "veh.aero.downforce"), params.get_double("vehicle.aero.cla_m2") * at.cla_factor * q, 1e-6);
  EXPECT_NEAR(signal(sim, "veh.aero.drag"), params.get_double("vehicle.aero.cda_m2") * at.cda_factor * q, 1e-6);
  EXPECT_NEAR(signal(sim, "veh.aero.front_ratio"),
              params.get_double("vehicle.aero.front_ratio") + at.front_ratio_shift, 1e-12);

  // Softer front springs put the nose below the stall point: less downforce
  // there and the balance moves rearwards instead of forwards.
  Simulation soft(load_example("straight_accel", {{"initial.speed_mps", "30"},
                                                  {"driver.throttle.value", "0.3"},
                                                  {"vehicle.suspension.front.spring_rate_n_per_mm", "14"}}));
  soft.run_for(1.5);
  EXPECT_LT(signal(soft, "veh.aero.ride_height_front"), -0.022);
  EXPECT_LT(signal(soft, "veh.aero.front_ratio"), params.get_double("vehicle.aero.front_ratio"));
  EXPECT_GT(signal(sim, "veh.aero.front_ratio"), params.get_double("vehicle.aero.front_ratio"));
  // The books still balance with the map moving the forces (V-03).
  EXPECT_LT(std::abs(signal(soft, "energy.residual")), 1e-6 * signal(soft, "energy.battery_chemical"));

  // Without the map, the fixed figures.
  Simulation fixed(load_example("straight_accel", {{"initial.speed_mps", "28"},
                                                   {"driver.throttle.value", "0.25"},
                                                   {"vehicle.aero.ride_height_map", "off"}}));
  fixed.run_for(0.5);
  EXPECT_DOUBLE_EQ(signal(fixed, "veh.aero.front_ratio"), params.get_double("vehicle.aero.front_ratio"));
}

TEST(Full, RollingRadiusFollowsPressureAndLoad) {
  const auto radius_after = [](const std::string& tire_temp, const char* corner, double seconds,
                               Scenario::Overrides extra, const char* scenario = "sine_steer") {
    extra["initial.tire_temp_c"] = tire_temp;
    Simulation sim(load_example(scenario, extra));
    sim.run_for(seconds);
    return std::pair{signal(sim, std::string("veh.wheel.") + corner + ".rolling_radius"),
                     signal(sim, std::string("veh.wheel.") + corner + ".tire_pressure")};
  };
  const ParamTree params = load_example("sine_steer").params;
  const double nominal = params.get_double("vehicle.wheel.radius_m");
  const double per_kpa = 1e-3 * params.get_double("vehicle.tire.rolling_radius.pressure_mm_per_kpa");
  const double optimum = params.get_double("vehicle.tire.pressure.optimum_kpa");
  ASSERT_GT(per_kpa, 0.0);
  // Cold tires: lower pressure, smaller radius, as the pressure says (static load).
  const auto [cold, cold_pressure] = radius_after("20", "fl", 0.5, {{"driver.throttle.value", "0"}}, "straight_accel");
  EXPECT_LT(cold_pressure, optimum - 20.0);
  EXPECT_NEAR(cold, nominal + per_kpa * (cold_pressure - optimum), 2e-6);
  // Cornering left: the loaded outer tire rolls on a smaller radius.
  const Scenario::Overrides turn = {{"driver.steer.type", "const"}, {"driver.steer.value", "30"}};
  EXPECT_LT(radius_after("75", "fr", 6.0, turn).first + 5e-4, radius_after("75", "fl", 6.0, turn).first);
}


TEST(AeroMap, BilinearAndHeldAtTheEdges) {
  const ParamTree params = ParamTree::from_yaml_string(R"(
vehicle:
  aero:
    ride_height_map:
      front_mm: [-10, 0, 10]
      rear_mm: [0, 20]
      cla_factor: [[1.2, 1.0], [1.0, 0.8], [0.9, 0.7]]
      front_ratio_shift: [[0.1, 0.0], [0.0, -0.1], [-0.1, -0.2]]
)");
  const AeroMap map = AeroMap::from_params(params);
  ASSERT_TRUE(map.enabled);
  EXPECT_DOUBLE_EQ(map.at(0.0, 0.0).cla_factor, 1.0);
  EXPECT_NEAR(map.at(-0.005, 0.01).cla_factor, 0.25 * (1.2 + 1.0 + 1.0 + 0.8), 1e-12);
  EXPECT_NEAR(map.at(0.005, 0.005).front_ratio_shift, 0.5 * (0.75 * 0.0 + 0.25 * -0.1) + 0.5 * (0.75 * -0.1 + 0.25 * -0.2),
              1e-12);
  EXPECT_DOUBLE_EQ(map.at(-0.05, -0.05).cla_factor, 1.2);  // held at the corner
  EXPECT_DOUBLE_EQ(map.at(0.05, 0.05).cla_factor, 0.7);
  EXPECT_DOUBLE_EQ(map.at(0.0, 0.0).cda_factor, 1.0);  // not given: 1

  ParamTree bad = params;
  bad.set("vehicle.aero.ride_height_map.cla_factor.2", "[0.9]");
  EXPECT_THROW(AeroMap::from_params(bad), std::invalid_argument);
  bad = params;
  bad.set("vehicle.aero.ride_height_map.front_mm.1", "-20");
  EXPECT_THROW(AeroMap::from_params(bad), std::invalid_argument);
  EXPECT_FALSE(AeroMap::from_params(ParamTree{}).enabled);
  EXPECT_DOUBLE_EQ(AeroMap{}.at(-0.02, 0.01).cla_factor, 1.0);
}

}  // namespace
}  // namespace vsim

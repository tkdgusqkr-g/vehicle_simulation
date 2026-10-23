#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

#include "vsim/models/driveline/driveline.hpp"

namespace vsim {
namespace {

constexpr double kWheelInertia = 0.3;
constexpr double kRotorInertia = 4e-4;
constexpr std::size_t kFl = 0, kFr = 1, kRl = 2, kRr = 3;

DrivelineConfig config(const char* yaml) {
  return DrivelineConfig::from_params(ParamTree::from_yaml_string(yaml));
}

constexpr const char* kInWheel = R"(
vehicle:
  driveline:
    - {motor: fl, wheel: fl, gear_ratio: 14.0, efficiency: 0.97}
    - {motor: fr, wheel: fr, gear_ratio: 14.0, efficiency: 0.97}
    - {motor: rl, wheel: rl, gear_ratio: 14.0, efficiency: 0.97}
    - {motor: rr, wheel: rr, gear_ratio: 14.0, efficiency: 0.97}
)";

std::string rear_diff(const std::string& diff) {
  return "vehicle:\n  driveline:\n    - {motor: rear, wheels: [rl, rr], gear_ratio: 4.0, "
         "efficiency: 1.0, diff: " + diff + "}\n";
}

struct Result {
  std::vector<double> qdot;
  Driveline::Losses losses;
};

Result accelerate(const Driveline& line, const std::vector<double>& q, Driveline::Loads loads,
                  const std::vector<double>& motor_torque) {
  loads.motor_torque = motor_torque;
  Result r;
  r.qdot.assign(line.dof_count(), 0.0);
  line.accelerations(q, loads, r.qdot, r.losses);
  return r;
}

// V-06: each in-wheel motor drives only its own wheel.
TEST(Driveline, InWheelMotorsAreIndependent) {
  Driveline line;
  line.init(config(kInWheel), kWheelInertia, kRotorInertia);
  ASSERT_EQ(line.dof_count(), 4u);
  ASSERT_EQ(line.motor_count(), 4u);

  const std::vector<double> q(4, 10.0);
  const Result r = accelerate(line, q, {}, {20.0, 0.0, 0.0, 0.0});
  const double inertia = kWheelInertia + kRotorInertia * 14.0 * 14.0;
  EXPECT_NEAR(line.wheel_speed(kFl, r.qdot), 20.0 * 14.0 * 0.97 / inertia, 1e-9);
  for (const std::size_t wheel : {kFr, kRl, kRr}) {
    EXPECT_EQ(line.wheel_speed(wheel, r.qdot), 0.0);
  }
  EXPECT_DOUBLE_EQ(line.motor_speed(0, q), 140.0);
  EXPECT_NEAR(r.losses.gear, 20.0 * 140.0 * 0.03, 1e-9);
  EXPECT_NEAR(line.kinetic_energy(q), 0.5 * 4.0 * inertia * 100.0, 1e-9);
}

// Efficiency costs power when regenerating too: less torque reaches the motor.
TEST(Driveline, GearLossesInBothDirections) {
  Driveline line;
  line.init(config(kInWheel), kWheelInertia, 0.0);
  const std::vector<double> q(4, 10.0);
  const Result r = accelerate(line, q, {}, {-20.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(line.wheel_speed(kFl, r.qdot) * kWheelInertia, -20.0 * 14.0 / 0.97, 1e-9);
  EXPECT_GT(r.losses.gear, 0.0);
}

// V-06: an open differential splits torque equally and the motor turns at
// the mean of the two wheel speeds.
TEST(Driveline, OpenDiffSplitsTorqueEqually) {
  Driveline line;
  line.init(config(rear_diff("{type: open}").c_str()), kWheelInertia, 0.0);
  ASSERT_EQ(line.dof_count(), 4u);  // two diff sides + two free front wheels

  std::vector<double> q(4, 0.0);
  Driveline::Loads loads;
  loads.wheel_torque[kRl] = -30.0;  // left wheel has more grip
  const Result r = accelerate(line, q, loads, {50.0});
  const double each = 0.5 * 50.0 * 4.0;
  EXPECT_NEAR(line.wheel_speed(kRl, r.qdot) * kWheelInertia, each - 30.0, 1e-9);
  EXPECT_NEAR(line.wheel_speed(kRr, r.qdot) * kWheelInertia, each, 1e-9);

  q = {20.0, 30.0, 0.0, 0.0};  // the diff's sides come first, then free wheels
  EXPECT_DOUBLE_EQ(line.wheel_speed(kRl, q), 20.0);
  EXPECT_DOUBLE_EQ(line.wheel_speed(kRr, q), 30.0);
  EXPECT_DOUBLE_EQ(line.motor_speed(0, q), 4.0 * 25.0);
}

TEST(Driveline, OpenDiffWithRotorInertiaKeepsTheKinematicConstraint) {
  Driveline line;
  line.init(config(rear_diff("{type: open}").c_str()), kWheelInertia, kRotorInertia);
  // The motor inertia couples the two sides: M = [Iw + J g^2/4, J g^2/4; ...].
  const double coupling = kRotorInertia * 4.0;  // J (g/2)^2
  EXPECT_NEAR(line.mass(0, 1), coupling, 1e-12);
  EXPECT_NEAR(line.mass(0, 0), kWheelInertia + coupling, 1e-12);

  // Equal loads: both sides accelerate together, as a rigid axle would.
  const Result r = accelerate(line, std::vector<double>(4, 0.0), {}, {50.0});
  EXPECT_NEAR(line.wheel_speed(kRl, r.qdot), line.wheel_speed(kRr, r.qdot), 1e-9);
  const double rigid = 50.0 * 4.0 / (2.0 * kWheelInertia + kRotorInertia * 16.0);
  EXPECT_NEAR(line.wheel_speed(kRl, r.qdot), rigid, 1e-9);
}

// V-06: a locked differential forces equal wheel speeds.
TEST(Driveline, LockedDiffSharesOneSpeed) {
  Driveline line;
  line.init(config(rear_diff("{type: locked}").c_str()), kWheelInertia, 0.0);
  ASSERT_EQ(line.dof_count(), 3u);
  Driveline::Loads loads;
  loads.wheel_torque[kRl] = -30.0;
  const Result r = accelerate(line, std::vector<double>(3, 0.0), loads, {50.0});
  EXPECT_DOUBLE_EQ(line.wheel_speed(kRl, r.qdot), line.wheel_speed(kRr, r.qdot));
  EXPECT_NEAR(line.wheel_speed(kRl, r.qdot), (200.0 - 30.0) / (2.0 * kWheelInertia), 1e-9);
}

// A limited-slip diff moves torque from the faster to the slower side.
TEST(Driveline, LimitedSlipDiffResistsSpeedDifference) {
  Driveline line;
  line.init(config(rear_diff("{type: lsd, preload_nm: 20, ramp: 0.5}").c_str()), kWheelInertia,
            0.0);
  const std::vector<double> q = {30.0, 20.0, 0.0, 0.0};  // left spins faster
  const Result r = accelerate(line, q, {}, {50.0});
  const double locking = (20.0 + 0.5 * 50.0 * 4.0) * std::tanh(10.0 / 0.5);
  EXPECT_NEAR(line.wheel_speed(kRl, r.qdot) * kWheelInertia, 100.0 - locking, 1e-9);
  EXPECT_NEAR(line.wheel_speed(kRr, r.qdot) * kWheelInertia, 100.0 + locking, 1e-9);
  EXPECT_NEAR(r.losses.diff, locking * 10.0, 1e-9);
}

TEST(Driveline, BrakeHoldsAWheelAtRestBelowCapacity) {
  Driveline line;
  line.init(config(kInWheel), kWheelInertia, 0.0);
  Driveline::Loads loads;
  loads.brake_capacity[kFl] = 500.0;
  loads.brake_capacity[kFr] = 100.0;
  const Result r = accelerate(line, std::vector<double>(4, 0.0), loads, {20.0, 20.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(line.wheel_speed(kFl, r.qdot), 0.0);
  // 272 N m of drive against 100 N m of brake.
  EXPECT_NEAR(line.wheel_speed(kFr, r.qdot) * kWheelInertia, 20.0 * 14.0 * 0.97 - 100.0, 1e-9);
  EXPECT_DOUBLE_EQ(r.losses.brake, 0.0);  // nothing moves yet
}

TEST(Driveline, BrakeOpposesRotationAtFullCapacity) {
  Driveline line;
  line.init(config(kInWheel), kWheelInertia, 0.0);
  Driveline::Loads loads;
  loads.brake_capacity[kRl] = 200.0;
  const std::vector<double> q = {0.0, 0.0, 50.0, 0.0};
  const Result r = accelerate(line, q, loads, {0.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(line.wheel_speed(kRl, r.qdot) * kWheelInertia, -200.0, 1e-9);
  EXPECT_NEAR(r.losses.brake, 200.0 * 50.0, 1e-9);
}

TEST(Driveline, UndrivenWheelsRollFreely) {
  Driveline line;
  line.init(config(rear_diff("{type: open}").c_str()), kWheelInertia, kRotorInertia);
  Driveline::Loads loads;
  loads.wheel_torque[kFl] = 6.0;
  const Result r = accelerate(line, std::vector<double>(4, 0.0), loads, {0.0});
  EXPECT_NEAR(line.wheel_speed(kFl, r.qdot), 6.0 / kWheelInertia, 1e-12);
  EXPECT_EQ(line.wheel_speed(kFr, r.qdot), 0.0);
}

TEST(Driveline, RejectsInvalidTopologies) {
  EXPECT_THROW(config("vehicle: {driveline: []}"), std::invalid_argument);
  EXPECT_THROW(config(R"(
vehicle:
  driveline:
    - {motor: a, wheel: rl, gear_ratio: 10}
    - {motor: b, wheels: [rl, rr], gear_ratio: 4}
)"),
               std::invalid_argument);
  EXPECT_THROW(config(R"(
vehicle:
  driveline:
    - {motor: a, wheel: rl, gear_ratio: 10}
    - {motor: a, wheel: rr, gear_ratio: 10}
)"),
               std::invalid_argument);
  EXPECT_THROW(config("vehicle: {driveline: [{motor: a, wheel: xx, gear_ratio: 10}]}"),
               std::invalid_argument);
  EXPECT_THROW(config("vehicle: {driveline: [{motor: a, wheels: [fl, fr, rl], gear_ratio: 4}]}"),
               std::invalid_argument);
  EXPECT_THROW(config("vehicle: {driveline: [{motor: a, wheel: rl, gear_ratio: 4, "
                      "diff: {type: open}}]}"),
               std::invalid_argument);
  EXPECT_THROW(config("vehicle: {driveline: [{motor: a, wheels: [rl, rr], gear_ratio: 4, "
                      "diff: {type: torsen}}]}"),
               std::invalid_argument);
  EXPECT_THROW(config("vehicle: {driveline: [{motor: a, wheel: rl, gear_ratio: 0}]}"),
               std::invalid_argument);
  EXPECT_EQ(config(kInWheel).motor_ids(), (std::vector<std::string>{"fl", "fr", "rl", "rr"}));
}


constexpr const char* kCompliant = R"(
vehicle:
  driveline:
    - {motor: fl, wheel: fl, gear_ratio: 14.0, efficiency: 1.0,
       shaft: {stiffness_nm_per_rad: 6000, damping_nms_per_rad: 0, backlash_deg: 1.0}}
)";

// Integrates the free driveline (no road, no motor torque) with RK4.
void run_free(const Driveline& line, std::vector<double>& q, std::vector<double>& twist, double seconds,
              const std::function<void(double)>& each = {}) {
  Driveline::Loads loads;
  const std::vector<double> torque(line.motor_count(), 0.0);
  loads.motor_torque = torque;
  const double h = 1e-5;
  const std::size_t n = q.size(), m = twist.size();
  const auto f = [&](const std::vector<double>& x, std::vector<double>& dx) {
    Driveline::Losses losses;
    line.accelerations(std::span<const double>(x.data(), n), std::span<const double>(x.data() + n, m), loads,
                       std::span<double>(dx.data(), n), std::span<double>(dx.data() + n, m), losses);
  };
  std::vector<double> x(n + m), k1(n + m), k2(n + m), k3(n + m), k4(n + m), t(n + m);
  std::copy(q.begin(), q.end(), x.begin());
  std::copy(twist.begin(), twist.end(), x.begin() + static_cast<std::ptrdiff_t>(n));
  for (int i = 0; i < static_cast<int>(seconds / h); ++i) {
    f(x, k1);
    for (std::size_t j = 0; j < x.size(); ++j) t[j] = x[j] + 0.5 * h * k1[j];
    f(t, k2);
    for (std::size_t j = 0; j < x.size(); ++j) t[j] = x[j] + 0.5 * h * k2[j];
    f(t, k3);
    for (std::size_t j = 0; j < x.size(); ++j) t[j] = x[j] + h * k3[j];
    f(t, k4);
    for (std::size_t j = 0; j < x.size(); ++j) x[j] += h / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
    if (each) each(x[n]);  // the twist
  }
  std::copy(x.begin(), x.begin() + static_cast<std::ptrdiff_t>(n), q.begin());
  std::copy(x.begin() + static_cast<std::ptrdiff_t>(n), x.end(), twist.begin());
}

// PH-04 building block: rotor and wheel on a spring with free play.
TEST(Driveline, CompliantShaftRingsAndRattlesWithoutLosingEnergy) {
  Driveline line;
  line.init(config(kCompliant), kWheelInertia, kRotorInertia);
  ASSERT_EQ(line.dof_count(), 5u);  // rotor + wheel, and the three free wheels
  ASSERT_EQ(line.shaft_count(), 1u);
  const double rotor = kRotorInertia * 14.0 * 14.0;  // at the wheel
  // No torque inside the free play.
  std::vector<double> q(line.dof_count(), 0.0), twist{0.4 * 3.14159265358979 / 180.0};
  EXPECT_EQ(line.shaft_torque(0, q, twist), 0.0);
  twist[0] = 1.0 * 3.14159265358979 / 180.0;  // 0.5 deg past the play
  EXPECT_NEAR(line.shaft_torque(0, q, twist), 6000.0 * 0.5 * 3.14159265358979 / 180.0, 1e-9);

  // Without play: rotor and wheel ring at sqrt(k (1/Jr + 1/Jw)).
  Driveline tight;
  tight.init(config(R"(
vehicle:
  driveline:
    - {motor: fl, wheel: fl, gear_ratio: 14.0, shaft: {stiffness_nm_per_rad: 6000}}
)"), kWheelInertia, kRotorInertia);
  q.assign(tight.dof_count(), 0.0);
  q[0] = 1.0;  // the rotor turns, the wheel does not
  twist = {0.0};
  const double energy0 = tight.kinetic_energy(q);
  double last = 0.0, crossings = 0.0, first = -1.0, last_time = 0.0, time = 0.0;
  run_free(tight, q, twist, 0.2, [&](double tw) {
    time += 1e-5;
    if (last < 0.0 && tw >= 0.0) {
      if (first < 0.0) first = time;
      last_time = time;
      crossings += 1.0;
    }
    last = tw;
  });
  const double period = (last_time - first) / (crossings - 1.0);
  const double expected = 2.0 * 3.14159265358979 / std::sqrt(6000.0 * (1.0 / rotor + 1.0 / kWheelInertia));
  EXPECT_NEAR(period, expected, 1e-3 * expected);
  EXPECT_NEAR(tight.kinetic_energy(q) + tight.spring_energy(twist), energy0, 1e-6 * energy0);

  // With play: the same total energy, just slower (the parts fly across the gap).
  q.assign(line.dof_count(), 0.0);
  q[0] = 1.0;
  twist = {0.0};
  run_free(line, q, twist, 0.2);
  // (the force has a corner at the edge of the play: a little integration error there)
  EXPECT_NEAR(line.kinetic_energy(q) + line.spring_energy(twist), energy0, 1e-4 * energy0);

  EXPECT_THROW(config(R"(
vehicle:
  driveline:
    - {motor: rear, wheels: [rl, rr], gear_ratio: 4.0, shaft: {stiffness_nm_per_rad: 6000}}
)"), std::invalid_argument);
}

}  // namespace
}  // namespace vsim

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <stdexcept>

#include "test_support.hpp"
#include "vsim/input/force_feedback.hpp"
#include "vsim/input/wheel_profile.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

constexpr double kDegree = std::numbers::pi / 180.0;

TEST(PedalAxis, MapsRawTravelWithDeadzoneAndGamma) {
  PedalAxis pedal;
  pedal.axis = 1;
  pedal.released = 32767;
  pedal.pressed = -32768;
  pedal.deadzone = 0.0;
  EXPECT_DOUBLE_EQ(pedal.value(32767), 0.0);
  EXPECT_DOUBLE_EQ(pedal.value(-32768), 1.0);
  EXPECT_NEAR(pedal.value(0), 0.5, 1e-4);
  pedal.deadzone = 0.1;
  EXPECT_DOUBLE_EQ(pedal.value(32767 - 0.05 * 65535), 0.0);  // inside the rest deadzone
  EXPECT_DOUBLE_EQ(pedal.value(-32768 + 0.05 * 65535), 1.0);
  pedal.deadzone = 0.0;
  pedal.gamma = 2.0;
  EXPECT_NEAR(pedal.value(0), 0.25, 1e-4);
  PedalAxis missing;
  EXPECT_EQ(missing.value(-32768), 0.0);
}

TEST(WheelProfile, SteeringAngleFollowsRangeAndDirection) {
  WheelProfile p = WheelProfile::logitech_g29();
  p.steer_range_deg = 900.0;
  // SDL: negative is left; vsim: positive is left.
  EXPECT_NEAR(p.steer_angle(-32768), 450.0 * kDegree, 1e-9);
  EXPECT_NEAR(p.steer_angle(16384), -225.0 * kDegree, 1e-9);
  p.steer_center = 100.0;
  EXPECT_DOUBLE_EQ(p.steer_angle(100.0), 0.0);
}

TEST(WheelProfile, MatchesByUsbIdOrName) {
  const WheelProfile g29 = WheelProfile::logitech_g29();
  EXPECT_TRUE(g29.matches("Some Wheel", 0x046d, 0xc24f));
  EXPECT_TRUE(g29.matches("Logitech G29 Driving Force Racing Wheel", 0, 0));
  EXPECT_TRUE(g29.matches("logitech g29", 0, 0));
  EXPECT_FALSE(g29.matches("Xbox Controller", 0x045e, 0x02ea));
}

TEST(WheelProfile, RoundTripsThroughYaml) {
  WheelProfile p = WheelProfile::logitech_g29();
  p.name = "Test Wheel";
  p.priority = 10;
  p.steer_axis = 2;
  p.steer_range_deg = 540.0;
  p.steer_invert = false;
  p.throttle = {5, -100.0, 30000.0, 0.03, 1.5};
  p.clutch.axis = -1;
  p.button_reset = 7;
  p.ffb.gain = 0.2;
  p.ffb.invert = false;

  const auto file = std::filesystem::temp_directory_path() /
                    ("vsim_profile_" + std::to_string(std::random_device{}()) + ".yaml");
  {
    std::ofstream out(file);
    out << p.to_yaml();
  }
  const WheelProfile q = WheelProfile::from_yaml_file(file);
  std::filesystem::remove(file);
  EXPECT_EQ(q.name, "Test Wheel");
  EXPECT_EQ(q.priority, 10);
  EXPECT_EQ(q.usb_ids, p.usb_ids);
  EXPECT_EQ(q.match_names, p.match_names);
  EXPECT_EQ(q.steer_axis, 2);
  EXPECT_DOUBLE_EQ(q.steer_range_deg, 540.0);
  EXPECT_FALSE(q.steer_invert);
  EXPECT_EQ(q.throttle.axis, 5);
  EXPECT_DOUBLE_EQ(q.throttle.released, -100.0);
  EXPECT_DOUBLE_EQ(q.throttle.gamma, 1.5);
  EXPECT_EQ(q.clutch.axis, -1);
  EXPECT_EQ(q.button_reset, 7);
  EXPECT_DOUBLE_EQ(q.ffb.gain, 0.2);
  EXPECT_FALSE(q.ffb.invert);
}

TEST(WheelProfile, ShippedProfilesLoad) {
  const auto dir = std::filesystem::path(VSIM_SOURCE_DIR) / "config" / "input";
  const std::vector<WheelProfile> profiles = WheelProfile::load_directory(dir);
  ASSERT_FALSE(profiles.empty());
  const WheelProfile* g29 = find_profile(profiles, "Logitech G29 Driving Force Racing Wheel USB", 0x046d, 0xc24f);
  ASSERT_NE(g29, nullptr);
  EXPECT_TRUE(g29->ffb_enabled);
}

TEST(WheelProfile, RejectsBadValues) {
  EXPECT_THROW(WheelProfile::from_params(ParamTree::from_yaml_string("name: x\nmatch: {usb_ids: [\"zz\"]}")),
               std::invalid_argument);
  EXPECT_THROW(WheelProfile::from_params(ParamTree::from_yaml_string(
                   "name: x\npedals: {brake: {axis: 1, released: 5, pressed: 5}}")),
               std::invalid_argument);
}

ForceFeedbackSettings plain() {
  ForceFeedbackSettings s;
  s.gain = 0.1;
  s.max_torque = 2.0;
  s.damping = 0.0;
  s.friction = 0.0;
  s.end_stop = 10.0;
  s.fade_time = 0.1;
  s.invert = false;
  return s;
}

TEST(ForceFeedback, RoadTorqueFadesInAndOut) {
  ForceFeedbackShaper shaper(plain());
  const double dt = 0.002;
  double level = 0.0;
  for (int i = 0; i < 25; ++i) {  // half the fade time
    level = shaper.update(dt, 0.0, -10.0, 1.9, true);
  }
  EXPECT_NEAR(level, -0.5 * 0.5, 0.02);  // -10 N m * 0.1 = -1 N m of 2 N m, half faded in
  for (int i = 0; i < 100; ++i) {
    level = shaper.update(dt, 0.0, -10.0, 1.9, true);
  }
  EXPECT_NEAR(level, -0.5, 1e-9);
  for (int i = 0; i < 100; ++i) {  // updates stopped (paused sim)
    level = shaper.update(dt, 0.0, -10.0, 1.9, false);
  }
  EXPECT_DOUBLE_EQ(level, 0.0);
}

TEST(ForceFeedback, EndStopPushesBackPastTheLock) {
  ForceFeedbackShaper shaper(plain());
  const double lock = 110.0 * kDegree;
  EXPECT_DOUBLE_EQ(shaper.update(0.002, lock - 0.01, 0.0, lock, false), 0.0);
  ForceFeedbackShaper left(plain());
  EXPECT_LT(left.update(0.002, lock + 0.1, 0.0, lock, false), 0.0);  // past left lock: push right
  ForceFeedbackShaper right(plain());
  EXPECT_GT(right.update(0.002, -lock - 0.1, 0.0, lock, false), 0.0);
  ForceFeedbackShaper far(plain());
  EXPECT_DOUBLE_EQ(far.update(0.002, lock + 1.0, 0.0, lock, false), -1.0);
  EXPECT_TRUE(far.clipping());
}

TEST(ForceFeedback, DampingOpposesMotionAndInvertFlipsTheDevice) {
  ForceFeedbackSettings s = plain();
  s.damping = 0.5;
  ForceFeedbackShaper shaper(s);
  shaper.update(0.001, 0.0, 0.0, 2.0, false);
  double level = 0.0;
  for (int i = 1; i <= 200; ++i) {  // turning left at 1 rad/s
    level = shaper.update(0.001, 0.001 * i, 0.0, 2.0, false);
  }
  EXPECT_NEAR(shaper.angular_velocity(), 1.0, 1e-3);
  EXPECT_NEAR(level, -0.25, 1e-3);
  s.invert = true;
  ForceFeedbackShaper inverted(s);
  inverted.update(0.001, 0.0, 0.0, 2.0, false);
  for (int i = 1; i <= 200; ++i) {
    level = inverted.update(0.001, 0.001 * i, 0.0, 2.0, false);
  }
  EXPECT_NEAR(level, 0.25, 1e-3);
}

TEST(ForceFeedback, SuspendsWhenTheWheelStaysFarPastTheLock) {
  ForceFeedbackShaper shaper(plain());
  const double lock = 110.0 * kDegree;
  const double far = lock + 100.0 * kDegree;
  double level = 0.0;
  for (int i = 0; i < 200; ++i) {  // 0.4 s: still pushing back
    level = shaper.update(0.002, far, 0.0, lock, true);
  }
  EXPECT_FALSE(shaper.suspended());
  EXPECT_LT(level, 0.0);
  for (int i = 0; i < 100; ++i) {
    level = shaper.update(0.002, far, 0.0, lock, true);
  }
  EXPECT_TRUE(shaper.suspended());
  EXPECT_EQ(level, 0.0);
  shaper.update(0.002, lock + 0.1, 0.0, lock, true);  // still outside the lock
  EXPECT_TRUE(shaper.suspended());
  shaper.update(0.002, 0.0, 0.0, lock, true);
  EXPECT_FALSE(shaper.suspended());
}

// The DIL driver takes the wheel when one is connected, else the keyboard,
// and posts the road torque for force feedback.
TEST(DilDriver, UsesTheWheelWhenConnected) {
  Simulation sim(load_example("drive", {{"time.t_end", "5"}}));
  WheelState wheel;
  wheel.connected = true;
  wheel.steer = 0.5;
  wheel.throttle = 0.4;
  sim.inputs().set_wheel(wheel);
  sim.run_for(1.0);
  EXPECT_DOUBLE_EQ(signal(sim, "drv.cmd.steer"), 0.5);
  EXPECT_DOUBLE_EQ(signal(sim, "drv.cmd.throttle"), 0.4);
  EXPECT_EQ(signal(sim, "drv.input.wheel"), 1.0);
  const ForceFeedbackRequest ffb = sim.inputs().force_feedback();
  EXPECT_LT(ffb.torque, 0.0);  // turning left: the road pushes the wheel right
  EXPECT_NEAR(ffb.lock, sim.scenario().params.get_double("vehicle.steering.lock_deg") * kDegree, 1e-12);

  // Unplugged: back to the keyboard.
  sim.inputs().set_wheel({});
  sim.inputs().set_keys({true, false, false, false});
  sim.run_for(0.5);
  EXPECT_EQ(signal(sim, "drv.input.wheel"), 0.0);
  EXPECT_GT(signal(sim, "drv.cmd.throttle"), 0.5);
}

TEST(DilDriver, WheelModeIgnoresTheKeyboard) {
  Simulation sim(load_example("drive", {{"driver.input", "wheel"}}));
  sim.inputs().set_keys({true, false, true, false});
  sim.run_for(0.5);
  EXPECT_EQ(signal(sim, "drv.cmd.throttle"), 0.0);
  EXPECT_EQ(signal(sim, "drv.cmd.steer"), 0.0);
  EXPECT_THROW(Simulation(load_example("drive", {{"driver.input", "joystick"}})), std::invalid_argument);
}

}  // namespace
}  // namespace vsim

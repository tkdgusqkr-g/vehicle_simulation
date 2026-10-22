// InputService against an SDL virtual joystick that claims to be a G29:
// hotplug, profile matching, axis mapping and unplugging. Force feedback
// needs real hardware (virtual joysticks have no haptics); see vsim-input.
#include <gtest/gtest.h>

#include <SDL3/SDL.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <numbers>
#include <thread>

#include "vsim/input/input_service.hpp"

namespace vsim {
namespace {

using Clock = std::chrono::steady_clock;

bool wait_for(const std::function<bool()>& condition, std::chrono::milliseconds timeout) {
  const auto deadline = Clock::now() + timeout;
  while (Clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return condition();
}

TEST(InputService, ReadsAHotpluggedWheel) {
  ASSERT_TRUE(SDL_Init(SDL_INIT_JOYSTICK)) << SDL_GetError();
  InputHub hub;
  InputService service(hub, {});
  ASSERT_FALSE(service.start().has_value());
  EXPECT_FALSE(hub.wheel().connected);

  SDL_VirtualJoystickDesc desc;
  SDL_INIT_INTERFACE(&desc);
  desc.type = SDL_JOYSTICK_TYPE_WHEEL;
  desc.vendor_id = 0x046d;
  desc.product_id = 0xc24f;
  desc.naxes = 4;
  desc.nbuttons = 8;
  desc.name = "Virtual G29";
  const SDL_JoystickID id = SDL_AttachVirtualJoystick(&desc);
  ASSERT_NE(id, 0u) << SDL_GetError();
  SDL_Joystick* joystick = SDL_OpenJoystick(id);
  ASSERT_NE(joystick, nullptr);
  // Pedals at rest (G29 pedals read +32767 released).
  for (int axis = 1; axis < 4; ++axis) {
    SDL_SetJoystickVirtualAxis(joystick, axis, 32767);
  }

  ASSERT_TRUE(wait_for([&] { return hub.wheel().connected; }, std::chrono::seconds(3)));
  EXPECT_EQ(service.status().profile, "Logitech G29");
  EXPECT_EQ(hub.wheel().device, "Virtual G29");

  // Quarter of the axis to the left = 112.5 degrees with the 900 degree default.
  SDL_SetJoystickVirtualAxis(joystick, 0, -8192);
  SDL_SetJoystickVirtualAxis(joystick, 1, -32768);  // throttle floored
  SDL_SetJoystickVirtualAxis(joystick, 2, 0);       // brake half way
  ASSERT_TRUE(wait_for([&] { return hub.wheel().throttle > 0.99; }, std::chrono::seconds(1)));
  const WheelState wheel = hub.wheel();
  EXPECT_NEAR(wheel.steer, 112.5 * std::numbers::pi / 180.0, 1e-6);
  EXPECT_NEAR(wheel.brake, 0.5, 0.01);
  EXPECT_EQ(wheel.clutch, 0.0);

  SDL_CloseJoystick(joystick);
  ASSERT_TRUE(SDL_DetachVirtualJoystick(id));
  EXPECT_TRUE(wait_for([&] { return !hub.wheel().connected; }, std::chrono::seconds(2)));
  EXPECT_FALSE(service.status().connected);

  service.stop();
  SDL_Quit();
}

// The --virtual-wheel path used for UI tests, the raw stream and a profile
// applied while the wheel is connected (what the calibration page does).
TEST(InputService, VirtualWheelRawInputAndProfileSwap) {
  InputHub hub;
  InputService::Options options;
  options.virtual_wheel = true;
  options.force_feedback = false;
  InputService service(hub, options);
  ASSERT_FALSE(service.start().has_value());
  ASSERT_TRUE(wait_for([&] { return hub.wheel().connected; }, std::chrono::seconds(3)));
  EXPECT_EQ(hub.wheel().device, "vsim virtual wheel");
  EXPECT_EQ(service.profile().name, "generic (G29 layout)");  // no profile matches

  service.set_virtual({-16384}, {0, 0, 0, 1});
  ASSERT_TRUE(wait_for([&] { const auto r = service.raw(); return !r.axes.empty() && r.axes[0] == -16384; },
                       std::chrono::seconds(1)));
  const InputService::RawInput raw = service.raw();
  EXPECT_TRUE(raw.connected);
  EXPECT_EQ(raw.axes.size(), 6u);
  ASSERT_EQ(raw.buttons.size(), 16u);
  EXPECT_EQ(raw.buttons[3], 1);
  EXPECT_EQ(raw.axes[1], 32767);  // pedals start released
  // G29 layout: inverted axis, 900 degrees: half the axis to the left is 225 degrees.
  EXPECT_NEAR(hub.wheel().steer, 225.0 * std::numbers::pi / 180.0, 1e-6);
  EXPECT_EQ(hub.wheel().throttle, 0.0);

  WheelProfile calibrated = WheelProfile::logitech_g29();
  calibrated.name = "calibrated";
  calibrated.priority = 10;
  calibrated.match_names.clear();
  calibrated.usb_ids.clear();
  calibrated.steer_axis = 4;
  calibrated.steer_invert = false;
  calibrated.steer_range_deg = 540.0;
  service.apply_profile(calibrated);
  service.set_virtual({0, 32767, 32767, 32767, 16384}, {});
  ASSERT_TRUE(wait_for([&] { return std::abs(hub.wheel().steer - 135.0 * std::numbers::pi / 180.0) < 1e-6; },
                       std::chrono::seconds(1)));
  EXPECT_EQ(service.profile().name, "calibrated");
  EXPECT_EQ(service.status().profile, "calibrated");
  EXPECT_TRUE(service.status().calibrated);
  ASSERT_EQ(service.profile().match_names.size(), 1u);
  EXPECT_EQ(service.profile().match_names[0], "vsim virtual wheel");
  service.stop();
}

}  // namespace
}  // namespace vsim

// InputService against an SDL virtual joystick that claims to be a G29:
// hotplug, profile matching, axis mapping and unplugging. Force feedback
// needs real hardware (virtual joysticks have no haptics); see vsim-input.
#include <gtest/gtest.h>

#include <SDL3/SDL.h>

#include <chrono>
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

}  // namespace
}  // namespace vsim

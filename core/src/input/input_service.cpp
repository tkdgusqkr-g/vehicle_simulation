#include "vsim/input/input_service.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>

namespace vsim {
namespace {

using Clock = std::chrono::steady_clock;

// Device names that suggest a wheel when no profile matches.
bool looks_like_wheel(std::string_view name) {
  std::string lower(name);
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower.find("wheel") != std::string::npos || lower.find("racing") != std::string::npos;
}

}  // namespace

struct InputService::Device {
  SDL_Joystick* joystick = nullptr;
  SDL_Haptic* haptic = nullptr;
  SDL_HapticEffectID effect = -1;
  SDL_HapticEffect effect_data{};
  WheelProfile profile;
  std::unique_ptr<ForceFeedbackShaper> shaper;
  Sint16 last_level = 0;
  bool reset_button = false;
  bool pause_button = false;
};

InputService::InputService(InputHub& hub, Options options) : hub_(hub), options_(std::move(options)) {
  options_.profiles.push_back(WheelProfile::logitech_g29());
}

InputService::~InputService() { stop(); }

std::optional<std::string> InputService::start() {
  if (thread_.joinable()) {
    return std::nullopt;
  }
  stop_ = false;
  started_ = false;
  // SDL must live on one thread; wait until that thread reports init.
  thread_ = std::thread([this] { run(); });
  while (!started_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::lock_guard lock(mutex_);
  if (start_error_) {
    thread_.join();
    return start_error_;
  }
  return std::nullopt;
}

void InputService::stop() {
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
}

InputService::Status InputService::status() const {
  std::lock_guard lock(mutex_);
  return status_;
}

void InputService::open_device(Device& device) {
  const char* name = SDL_GetJoystickName(device.joystick);
  const std::string device_name = name ? name : "unknown";
  const WheelProfile* profile = find_profile(options_.profiles, device_name, SDL_GetJoystickVendor(device.joystick),
                                             SDL_GetJoystickProduct(device.joystick));
  device.profile = profile ? *profile : WheelProfile::logitech_g29();
  if (!profile) {
    device.profile.name = "generic (G29 layout)";
  }
  device.shaper = std::make_unique<ForceFeedbackShaper>(device.profile.ffb);

  Status status;
  status.connected = true;
  status.device = device_name;
  status.profile = device.profile.name;
  status.calibrated = device.profile.priority > 0;

  if (options_.force_feedback && device.profile.ffb_enabled) {
    device.haptic = SDL_OpenHapticFromJoystick(device.joystick);
    if (!device.haptic) {
      status.ffb_error = std::string("no force feedback: ") + SDL_GetError();
    } else if (!(SDL_GetHapticFeatures(device.haptic) & SDL_HAPTIC_CONSTANT)) {
      status.ffb_error = "device has no constant-force effect";
      SDL_CloseHaptic(device.haptic);
      device.haptic = nullptr;
    } else {
      const Uint32 features = SDL_GetHapticFeatures(device.haptic);
      if (features & SDL_HAPTIC_AUTOCENTER) {
        SDL_SetHapticAutocenter(device.haptic, 0);  // the simulation provides centring
      }
      if (features & SDL_HAPTIC_GAIN) {
        SDL_SetHapticGain(device.haptic, 100);
      }
      SDL_HapticEffect& effect = device.effect_data;
      effect = {};
      effect.type = SDL_HAPTIC_CONSTANT;
      effect.constant.direction.type = SDL_HAPTIC_STEERING_AXIS;
      effect.constant.length = SDL_HAPTIC_INFINITY;
      effect.constant.level = 0;
      device.effect = SDL_CreateHapticEffect(device.haptic, &effect);
      if (device.effect < 0 || !SDL_RunHapticEffect(device.haptic, device.effect, 1)) {
        status.ffb_error = std::string("cannot start constant force: ") + SDL_GetError();
        SDL_CloseHaptic(device.haptic);
        device.haptic = nullptr;
        device.effect = -1;
      } else {
        status.force_feedback = true;
      }
    }
  }
  std::lock_guard lock(mutex_);
  status_ = status;
}

void InputService::close_device(Device& device) {
  if (device.haptic) {
    if (device.effect >= 0) {
      SDL_StopHapticEffect(device.haptic, device.effect);
      SDL_DestroyHapticEffect(device.haptic, device.effect);
    }
    SDL_CloseHaptic(device.haptic);
  }
  if (device.joystick) {
    SDL_CloseJoystick(device.joystick);
  }
  device = Device{};
  hub_.set_wheel({});
  std::lock_guard lock(mutex_);
  status_ = {};
}

void InputService::run() {
  // No window of ours has focus; keep reading the wheel in the background.
  SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
  if (!SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC)) {
    std::lock_guard lock(mutex_);
    start_error_ = std::string("SDL_Init failed: ") + SDL_GetError();
    started_ = true;
    return;
  }
  started_ = true;

  Device device;
  const auto poll_period = std::chrono::duration<double>(1.0 / options_.poll_hz);
  auto last = Clock::now();
  auto next_scan = last;
  auto next_ffb = last;

  while (!stop_) {
    SDL_UpdateJoysticks();
    const auto now = Clock::now();

    if (device.joystick && !SDL_JoystickConnected(device.joystick)) {
      close_device(device);
    }
    if (!device.joystick && now >= next_scan) {
      next_scan = now + std::chrono::milliseconds(500);
      int count = 0;
      SDL_JoystickID* ids = SDL_GetJoysticks(&count);
      // Prefer a device with a profile, then anything that looks like a wheel.
      SDL_JoystickID chosen = 0;
      for (int pass = 0; pass < 2 && chosen == 0; ++pass) {
        for (int i = 0; i < count && chosen == 0; ++i) {
          const char* name = SDL_GetJoystickNameForID(ids[i]);
          const std::string device_name = name ? name : "";
          const bool known = find_profile(options_.profiles, device_name, SDL_GetJoystickVendorForID(ids[i]),
                                          SDL_GetJoystickProductForID(ids[i])) != nullptr;
          if (pass == 0 ? known : looks_like_wheel(device_name)) {
            chosen = ids[i];
          }
        }
      }
      SDL_free(ids);
      if (chosen != 0) {
        device.joystick = SDL_OpenJoystick(chosen);
        if (device.joystick) {
          open_device(device);
        }
      }
    }

    if (device.joystick) {
      const WheelProfile& p = device.profile;
      const auto axis = [&](int index) {
        return index >= 0 && index < SDL_GetNumJoystickAxes(device.joystick)
                   ? static_cast<double>(SDL_GetJoystickAxis(device.joystick, index))
                   : 0.0;
      };
      const auto button = [&](int index) {
        return index >= 0 && index < SDL_GetNumJoystickButtons(device.joystick) &&
               SDL_GetJoystickButton(device.joystick, index);
      };
      WheelState wheel;
      wheel.connected = true;
      wheel.device = SDL_GetJoystickName(device.joystick) ? SDL_GetJoystickName(device.joystick) : "";
      wheel.steer = p.steer_angle(axis(p.steer_axis));
      wheel.throttle = p.throttle.value(axis(p.throttle.axis));
      wheel.brake = p.brake.value(axis(p.brake.axis));
      wheel.clutch = p.clutch.value(axis(p.clutch.axis));
      wheel.reset = button(p.button_reset);
      wheel.pause = button(p.button_pause);
      wheel.stamp = now;
      hub_.set_wheel(wheel);

      if (device.haptic && now >= next_ffb) {
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;
        next_ffb = now + std::chrono::duration_cast<Clock::duration>(
                             std::chrono::duration<double>(1.0 / p.ffb.update_hz));
        const ForceFeedbackRequest request = hub_.force_feedback();
        const bool fresh = std::chrono::duration<double>(now - request.stamp).count() < p.ffb.timeout;
        const double level = device.shaper->update(dt, wheel.steer, request.torque, request.lock, fresh);
        const auto command = static_cast<Sint16>(std::lround(level * 32767.0));
        if (command != device.last_level) {
          device.effect_data.constant.level = command;
          SDL_UpdateHapticEffect(device.haptic, device.effect, &device.effect_data);
          device.last_level = command;
        }
        std::lock_guard lock(mutex_);
        status_.ffb_torque = device.shaper->torque();
        status_.ffb_clipping = device.shaper->clipping();
        status_.ffb_suspended = device.shaper->suspended();
      }
    }
    std::this_thread::sleep_for(poll_period);
  }

  close_device(device);
  SDL_Quit();
}

}  // namespace vsim

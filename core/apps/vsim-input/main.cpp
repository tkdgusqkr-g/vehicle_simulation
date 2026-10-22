// vsim-input: lists steering wheels, shows live values, calibrates a profile
// (axes, pedal travel, force feedback direction) and tests force feedback.
#include <SDL3/SDL.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "vsim/input/wheel_profile.hpp"
#include "vsim/util/paths.hpp"
#include "vsim/version.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr const char* kUsage = R"(usage: vsim-input <command> [options]

commands:
  list                    list game controllers and wheels
  monitor [--device N]    show live steering, pedal and button values (Ctrl+C to stop)
  calibrate [--device N] [--out file.yaml] [--range DEG]
                          detect the steering and pedal axes and the force feedback
                          direction, then write a profile (default config/input/<name>.yaml)
  ffb-test [--device N] [--level F]
                          push the wheel left, then right, then release (F = 0..1, default 0.25)

The simulator loads every profile in config/input/ and uses the first that
matches the connected device.
)";

struct Args {
  std::string command;
  int device = -1;
  std::string out;
  std::optional<double> range;
  double level = 0.25;
};

Args parse(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument(arg + " needs a value");
      }
      return argv[++i];
    };
    if (arg == "--device") {
      args.device = std::stoi(value());
    } else if (arg == "--out") {
      args.out = value();
    } else if (arg == "--range") {
      args.range = std::stod(value());
    } else if (arg == "--level") {
      args.level = std::clamp(std::stod(value()), 0.0, 1.0);
    } else if (args.command.empty() && arg[0] != '-') {
      args.command = arg;
    } else {
      throw std::invalid_argument("unknown argument '" + arg + "'");
    }
  }
  return args;
}

std::vector<SDL_JoystickID> joysticks() {
  int count = 0;
  SDL_JoystickID* ids = SDL_GetJoysticks(&count);
  std::vector<SDL_JoystickID> out(ids, ids + count);
  SDL_free(ids);
  return out;
}

std::string name_of(SDL_JoystickID id) {
  const char* name = SDL_GetJoystickNameForID(id);
  return name ? name : "unknown";
}

// The chosen device, or the first one that looks like a wheel, or the first.
SDL_Joystick* open_device(int index) {
  // Hotplug detection may need a moment after SDL_Init.
  for (int attempt = 0; attempt < 20 && joysticks().empty(); ++attempt) {
    SDL_UpdateJoysticks();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const std::vector<SDL_JoystickID> ids = joysticks();
  if (ids.empty()) {
    throw std::runtime_error("no game controller or wheel found");
  }
  std::size_t pick = 0;
  if (index >= 0) {
    if (static_cast<std::size_t>(index) >= ids.size()) {
      throw std::runtime_error("--device " + std::to_string(index) + " does not exist (see vsim-input list)");
    }
    pick = static_cast<std::size_t>(index);
  } else {
    const vsim::WheelProfile g29 = vsim::WheelProfile::logitech_g29();
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const std::string name = name_of(ids[i]);
      if (g29.matches(name, SDL_GetJoystickVendorForID(ids[i]), SDL_GetJoystickProductForID(ids[i])) ||
          name.find("Wheel") != std::string::npos || name.find("wheel") != std::string::npos) {
        pick = i;
        break;
      }
    }
  }
  SDL_Joystick* joystick = SDL_OpenJoystick(ids[pick]);
  if (!joystick) {
    throw std::runtime_error(std::string("cannot open device: ") + SDL_GetError());
  }
  std::cout << "device: " << SDL_GetJoystickName(joystick) << "\n";
  return joystick;
}

std::vector<int> read_axes(SDL_Joystick* joystick) {
  SDL_UpdateJoysticks();
  std::vector<int> axes(static_cast<std::size_t>(std::max(0, SDL_GetNumJoystickAxes(joystick))));
  for (std::size_t i = 0; i < axes.size(); ++i) {
    axes[i] = SDL_GetJoystickAxis(joystick, static_cast<int>(i));
  }
  return axes;
}

// Samples the axes for a moment and averages them.
std::vector<double> settle(SDL_Joystick* joystick) {
  std::vector<double> sum;
  const int samples = 30;
  for (int n = 0; n < samples; ++n) {
    const std::vector<int> axes = read_axes(joystick);
    sum.resize(axes.size(), 0.0);
    for (std::size_t i = 0; i < axes.size(); ++i) {
      sum[i] += axes[i];
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  for (double& value : sum) {
    value /= samples;
  }
  return sum;
}

void wait_enter(const std::string& prompt) {
  std::cout << "\n" << prompt << "\n  then press Enter... " << std::flush;
  std::string line;
  std::getline(std::cin, line);
}

// The axis that moved most between two snapshots, skipping `exclude`.
int moved_axis(const std::vector<double>& before, const std::vector<double>& after,
               const std::vector<int>& exclude, double* delta) {
  int best = -1;
  double best_change = 0.0;
  for (std::size_t i = 0; i < std::min(before.size(), after.size()); ++i) {
    if (std::find(exclude.begin(), exclude.end(), static_cast<int>(i)) != exclude.end()) {
      continue;
    }
    const double change = after[i] - before[i];
    if (std::abs(change) > std::abs(best_change)) {
      best_change = change;
      best = static_cast<int>(i);
    }
  }
  if (delta) {
    *delta = best_change;
  }
  return std::abs(best_change) > 8000.0 ? best : -1;
}

int command_list() {
  SDL_UpdateJoysticks();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  SDL_UpdateJoysticks();
  const std::vector<SDL_JoystickID> ids = joysticks();
  if (ids.empty()) {
    std::cout << "no game controllers or wheels found\n";
    return 0;
  }
  for (std::size_t i = 0; i < ids.size(); ++i) {
    SDL_Joystick* joystick = SDL_OpenJoystick(ids[i]);
    std::printf("[%zu] %s  usb %04x:%04x", i, name_of(ids[i]).c_str(), SDL_GetJoystickVendorForID(ids[i]),
                SDL_GetJoystickProductForID(ids[i]));
    if (joystick) {
      std::printf("  axes %d, buttons %d, hats %d", SDL_GetNumJoystickAxes(joystick),
                  SDL_GetNumJoystickButtons(joystick), SDL_GetNumJoystickHats(joystick));
      SDL_Haptic* haptic = SDL_OpenHapticFromJoystick(joystick);
      if (haptic) {
        const Uint32 features = SDL_GetHapticFeatures(haptic);
        std::printf("  force feedback%s%s", features & SDL_HAPTIC_CONSTANT ? " (constant)" : "",
                    features & SDL_HAPTIC_AUTOCENTER ? " (autocenter)" : "");
        SDL_CloseHaptic(haptic);
      } else {
        std::printf("  no force feedback");
      }
      SDL_CloseJoystick(joystick);
    }
    std::printf("\n");
  }
  return 0;
}

int command_monitor(const Args& args) {
  SDL_Joystick* joystick = open_device(args.device);
  const std::filesystem::path dir = vsim::find_resource_dir("config/input");
  std::vector<vsim::WheelProfile> profiles = dir.empty() ? std::vector<vsim::WheelProfile>{}
                                                         : vsim::WheelProfile::load_directory(dir);
  profiles.push_back(vsim::WheelProfile::logitech_g29());
  const vsim::WheelProfile* profile =
      vsim::find_profile(profiles, SDL_GetJoystickName(joystick), SDL_GetJoystickVendor(joystick),
                         SDL_GetJoystickProduct(joystick));
  const vsim::WheelProfile& p = profile ? *profile : profiles.back();
  std::cout << "profile: " << p.name << "  (Ctrl+C to stop)\n";
  while (true) {
    const std::vector<int> axes = read_axes(joystick);
    const auto axis = [&](int i) { return i >= 0 && i < static_cast<int>(axes.size()) ? axes[i] : 0; };
    std::printf("\rsteer %7.1f deg  throttle %5.2f  brake %5.2f  clutch %5.2f  raw [",
                p.steer_angle(axis(p.steer_axis)) * 180.0 / std::numbers::pi, p.throttle.value(axis(p.throttle.axis)),
                p.brake.value(axis(p.brake.axis)), p.clutch.value(axis(p.clutch.axis)));
    for (std::size_t i = 0; i < axes.size(); ++i) {
      std::printf("%s%6d", i ? " " : "", axes[i]);
    }
    std::printf("]  buttons ");
    for (int b = 0; b < SDL_GetNumJoystickButtons(joystick); ++b) {
      if (SDL_GetJoystickButton(joystick, b)) {
        std::printf("%d ", b);
      }
    }
    std::printf("        ");
    std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

struct Ffb {
  SDL_Haptic* haptic = nullptr;
  SDL_HapticEffect effect{};
  SDL_HapticEffectID id = -1;

  explicit Ffb(SDL_Joystick* joystick) {
    haptic = SDL_OpenHapticFromJoystick(joystick);
    if (!haptic || !(SDL_GetHapticFeatures(haptic) & SDL_HAPTIC_CONSTANT)) {
      if (haptic) {
        SDL_CloseHaptic(haptic);
        haptic = nullptr;
      }
      return;
    }
    if (SDL_GetHapticFeatures(haptic) & SDL_HAPTIC_AUTOCENTER) {
      SDL_SetHapticAutocenter(haptic, 0);
    }
    effect.type = SDL_HAPTIC_CONSTANT;
    effect.constant.direction.type = SDL_HAPTIC_STEERING_AXIS;
    effect.constant.length = SDL_HAPTIC_INFINITY;
    id = SDL_CreateHapticEffect(haptic, &effect);
    if (id < 0 || !SDL_RunHapticEffect(haptic, id, 1)) {
      SDL_CloseHaptic(haptic);
      haptic = nullptr;
    }
  }
  ~Ffb() {
    if (haptic) {
      set(0.0);
      SDL_StopHapticEffect(haptic, id);
      SDL_DestroyHapticEffect(haptic, id);
      SDL_CloseHaptic(haptic);
    }
  }
  bool ok() const { return haptic != nullptr; }
  void set(double level) {
    effect.constant.level = static_cast<Sint16>(std::lround(std::clamp(level, -1.0, 1.0) * 32767.0));
    SDL_UpdateHapticEffect(haptic, id, &effect);
  }
};

int command_ffb_test(const Args& args) {
  SDL_Joystick* joystick = open_device(args.device);
  Ffb ffb(joystick);
  if (!ffb.ok()) {
    std::cerr << "this device has no usable constant-force effect: " << SDL_GetError() << "\n";
    return 1;
  }
  std::cout << "hold the wheel loosely\n";
  for (const double sign : {1.0, -1.0}) {
    std::cout << "level " << sign * args.level << "\n";
    ffb.set(sign * args.level);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }
  ffb.set(0.0);
  std::cout << "released\n";
  return 0;
}

int command_calibrate(const Args& args) {
  SDL_Joystick* joystick = open_device(args.device);
  vsim::WheelProfile profile = vsim::WheelProfile::logitech_g29();
  profile.name = SDL_GetJoystickName(joystick);
  profile.match_names = {profile.name};
  profile.usb_ids = {{SDL_GetJoystickVendor(joystick), SDL_GetJoystickProduct(joystick)}};
  if (profile.usb_ids.front().first == 0) {
    profile.usb_ids.clear();
  }

  wait_enter("1/5  Centre the wheel and take your feet off the pedals,");
  const std::vector<double> rest = settle(joystick);

  wait_enter("2/5  Turn the wheel about 90 degrees to the LEFT and hold it,");
  const std::vector<double> left = settle(joystick);
  double delta = 0.0;
  profile.steer_axis = moved_axis(rest, left, {}, &delta);
  if (profile.steer_axis < 0) {
    std::cerr << "no axis moved enough; is the wheel connected?\n";
    return 1;
  }
  profile.steer_center = rest[static_cast<std::size_t>(profile.steer_axis)];
  // vsim steering is positive to the left.
  profile.steer_invert = delta < 0.0;
  std::cout << "  steering: axis " << profile.steer_axis << (profile.steer_invert ? " (inverted)" : "") << "\n";
  if (args.range) {
    profile.steer_range_deg = *args.range;
  } else {
    std::cout << "  rotation range set in the wheel software (G HUB), lock to lock [900]: " << std::flush;
    std::string line;
    std::getline(std::cin, line);
    if (!line.empty()) {
      profile.steer_range_deg = std::stod(line);
    }
  }

  std::vector<int> used = {profile.steer_axis};
  const auto pedal = [&](const char* label, vsim::PedalAxis& axis, int step) {
    wait_enter(std::to_string(step) + "/5  Centre the wheel and press the " + label +
               " pedal all the way (Enter alone if you have none),");
    const std::vector<double> pressed = settle(joystick);
    axis.axis = moved_axis(rest, pressed, used, nullptr);
    if (axis.axis < 0) {
      std::cout << "  " << label << ": none\n";
      return;
    }
    used.push_back(axis.axis);
    axis.released = rest[static_cast<std::size_t>(axis.axis)];
    axis.pressed = pressed[static_cast<std::size_t>(axis.axis)];
    std::cout << "  " << label << ": axis " << axis.axis << " (" << axis.released << " -> " << axis.pressed
              << ")\n";
  };
  pedal("throttle", profile.throttle, 3);
  pedal("brake", profile.brake, 4);
  pedal("clutch", profile.clutch, 5);

  // Force feedback direction: push with positive level and see where it goes.
  Ffb ffb(joystick);
  if (!ffb.ok()) {
    std::cout << "\nno usable force feedback on this device; ffb disabled in the profile\n";
    profile.ffb_enabled = false;
  } else {
    wait_enter("FFB  Let go of the wheel completely (it will turn by itself briefly),");
    const double start = settle(joystick)[static_cast<std::size_t>(profile.steer_axis)];
    ffb.set(0.3);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const double moved = settle(joystick)[static_cast<std::size_t>(profile.steer_axis)] - start;
    ffb.set(0.0);
    if (std::abs(moved) < 500.0) {
      std::cout << "  the wheel hardly moved; keeping the default direction (check with ffb-test)\n";
    } else {
      const double angle = profile.steer_angle(start + moved) - profile.steer_angle(start);
      // Positive level should turn the wheel left (positive angle) unless inverted.
      profile.ffb.invert = angle < 0.0;
      std::cout << "  force feedback direction: " << (profile.ffb.invert ? "inverted" : "normal") << "\n";
    }
  }

  std::filesystem::path out = vsim::path_from_utf8(args.out);
  if (out.empty()) {
    std::filesystem::path dir = vsim::find_resource_dir("config/input");
    if (dir.empty()) {
      dir = "config/input";
      std::filesystem::create_directories(dir);
    }
    std::string file;
    for (const char c : profile.name) {
      file += std::isalnum(static_cast<unsigned char>(c)) ? static_cast<char>(std::tolower(c)) : '_';
    }
    out = dir / (file + ".yaml");
  }
  std::ofstream stream(out, std::ios::binary);
  stream << profile.to_yaml();
  if (!stream) {
    std::cerr << "cannot write " << vsim::path_to_utf8(out) << "\n";
    return 1;
  }
  std::cout << "\nprofile written to " << vsim::path_to_utf8(out)
            << "\nedit ffb.gain there to make the steering heavier or lighter.\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = parse(argc, argv);
    if (args.command.empty() || args.command == "help" || args.command == "--help") {
      std::cout << kUsage;
      return args.command.empty() ? 1 : 0;
    }
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    if (!SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC)) {
      std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
      return 1;
    }
    int status = 1;
    if (args.command == "list") {
      status = command_list();
    } else if (args.command == "monitor") {
      status = command_monitor(args);
    } else if (args.command == "calibrate") {
      status = command_calibrate(args);
    } else if (args.command == "ffb-test") {
      status = command_ffb_test(args);
    } else if (args.command == "version") {
      std::cout << "vsim-input " << vsim::version() << " (" << vsim::git_commit() << ")\n";
      status = 0;
    } else {
      std::cerr << "unknown command '" << args.command << "'\n\n" << kUsage;
    }
    SDL_Quit();
    return status;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    SDL_Quit();
    return 1;
  }
}

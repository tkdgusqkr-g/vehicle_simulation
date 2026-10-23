// vsim-link: looks at the shared-memory link from the viewer's side, to check
// that a running vsim-core publishes and to drive it without a renderer.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <timeapi.h>
#endif

#include "vsim/link/state_link.hpp"

namespace {

constexpr const char* kUsage = R"(usage: vsim-link <command> [options]

  monitor [--seconds <s>]     print the published state twice a second
  wait [--seconds <s>]        wait until a simulator publishes (exit 1 on timeout)
  drive <keys> [--seconds <s>]
                              hold keys (any of throttle,brake,left,right) and
                              report speed, track mesh and input latency
  reset | pause               send a command

  --name <name>               link name (default vsim_state_v2)

Start vsim-core in realtime mode first.
)";

double ms(std::int64_t ns) { return static_cast<double>(ns) / 1e6; }

void print_state(const vsim_link::VehicleState& s, std::int64_t now) {
  std::cout << std::fixed << std::setprecision(2) << "t " << s.sim_time << " s  frame " << s.frame << "  x "
            << s.x << "  y " << s.y << "  speed " << s.speed * 3.6 << " km/h  lap " << s.lap << "  track gen "
            << s.track_generation << "  car gen " << s.car_generation << "  age " << ms(now - s.publish_ns) << " ms";
  if (s.input_ns != 0) {
    std::cout << "  input " << ms(now - s.input_ns) << " ms";
  }
  std::cout << (s.paused ? "  (paused)" : "") << "\n";
}

std::uint32_t parse_keys(const std::string& text) {
  std::uint32_t keys = 0;
  std::size_t begin = 0;
  while (begin <= text.size()) {
    const std::size_t end = std::min(text.find(',', begin), text.size());
    const std::string key = text.substr(begin, end - begin);
    if (key == "throttle") {
      keys |= vsim_link::kKeyThrottle;
    } else if (key == "brake") {
      keys |= vsim_link::kKeyBrake;
    } else if (key == "left") {
      keys |= vsim_link::kKeyLeft;
    } else if (key == "right") {
      keys |= vsim_link::kKeyRight;
    } else if (!key.empty() && key != "none") {
      throw std::invalid_argument("unknown key '" + key + "'");
    }
    begin = end + 1;
  }
  return keys;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::vector<std::string> args(argv + 1, argv + argc);
    std::string name = vsim_link::kStateName;
    double seconds = -1.0;
    std::vector<std::string> positional;
    for (std::size_t i = 0; i < args.size(); ++i) {
      if ((args[i] == "--name" || args[i] == "--seconds") && i + 1 < args.size()) {
        if (args[i] == "--name") {
          name = args[++i];
        } else {
          seconds = std::stod(args[++i]);
        }
      } else if (args[i] == "--help" || args[i] == "-h") {
        std::cout << kUsage;
        return 0;
      } else {
        positional.push_back(args[i]);
      }
    }
    if (positional.empty()) {
      std::cerr << kUsage;
      return 2;
    }
    const std::string& command = positional[0];
#ifdef _WIN32
    timeBeginPeriod(1);  // 1 ms sleeps, as a renderer's input thread would have
#endif
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - start).count(); };

    vsim::StateLinkReader reader(name);
    const double wait_limit = command == "wait" && seconds > 0.0 ? seconds : 5.0;
    while (!reader.connect()) {
      if (elapsed() > wait_limit) {
        std::cerr << "no simulator publishes '" << name << "' (is vsim-core running in realtime mode?)\n";
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    vsim_link::VehicleState state{};

    if (command == "wait") {
      reader.read(state);
      std::cout << "connected: scenario '" << state.info.scenario << "', driver " << state.info.driver << "\n";
      return 0;
    }
    if (command == "reset" || command == "pause") {
      reader.send_command(command == "reset" ? vsim_link::kCommandReset : vsim_link::kCommandTogglePause);
      return 0;
    }
    if (command == "monitor") {
      while (seconds < 0.0 || elapsed() < seconds) {
        if (reader.read(state)) {
          print_state(state, vsim_link::now_ns());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!reader.connect()) {
          std::cout << "simulator gone\n";
          return 1;
        }
      }
      return 0;
    }
    if (command == "drive") {
      const std::uint32_t keys = parse_keys(positional.size() > 1 ? positional[1] : "throttle");
      const double duration = seconds > 0.0 ? seconds : 2.0;
      // Keys at 1 kHz like a steering wheel's polling; the age of the newest
      // input inside each new state is the simulator's share of the latency.
      double sum = 0.0, worst = 0.0;
      int samples = 0;
      std::uint64_t last_frame = 0;
      const auto drive_start = Clock::now();
      while (std::chrono::duration<double>(Clock::now() - drive_start).count() < duration) {
        reader.send_keys(keys);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (reader.read(state) && state.input_ns != 0 && state.frame != last_frame) {
          last_frame = state.frame;
          const double latency = ms(state.publish_ns - state.input_ns);
          sum += latency;
          worst = std::max(worst, latency);
          ++samples;
        }
      }
      reader.send_keys(0);
      reader.read(state);
      print_state(state, vsim_link::now_ns());
      if (const vsim_link::MeshRegion* track = reader.track()) {
        std::cout << "track '" << track->name << "': " << track->section_count << " sections, " << track->size
                  << " bytes\n";
      } else {
        std::cout << "track: none\n";
      }
      if (const vsim_link::MeshRegion* car = reader.car()) {
        std::cout << "car '" << car->name << "': " << car->part_count << " parts, " << car->section_count
                  << " sections, " << car->size << " bytes\n";
      } else {
        std::cout << "car: none\n";
      }
      if (samples > 0) {
        std::cout << std::setprecision(1) << "input -> state: mean " << sum / samples << " ms, max " << worst
                  << " ms (" << samples << " states; a renderer adds its frame time)\n";
      }
      std::cout << "speed_mps " << std::setprecision(3) << state.speed << "\n";
      return 0;
    }
    std::cerr << "unknown command '" << command << "'\n" << kUsage;
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  }
}

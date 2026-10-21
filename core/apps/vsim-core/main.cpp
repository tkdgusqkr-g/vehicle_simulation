// vsim-core: runs a scenario in real time (with the live view) or headless.
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
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
#include <shellapi.h>
#include <timeapi.h>
#endif

#include "vsim/api/api_server.hpp"
#include "vsim/models/vehicle/single_track.hpp"
#include "vsim/sim/simulation.hpp"
#include "vsim/util/paths.hpp"
#include "vsim/version.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using nlohmann::json;

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop = true; }

struct Options {
  std::string scenario;
  std::string mode;  // realtime | fast; empty = scenario's time.mode
  std::string host = "127.0.0.1";
  int port = 8765;
  std::string log_dir;
  std::optional<double> duration;
  vsim::Scenario::Overrides overrides;
};

constexpr const char* kUsage = R"(usage: vsim-core --scenario <file.yaml> [options]

  --scenario <file>    scenario YAML (required)
  --mode <m>           realtime (default) or fast; overrides time.mode
  --duration <s>       stop after this many simulated seconds
  --log-dir <dir>      write run outputs (MCAP log, metadata) under <dir>
  --set <key>=<value>  override a scenario parameter (repeatable)
  --port <n>           Sim API WebSocket port (default 8765)
  --host <addr>        bind address (default 127.0.0.1; 0.0.0.0 for other machines)
  --version            print the version and exit

In realtime mode open ui/live/index.html in a browser to watch and drive.
)";

// Command-line arguments as UTF-8, also on Windows where argv uses the ANSI
// code page and would mangle non-ASCII paths.
std::vector<std::string> utf8_args(int argc, char** argv) {
#ifdef _WIN32
  (void)argc;
  (void)argv;
  int count = 0;
  LPWSTR* wide = CommandLineToArgvW(GetCommandLineW(), &count);
  std::vector<std::string> args;
  for (int i = 0; i < count; ++i) {
    const int size = WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, nullptr, 0, nullptr, nullptr);
    std::string arg(static_cast<std::size_t>(size > 0 ? size - 1 : 0), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, arg.data(), size, nullptr, nullptr);
    args.push_back(std::move(arg));
  }
  LocalFree(wide);
  return args;
#else
  return {argv, argv + argc};
#endif
}

std::optional<Options> parse_args(const std::vector<std::string>& args) {
  Options options;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string& arg = args[i];
    const auto value = [&]() -> const std::string& {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument(arg + " needs a value");
      }
      return args[++i];
    };
    if (arg == "--scenario") {
      options.scenario = value();
    } else if (arg == "--mode") {
      options.mode = value();
    } else if (arg == "--duration") {
      options.duration = std::stod(value());
    } else if (arg == "--log-dir") {
      options.log_dir = value();
    } else if (arg == "--port") {
      options.port = std::stoi(value());
    } else if (arg == "--host") {
      options.host = value();
    } else if (arg == "--set") {
      const std::string& assignment = value();
      const auto eq = assignment.find('=');
      if (eq == std::string::npos || eq == 0) {
        throw std::invalid_argument("--set expects key=value, got '" + assignment + "'");
      }
      options.overrides[assignment.substr(0, eq)] = assignment.substr(eq + 1);
    } else if (arg == "--version") {
      std::cout << "vsim-core " << vsim::version() << " (" << vsim::git_commit() << ")\n";
      return std::nullopt;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << kUsage;
      return std::nullopt;
    } else {
      throw std::invalid_argument("unknown argument '" + arg + "'");
    }
  }
  if (options.scenario.empty()) {
    throw std::invalid_argument("--scenario is required");
  }
  if (!options.mode.empty() && options.mode != "realtime" && options.mode != "fast") {
    throw std::invalid_argument("--mode must be realtime or fast");
  }
  return options;
}

// Signals streamed to the live view, with their short names in the message.
struct StateStream {
  struct Field {
    const char* key;
    const char* signal;
    std::optional<vsim::SignalId> id;
  };

  std::vector<Field> fields = {
      {"x", "veh.chassis.x", {}},
      {"y", "veh.chassis.y", {}},
      {"yaw", "veh.chassis.yaw", {}},
      {"vx", "veh.chassis.vx", {}},
      {"vy", "veh.chassis.vy", {}},
      {"yaw_rate", "veh.chassis.yaw_rate", {}},
      {"speed", "veh.chassis.speed", {}},
      {"ax", "veh.chassis.ax", {}},
      {"ay", "veh.chassis.ay", {}},
      {"beta", "veh.chassis.beta", {}},
      {"delta", "veh.steer.road_wheel_angle", {}},
      {"steer", "drv.cmd.steer", {}},
      {"throttle", "drv.cmd.throttle", {}},
      {"brake", "drv.cmd.brake", {}},
      {"power", "pt.power_mech", {}},
  };

  void resolve(const vsim::SignalBus& bus) {
    for (Field& field : fields) {
      field.id = bus.find(field.signal);
    }
  }

  std::string message(const vsim::Simulation& sim, bool paused, double rtf) const {
    json params;
    params["t"] = sim.time();
    params["paused"] = paused;
    params["rtf"] = rtf;
    for (const Field& field : fields) {
      if (field.id) {
        params[field.key] = sim.bus().get(*field.id);
      }
    }
    return json{{"jsonrpc", "2.0"}, {"method", "state"}, {"params", params}}.dump();
  }
};

std::string hello_message(const vsim::Simulation& sim) {
  const vsim::ParamTree& params = sim.scenario().params;
  const auto p = vsim::SingleTrackModel::params_from(params);
  json vehicle;
  vehicle["wheelbase"] = p.wheelbase;
  vehicle["cg_to_front"] = p.cg_to_front;
  vehicle["track_width"] = params.get_double("vehicle.track_width_m", 1.2);
  vehicle["steering_ratio"] = p.steering_ratio;
  vehicle["max_steer_wheel"] = p.max_steer_wheel;
  vehicle["tire_radius"] = p.tire_radius;
  json hello;
  hello["vsim_version"] = vsim::version();
  hello["scenario"] = sim.scenario().name;
  hello["driver"] = params.get_string("driver.type", "external");
  hello["dt"] = sim.dt();
  hello["vehicle"] = vehicle;
  return json{{"jsonrpc", "2.0"}, {"method", "hello"}, {"params", hello}}.dump();
}

void print_summary(const vsim::Simulation& sim, double wall_seconds) {
  const auto speed_id = sim.bus().find("veh.chassis.speed");
  std::cout << "sim time   " << sim.time() << " s\n";
  std::cout << "wall time  " << wall_seconds << " s";
  if (wall_seconds > 0.0) {
    std::cout << " (" << sim.time() / wall_seconds << "x real time)";
  }
  std::cout << "\n";
  if (speed_id) {
    std::cout << "speed      " << sim.bus().get(*speed_id) << " m/s at end\n";
  }
  if (!sim.run_path().empty()) {
    std::cout << "run output " << vsim::path_to_utf8(sim.run_path()) << "\n";
  }
}

int run_fast(vsim::Simulation& sim, const Options& options) {
  if (sim.scenario().params.get_string("driver.type", "external") == "keyboard") {
    std::cerr << "note: keyboard driver in fast mode has no input; the car will not move\n";
  }
  const auto start = Clock::now();
  if (options.duration) {
    sim.run_for(*options.duration);
  } else {
    sim.run_to_end();
  }
  sim.finish();
  print_summary(sim, std::chrono::duration<double>(Clock::now() - start).count());
  return 0;
}

int run_realtime(vsim::Simulation& sim, const Options& options) {
  vsim::ApiServer server(options.port, options.host);
  StateStream stream;
  stream.resolve(sim.bus());
  server.set_hello(hello_message(sim));
  if (const auto error = server.start()) {
    std::cerr << "error: cannot listen on " << options.host << ":" << options.port << ": "
              << *error << "\n";
    return 1;
  }
  std::cout << "Sim API on ws://" << options.host << ":" << options.port
            << "  - open ui/live/index.html to watch and drive, Ctrl+C to stop\n";

  // Fixed-step accumulator: the sim catches up with wall time in whole steps.
  // If it falls more than max_lag behind, the excess is dropped (slow motion)
  // instead of spiralling.
  constexpr double kMaxLag = 0.1;
  constexpr auto kPublishPeriod = std::chrono::microseconds(16'667);
  constexpr auto kKeyTimeout = std::chrono::milliseconds(500);

  const double end_time = options.duration ? sim.time() + *options.duration : sim.end_time();
  bool paused = false;
  double lag = 0.0;
  std::int64_t overruns = 0;
  double rtf = 0.0;
  auto last = Clock::now();
  auto last_keys = last;
  auto next_publish = last;
  auto rtf_window = last;
  double rtf_sim_start = sim.time();
  const auto start = last;

  while (!g_stop && sim.time() < end_time) {
    for (const std::string& text : server.take_messages()) {
      json message = json::parse(text, nullptr, false);
      if (message.is_discarded() || !message.contains("method")) {
        continue;
      }
      const std::string method = message.value("method", "");
      const json params = message.value("params", json::object());
      if (method == "input.keys") {
        vsim::KeyState keys;
        keys.throttle = params.value("throttle", false);
        keys.brake = params.value("brake", false);
        keys.left = params.value("left", false);
        keys.right = params.value("right", false);
        sim.inputs().set_keys(keys);
        last_keys = Clock::now();
      } else if (method == "sim.reset") {
        sim.reset();
        stream.resolve(sim.bus());
        rtf_sim_start = sim.time();
        lag = 0.0;
      } else if (method == "sim.pause") {
        paused = true;
      } else if (method == "sim.resume") {
        paused = false;
      } else if (method == "sim.toggle_pause") {
        paused = !paused;
      }
    }

    const auto now = Clock::now();
    if (now - last_keys > kKeyTimeout) {
      // A browser that lost focus or disconnected must not leave a pedal pressed.
      sim.inputs().set_keys({});
    }
    const double elapsed = std::chrono::duration<double>(now - last).count();
    last = now;
    if (!paused) {
      lag += elapsed;
      if (lag > kMaxLag) {
        lag = kMaxLag;
        ++overruns;
      }
      const auto steps = static_cast<std::int64_t>(lag / sim.dt());
      if (steps > 0) {
        sim.step(steps);
        lag -= static_cast<double>(steps) * sim.dt();
      }
    }

    if (now - rtf_window >= std::chrono::seconds(1)) {
      rtf = (sim.time() - rtf_sim_start) / std::chrono::duration<double>(now - rtf_window).count();
      rtf_window = now;
      rtf_sim_start = sim.time();
    }
    if (now >= next_publish) {
      server.broadcast(stream.message(sim, paused, rtf));
      next_publish = now + kPublishPeriod;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  server.stop();
  sim.finish();
  print_summary(sim, std::chrono::duration<double>(Clock::now() - start).count());
  if (overruns > 0) {
    std::cout << "overruns   " << overruns << " (sim fell behind real time)\n";
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_args(utf8_args(argc, argv));
    if (!options) {
      return 0;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
#ifdef _WIN32
    // 1 ms timer resolution so the real-time loop can sleep in small steps.
    timeBeginPeriod(1);
#endif

    vsim::SimulationOptions sim_options;
    sim_options.log_dir = vsim::path_from_utf8(options->log_dir);
    sim_options.record = false;
    vsim::Simulation sim(vsim::Scenario::load(vsim::path_from_utf8(options->scenario),
                                              options->overrides),
                         sim_options);

    const std::string mode = options->mode.empty()
                                 ? sim.scenario().params.get_string("time.mode", "realtime")
                                 : options->mode;
    const int status = mode == "fast" ? run_fast(sim, *options) : run_realtime(sim, *options);
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    return status;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    if (argc <= 1) {
      std::cerr << "\n" << kUsage;
    }
    return 1;
  }
}

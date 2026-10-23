// vsim-core: runs a scenario in real time (with the live view) or headless.
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <functional>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <numbers>
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
#ifdef VSIM_HAS_SDL
#include "vsim/input/input_service.hpp"
#endif
#include "vsim/link/state_link.hpp"
#include "vsim/models/vehicle/suspension.hpp"
#include "vsim/sim/simulation.hpp"
#include "vsim/track/track.hpp"
#include "vsim/track/track_mesh.hpp"
#include "vsim/visual/gltf.hpp"
#include "vsim/visual/vehicle_visual.hpp"
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
  std::string input_profiles;
  std::string export_track;
  std::string export_car;
  std::string export_kc;
  std::string link_name = vsim_link::kStateName;
  bool no_wheel = false;
  bool virtual_wheel = false;
  bool no_link = false;
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
  --no-wheel           do not look for a steering wheel (driver.type dil uses the keyboard)
  --no-link            do not publish to shared memory (the Unreal viewer reads it)
  --link-name <name>   shared-memory name (default vsim_state_v2)
  --input-profiles <d> wheel profile directory (default: config/input in the source tree)
  --virtual-wheel      add a virtual steering wheel driven by input.virtual (UI tests)
  --export-track-glb <file>  write the scenario's track as a 3D model (glTF binary) and exit
  --export-car-glb <file>    write the car's 3D model (glTF binary) and exit
  --export-kc <file.csv>     write the suspension K&C curves solved from the hardpoints and exit
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

// Suspension K&C curves (design doc §5.5) as CSV: per axle over wheel travel
// at the rack's centre, and the front over the rack at design height; the
// static figures go to the console.
void write_kc(const vsim::ParamTree& params, const std::string& file) {
  std::ofstream out(vsim::path_from_utf8(file));
  if (!out) {
    throw std::runtime_error("cannot write " + file);
  }
  constexpr double kDeg = std::numbers::pi / 180.0;
  out << "axle,travel_mm,rack_mm,steer_deg,camber_deg,contact_x_mm,contact_y_mm,jacking_x,jacking_y,"
         "spring_mm,motion_ratio,kingpin_arm_x_mm,kingpin_arm_y_mm,kingpin_arm_z_mm\n";
  const double wheelbase = params.get_double("vehicle.wheelbase_m", 1.53);
  for (const bool front : {true, false}) {
    const vsim::WishboneSolver solver(vsim::WishboneGeometry::from_params(params, front));
    const double half = 0.5 * params.get_double(front ? "vehicle.track_front_m" : "vehicle.track_rear_m", 1.2);
    const vsim::KcSummary k = vsim::summarize(solver, half, wheelbase, front);
    std::printf("%s: kingpin %.2f deg, caster %.2f deg, scrub %.1f mm, trail %.1f mm, roll centre %.1f mm, "
                "camber gain %.2f deg/10 mm, bump steer %.3f deg/10 mm, motion ratio %.3f%s\n",
                front ? "front" : "rear", k.kingpin_inclination_deg, k.caster_deg, 1000.0 * k.scrub_radius_m,
                1000.0 * k.mechanical_trail_m, 1000.0 * k.roll_centre_height_m, k.camber_gain_deg_per_m / 100.0,
                k.bump_steer_deg_per_m / 100.0, k.motion_ratio,
                front ? (", Ackermann " + std::to_string(static_cast<int>(std::lround(k.ackermann_percent))) + " %").c_str()
                      : "");
    double rack_max = 0.0;
    if (front) {
      constexpr double h = 1e-4;
      const double slope = (solver.solve(0.0, h).steer - solver.solve(0.0, -h).steer) / (2.0 * h);
      rack_max = params.get_double("vehicle.steering.lock_deg", 110.0) * kDeg /
                 (params.get_double("vehicle.steering.ratio", 4.5) * slope);
    }
    const vsim::KcTable table = vsim::KcTable::build(solver, 0.05, 41, rack_max, front ? 21 : 1);
    const auto row = [&](double travel, double rack) {
      const vsim::KcPoint p = table.at(travel, rack);
      char line[512];
      std::snprintf(line, sizeof line, "%s,%.1f,%.2f,%.4f,%.4f,%.3f,%.3f,%.5f,%.5f,%.3f,%.4f,%.2f,%.2f,%.2f\n",
                    front ? "front" : "rear", 1000.0 * travel, 1000.0 * rack, p.steer / kDeg, p.camber / kDeg,
                    1000.0 * p.cp_x, 1000.0 * p.cp_y, p.dcp_x, p.dcp_y, 1000.0 * p.spring, 1.0 / p.dspring,
                    1000.0 * p.arm_x, 1000.0 * p.arm_y, 1000.0 * p.arm_z);
      out << line;
    };
    for (int i = -20; i <= 20; ++i) {
      row(0.0025 * i, 0.0);
    }
    if (front) {
      for (int i = -10; i <= 10; ++i) {
        if (i != 0) {
          row(0.0, rack_max * i / 10.0);
        }
      }
    }
  }
  std::cout << "K&C written to " << file << "\n";
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
    } else if (arg == "--no-wheel") {
      options.no_wheel = true;
    } else if (arg == "--virtual-wheel") {
      options.virtual_wheel = true;
    } else if (arg == "--no-link") {
      options.no_link = true;
    } else if (arg == "--link-name") {
      options.link_name = value();
    } else if (arg == "--input-profiles") {
      options.input_profiles = value();
    } else if (arg == "--export-track-glb") {
      options.export_track = value();
    } else if (arg == "--export-kc") {
      options.export_kc = value();
    } else if (arg == "--export-car-glb") {
      options.export_car = value();
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
      {"power_elec", "bat.power", {}},
      {"soc", "bat.soc", {}},
      {"bat_v", "bat.voltage", {}},
      {"bat_i", "bat.current", {}},
      {"bat_temp", "bat.temp", {}},
      {"regen", "vcu.regen", {}},
      {"ffb", "veh.ffb.torque_sw", {}},
      {"wheel", "drv.input.wheel", {}},
      {"target_speed", "drv.target_speed", {}},
      {"track_s", "track.s", {}},
      {"track_n", "track.n", {}},
      {"lap", "track.lap", {}},
      {"lap_time", "track.lap_time", {}},
      {"last_lap", "track.last_lap", {}},
      {"best_lap", "track.best_lap", {}},
      {"last_lap_valid", "track.last_lap_valid", {}},
      {"lap_valid", "track.lap_valid", {}},
      {"sector", "track.sector", {}},
      {"last_sector", "track.last_sector", {}},
      {"off_track", "track.off_track", {}},
      {"finished", "track.finished", {}},
      {"roll", "veh.chassis.roll", {}},
      {"pitch", "veh.chassis.pitch", {}},
      {"heave", "veh.chassis.heave", {}},
      {"road_z", "veh.chassis.road_z", {}},
  };
  std::vector<vsim::SignalId> motor_temps;
  // Per wheel (FL, FR, RL, RR) arrays for the 3D view; empty if the model has none.
  struct WheelArray {
    const char* key;
    const char* field;
    std::vector<vsim::SignalId> ids;
  };
  std::vector<WheelArray> wheel_arrays = {{"wheel_steer", "steer", {}},
                                          {"wheel_omega", "omega", {}},
                                          {"wheel_slip_ratio", "slip_ratio", {}},
                                          {"wheel_slip_angle", "slip_angle", {}},
                                          {"wheel_load", "fz", {}},
                                          {"wheel_hub", "hub_offset", {}},
                                          {"wheel_camber", "camber", {}},
                                          {"wheel_surface", "surface", {}}};

  void resolve(const vsim::SignalBus& bus) {
    for (Field& field : fields) {
      field.id = bus.find(field.signal);
    }
    motor_temps = bus.match("pt.motor.*.temp");
    for (WheelArray& array : wheel_arrays) {
      array.ids.clear();
      for (const char* corner : {"fl", "fr", "rl", "rr"}) {
        if (const auto id = bus.find(std::string("veh.wheel.") + corner + "." + array.field)) {
          array.ids.push_back(*id);
        }
      }
      if (array.ids.size() != 4) {
        array.ids.clear();
      }
    }
  }

  std::string message(const vsim::Simulation& sim, bool paused, double rtf, const json& extra) const {
    json params = extra;
    params["t"] = sim.time();
    params["paused"] = paused;
    params["rtf"] = rtf;
    for (const Field& field : fields) {
      if (field.id) {
        params[field.key] = sim.bus().get(*field.id);
      }
    }
    if (!motor_temps.empty()) {
      double hottest = sim.bus().get(motor_temps.front());
      for (const vsim::SignalId id : motor_temps) {
        hottest = std::max(hottest, sim.bus().get(id));
      }
      params["motor_temp"] = hottest;
    }
    for (const WheelArray& array : wheel_arrays) {
      if (!array.ids.empty()) {
        json values = json::array();
        for (const vsim::SignalId id : array.ids) {
          values.push_back(sim.bus().get(id));
        }
        params[array.key] = values;
      }
    }
    return json{{"jsonrpc", "2.0"}, {"method", "state"}, {"params", params}}.dump();
  }
};

// Track outline for drawing: centerline, boundaries and lines every ~1 m,
// start/finish and sector lines.
json track_geometry(const vsim::Track& track) {
  const auto every = static_cast<std::size_t>(std::max(1.0, std::round(1.0 / vsim::Track::kStep)));
  const auto round_cm = [](double v) { return std::round(v * 100.0) / 100.0; };
  json center = json::array(), left = json::array(), right = json::array(), elevation = json::array();
  for (std::size_t i = 0; i < track.samples().size(); i += every) {
    const vsim::CurveSample& c = track.samples()[i];
    const vsim::Track::Widths w = track.widths_at(c.s);
    const double nx = -std::sin(c.heading);
    const double ny = std::cos(c.heading);
    center.push_back({round_cm(c.x), round_cm(c.y)});
    elevation.push_back(round_cm(track.elevation(c.s)));
    left.push_back({round_cm(c.x + w.left * nx), round_cm(c.y + w.left * ny)});
    right.push_back({round_cm(c.x - w.right * nx), round_cm(c.y - w.right * ny)});
  }
  json lines = json::object();
  for (const auto& definition : track.definition().lines) {
    const vsim::SampledCurve line = track.line(definition.name);
    json points = json::array();
    for (std::size_t i = 0; i < line.samples().size(); i += every) {
      points.push_back({round_cm(line.samples()[i].x), round_cm(line.samples()[i].y)});
    }
    lines[definition.name] = points;
  }
  const auto marker = [&](double s) {
    const vsim::CurveSample c = track.at(s);
    const vsim::Track::Widths w = track.widths_at(s);
    return json{{"x", c.x}, {"y", c.y}, {"heading", c.heading}, {"left", w.left}, {"right", w.right}};
  };
  json sectors = json::array();
  for (const double s : track.sector_s()) {
    sectors.push_back(marker(s));
  }
  return {{"name", track.name()},   {"closed", track.closed()},     {"length", track.length()},
          {"center", center},       {"left", left},                 {"right", right},
          {"lines", lines},         {"start", marker(track.start_s())}, {"finish", marker(track.finish_s())},
          {"sectors", sectors},     {"elevation", elevation}};
}

std::string hello_message(const vsim::Simulation& sim, std::uint64_t track_generation,
                          std::uint64_t car_generation) {
  const vsim::ParamTree& params = sim.scenario().params;
  json vehicle;
  vehicle["model"] = params.get_string("vehicle.model", "two_track");
  vehicle["wheelbase"] = params.get_double("vehicle.wheelbase_m");
  vehicle["cg_to_front"] = params.get_double("vehicle.cg_to_front_axle_m");
  vehicle["track_front"] = params.get_double("vehicle.track_front_m", 1.2);
  vehicle["track_rear"] = params.get_double("vehicle.track_rear_m", 1.2);
  vehicle["steering_ratio"] = params.get_double("vehicle.steering.ratio");
  vehicle["max_steer_wheel"] = params.get_double("vehicle.steering.lock_deg") * std::numbers::pi / 180.0;
  vehicle["tire_radius"] = params.get_double("vehicle.wheel.radius_m");
  vehicle["tire_width"] = params.get_double("vehicle.tire.width_m", 0.2);
  vehicle["cg_height"] = params.get_double("vehicle.cg_height_m", 0.3);
  json hello;
  hello["vsim_version"] = vsim::version();
  hello["scenario"] = sim.scenario().name;
  hello["driver"] = params.get_string("driver.type", "external");
  hello["dt"] = sim.dt();
  hello["vehicle"] = vehicle;
  const std::shared_ptr<const vsim::Track> track = sim.track();
  hello["track"] = track ? track_geometry(*track) : json(nullptr);
  hello["track_generation"] = track_generation;  // track.mesh fetches the 3D model for it
  hello["car_generation"] = car_generation;      // vehicle.model fetches the car
  return json{{"jsonrpc", "2.0"}, {"method", "hello"}, {"params", hello}}.dump();
}

std::string base64(const std::string& data) {
  static constexpr char kDigits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((data.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < data.size(); i += 3) {
    std::uint32_t chunk = static_cast<std::uint8_t>(data[i]) << 16;
    if (i + 1 < data.size()) {
      chunk |= static_cast<std::uint32_t>(static_cast<std::uint8_t>(data[i + 1])) << 8;
    }
    if (i + 2 < data.size()) {
      chunk |= static_cast<std::uint8_t>(data[i + 2]);
    }
    out += kDigits[(chunk >> 18) & 63];
    out += kDigits[(chunk >> 12) & 63];
    out += i + 1 < data.size() ? kDigits[(chunk >> 6) & 63] : '=';
    out += i + 2 < data.size() ? kDigits[chunk & 63] : '=';
  }
  return out;
}

// The track's 3D scenery as a glTF binary for the 3D views (null without a track).
std::string track_mesh_message(const vsim::Simulation& sim, std::uint64_t generation) {
  const std::shared_ptr<const vsim::Track> track = sim.track();
  json params = {{"generation", generation}, {"glb", nullptr}};
  if (track) {
    params["name"] = track->name();
    params["glb"] = base64(vsim::to_glb(vsim::make_track_mesh(*track)));
  }
  return json{{"jsonrpc", "2.0"}, {"method", "track.mesh"}, {"params", params}}.dump();
}

std::int64_t to_ns(Clock::time_point t) {
  return t == Clock::time_point{} ? 0 : std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

// The car's 3D model as a glTF binary, parts tagged with their roles.
std::string car_model_message(const vsim::VisualModel& car, std::uint64_t generation) {
  return json{{"jsonrpc", "2.0"},
              {"method", "vehicle.model"},
              {"params", {{"generation", generation}, {"name", car.name}, {"glb", base64(vsim::to_glb(car))}}}}
      .dump();
}

std::string base64_decode(const std::string& text) {
  static const auto table = [] {
    std::array<int, 256> t{};
    t.fill(-1);
    const char* digits = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (int i = 0; i < 64; ++i) {
      t[static_cast<unsigned char>(digits[i])] = i;
    }
    return t;
  }();
  std::string out;
  out.reserve(text.size() / 4 * 3);
  std::uint32_t buffer = 0;
  int bits = 0;
  for (const unsigned char c : text) {
    if (c == '=') {
      break;
    }
    const int v = table[c];
    if (v < 0) {
      continue;  // whitespace
    }
    buffer = (buffer << 6) | static_cast<std::uint32_t>(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<char>((buffer >> bits) & 0xff));
    }
  }
  return out;
}

// Parameters under `prefix` as nested JSON (numeric keys become arrays).
json params_json(const vsim::ParamTree& tree, const std::string& prefix) {
  json root = json::object();
  for (const auto& [key, value] : tree.entries()) {
    if (key.rfind(prefix + ".", 0) != 0) {
      continue;
    }
    json* node = &root;
    std::size_t start = prefix.size() + 1;
    while (true) {
      const std::size_t dot = key.find('.', start);
      const std::string part = key.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
      if (dot == std::string::npos) {
        const json parsed = json::parse(value, nullptr, false);
        (*node)[part] = !parsed.is_discarded() && (parsed.is_number() || parsed.is_boolean()) ? parsed : json(value);
        break;
      }
      node = &(*node)[part];
      start = dot + 1;
    }
  }
  // {"0": a, "1": b} -> [a, b]
  std::function<void(json&)> listify = [&](json& j) {
    if (!j.is_object()) {
      return;
    }
    for (auto& [k, v] : j.items()) {
      listify(v);
    }
    bool numeric = !j.empty();
    for (std::size_t i = 0; numeric && i < j.size(); ++i) {
      numeric = j.contains(std::to_string(i));
    }
    if (numeric) {
      json list = json::array();
      for (std::size_t i = 0; i < j.size(); ++i) {
        list.push_back(j[std::to_string(i)]);
      }
      j = list;
    }
  };
  listify(root);
  return root;
}

// Writes files from the vehicle builder into the project: only below
// vehicle/, with known extensions. Returns the written paths.
std::vector<std::string> import_vehicle_files(const std::filesystem::path& project, const json& files) {
  std::vector<std::string> written;
  for (const json& f : files) {
    const std::string rel = f.at("path");
    const std::filesystem::path path = vsim::path_from_utf8(rel);
    const std::string ext = vsim::path_to_utf8(path.extension());
    const bool allowed_ext = ext == ".glb" || ext == ".yaml" || ext == ".step" || ext == ".stp" || ext == ".stl";
    bool inside = rel.rfind("vehicle/", 0) == 0 && path.is_relative();
    for (const auto& segment : path) {
      inside = inside && segment != "..";
    }
    if (!inside || !allowed_ext) {
      throw std::invalid_argument("refusing to write '" + rel + "' (only vehicle/*.glb|yaml|step|stl)");
    }
    const std::filesystem::path target = project / path;
    std::filesystem::create_directories(target.parent_path());
    std::ofstream out(target, std::ios::binary);
    out << (f.contains("base64") ? base64_decode(f["base64"].get<std::string>()) : f.value("text", std::string()));
    if (!out) {
      throw std::runtime_error("cannot write " + vsim::path_to_utf8(target));
    }
    written.push_back(vsim::path_to_utf8(target));
  }
  return written;
}

std::string notice(const std::string& level, const std::string& text) {
  return json{{"jsonrpc", "2.0"}, {"method", "notice"}, {"params", {{"level", level}, {"text", text}}}}.dump();
}

#ifdef VSIM_HAS_SDL
// Wheel profile as JSON with the same keys as the YAML file, so the
// calibration page can edit it and send it back (JSON is YAML).
json profile_json(const vsim::WheelProfile& p) {
  const auto pedal = [](const vsim::PedalAxis& a) {
    return json{{"axis", a.axis}, {"released", a.released}, {"pressed", a.pressed}, {"deadzone", a.deadzone},
                {"gamma", a.gamma}};
  };
  json usb_ids = json::array();
  for (const auto& [vendor, product] : p.usb_ids) {
    char id[16];
    std::snprintf(id, sizeof id, "%04x:%04x", vendor, product);
    usb_ids.push_back(id);
  }
  return {{"name", p.name},
          {"priority", p.priority},
          {"match", {{"names", p.match_names}, {"usb_ids", usb_ids}}},
          {"steer", {{"axis", p.steer_axis}, {"range_deg", p.steer_range_deg}, {"center", p.steer_center},
                     {"invert", p.steer_invert}}},
          {"pedals", {{"throttle", pedal(p.throttle)}, {"brake", pedal(p.brake)}, {"clutch", pedal(p.clutch)}}},
          {"buttons", {{"reset", p.button_reset}, {"pause", p.button_pause}}},
          {"ffb", {{"enabled", p.ffb_enabled},
                   {"invert", p.ffb.invert},
                   {"gain", p.ffb.gain},
                   {"max_torque_nm", p.ffb.max_torque},
                   {"damping_nm_s_per_rad", p.ffb.damping},
                   {"friction_nm", p.ffb.friction},
                   {"end_stop_nm_per_rad", p.ffb.end_stop},
                   {"timeout_s", p.ffb.timeout},
                   {"fade_s", p.ffb.fade_time},
                   {"update_hz", p.ffb.update_hz}}}};
}

std::string profile_file_name(const std::string& name) {
  std::string file;
  for (const unsigned char c : name) {
    if (std::isalnum(c)) {
      file += static_cast<char>(std::tolower(c));
    } else if (!file.empty() && file.back() != '_') {
      file += '_';
    }
  }
  while (!file.empty() && file.back() == '_') {
    file.pop_back();
  }
  return (file.empty() ? std::string("wheel") : file) + ".yaml";
}
#endif

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

// run_realtime asks main to load the scenario again (the vehicle changed).
constexpr int kReload = 100;

int run_realtime(vsim::Simulation& sim, const Options& options) {
  vsim::ApiServer server(options.port, options.host);
  StateStream stream;
  stream.resolve(sim.bus());
  std::uint64_t track_generation = 1;
  const std::uint64_t car_generation = 1;
  vsim::VisualModel car_model;
  try {
    car_model = vsim::make_vehicle_visual(sim.scenario().params);
  } catch (const std::exception& error) {
    std::cerr << "warning: vehicle 3D model: " << error.what() << "; showing the procedural car\n";
    car_model = vsim::make_formula_car(sim.scenario().params);
  }
  server.set_hello(hello_message(sim, track_generation, car_generation));
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

#ifdef VSIM_HAS_SDL
  // Steering wheel and force feedback for the driver-in-the-loop driver.
  std::unique_ptr<vsim::InputService> wheel_service;
  std::filesystem::path profile_dir = options.input_profiles.empty() ? vsim::find_resource_dir("config/input")
                                                                     : vsim::path_from_utf8(options.input_profiles);
  if (!options.no_wheel && sim.scenario().params.get_string("driver.type", "external") == "dil") {
    vsim::InputService::Options input;
    const std::filesystem::path& dir = profile_dir;
    if (!dir.empty()) {
      input.profiles = vsim::WheelProfile::load_directory(dir);
    }
    input.virtual_wheel = options.virtual_wheel;
    wheel_service = std::make_unique<vsim::InputService>(sim.inputs(), std::move(input));
    if (const auto error = wheel_service->start()) {
      std::cerr << "warning: steering wheel input unavailable: " << *error << "\n";
      wheel_service.reset();
    } else {
      std::cout << "Steering wheel: waiting for a device (profiles: "
                << (dir.empty() ? std::string("built-in") : vsim::path_to_utf8(dir))
                << "); the keyboard works meanwhile\n";
    }
  }
  vsim::InputService::Status wheel_status;
  if (profile_dir.empty()) {
    profile_dir = std::filesystem::current_path() / "config" / "input";  // where a saved profile goes
  }
  const auto send_profile = [&] {
    json message = {{"available", wheel_service != nullptr}, {"dir", vsim::path_to_utf8(profile_dir)}};
    if (wheel_service) {
      const vsim::InputService::Status status = wheel_service->status();
      message["profile"] = profile_json(wheel_service->profile());
      message["status"] = {{"connected", status.connected},       {"device", status.device},
                           {"profile", status.profile},           {"force_feedback", status.force_feedback},
                           {"ffb_error", status.ffb_error},       {"calibrated", status.calibrated}};
    }
    server.broadcast(json{{"jsonrpc", "2.0"}, {"method", "input.profile"}, {"params", message}}.dump());
  };
  auto raw_watch_until = Clock::time_point{};
#endif
  vsim::WheelState last_wheel;

  // Shared-memory link for native renderers (Unreal viewer, vsim_link.h).
  std::unique_ptr<vsim::StateLink> link;
  vsim::LinkSnapshot snapshot;
  vsim_link::VehicleState link_state{};
  bool viewer_alive = false;
  if (!options.no_link) {
    try {
      link = std::make_unique<vsim::StateLink>(options.link_name);
      snapshot.reset(sim);
    } catch (const std::exception& error) {
      std::cerr << "warning: shared-memory link off: " << error.what() << "\n";
    }
  }
  const auto publish_track = [&] {
    if (!link) {
      return;
    }
    if (const auto track = sim.track()) {
      link->publish_track(vsim::make_track_mesh(*track), track->name(), track->length());
    } else {
      link->clear_track();
    }
  };
  publish_track();
  if (link) {
    link->publish_car(car_model);
    std::cout << "Shared-memory link '" << link->name() << "' for the Unreal viewer (ui/unreal)\n";
  }

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
  vsim::KeyState browser_keys;
  std::int64_t applied_input_ns = 0;
  const auto after_reset = [&] {
    stream.resolve(sim.bus());
    if (link) {
      snapshot.reset(sim);
    }
    rtf_sim_start = sim.time();
    lag = 0.0;
  };

  bool reload = false;
  while (!g_stop && !reload && sim.time() < end_time) {
    for (const std::string& text : server.take_messages()) {
      json message = json::parse(text, nullptr, false);
      if (message.is_discarded() || !message.contains("method")) {
        continue;
      }
      const std::string method = message.value("method", "");
      const json params = message.value("params", json::object());
      // A bad request must not stop the simulation.
      try {
        if (method == "input.keys") {
          browser_keys.throttle = params.value("throttle", false);
          browser_keys.brake = params.value("brake", false);
          browser_keys.left = params.value("left", false);
          browser_keys.right = params.value("right", false);
          browser_keys.stamp = Clock::now();
          last_keys = browser_keys.stamp;
        } else if (method == "sim.reset") {
          sim.reset();
          after_reset();
        } else if (method == "sim.pause") {
          paused = true;
        } else if (method == "sim.resume") {
          paused = false;
        } else if (method == "sim.toggle_pause") {
          paused = !paused;
        } else if (method == "track.set" || method == "track.reset") {
          // A track from the editor (track.set) or back to the scenario's.
          try {
            std::shared_ptr<const vsim::Track> track;
            if (method == "track.set") {
              track = std::make_shared<const vsim::Track>(
                  vsim::TrackDefinition::from_json(params.value("track", json::object()).dump()));
            }
            sim.set_track(std::move(track));
            after_reset();
            publish_track();
            ++track_generation;
            server.set_hello(hello_message(sim, track_generation, car_generation));
            server.broadcast(hello_message(sim, track_generation, car_generation));
            const auto current = sim.track();
            const std::string name = current ? current->name() : std::string("(none)");
            server.broadcast(notice("info", "track: " + name));
            std::cout << "Track: " << name << "\n";
            if (current) {
              for (const std::string& warning : current->warnings()) {
                server.broadcast(notice("warning", warning));
              }
            }
          } catch (const std::exception& error) {
            server.broadcast(notice("error", std::string("track rejected: ") + error.what()));
          }
  #ifdef VSIM_HAS_SDL
        } else if (method == "input.watch") {
          raw_watch_until = Clock::now() + std::chrono::seconds(3);
        } else if (method == "input.profile.get") {
          send_profile();
        } else if (method == "input.profile.apply") {
          if (!wheel_service) {
            server.broadcast(notice("error", "steering wheel input is off: run a driver.type dil scenario without --no-wheel"));
            continue;
          }
          try {
            vsim::WheelProfile profile = vsim::WheelProfile::from_params(
                vsim::ParamTree::from_yaml_string(params.value("profile", json::object()).dump()));
            if (params.value("save", false)) {
              std::filesystem::create_directories(profile_dir);
              const std::filesystem::path file = profile_dir / profile_file_name(profile.name);
              std::ofstream out(file);
              out << profile.to_yaml();
              if (!out) {
                throw std::runtime_error("cannot write " + vsim::path_to_utf8(file));
              }
              server.broadcast(notice("info", "profile saved: " + vsim::path_to_utf8(file)));
              std::cout << "Wheel profile saved: " << vsim::path_to_utf8(file) << "\n";
            } else {
              server.broadcast(notice("info", "profile applied (not saved): " + profile.name));
            }
            wheel_service->apply_profile(std::move(profile));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // let the input thread take it
            send_profile();
          } catch (const std::exception& error) {
            server.broadcast(notice("error", std::string("profile rejected: ") + error.what()));
          }
        } else if (method == "input.ffb_test") {
          if (wheel_service) {
            wheel_service->test_force(params.value("level", 0.0), params.value("seconds", 0.5));
          }
        } else if (method == "input.virtual") {
          if (wheel_service) {
            std::vector<int> axes;
            std::vector<std::uint8_t> buttons;
            for (const auto& v : params.value("axes", json::array())) {
              axes.push_back(v.get<int>());
            }
            for (const auto& v : params.value("buttons", json::array())) {
              buttons.push_back(v.is_boolean() ? v.get<bool>() : v.get<int>() != 0);
            }
            wheel_service->set_virtual(axes, buttons);
          }
  #endif
        } else if (method == "vehicle.params") {
          server.broadcast(json{{"jsonrpc", "2.0"},
                                {"method", "vehicle.params"},
                                {"params",
                                 {{"project", vsim::path_to_utf8(sim.scenario().project_dir)},
                                  {"project_name", sim.scenario().params.get_string("project.name", "")},
                                  {"vehicle", params_json(sim.scenario().params, "vehicle")}}}}
                               .dump());
        } else if (method == "vehicle.import") {
          const auto written = import_vehicle_files(sim.scenario().project_dir, params.at("files"));
          for (const std::string& file : written) {
            std::cout << "Vehicle builder wrote " << file << "\n";
          }
          server.broadcast(notice("info", "saved " + std::to_string(written.size()) + " file(s) to the project"));
          if (params.value("reload", false)) {
            server.broadcast(notice("info", "reloading the simulation with the new vehicle"));
            reload = true;
          }
        } else if (method == "vehicle.model") {
          server.broadcast(car_model_message(car_model, car_generation));
        } else if (method == "track.mesh") {
          server.broadcast(track_mesh_message(sim, track_generation));
        } else if (method == "track.get") {
          const auto track = sim.track();
          server.broadcast(json{{"jsonrpc", "2.0"},
                                {"method", "track.definition"},
                                {"params", {{"track", track ? json::parse(track->definition().to_json()) : json(nullptr)}}}}
                               .dump());
        }
      } catch (const std::exception& error) {
        server.broadcast(notice("error", method + ": " + error.what()));
      }
    }

    const auto now = Clock::now();
    // Wheel buttons mapped by the profile act on the press.
    const vsim::WheelState wheel = sim.inputs().wheel();
    if (wheel.reset && !last_wheel.reset) {
      sim.reset();
      after_reset();
    }
    if (wheel.pause && !last_wheel.pause) {
      paused = !paused;
    }
    last_wheel = wheel;

    // Keys from the browser and from the 3D viewer; a source that went quiet
    // (lost focus, disconnected) must not leave a pedal pressed.
    vsim::KeyState keys;
    if (now - last_keys <= kKeyTimeout) {
      keys = browser_keys;
    }
    vsim::StateLink::ViewerInput viewer;
    if (link) {
      viewer = link->poll_viewer();
      if (viewer.keys) {
        keys.throttle = keys.throttle || viewer.keys->throttle;
        keys.brake = keys.brake || viewer.keys->brake;
        keys.left = keys.left || viewer.keys->left;
        keys.right = keys.right || viewer.keys->right;
        keys.stamp = std::max(keys.stamp, Clock::time_point(std::chrono::duration_cast<Clock::duration>(
                                              std::chrono::nanoseconds(viewer.keys_ns))));
      }
      for (const vsim_link::Command command : viewer.commands) {
        if (command == vsim_link::kCommandReset) {
          sim.reset();
          after_reset();
        } else if (command == vsim_link::kCommandTogglePause) {
          paused = !paused;
        }
      }
      if (viewer.alive != viewer_alive) {
        viewer_alive = viewer.alive;
        std::cout << (viewer_alive ? "3D viewer connected\n" : "3D viewer disconnected\n");
      }
    }
    sim.inputs().set_keys(keys);
    // The newest driver input the coming steps apply, for latency.
    const std::int64_t input_ns = std::max(to_ns(keys.stamp), wheel.connected ? to_ns(wheel.stamp) : 0);
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
        applied_input_ns = input_ns;
      }
    }

    if (link) {
      snapshot.update(sim, link_state);
      link_state.publish_ns = vsim_link::now_ns();
      link_state.input_ns = applied_input_ns;
      link_state.rtf = static_cast<float>(rtf);
      link_state.paused = paused;
      link_state.wheel_connected = wheel.connected;
      link_state.track_generation = link->track_generation();
      link_state.car_generation = link->car_generation();
      link->publish(link_state);
    }

    if (now - rtf_window >= std::chrono::seconds(1)) {
      rtf = (sim.time() - rtf_sim_start) / std::chrono::duration<double>(now - rtf_window).count();
      rtf_window = now;
      rtf_sim_start = sim.time();
    }
    if (now >= next_publish) {
      json input = {{"device", wheel.connected ? wheel.device : ""}};
      if (viewer.alive) {
        input["viewer"] = {{"latency_ms", viewer.latency_ms}, {"frame_ms", viewer.frame_ms}};
      }
#ifdef VSIM_HAS_SDL
      if (wheel_service) {
        const vsim::InputService::Status status = wheel_service->status();
        if (status.connected != wheel_status.connected || status.device != wheel_status.device) {
          if (status.connected) {
            std::cout << "Steering wheel: " << status.device << " (profile " << status.profile
                      << ", force feedback " << (status.force_feedback ? "on" : "off: " + status.ffb_error)
                      << ")\n";
            if (!status.calibrated) {
              std::cout << "  default profile: if the wheel pulls away from the centre or the pedals act\n"
                           "  wrongly, run `vsim-input calibrate` once (docs/build.md)\n";
            }
          } else {
            std::cout << "Steering wheel: disconnected, using the keyboard\n";
          }
        }
        if (status.ffb_suspended && !wheel_status.ffb_suspended) {
          std::cout << "Force feedback paused: the wheel stayed far past the steering lock. If it was\n"
                       "  turning there by itself, the FFB direction is reversed (ffb.invert in the\n"
                       "  profile, or `vsim-input calibrate`). It resumes inside the lock.\n";
        }
        wheel_status = status;
        input["ffb"] = wheel_status.force_feedback;
        input["ffb_torque"] = wheel_status.ffb_torque;
        input["ffb_clipping"] = wheel_status.ffb_clipping;
        input["ffb_suspended"] = wheel_status.ffb_suspended;
      }
      if (now < raw_watch_until) {
        json raw = {{"available", wheel_service != nullptr}};
        if (wheel_service) {
          const vsim::InputService::RawInput r = wheel_service->raw();
          raw.update({{"connected", r.connected}, {"device", r.device},   {"vendor", r.vendor},
                      {"product", r.product},     {"axes", r.axes},       {"buttons", r.buttons},
                      {"steer", wheel.steer},     {"throttle", wheel.throttle}, {"brake", wheel.brake},
                      {"clutch", wheel.clutch},   {"force_feedback", wheel_status.force_feedback},
                      {"ffb_torque", wheel_status.ffb_torque}});
        }
        server.broadcast(json{{"jsonrpc", "2.0"}, {"method", "input.raw"}, {"params", raw}}.dump());
      }
#endif
      server.broadcast(stream.message(sim, paused, rtf, {{"input", input}}));
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
  return reload ? kReload : 0;
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
    for (;;) {
    vsim::Simulation sim(vsim::Scenario::load(vsim::path_from_utf8(options->scenario),
                                              options->overrides),
                         sim_options);

    if (!options->export_kc.empty()) {
      write_kc(sim.scenario().params, options->export_kc);
      if (options->export_car.empty() && options->export_track.empty()) {
        return 0;
      }
    }
    if (!options->export_car.empty()) {
      std::ofstream out(vsim::path_from_utf8(options->export_car), std::ios::binary);
      out << vsim::to_glb(vsim::make_vehicle_visual(sim.scenario().params));
      if (!out) {
        throw std::runtime_error("cannot write " + options->export_car);
      }
      std::cout << "Car model written to " << options->export_car << "\n";
      if (options->export_track.empty()) {
        return 0;
      }
    }
    if (!options->export_track.empty()) {
      const auto track = sim.track();
      if (!track) {
        throw std::invalid_argument("the scenario has no track (track.file or track.template)");
      }
      std::ofstream out(vsim::path_from_utf8(options->export_track), std::ios::binary);
      out << vsim::to_glb(vsim::make_track_mesh(*track));
      if (!out) {
        throw std::runtime_error("cannot write " + options->export_track);
      }
      std::cout << "Track '" << track->name() << "' (" << track->length() << " m) written to "
                << options->export_track << "\n";
      return 0;
    }

    const std::string mode = options->mode.empty()
                                 ? sim.scenario().params.get_string("time.mode", "realtime")
                                 : options->mode;
    const int status = mode == "fast" ? run_fast(sim, *options) : run_realtime(sim, *options);
    if (status == kReload && !g_stop) {
      std::cout << "\nReloading " << options->scenario << " with the new vehicle\n";
      continue;
    }
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    return status;
    }
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    if (argc <= 1) {
      std::cerr << "\n" << kUsage;
    }
    return 1;
  }
}

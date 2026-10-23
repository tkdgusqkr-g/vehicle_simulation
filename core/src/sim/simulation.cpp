#include "vsim/sim/simulation.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "vsim/control/reference_vcu.hpp"
#include "vsim/driver/drivers.hpp"
#include "vsim/log/mcap_logger.hpp"
#include "vsim/models/vehicle/single_track.hpp"
#include "vsim/models/vehicle/two_track.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/integrator.hpp"
#include "vsim/track/lap_timer.hpp"
#include "vsim/track/track.hpp"
#include "vsim/util/paths.hpp"
#include "vsim/version.hpp"

namespace vsim {
namespace {

// Number of plant steps in `period`, which must be a whole multiple of dt.
std::int64_t ticks_for(double period, double dt, std::string_view what) {
  const double ratio = period / dt;
  const double rounded = std::round(ratio);
  if (!(period > 0.0) || rounded < 1.0 || std::abs(ratio - rounded) > 1e-6) {
    throw std::invalid_argument(std::string(what) + ": period " + std::to_string(period) +
                                " s must be a positive multiple of time.dt (" +
                                std::to_string(dt) + " s)");
  }
  return static_cast<std::int64_t>(rounded);
}

std::string local_timestamp(const char* format) {
  const std::time_t now = std::time(nullptr);
  std::tm local{};
#ifdef _WIN32
  localtime_s(&local, &now);
#else
  localtime_r(&now, &local);
#endif
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), format, &local);
  return buffer;
}

std::filesystem::path make_run_dir(const std::filesystem::path& base, const std::string& name) {
  const std::string stem = local_timestamp("%Y%m%d-%H%M%S") + "_" + name;
  std::filesystem::path dir = base / path_from_utf8(stem);
  for (int n = 2; std::filesystem::exists(dir); ++n) {
    dir = base / path_from_utf8(stem + "_" + std::to_string(n));
  }
  std::filesystem::create_directories(dir);
  return dir;
}

void write_text(const std::filesystem::path& file, const std::string& text) {
  std::ofstream out(file, std::ios::binary);
  if (!out) {
    throw std::runtime_error("cannot write '" + path_to_utf8(file) + "'");
  }
  out << text;
}

}  // namespace

struct Simulation::Impl {
  struct ScheduledBlock {
    std::unique_ptr<Block> block;
    std::int64_t period_ticks = 1;
  };

  SignalBus bus;
  std::unique_ptr<Context> context;
  std::unique_ptr<PlantModel> plant;
  std::vector<double> state;
  Rk4Integrator integrator;
  std::vector<ScheduledBlock> blocks;

  std::int64_t tick = 0;
  double dt = 0.001;
  std::int64_t dt_ns = 1'000'000;
  double t_end = std::numeric_limits<double>::infinity();

  std::vector<SignalId> log_ids;
  std::vector<double> log_values;
  std::int64_t log_period_ticks = 10;
  bool record = true;
  Recorder recorder;
  std::unique_ptr<McapLogger> mcap;
  std::filesystem::path run_path;

  std::optional<SignalId> speed_id;
  double distance = 0.0;
  double max_speed = 0.0;
  bool finished = false;

  struct Lap {
    int number;
    double time;
    bool valid;
  };
  std::optional<SignalId> lap_id, last_lap_id, last_lap_valid_id;
  std::vector<Lap> laps;

  void build(const Scenario& scenario, const SimulationOptions& options, InputHub& inputs);
  void step_once();
  void log();
  void finish();
};

void Simulation::Impl::build(const Scenario& scenario, const SimulationOptions& options,
                             InputHub& inputs) {
  const ParamTree& params = scenario.params;

  dt = params.get_double("time.dt", 0.001);
  dt_ns = std::llround(dt * 1e9);
  if (!(dt > 0.0) || std::abs(static_cast<double>(dt_ns) * 1e-9 - dt) > 1e-15) {
    throw std::invalid_argument("time.dt must be a positive whole number of nanoseconds");
  }
  t_end = params.get_double("time.t_end", std::numeric_limits<double>::infinity());
  const auto seed = static_cast<std::uint64_t>(params.get_int("seed", 0));
  context = std::make_unique<Context>(bus, inputs, RealismConfig::from_params(params), seed, dt);

  // Track: from the options (editor), else track.file / track.template.
  std::shared_ptr<const Track> track = options.track;
  if (!track) {
    std::string file = params.get_string("track.file", "");
    if (file.empty() && params.has("track")) {
      file = params.get_string("track");
    }
    if (const auto path = resolve_track_file(file, params.get_string("track.template", ""),
                                             scenario.file.parent_path())) {
      track = std::make_shared<Track>(TrackDefinition::from_file(*path));
    }
  }
  context->set_track(track);
  // Models see the start pose on the track unless the scenario sets one.
  ParamTree effective = params;
  if (track && !params.has("initial.x_m") && !params.has("initial.y_m")) {
    const TrackPose pose = start_pose(*track, params.get_double("track.start_distance_m", 5.0));
    effective.set("initial.x_m", std::to_string(pose.x));
    effective.set("initial.y_m", std::to_string(pose.y));
    effective.set("initial.yaw_deg", std::to_string(pose.yaw * 180.0 / std::numbers::pi));
  }

  const std::string model = params.get_string("vehicle.model", "two_track");
  if (model == "two_track" || model == "full") {
    plant = std::make_unique<TwoTrackModel>();
  } else if (model == "single_track") {
    plant = std::make_unique<SingleTrackModel>();
  } else {
    throw std::invalid_argument("vehicle.model '" + model +
                                "' is not available (expected full, two_track or single_track)");
  }
  plant->init(effective, *context);

  // Execution order within a tick: driver, VCU, then lap timing.
  std::vector<std::unique_ptr<Block>> ordered;
  ordered.push_back(make_driver(effective));
  ordered.push_back(std::make_unique<ReferenceVcu>());
  if (track) {
    ordered.push_back(std::make_unique<LapTimer>());
  }
  for (auto& block : ordered) {
    block->init(effective, *context);
    const std::int64_t period_ticks = ticks_for(block->period(), dt, block->name());
    blocks.push_back({std::move(block), period_ticks});
  }

  state.assign(plant->state_size(), 0.0);
  plant->initial_state(state);
  plant->write_outputs(0.0, state, bus);
  speed_id = bus.find("veh.chassis.speed");
  lap_id = bus.find("track.lap");
  last_lap_id = bus.find("track.last_lap");
  last_lap_valid_id = bus.find("track.last_lap_valid");

  // Logging: union of the glob patterns, in declaration order.
  std::vector<std::string> patterns = params.get_string_list("logging.signals");
  if (patterns.empty()) {
    patterns.push_back("*");
  }
  std::vector<bool> selected(bus.size(), false);
  for (const std::string& pattern : patterns) {
    const std::vector<SignalId> matches = bus.match(pattern);
    if (matches.empty()) {
      throw std::invalid_argument("logging.signals: pattern '" + pattern + "' matches no signal");
    }
    for (const SignalId id : matches) {
      selected[id] = true;
    }
  }
  for (SignalId id = 0; id < bus.size(); ++id) {
    if (selected[id]) {
      log_ids.push_back(id);
    }
  }
  log_values.resize(log_ids.size());
  log_period_ticks = ticks_for(1.0 / params.get_double("logging.rate_hz", 100.0), dt, "logging");

  record = options.record;
  if (record) {
    recorder.configure(bus, log_ids);
  }
  if (!options.log_dir.empty()) {
    run_path = make_run_dir(options.log_dir, scenario.name);
    write_text(run_path / "scenario.resolved.yaml", params.to_yaml());

    nlohmann::json meta;
    meta["scenario"] = scenario.name;
    meta["scenario_file"] = path_to_utf8(scenario.file);
    meta["project_dir"] = path_to_utf8(scenario.project_dir);
    meta["seed"] = seed;
    meta["dt"] = dt;
    meta["vsim_version"] = version();
    meta["git_commit"] = git_commit();
    meta["created_local"] = local_timestamp("%Y-%m-%dT%H:%M:%S");
    for (const RealismCategory category : kRealismCategories) {
      meta["realism"][std::string(to_string(category))] =
          context->realism().is_real(category) ? "real" : "ideal";
    }
    std::vector<std::string> names;
    for (const SignalId id : log_ids) {
      names.push_back(bus.info(id).name);
      meta["signals"][bus.info(id).name] = bus.info(id).unit;
    }
    write_text(run_path / "meta.json", meta.dump(2) + "\n");
    mcap = std::make_unique<McapLogger>(run_path / "log.mcap", std::move(names));
  }
  log();
}

void Simulation::Impl::step_once() {
  const double t = static_cast<double>(tick) * dt;
  for (auto& scheduled : blocks) {
    if (tick % scheduled.period_ticks == 0) {
      scheduled.block->step(t, bus);
    }
  }
  plant->read_inputs(bus);
  integrator.step(*plant, t, dt, state);
  ++tick;
  plant->write_outputs(static_cast<double>(tick) * dt, state, bus);

  if (speed_id) {
    const double speed = bus.get(*speed_id);
    distance += speed * dt;
    max_speed = std::max(max_speed, speed);
  }
  if (lap_id && bus.get(*lap_id) > static_cast<double>(laps.size())) {
    laps.push_back({static_cast<int>(laps.size()) + 1, bus.get(*last_lap_id), bus.get(*last_lap_valid_id) > 0.5});
  }
  if (tick % log_period_ticks == 0) {
    log();
  }
}

void Simulation::Impl::log() {
  const double t = static_cast<double>(tick) * dt;
  if (record) {
    recorder.record(t, bus);
  }
  if (mcap) {
    for (std::size_t i = 0; i < log_ids.size(); ++i) {
      log_values[i] = bus.get(log_ids[i]);
    }
    mcap->write(static_cast<std::uint64_t>(tick * dt_ns), log_values);
  }
}

void Simulation::Impl::finish() {
  if (finished) {
    return;
  }
  finished = true;
  if (!run_path.empty()) {
    nlohmann::json summary;
    summary["duration_s"] = static_cast<double>(tick) * dt;
    summary["ticks"] = tick;
    summary["distance_m"] = distance;
    summary["max_speed_mps"] = max_speed;
    if (const Track* track = context->track().get()) {
      summary["track"] = track->name();
      nlohmann::json laps_json = nlohmann::json::array();
      double best = 0.0;
      for (const Lap& lap : laps) {
        laps_json.push_back({{"lap", lap.number}, {"time_s", lap.time}, {"valid", lap.valid}});
        if (lap.valid && (best == 0.0 || lap.time < best)) {
          best = lap.time;
        }
      }
      summary["laps"] = laps_json;
      summary["best_lap_s"] = best > 0.0 ? nlohmann::json(best) : nlohmann::json(nullptr);
    }
    write_text(run_path / "summary.json", summary.dump(2) + "\n");
  }
  if (mcap) {
    mcap->close();
  }
}

Simulation::Simulation(Scenario scenario, SimulationOptions options)
    : scenario_(std::move(scenario)), options_(std::move(options)) {
  reset();
}

Simulation::~Simulation() {
  try {
    finish();
  } catch (...) {
    // Destructors must not throw; finish() explicitly to see errors.
  }
}

void Simulation::step(std::int64_t ticks) {
  for (std::int64_t i = 0; i < ticks; ++i) {
    impl_->step_once();
  }
}

void Simulation::run_for(double seconds) { step(std::llround(seconds / impl_->dt)); }

void Simulation::run_to_end() {
  if (!std::isfinite(impl_->t_end)) {
    throw std::logic_error("scenario '" + scenario_.name + "' has no time.t_end");
  }
  step(std::llround(impl_->t_end / impl_->dt) - impl_->tick);
}

std::int64_t Simulation::tick() const { return impl_->tick; }
double Simulation::time() const { return static_cast<double>(impl_->tick) * impl_->dt; }
double Simulation::dt() const { return impl_->dt; }
double Simulation::end_time() const { return impl_->t_end; }
const SignalBus& Simulation::bus() const { return impl_->bus; }
void Simulation::set_signal(const std::string& name, double value) { impl_->bus.set(impl_->bus.require(name), value); }
const Recorder& Simulation::recorder() const { return impl_->recorder; }
const std::filesystem::path& Simulation::run_path() const { return impl_->run_path; }

void Simulation::reset() {
  if (impl_) {
    impl_->finish();
  }
  auto fresh = std::make_unique<Impl>();
  fresh->build(scenario_, options_, inputs_);
  impl_ = std::move(fresh);
}

void Simulation::finish() { impl_->finish(); }

std::shared_ptr<const Track> Simulation::track() const { return impl_->context->track(); }

void Simulation::set_track(std::shared_ptr<const Track> track) {
  options_.track = std::move(track);
  reset();
}

}  // namespace vsim

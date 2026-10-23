#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "vsim/sim/input_hub.hpp"
#include "vsim/sim/recorder.hpp"
#include "vsim/sim/scenario.hpp"
#include "vsim/sim/signal_bus.hpp"

namespace vsim {

class PluginBlock;

class Track;

struct SimulationOptions {
  // Base directory for run outputs; each run gets its own subdirectory.
  // Empty: nothing is written to disk.
  std::filesystem::path log_dir;
  // Keep logged signals in memory (Python access, tests).
  bool record = true;
  // Replaces the scenario's track (track.file / track.template), e.g. one
  // sent from the track editor.
  std::shared_ptr<const Track> track;
};

// Owns one run of a scenario: the signal bus, the plant, the discrete blocks
// and logging. Time advances in fixed plant steps ("ticks").
//
// Order within a tick (design doc §15.1):
//   1. due blocks run in order (driver, VCU), writing commands for this step
//   2. the plant latches its inputs and is integrated over one step
//   3. plant outputs are written for the new time
//   4. logging, when due
class Simulation {
 public:
  explicit Simulation(Scenario scenario, SimulationOptions options = {});
  ~Simulation();
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  void step(std::int64_t ticks = 1);
  void run_for(double seconds);
  // Runs until time.t_end; throws if the scenario has none.
  void run_to_end();

  std::int64_t tick() const;
  double time() const;
  double dt() const;
  double end_time() const;  // +inf without time.t_end

  const Scenario& scenario() const { return scenario_; }
  const SignalBus& bus() const;
  // Sets a signal from outside (e.g. the full model's four-post rig inputs
  // rig.<corner>.z / .vz); its producer, if any, overwrites it next step.
  void set_signal(const std::string& name, double value);
  // Survives reset(), so a connected keyboard keeps working.
  InputHub& inputs() { return inputs_; }
  const Recorder& recorder() const;
  // The project's plugin components in execution order (design doc §11).
  std::vector<const PluginBlock*> plugins() const;
  // Directory of the current run's files; empty when not writing.
  const std::filesystem::path& run_path() const;

  // Rebuilds everything from the scenario (same seed, so same results).
  // When writing logs, the finished run is closed and a new one started.
  void reset();
  // Writes the run summary and closes log files. Safe to call twice.
  void finish();

  // The track in use (nullptr without one).
  std::shared_ptr<const Track> track() const;
  // Uses `track` from now on (nullptr: back to the scenario's) and resets.
  void set_track(std::shared_ptr<const Track> track);

 private:
  struct Impl;

  Scenario scenario_;
  SimulationOptions options_;
  InputHub inputs_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vsim

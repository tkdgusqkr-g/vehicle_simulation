#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

#include "test_support.hpp"

namespace vsim {
namespace {

using test::load_example;
using test::signal;

// A fresh directory under the system temp dir, removed afterwards.
class TempDir {
 public:
  TempDir() {
    path_ = std::filesystem::temp_directory_path() /
            ("vsim_test_" + std::to_string(std::random_device{}()));
    std::filesystem::create_directories(path_);
  }
  ~TempDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

// V-04: same scenario, same seed -> bit-identical logs.
TEST(Simulation, DeterministicAcrossRuns) {
  Simulation a(load_example("sine_steer"));
  Simulation b(load_example("sine_steer"));
  a.run_for(12.0);
  b.run_for(12.0);
  ASSERT_EQ(a.recorder().names(), b.recorder().names());
  EXPECT_EQ(a.recorder().times(), b.recorder().times());
  EXPECT_EQ(a.recorder().columns(), b.recorder().columns());
}

TEST(Simulation, ResetReproducesTheRun) {
  Simulation sim(load_example("sine_steer"));
  sim.run_for(8.0);
  const double x = signal(sim, "veh.chassis.x");
  const double y = signal(sim, "veh.chassis.y");
  sim.reset();
  EXPECT_EQ(sim.tick(), 0);
  sim.run_for(8.0);
  EXPECT_EQ(signal(sim, "veh.chassis.x"), x);
  EXPECT_EQ(signal(sim, "veh.chassis.y"), y);
}

TEST(Simulation, LogsAtTheConfiguredRate) {
  Simulation sim(load_example("sine_steer", {{"logging.rate_hz", "50"}}));
  sim.run_for(1.0);
  const auto& times = sim.recorder().times();
  ASSERT_EQ(times.size(), 51u);
  EXPECT_DOUBLE_EQ(times[1], 0.02);
  EXPECT_DOUBLE_EQ(times.back(), 1.0);
}

TEST(Simulation, RejectsPeriodsThatAreNotMultiplesOfDt) {
  EXPECT_THROW(Simulation(load_example("sine_steer", {{"driver.period_s", "0.0015"}})),
               std::invalid_argument);
  EXPECT_THROW(Simulation(load_example("sine_steer", {{"logging.rate_hz", "300"}})),
               std::invalid_argument);
}

TEST(Simulation, RejectsLoggingPatternsThatMatchNothing) {
  EXPECT_THROW(Simulation(load_example("sine_steer", {{"logging.signals", "nope.*"}})),
               std::invalid_argument);
}

TEST(Simulation, RunToEndNeedsAnEndTime) {
  Simulation sim(load_example("keyboard"));
  EXPECT_THROW(sim.run_to_end(), std::logic_error);
}

TEST(Simulation, ExternalDriverTakesCommandsFromTheInputHub) {
  Simulation sim(load_example("straight_accel", {{"driver.type", "external"}}));
  sim.inputs().set_command({0.0, 1.0, 0.0});
  sim.run_for(2.0);
  EXPECT_GT(signal(sim, "veh.chassis.vx"), 5.0);
  EXPECT_EQ(signal(sim, "drv.cmd.throttle"), 1.0);
}

TEST(Scenario, OverridePrecedence) {
  TempDir dir;
  const std::filesystem::path file = dir.path() / "scenario.yaml";
  {
    std::ofstream out(file);
    out << "name: precedence\n"
        << "project: " << VSIM_SOURCE_DIR << "/templates/projects/example_fs_ev\n"
        << "driver: {type: external}\n"
        << "overrides:\n"
        << "  vehicle.mass_kg: 350\n";
  }
  EXPECT_EQ(Scenario::load(file).params.get_double("vehicle.mass_kg"), 350.0);
  EXPECT_EQ(Scenario::load(file, {{"vehicle.mass_kg", "400"}}).params.get_double("vehicle.mass_kg"),
            400.0);
  const Scenario scenario = Scenario::load(file);
  EXPECT_EQ(scenario.name, "precedence");
  EXPECT_EQ(scenario.params.get_string("project.name"), "example_fs_ev");
  EXPECT_EQ(scenario.project_dir.filename(), "example_fs_ev");
}

TEST(Simulation, WritesRunFiles) {
  TempDir dir;
  std::filesystem::path run;
  {
    SimulationOptions options;
    options.log_dir = dir.path();
    Simulation sim(load_example("sine_steer"), options);
    sim.run_for(2.0);
    sim.finish();
    run = sim.run_path();
  }
  ASSERT_FALSE(run.empty());
  for (const char* name : {"meta.json", "scenario.resolved.yaml", "log.mcap", "summary.json"}) {
    EXPECT_TRUE(std::filesystem::exists(run / name)) << name;
  }
  EXPECT_GT(std::filesystem::file_size(run / "log.mcap"), 1000u);
  std::ifstream summary_file(run / "summary.json");
  const auto summary = nlohmann::json::parse(summary_file);
  EXPECT_DOUBLE_EQ(summary["duration_s"].get<double>(), 2.0);
  EXPECT_GT(summary["distance_m"].get<double>(), 0.0);
  // The resolved scenario loads back as a parameter tree with the same content.
  const ParamTree resolved = ParamTree::from_yaml_file(run / "scenario.resolved.yaml");
  EXPECT_EQ(resolved.entries(), load_example("sine_steer").params.entries());
}

}  // namespace
}  // namespace vsim

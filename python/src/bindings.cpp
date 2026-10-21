// Python module `vsim._vsim`: runs the C++ simulation core in-process.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "vsim/sim/simulation.hpp"
#include "vsim/util/paths.hpp"
#include "vsim/version.hpp"

namespace py = pybind11;

namespace {

// Parameter values are text in the scenario tree; convert Python values the
// way YAML would write them.
vsim::Scenario::Overrides to_overrides(const py::dict& values) {
  vsim::Scenario::Overrides overrides;
  for (const auto& item : values) {
    const std::string key = py::str(item.first);
    const py::handle value = item.second;
    if (py::isinstance<py::bool_>(value)) {
      overrides[key] = value.cast<bool>() ? "true" : "false";
    } else if (py::isinstance<py::str>(value)) {
      overrides[key] = value.cast<std::string>();
    } else if (py::isinstance<py::int_>(value) || py::isinstance<py::float_>(value)) {
      overrides[key] = py::repr(value).cast<std::string>();  // shortest round-trip form
    } else {
      throw py::type_error("override '" + key + "' must be a bool, int, float or str");
    }
  }
  return overrides;
}

py::array_t<double> to_array(const std::vector<double>& values) {
  py::array_t<double> array(static_cast<py::ssize_t>(values.size()));
  std::copy(values.begin(), values.end(), array.mutable_data());
  return array;
}

}  // namespace

PYBIND11_MODULE(_vsim, m) {
  m.doc() = "C++ simulation core of vsim";
  m.attr("__version__") = vsim::version();
  m.attr("git_commit") = vsim::git_commit();

  py::class_<vsim::Simulation>(m, "Simulation", R"doc(
One run of a scenario. Time advances in fixed steps of `dt`.

    sim = Simulation("scenario.yaml", overrides={"vehicle.mass_kg": 300})
    sim.run(10.0)              # simulate 10 s
    rec = sim.recording()      # {"t": array, "veh.chassis.vx": array, ...}
)doc")
      .def(py::init([](const std::filesystem::path& scenario, const py::dict& overrides,
                       const std::optional<std::filesystem::path>& log_dir, bool record) {
             vsim::SimulationOptions options;
             if (log_dir) {
               options.log_dir = *log_dir;
             }
             options.record = record;
             return std::make_unique<vsim::Simulation>(
                 vsim::Scenario::load(scenario, to_overrides(overrides)), options);
           }),
           py::arg("scenario"), py::kw_only(), py::arg("overrides") = py::dict(),
           py::arg("log_dir") = py::none(), py::arg("record") = true,
           "Loads a scenario file. `overrides` maps dotted parameter keys to values; "
           "`log_dir` enables run outputs (MCAP log, metadata) under that directory.")
      .def("step", &vsim::Simulation::step, py::arg("ticks") = 1,
           py::call_guard<py::gil_scoped_release>(), "Advances by `ticks` plant steps.")
      .def(
          "run",
          [](vsim::Simulation& sim, std::optional<double> duration) {
            py::gil_scoped_release release;
            if (duration) {
              sim.run_for(*duration);
            } else {
              sim.run_to_end();
            }
          },
          py::arg("duration") = py::none(),
          "Simulates `duration` seconds, or until time.t_end when omitted.")
      .def_property_readonly("time", &vsim::Simulation::time, "Sim time [s].")
      .def_property_readonly("tick", &vsim::Simulation::tick)
      .def_property_readonly("dt", &vsim::Simulation::dt, "Plant step [s].")
      .def_property_readonly("end_time", &vsim::Simulation::end_time)
      .def_property_readonly("scenario_name",
                             [](const vsim::Simulation& sim) { return sim.scenario().name; })
      .def_property_readonly("run_dir",
                             [](const vsim::Simulation& sim) -> std::optional<std::string> {
                               if (sim.run_path().empty()) {
                                 return std::nullopt;
                               }
                               return vsim::path_to_utf8(sim.run_path());
                             })
      .def(
          "get",
          [](const vsim::Simulation& sim, const std::string& name) {
            return sim.bus().get(sim.bus().require(name));
          },
          py::arg("name"), "Current value of a signal.")
      .def(
          "signals",
          [](const vsim::Simulation& sim) {
            std::vector<std::string> names;
            for (vsim::SignalId id = 0; id < sim.bus().size(); ++id) {
              names.push_back(sim.bus().info(id).name);
            }
            return names;
          },
          "Names of all signals.")
      .def(
          "units",
          [](const vsim::Simulation& sim) {
            py::dict units;
            for (vsim::SignalId id = 0; id < sim.bus().size(); ++id) {
              units[py::str(sim.bus().info(id).name)] = sim.bus().info(id).unit;
            }
            return units;
          },
          "Unit of every signal.")
      .def(
          "set_driver_command",
          [](vsim::Simulation& sim, double steer, double throttle, double brake) {
            sim.inputs().set_command({steer, throttle, brake});
          },
          py::kw_only(), py::arg("steer") = 0.0, py::arg("throttle") = 0.0,
          py::arg("brake") = 0.0,
          "Command for driver.type 'external': steering wheel angle [rad], pedals [0, 1].")
      .def(
          "recording",
          [](const vsim::Simulation& sim) {
            const vsim::Recorder& recorder = sim.recorder();
            py::dict out;
            out["t"] = to_array(recorder.times());
            for (std::size_t i = 0; i < recorder.names().size(); ++i) {
              out[py::str(recorder.names()[i])] = to_array(recorder.columns()[i]);
            }
            return out;
          },
          "Logged signals as numpy arrays, keyed by name, plus 't'.")
      .def("reset", &vsim::Simulation::reset,
           "Restarts the scenario from t = 0 (same seed, same results).")
      .def("finish", &vsim::Simulation::finish, "Writes the run summary and closes log files.")
      .def("__enter__", [](vsim::Simulation& sim) -> vsim::Simulation& { return sim; },
           py::return_value_policy::reference)
      .def("__exit__", [](vsim::Simulation& sim, const py::args&) { sim.finish(); });
}

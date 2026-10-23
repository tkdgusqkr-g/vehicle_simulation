// Python plugins: a class with `inputs`, `outputs`, `period`, init(params)
// and step(t, u) -> outputs (vsim.plugin.Block), called every period.

#include "python_plugin.hpp"

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include "vsim/plugin/plugin.hpp"
#include "vsim/util/paths.hpp"

namespace py = pybind11;

namespace vsim {
namespace {

bool g_embedded = false;

// vsim-core: an interpreter on first use, the GIL released between calls.
void ensure_interpreter() {
  static std::once_flag once;
  std::call_once(once, [] {
#if defined(_WIN32) && defined(VSIM_PYTHON_DLL)
    // The interpreter DLL is delay-loaded: check it exists before touching it.
    if (LoadLibraryA(VSIM_PYTHON_DLL) == nullptr) {
      throw std::runtime_error(std::string("python plugins need ") + VSIM_PYTHON_DLL +
                               " (the Python vsim-core was built with) on the PATH");
    }
#endif
    if (!Py_IsInitialized()) {
      py::initialize_interpreter();
#ifdef VSIM_PYTHON_DIR
      py::module_::import("sys").attr("path").attr("insert")(0, VSIM_PYTHON_DIR);
#endif
      PyEval_SaveThread();  // plugin calls take the GIL themselves
    }
  });
}

std::vector<std::pair<std::string, std::string>> port_list(const py::object& spec, const char* what) {
  std::vector<std::pair<std::string, std::string>> out;
  if (py::isinstance<py::dict>(spec)) {
    for (const auto& [name, unit] : spec.cast<py::dict>()) {
      out.emplace_back(py::str(name), unit.is_none() ? std::string() : std::string(py::str(unit)));
    }
  } else if (!spec.is_none()) {
    for (const auto& name : spec) {
      out.emplace_back(py::str(name), "");
    }
  }
  if (out.empty() && std::string(what) == "outputs") {
    throw std::invalid_argument("the plugin class declares no outputs");
  }
  return out;
}

class PythonPlugin final : public PluginInstance {
 public:
  PythonPlugin(const ParamTree& params, const ComponentSpec& spec, double period) : name_(spec.name) {
    py::gil_scoped_acquire gil;
    try {
      const std::string target = params.get_string(spec.prefix + ".module", "");
      const std::size_t colon = target.rfind(':');
      if (target.empty() || colon == std::string::npos || colon + 1 == target.size()) {
        throw std::invalid_argument(spec.prefix + ".module: give 'module:Class' (e.g. tv_py:TorqueVectoring)");
      }
      const std::string module_name = target.substr(0, colon), class_name = target.substr(colon + 1);
      // The project's plugins/ folder and the project itself are importable.
      py::module_ sys = py::module_::import("sys");
      py::list path = sys.attr("path");
      for (const auto& dir : {spec.project_dir, spec.project_dir / "plugins"}) {
        const py::str entry(path_to_utf8(dir));
        if (!path.contains(entry)) {
          path.insert(0, entry);
        }
      }
      py::module_ importlib = py::module_::import("importlib");
      py::module_ module;
      if (module_name.size() > 3 && module_name.compare(module_name.size() - 3, 3, ".py") == 0) {
        // A file, relative to the project.
        std::filesystem::path file = path_from_utf8(module_name);
        if (file.is_relative()) {
          file = spec.project_dir / file;
        }
        py::module_ util = py::module_::import("importlib.util");
        py::object loader_spec =
            util.attr("spec_from_file_location")("vsim_plugin_" + spec.name, path_to_utf8(file));
        if (loader_spec.is_none()) {
          throw std::invalid_argument(spec.prefix + ".module: cannot load '" + path_to_utf8(file) + "'");
        }
        module = util.attr("module_from_spec")(loader_spec);
        loader_spec.attr("loader").attr("exec_module")(module);
      } else {
        const bool loaded = sys.attr("modules").contains(module_name);
        module = importlib.attr("import_module")(module_name);
        if (loaded) {
          module = importlib.attr("reload")(module);  // pick up edits between runs
        }
      }
      if (!py::hasattr(module, class_name.c_str())) {
        throw std::invalid_argument(spec.prefix + ".module: '" + module_name + "' has no '" + class_name + "'");
      }
      py::object cls = module.attr(class_name.c_str());
      object_ = cls();
      for (const auto& [name, unit] : port_list(py::getattr(object_, "inputs", py::none()), "inputs")) {
        ports_.push_back({name, unit, true});
      }
      for (const auto& [name, unit] : port_list(py::getattr(object_, "outputs", py::none()), "outputs")) {
        ports_.push_back({name, unit, false});
        output_names_.push_back(py::str(name));
      }
      for (const PortInfo& p : ports_) {
        if (p.input) {
          input_names_.push_back(py::str(p.name));
        }
      }
      period_ = period > 0.0 ? period : py::getattr(object_, "period", py::float_(0.0)).cast<double>();
      if (!(period_ > 0.0)) {
        throw std::invalid_argument(spec.prefix + ".period: the plugin class sets no period; give one");
      }
      if (py::hasattr(object_, "init")) {
        py::object json = py::module_::import("json");
        object_.attr("init")(json.attr("loads")(component_params_json(params, spec)));
      }
      step_ = object_.attr("step");
      info_ = {"python", py::hasattr(module, "__file__") ? std::string(py::str(module.attr("__file__"))) : "",
               std::string(py::str(cls.attr("__name__"))),
               std::string(py::str(py::getattr(object_, "version", py::str(""))))};
    } catch (py::error_already_set& e) {
      throw std::runtime_error("python plugin " + name_ + ": " + e.what());
    }
  }

  ~PythonPlugin() override {
    py::gil_scoped_acquire gil;
    step_ = py::object();
    object_ = py::object();
    input_names_.clear();
    output_names_.clear();
  }

  const std::vector<PortInfo>& ports() const override { return ports_; }
  double period() const override { return period_; }

  void step(double t, const double* in, double* out) override {
    py::gil_scoped_acquire gil;
    try {
      py::dict u;
      for (std::size_t i = 0; i < input_names_.size(); ++i) {
        u[input_names_[i]] = in[i];
      }
      const py::object y = step_(t, u);
      if (py::isinstance<py::dict>(y)) {
        const py::dict d = y.cast<py::dict>();
        for (std::size_t i = 0; i < output_names_.size(); ++i) {
          if (!d.contains(output_names_[i])) {
            throw std::runtime_error("python plugin " + name_ + ": step() returned no '" +
                                     std::string(output_names_[i]) + "'");
          }
          out[i] = d[output_names_[i]].cast<double>();
        }
      } else {
        const py::sequence s = y.cast<py::sequence>();
        if (s.size() != output_names_.size()) {
          throw std::runtime_error("python plugin " + name_ + ": step() returned " + std::to_string(s.size()) +
                                   " values for " + std::to_string(output_names_.size()) + " outputs");
        }
        for (std::size_t i = 0; i < output_names_.size(); ++i) {
          out[i] = s[i].cast<double>();
        }
      }
    } catch (py::error_already_set& e) {
      throw std::runtime_error("python plugin " + name_ + " at t = " + std::to_string(t) + " s: " + e.what());
    } catch (py::cast_error& e) {
      throw std::runtime_error("python plugin " + name_ + ": step() output is not a number (" + e.what() + ")");
    }
  }

  Info info() const override { return info_; }

 private:
  std::string name_;
  py::object object_, step_;
  std::vector<PortInfo> ports_;
  std::vector<py::str> input_names_, output_names_;
  double period_ = 0.0;
  Info info_;
};

}  // namespace

void register_python_plugins(bool embedded) {
  g_embedded = embedded;
  register_plugin_type("python", [](const ParamTree& params, const ComponentSpec& spec, double period) {
    if (g_embedded) {
      ensure_interpreter();
    }
    return std::unique_ptr<PluginInstance>(std::make_unique<PythonPlugin>(params, spec, period));
  });
}

}  // namespace vsim

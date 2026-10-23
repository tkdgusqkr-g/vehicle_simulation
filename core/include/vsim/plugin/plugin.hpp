#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "vsim/plugin/units.hpp"
#include "vsim/sim/block.hpp"

namespace vsim {

// A port of a plugin: a value it reads or writes every period.
struct PortInfo {
  std::string name;
  std::string unit;  // "" if the plugin does not say
  bool input = true;
};

// One loaded plugin (native library, FMU, Python object) behind the common
// interface the host drives (design doc §11).
class PluginInstance {
 public:
  struct Info {
    std::string type;     // native | fmu | python
    std::string file;     // what was loaded (library, .fmu, module file)
    std::string name;     // the plugin's own name and version
    std::string version;
  };

  virtual ~PluginInstance() = default;
  virtual const std::vector<PortInfo>& ports() const = 0;
  // Execution period [s] the instance was created with.
  virtual double period() const = 0;
  // One period: inputs in input-port order, outputs in output-port order.
  virtual void step(double t, const double* in, double* out) = 0;
  virtual Info info() const = 0;
};

// One components.<name> entry of the vehicle project (design doc §11.4):
//
//   components:
//     tv:
//       enabled: true         # false keeps the entry but does not run it
//       type: native          # native | fmu | python | builtin
//       path: tv_cpp          # library or .fmu, relative to the project
//                             # (or module: file:Class for python)
//       slot: vcu             # replace a built-in block: driver | vcu
//       after: vcu            # otherwise runs after: driver | sensors | vcu (default)
//       period: 0.01          # [s]; default: the plugin's own
//       params: {gain: 400}
//       ports:
//         in:  {steer: drv.cmd.steer, vx: {signal: vcu.vx_est, unit: km/h}, mode: {value: 1}}
//         out: {tq_fl: pt.motor.fl.torque_cmd}
//
// A component named driver or vcu takes that slot unless `slot` says otherwise.
struct ComponentSpec {
  std::string name;
  std::string type;
  std::string slot;   // "driver", "vcu" or "" (an extra block)
  std::string after;  // for extra blocks
  std::string prefix; // "components.<name>"
  double period = 0.0;
  std::filesystem::path project_dir;
};

std::vector<ComponentSpec> components_from_params(const ParamTree& params);

// Creates a plugin instance for a component; `period` is the requested
// period (0: the plugin's default).
using PluginFactory =
    std::function<std::unique_ptr<PluginInstance>(const ParamTree& params, const ComponentSpec& spec, double period)>;

// Plugin types by name. native and fmu are built in; python is registered by
// whoever hosts an interpreter (the Python module, vsim-core).
void register_plugin_type(const std::string& type, PluginFactory factory);
bool plugin_type_available(const std::string& type);
std::vector<std::string> plugin_types();

// A plugin instance for a component (as PluginBlock::init makes it); for
// tools that want its ports without running it.
std::unique_ptr<PluginInstance> create_plugin(const ParamTree& params, const ComponentSpec& spec);

// A plugin file from the component's `path`: relative to the project
// directory, its plugins/ folder, the VSIM_PLUGIN_PATH directories, then the
// plugins built with vsim. Without an extension, the platform's
// shared-library extension is tried too. Throws if not found.
std::filesystem::path find_plugin_file(const ComponentSpec& spec, const std::string& path,
                                       const std::vector<std::string>& extensions, bool allow_directory = false);

// The component's params subtree as a JSON object text ("{}" if none).
std::string component_params_json(const ParamTree& params, const ComponentSpec& spec);

std::string sha256_file(const std::filesystem::path& file);

// A component as a block: its ports connected to bus signals with unit
// conversion, called every period with zero-order hold outputs.
class PluginBlock final : public Block {
 public:
  explicit PluginBlock(ComponentSpec spec);
  ~PluginBlock() override;

  std::string_view name() const override { return spec_.name; }
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override;

  struct Connection {
    std::string port, unit, signal, signal_unit;
    bool input = true;
    std::optional<double> constant;  // an input fed a fixed value
  };
  const ComponentSpec& spec() const { return spec_; }
  const std::vector<Connection>& connections() const { return connections_; }
  PluginInstance::Info info() const;
  std::string sha256() const { return sha256_; }

  // Wall time spent in the plugin (not on the bus: it varies run to run).
  struct Stats {
    std::int64_t calls = 0;
    double total_s = 0.0, max_s = 0.0;
  };
  const Stats& stats() const { return stats_; }

 private:
  ComponentSpec spec_;
  std::unique_ptr<PluginInstance> instance_;
  double period_ = 0.01;
  std::vector<Connection> connections_;
  struct Binding {
    std::optional<SignalId> id;
    double constant = 0.0;
    UnitConversion conversion;
  };
  std::vector<Binding> inputs_, outputs_;
  std::vector<double> in_, out_;
  std::string sha256_;
  Stats stats_;
};

}  // namespace vsim

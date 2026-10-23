#include "vsim/plugin/python_host.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "plugin_hosts.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

using CreateFn = PluginInstance* (*)(const ParamTree*, const ComponentSpec*, double, char*, int);
using DestroyFn = void (*)(PluginInstance*);

// An instance made by the host library, destroyed by it.
class HostedPlugin final : public PluginInstance {
 public:
  HostedPlugin(PluginInstance* plugin, DestroyFn destroy) : plugin_(plugin), destroy_(destroy) {}
  ~HostedPlugin() override { destroy_(plugin_); }
  const std::vector<PortInfo>& ports() const override { return plugin_->ports(); }
  double period() const override { return plugin_->period(); }
  void step(double t, const double* in, double* out) override { plugin_->step(t, in, out); }
  Info info() const override { return plugin_->info(); }

 private:
  PluginInstance* plugin_;
  DestroyFn destroy_;
};

}  // namespace

void register_python_host(const std::filesystem::path& library) {
  register_plugin_type("python", [library](const ParamTree& params, const ComponentSpec& spec, double period) {
    static std::mutex mutex;
    static std::unique_ptr<DynamicLibrary> lib;
    static CreateFn create = nullptr;
    static DestroyFn destroy = nullptr;
    {
      std::lock_guard lock(mutex);
      if (!lib) {
        if (!std::filesystem::exists(library)) {
          throw std::runtime_error("python plugins: " + path_to_utf8(library) +
                                   " is missing (vsim was built without Python)");
        }
        try {
          lib = std::make_unique<DynamicLibrary>(library, true);
        } catch (const std::exception& e) {
          throw std::runtime_error(std::string("python plugins need the Python vsim was built with on the PATH: ") +
                                   e.what());
        }
        create = reinterpret_cast<CreateFn>(lib->symbol("vsim_python_host_create"));
        destroy = reinterpret_cast<DestroyFn>(lib->symbol("vsim_python_host_destroy"));
        if (create == nullptr || destroy == nullptr) {
          lib.reset();
          throw std::runtime_error(path_to_utf8(library) + " is not the vsim Python host");
        }
      }
    }
    char error[2048] = {};
    PluginInstance* plugin = create(&params, &spec, period, error, static_cast<int>(sizeof error));
    if (plugin == nullptr) {
      throw std::runtime_error(error);
    }
    return std::unique_ptr<PluginInstance>(std::make_unique<HostedPlugin>(plugin, destroy));
  });
}

}  // namespace vsim

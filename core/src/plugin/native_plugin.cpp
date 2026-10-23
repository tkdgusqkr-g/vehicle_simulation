#include <cstdio>
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
#else
#include <dlfcn.h>
#endif

#include "plugin_hosts.hpp"
#include "vsim/util/paths.hpp"
#include "vsim_plugin.h"

namespace vsim {

std::string shared_library_extension() {
#if defined(_WIN32)
  return ".dll";
#elif defined(__APPLE__)
  return ".dylib";
#else
  return ".so";
#endif
}

DynamicLibrary::DynamicLibrary(const std::filesystem::path& file) : file_(file) {
#ifdef _WIN32
  // Dependencies next to the library load too.
  handle_ = LoadLibraryExW(file.wstring().c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
  if (handle_ == nullptr) {
    throw std::runtime_error("cannot load '" + path_to_utf8(file) + "' (error " + std::to_string(GetLastError()) +
                             "; a 64-bit library and its DLLs are needed)");
  }
#else
  handle_ = dlopen(file.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (handle_ == nullptr) {
    const char* error = dlerror();
    throw std::runtime_error("cannot load '" + path_to_utf8(file) + "': " + (error ? error : "unknown error"));
  }
#endif
}

DynamicLibrary::~DynamicLibrary() {
#ifdef _WIN32
  FreeLibrary(static_cast<HMODULE>(handle_));
#else
  dlclose(handle_);
#endif
}

void* DynamicLibrary::symbol(const char* name) const {
#ifdef _WIN32
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(handle_), name));
#else
  return dlsym(handle_, name);
#endif
}

namespace {

void host_log(void* context, int level, const char* message) {
  const auto* name = static_cast<const std::string*>(context);
  const char* tag = level >= VSIM_LOG_ERROR ? "error" : level == VSIM_LOG_WARNING ? "warning" : "info";
  std::fprintf(stderr, "plugin %s %s: %s\n", name->c_str(), tag, message ? message : "");
}

class NativePlugin final : public PluginInstance {
 public:
  NativePlugin(const ParamTree& params, const ComponentSpec& spec, double period) : name_(spec.name) {
    const std::filesystem::path file =
        find_plugin_file(spec, params.get_string(spec.prefix + ".path", ""), {shared_library_extension()});
    library_ = std::make_unique<DynamicLibrary>(file);
    auto* entry = reinterpret_cast<VsimPluginEntry>(library_->symbol("vsim_plugin_entry"));
    if (entry == nullptr) {
      throw std::invalid_argument(path_to_utf8(file) + " has no vsim_plugin_entry (is it a vsim plugin?)");
    }
    api_ = entry();
    if (api_ == nullptr || api_->abi_version != VSIM_PLUGIN_ABI_VERSION) {
      throw std::invalid_argument(path_to_utf8(file) + ": plugin ABI version " +
                                  (api_ ? std::to_string(api_->abi_version) : std::string("?")) +
                                  ", this vsim reads version " + std::to_string(VSIM_PLUGIN_ABI_VERSION) +
                                  " (rebuild the plugin with this SDK's vsim_plugin.h)");
    }
    if (api_->describe == nullptr || api_->create == nullptr || api_->step == nullptr || api_->destroy == nullptr) {
      throw std::invalid_argument(path_to_utf8(file) + ": the plugin table is incomplete");
    }
    const VsimPort* ports = nullptr;
    int n = 0;
    if (api_->describe(&ports, &n) != 0 || n < 0 || (n > 0 && ports == nullptr)) {
      throw std::invalid_argument(path_to_utf8(file) + ": describe() failed");
    }
    // Inputs first, then outputs, each in declaration order (the arrays step() gets).
    for (const int direction : {VSIM_IN, VSIM_OUT}) {
      for (int i = 0; i < n; ++i) {
        if (ports[i].direction == direction) {
          ports_.push_back({ports[i].name ? ports[i].name : "", ports[i].unit ? ports[i].unit : "",
                            direction == VSIM_IN});
        }
      }
    }
    period_ = period > 0.0 ? period : api_->default_period;
    if (!(period_ > 0.0)) {
      throw std::invalid_argument(spec.prefix + ".period: the plugin has no default period; give one");
    }
    info_ = {"native", path_to_utf8(file), api_->name ? api_->name : "", api_->version ? api_->version : ""};
    host_ = {&name_, host_log};
    const std::string json = component_params_json(params, spec);
    self_ = api_->create(json.c_str(), period_, &host_);
    if (self_ == nullptr) {
      throw std::runtime_error("plugin " + spec.name + ": create() failed (see its log above)");
    }
  }

  ~NativePlugin() override {
    if (self_ != nullptr) {
      api_->destroy(self_);
    }
  }

  const std::vector<PortInfo>& ports() const override { return ports_; }
  double period() const override { return period_; }
  void step(double t, const double* in, double* out) override {
    const int status = api_->step(self_, t, in, out);
    if (status != 0) {
      throw std::runtime_error("plugin " + name_ + ": step() returned " + std::to_string(status) + " at t = " +
                               std::to_string(t) + " s");
    }
  }
  Info info() const override { return info_; }

 private:
  std::string name_;
  std::unique_ptr<DynamicLibrary> library_;  // outlives the instance (declared first, destroyed last)
  const VsimPluginApi* api_ = nullptr;
  VsimHost host_{};
  void* self_ = nullptr;
  std::vector<PortInfo> ports_;
  double period_ = 0.0;
  Info info_;
};

}  // namespace

std::unique_ptr<PluginInstance> make_native_plugin(const ParamTree& params, const ComponentSpec& spec, double period) {
  return std::make_unique<NativePlugin>(params, spec, period);
}

}  // namespace vsim

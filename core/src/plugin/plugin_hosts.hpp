#pragma once

// Internal: the plugin hosts built into the core, and loading of shared libraries.

#include <filesystem>
#include <memory>
#include <string>

#include "vsim/plugin/plugin.hpp"

namespace vsim {

std::unique_ptr<PluginInstance> make_native_plugin(const ParamTree& params, const ComponentSpec& spec, double period);
std::unique_ptr<PluginInstance> make_fmu_plugin(const ParamTree& params, const ComponentSpec& spec, double period);
std::unique_ptr<PluginInstance> make_external_plugin(const ParamTree& params, const ComponentSpec& spec, double period);

// The platform's shared-library extension (".dll", ".so", ".dylib").
std::string shared_library_extension();

// A loaded shared library; unloaded on destruction.
class DynamicLibrary {
 public:
  // `global`: its symbols serve libraries loaded later (an embedded
  // interpreter's extension modules need libpython's).
  explicit DynamicLibrary(const std::filesystem::path& file, bool global = false);
  ~DynamicLibrary();
  DynamicLibrary(const DynamicLibrary&) = delete;
  DynamicLibrary& operator=(const DynamicLibrary&) = delete;

  // The exported symbol, or nullptr.
  void* symbol(const char* name) const;
  const std::filesystem::path& file() const { return file_; }

 private:
  std::filesystem::path file_;
  void* handle_ = nullptr;
};

}  // namespace vsim

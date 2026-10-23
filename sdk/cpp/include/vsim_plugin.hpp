// vsim_plugin.hpp - C++ helper for vsim native plugins (header only).
//
//   #include "vsim_plugin.hpp"
//   class Gain : public vsim::plugin::Plugin {
//    public:
//     Gain(const vsim::plugin::Params& p, double period) : k_(p.number("k", 1.0)) {}
//     void step(double t, const double* in, double* out) override { out[0] = k_ * in[0]; }
//    private:
//     double k_;
//   };
//   VSIM_PLUGIN(Gain, "gain", "1.0", 0.01,
//               {"u", "-", VSIM_IN},
//               {"y", "-", VSIM_OUT})
//
// Exceptions thrown from the constructor are logged and fail the load; from
// step() they stop the run.
#ifndef VSIM_PLUGIN_HPP
#define VSIM_PLUGIN_HPP

#include <cstdlib>
#include <exception>
#include <string>

#include "vsim_plugin.h"

namespace vsim::plugin {

// The component's params (a JSON object): top-level numbers, booleans and strings.
class Params {
 public:
  explicit Params(const char* json) : json_(json ? json : "{}") {}

  double number(const std::string& key, double fallback) const {
    const std::size_t at = value_at(key);
    if (at == std::string::npos) {
      return fallback;
    }
    if (json_.compare(at, 4, "true") == 0) {
      return 1.0;
    }
    if (json_.compare(at, 5, "false") == 0) {
      return 0.0;
    }
    char* end = nullptr;
    const double v = std::strtod(json_.c_str() + at, &end);
    return end == json_.c_str() + at ? fallback : v;
  }
  std::string text(const std::string& key, const std::string& fallback) const {
    const std::size_t at = value_at(key);
    if (at == std::string::npos || json_[at] != '"') {
      return fallback;
    }
    std::string out;
    for (std::size_t i = at + 1; i < json_.size() && json_[i] != '"'; ++i) {
      out += json_[i] == '\\' && i + 1 < json_.size() ? json_[++i] : json_[i];
    }
    return out;
  }
  bool has(const std::string& key) const { return value_at(key) != std::string::npos; }

 private:
  // Position of the value of a top-level "key" (nesting depth 1).
  std::size_t value_at(const std::string& key) const {
    int depth = 0;
    for (std::size_t i = 0; i < json_.size(); ++i) {
      const char c = json_[i];
      if (c == '{' || c == '[') {
        ++depth;
      } else if (c == '}' || c == ']') {
        --depth;
      } else if (c == '"') {
        std::size_t end = i + 1;
        std::string s;
        while (end < json_.size() && json_[end] != '"') {
          s += json_[end] == '\\' && end + 1 < json_.size() ? json_[++end] : json_[end];
          ++end;
        }
        std::size_t colon = end + 1;
        while (colon < json_.size() && (json_[colon] == ' ' || json_[colon] == '\n')) {
          ++colon;
        }
        if (depth == 1 && s == key && colon < json_.size() && json_[colon] == ':') {
          std::size_t v = colon + 1;
          while (v < json_.size() && (json_[v] == ' ' || json_[v] == '\n')) {
            ++v;
          }
          return v;
        }
        i = end;
      }
    }
    return std::string::npos;
  }
  std::string json_;
};

class Plugin {
 public:
  virtual ~Plugin() = default;
  virtual void step(double t, const double* in, double* out) = 0;

  void log(int level, const std::string& message) const {
    if (host_ != nullptr && host_->log != nullptr) {
      host_->log(host_->context, level, message.c_str());
    }
  }
  const VsimHost* host_ = nullptr;
};

}  // namespace vsim::plugin

#define VSIM_PLUGIN(Class, NAME, VERSION, PERIOD, ...)                                                       \
  namespace {                                                                                               \
  const VsimPort vsim_plugin_ports_[] = {__VA_ARGS__};                                                     \
  int vsim_plugin_describe_(const VsimPort** ports, int* n) {                                              \
    *ports = vsim_plugin_ports_;                                                                           \
    *n = static_cast<int>(sizeof vsim_plugin_ports_ / sizeof vsim_plugin_ports_[0]);                      \
    return 0;                                                                                              \
  }                                                                                                        \
  void* vsim_plugin_create_(const char* json, double period, const VsimHost* host) {                      \
    try {                                                                                                  \
      auto* self = new Class(::vsim::plugin::Params(json), period);                                       \
      self->host_ = host;                                                                                  \
      return self;                                                                                         \
    } catch (const std::exception& e) {                                                                    \
      if (host != nullptr && host->log != nullptr) host->log(host->context, VSIM_LOG_ERROR, e.what());     \
      return nullptr;                                                                                      \
    }                                                                                                      \
  }                                                                                                        \
  int vsim_plugin_step_(void* self, double t, const double* in, double* out) {                            \
    try {                                                                                                  \
      static_cast<Class*>(self)->step(t, in, out);                                                         \
      return 0;                                                                                            \
    } catch (const std::exception& e) {                                                                    \
      static_cast<Class*>(self)->log(VSIM_LOG_ERROR, e.what());                                            \
      return 1;                                                                                            \
    }                                                                                                      \
  }                                                                                                        \
  void vsim_plugin_destroy_(void* self) { delete static_cast<Class*>(self); }                              \
  const VsimPluginApi vsim_plugin_api_ = {VSIM_PLUGIN_ABI_VERSION, NAME, VERSION, PERIOD,                  \
                                          vsim_plugin_describe_, vsim_plugin_create_, vsim_plugin_step_,  \
                                          vsim_plugin_destroy_};                                           \
  }                                                                                                        \
  extern "C" VSIM_PLUGIN_EXPORT const VsimPluginApi* vsim_plugin_entry(void) { return &vsim_plugin_api_; }

#endif  // VSIM_PLUGIN_HPP

#include "vsim/plugin/plugin.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <stdexcept>

#include "plugin_hosts.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/util/paths.hpp"

#ifndef VSIM_DEFAULT_PLUGIN_DIR
#define VSIM_DEFAULT_PLUGIN_DIR ""
#endif

namespace vsim {
namespace {

std::map<std::string, PluginFactory>& registry() {
  static std::map<std::string, PluginFactory> factories = {
      {"native", make_native_plugin},
      {"fmu", make_fmu_plugin},
  };
  return factories;
}

std::mutex& registry_mutex() {
  static std::mutex m;
  return m;
}

bool is_index(std::string_view s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
}

nlohmann::json scalar_json(const std::string& text) {
  if (text == "true" || text == "false") {
    return text == "true";
  }
  if (text == "null" || text == "~") {
    return nullptr;
  }
  if (!text.empty()) {
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (end == text.c_str() + text.size() && std::isfinite(v)) {
      return v;
    }
  }
  return text;
}

// --- SHA-256 (FIPS 180-4) ---------------------------------------------------------------------
class Sha256 {
 public:
  void update(const unsigned char* data, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
      block_[fill_++] = data[i];
      if (fill_ == 64) {
        compress();
        fill_ = 0;
      }
    }
    length_ += n;
  }
  std::string hex() {
    const std::uint64_t bits = length_ * 8;
    const unsigned char pad = 0x80;
    update(&pad, 1);
    const unsigned char zero = 0;
    while (fill_ != 56) {
      update(&zero, 1);
    }
    for (int i = 7; i >= 0; --i) {
      block_[fill_++] = static_cast<unsigned char>(bits >> (8 * i));
    }
    compress();
    static const char* digits = "0123456789abcdef";
    std::string out;
    for (const std::uint32_t h : h_) {
      for (int i = 7; i >= 0; --i) {
        out += digits[(h >> (4 * i)) & 0xF];
      }
    }
    return out;
  }

 private:
  static std::uint32_t rotr(std::uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
  void compress() {
    static constexpr std::array<std::uint32_t, 64> k = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16; ++i) {
      w[i] = (std::uint32_t{block_[4 * i]} << 24) | (std::uint32_t{block_[4 * i + 1]} << 16) |
             (std::uint32_t{block_[4 * i + 2]} << 8) | std::uint32_t{block_[4 * i + 3]};
    }
    for (std::size_t i = 16; i < 64; ++i) {
      const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::array<std::uint32_t, 8> v = h_;
    for (std::size_t i = 0; i < 64; ++i) {
      const std::uint32_t s1 = rotr(v[4], 6) ^ rotr(v[4], 11) ^ rotr(v[4], 25);
      const std::uint32_t ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
      const std::uint32_t t1 = v[7] + s1 + ch + k[i] + w[i];
      const std::uint32_t s0 = rotr(v[0], 2) ^ rotr(v[0], 13) ^ rotr(v[0], 22);
      const std::uint32_t maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
      const std::uint32_t t2 = s0 + maj;
      v = {t1 + t2, v[0], v[1], v[2], v[3] + t1, v[4], v[5], v[6]};
    }
    for (std::size_t i = 0; i < 8; ++i) {
      h_[i] += v[i];
    }
  }

  std::array<std::uint32_t, 8> h_ = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                     0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::array<unsigned char, 64> block_{};
  std::size_t fill_ = 0;
  std::uint64_t length_ = 0;
};

}  // namespace

std::unique_ptr<PluginInstance> create_plugin(const ParamTree& params, const ComponentSpec& spec) {
  PluginFactory factory;
  {
    std::lock_guard lock(registry_mutex());
    const auto it = registry().find(spec.type);
    if (it == registry().end()) {
      std::string known;
      for (const auto& [name, f] : registry()) {
        known += (known.empty() ? "" : ", ") + name;
      }
      throw std::invalid_argument(spec.prefix + ".type '" + spec.type + "' is not available here (available: " +
                                  known + (spec.type == "python" ? "; python plugins need vsim-core built with "
                                                                   "Python or the Python module"
                                                                 : "") +
                                  ")");
    }
    factory = it->second;
  }
  return factory(params, spec, spec.period);
}

void register_plugin_type(const std::string& type, PluginFactory factory) {
  std::lock_guard lock(registry_mutex());
  registry()[type] = std::move(factory);
}

bool plugin_type_available(const std::string& type) {
  std::lock_guard lock(registry_mutex());
  return registry().count(type) > 0;
}

std::vector<std::string> plugin_types() {
  std::lock_guard lock(registry_mutex());
  std::vector<std::string> out;
  for (const auto& [name, f] : registry()) {
    out.push_back(name);
  }
  return out;
}

std::vector<ComponentSpec> components_from_params(const ParamTree& params) {
  std::vector<std::string> names;
  const ParamTree component_tree = params.subtree("components");
  for (const auto& [key, value] : component_tree.entries()) {
    const std::string name = key.substr(0, key.find('.'));
    if (std::find(names.begin(), names.end(), name) == names.end()) {
      names.push_back(name);
    }
  }
  std::vector<ComponentSpec> out;
  const std::filesystem::path project_dir = path_from_utf8(params.get_string("project.dir", "."));
  for (const std::string& name : names) {
    ComponentSpec c;
    c.name = name;
    c.prefix = "components." + name;
    if ((params.has(c.prefix) && params.get_string(c.prefix).empty()) ||
        !params.get_bool(c.prefix + ".enabled", true)) {
      continue;  // switched off: enabled: false, or components.<name>: "" from an override
    }
    c.type = params.get_string(c.prefix + ".type");
    c.slot = params.get_string(c.prefix + ".slot", name == "vcu" || name == "driver" ? name : "");
    c.after = params.get_string(c.prefix + ".after", "vcu");
    c.period = params.get_double(c.prefix + ".period", 0.0);
    c.project_dir = project_dir;
    if (!c.slot.empty() && c.slot != "vcu" && c.slot != "driver") {
      throw std::invalid_argument(c.prefix + ".slot '" + c.slot + "' (expected vcu or driver)");
    }
    if (c.after != "driver" && c.after != "sensors" && c.after != "vcu") {
      throw std::invalid_argument(c.prefix + ".after '" + c.after + "' (expected driver, sensors or vcu)");
    }
    if (c.period < 0.0) {
      throw std::invalid_argument(c.prefix + ".period must not be negative");
    }
    out.push_back(c);
  }
  // Extra blocks after the same slot run in `order`, then by name.
  std::stable_sort(out.begin(), out.end(), [&](const ComponentSpec& a, const ComponentSpec& b) {
    return params.get_double(a.prefix + ".order", 0.0) < params.get_double(b.prefix + ".order", 0.0);
  });
  return out;
}

std::filesystem::path find_plugin_file(const ComponentSpec& spec, const std::string& path,
                                       const std::vector<std::string>& extensions, bool allow_directory) {
  if (path.empty()) {
    throw std::invalid_argument(spec.prefix + ".path is missing");
  }
  const std::filesystem::path given = path_from_utf8(path);
  std::vector<std::filesystem::path> dirs;
  if (given.is_absolute()) {
    dirs.emplace_back();
  } else {
    dirs.push_back(spec.project_dir);
    dirs.push_back(spec.project_dir / "plugins");
    if (const char* env = std::getenv("VSIM_PLUGIN_PATH")) {
#ifdef _WIN32
      const char sep = ';';
#else
      const char sep = ':';
#endif
      std::string list = env;
      std::size_t start = 0;
      while (start <= list.size()) {
        const std::size_t end = std::min(list.find(sep, start), list.size());
        if (end > start) {
          dirs.push_back(path_from_utf8(list.substr(start, end - start)));
        }
        start = end + 1;
      }
    }
    if (std::strlen(VSIM_DEFAULT_PLUGIN_DIR) > 0) {
      dirs.push_back(path_from_utf8(VSIM_DEFAULT_PLUGIN_DIR));
    }
  }
  std::vector<std::filesystem::path> tried;
  for (const auto& dir : dirs) {
    const std::filesystem::path base = dir.empty() ? given : dir / given;
    std::vector<std::filesystem::path> candidates = {base};
    if (!given.has_extension()) {
      for (const std::string& ext : extensions) {
        candidates.push_back(base.string() + ext);
        // CMake names shared libraries lib<name>.so on Unix.
        candidates.push_back(base.parent_path() / ("lib" + base.filename().string() + ext));
      }
    }
    for (const auto& c : candidates) {
      tried.push_back(c);
      std::error_code ec;
      if (std::filesystem::is_regular_file(c, ec) || (allow_directory && std::filesystem::is_directory(c, ec))) {
        return std::filesystem::absolute(c).lexically_normal();
      }
    }
  }
  std::string where;
  for (std::size_t i = 0; i < std::min<std::size_t>(tried.size(), 6); ++i) {
    where += "\n  " + path_to_utf8(tried[i]);
  }
  throw std::invalid_argument(spec.prefix + ".path: '" + path + "' not found; tried" + where +
                              (tried.size() > 6 ? "\n  ..." : "") +
                              "\n(set VSIM_PLUGIN_PATH to add folders)");
}

std::string component_params_json(const ParamTree& params, const ComponentSpec& spec) {
  nlohmann::json root = nlohmann::json::object();
  const ParamTree plugin_params = params.subtree(spec.prefix + ".params");
  for (const auto& [key, value] : plugin_params.entries()) {
    nlohmann::json* node = &root;
    std::size_t start = 0;
    while (true) {
      const std::size_t end = key.find('.', start);
      const std::string segment = key.substr(start, end == std::string::npos ? std::string::npos : end - start);
      const bool last = end == std::string::npos;
      const std::string next = last ? "" : key.substr(end + 1, key.find('.', end + 1) - end - 1);
      if (node->is_array()) {
        const auto index = static_cast<std::size_t>(std::stoul(segment));
        while (node->size() <= index) {
          node->push_back(nullptr);
        }
        node = &(*node)[index];
      } else {
        node = &(*node)[segment];
      }
      if (last) {
        *node = scalar_json(value);
        break;
      }
      if (node->is_null()) {
        *node = is_index(next) ? nlohmann::json::array() : nlohmann::json::object();
      }
      start = end + 1;
    }
  }
  return root.dump();
}

std::string sha256_file(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    return "";
  }
  Sha256 sha;
  std::array<char, 65536> buffer{};
  while (in) {
    in.read(buffer.data(), buffer.size());
    sha.update(reinterpret_cast<const unsigned char*>(buffer.data()), static_cast<std::size_t>(in.gcount()));
  }
  return sha.hex();
}

// --- PluginBlock ------------------------------------------------------------------------------

PluginBlock::PluginBlock(ComponentSpec spec) : spec_(std::move(spec)) {}
PluginBlock::~PluginBlock() = default;

PluginInstance::Info PluginBlock::info() const { return instance_ ? instance_->info() : PluginInstance::Info{}; }

void PluginBlock::init(const ParamTree& params, Context& ctx) {
  SignalBus& bus = ctx.bus();
  instance_ = create_plugin(params, spec_);
  period_ = instance_->period();
  const PluginInstance::Info information = instance_->info();
  sha256_ = information.file.empty() ? "" : sha256_file(path_from_utf8(information.file));

  // Port names the project mentions must exist (typos).
  for (const char* dir : {"in", "out"}) {
    const ParamTree mapped_ports = params.subtree(spec_.prefix + ".ports." + dir);
    for (const auto& [key, value] : mapped_ports.entries()) {
      const std::string port = key.substr(0, key.find('.'));
      const auto& ports = instance_->ports();
      const bool input = std::string(dir) == "in";
      if (std::none_of(ports.begin(), ports.end(),
                       [&](const PortInfo& p) { return p.name == port && p.input == input; })) {
        std::string names;
        for (const PortInfo& p : ports) {
          if (p.input == input) {
            names += (names.empty() ? "" : ", ") + p.name;
          }
        }
        throw std::invalid_argument(spec_.prefix + ".ports." + dir + "." + port + ": the plugin has no " +
                                    (input ? "input" : "output") + " port '" + port + "' (it has: " + names + ")");
      }
    }
  }

  connections_.clear();
  inputs_.clear();
  outputs_.clear();
  std::vector<std::string> unconnected;
  for (const PortInfo& port : instance_->ports()) {
    const std::string key = spec_.prefix + ".ports." + (port.input ? "in." : "out.") + port.name;
    Connection c;
    c.port = port.name;
    c.input = port.input;
    c.unit = params.get_string(key + ".unit", port.unit);  // the project may say what the plugin does not
    if (params.has(key + ".signal") || params.has(key + ".value")) {
      c.signal = params.get_string(key + ".signal", "");
      if (params.has(key + ".value")) {
        c.constant = params.get_double(key + ".value");
      }
    } else if (params.has(key)) {
      c.signal = params.get_string(key);
    }
    Binding b;
    if (port.input) {
      if (c.signal.empty() && !c.constant) {
        if (bus.find(port.name)) {
          c.signal = port.name;  // named after a bus signal: connects itself
        } else {
          unconnected.push_back(port.name);
          continue;
        }
      }
      if (!c.constant) {
        b.id = bus.find(c.signal);
        if (!b.id) {
          throw std::invalid_argument(key + ": no signal '" + c.signal + "' on the bus");
        }
        c.signal_unit = bus.info(*b.id).unit;
        try {
          b.conversion = unit_conversion(c.signal_unit, c.unit);
        } catch (const std::invalid_argument& e) {
          throw std::invalid_argument(key + ": " + e.what());
        }
      } else {
        b.constant = *c.constant;
      }
      inputs_.push_back(b);
    } else {
      if (c.signal.empty()) {
        c.signal = bus.find(port.name) ? port.name : "plugin." + spec_.name + "." + port.name;
      }
      b.id = bus.find(c.signal);
      if (!b.id) {
        b.id = bus.declare(c.signal, c.unit, "output of plugin " + spec_.name);
      }
      c.signal_unit = bus.info(*b.id).unit;
      try {
        b.conversion = unit_conversion(c.unit, c.signal_unit);
      } catch (const std::invalid_argument& e) {
        throw std::invalid_argument(key + ": " + e.what());
      }
      outputs_.push_back(b);
    }
    connections_.push_back(c);
  }
  if (!unconnected.empty()) {
    std::string names;
    for (const std::string& n : unconnected) {
      names += (names.empty() ? "" : ", ") + n;
    }
    throw std::invalid_argument(spec_.prefix + ".ports.in: inputs not connected: " + names +
                                " (give each a signal or {value: ...})");
  }
  in_.assign(inputs_.size(), 0.0);
  out_.assign(outputs_.size(), 0.0);
  stats_ = {};
}

void PluginBlock::step(double t, SignalBus& bus) {
  for (std::size_t i = 0; i < inputs_.size(); ++i) {
    const Binding& b = inputs_[i];
    in_[i] = b.id ? b.conversion.apply(bus.get(*b.id)) : b.constant;
  }
  const auto start = std::chrono::steady_clock::now();
  instance_->step(t, in_.data(), out_.data());
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  ++stats_.calls;
  stats_.total_s += elapsed;
  stats_.max_s = std::max(stats_.max_s, elapsed);
  for (std::size_t i = 0; i < outputs_.size(); ++i) {
    if (!std::isfinite(out_[i])) {
      std::string port;
      std::size_t k = 0;
      for (const Connection& c : connections_) {
        if (!c.input && k++ == i) {
          port = c.port;
        }
      }
      throw std::runtime_error("plugin " + spec_.name + " output '" + port + "' is not finite at t = " +
                               std::to_string(t) + " s");
    }
    bus.set(*outputs_[i].id, outputs_[i].conversion.apply(out_[i]));
  }
}

}  // namespace vsim

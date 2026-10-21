#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace vsim {

// Hierarchical parameters flattened to dotted keys ("vehicle.tire.mu").
// Sequence items use their index as a key segment ("logging.signals.0").
// Values are kept as scalar text and converted on access, so any key can be
// overridden from the command line, Python or a sweep with a plain string.
class ParamTree {
 public:
  using Entries = std::map<std::string, std::string, std::less<>>;

  static ParamTree from_yaml_file(const std::filesystem::path& path);
  static ParamTree from_yaml_string(std::string_view yaml);

  bool has(std::string_view key) const;
  // True if any key lives under "prefix.".
  bool has_prefix(std::string_view prefix) const;

  double get_double(std::string_view key) const;
  double get_double(std::string_view key, double fallback) const;
  std::int64_t get_int(std::string_view key) const;
  std::int64_t get_int(std::string_view key, std::int64_t fallback) const;
  bool get_bool(std::string_view key, bool fallback) const;
  std::string get_string(std::string_view key) const;
  std::string get_string(std::string_view key, std::string_view fallback) const;
  // Items "key.0", "key.1", ... in index order; empty if absent.
  std::vector<std::string> get_string_list(std::string_view key) const;

  // Sets a scalar. Anything stored under "key." and any scalar stored at a
  // parent of key is removed, so the tree stays representable as YAML.
  void set(std::string_view key, std::string_view value);
  // Copies every entry of `other` under `prefix` (empty prefix = root).
  void merge(const ParamTree& other, std::string_view prefix = {});
  // Entries under "prefix." with the prefix stripped.
  ParamTree subtree(std::string_view prefix) const;

  const Entries& entries() const { return entries_; }
  bool empty() const { return entries_.empty(); }
  std::string to_yaml() const;

 private:
  const std::string& raw(std::string_view key) const;

  Entries entries_;
};

}  // namespace vsim

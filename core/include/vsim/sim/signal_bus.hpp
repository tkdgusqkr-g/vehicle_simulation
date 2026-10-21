#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace vsim {

using SignalId = std::size_t;

struct SignalInfo {
  std::string name;
  std::string unit;
  std::string description;
};

// Shared store of named scalar signals.
// Names are resolved to ids once during init; the run loop only uses ids.
class SignalBus {
 public:
  // Declares a signal and returns its id. Declaring an existing name again
  // returns the same id; declaring it with a different unit is an error.
  SignalId declare(std::string_view name, std::string_view unit, std::string_view description = {});

  std::optional<SignalId> find(std::string_view name) const;
  // Like find(), but throws std::out_of_range naming the missing signal.
  SignalId require(std::string_view name) const;

  double get(SignalId id) const { return values_[id]; }
  void set(SignalId id, double value) { values_[id] = value; }

  std::size_t size() const { return values_.size(); }
  const SignalInfo& info(SignalId id) const { return infos_[id]; }
  const std::vector<double>& values() const { return values_; }

  // Ids of all signals whose name matches the glob pattern, in declaration order.
  std::vector<SignalId> match(std::string_view pattern) const;

 private:
  std::vector<SignalInfo> infos_;
  std::vector<double> values_;
  std::unordered_map<std::string, SignalId> index_;
};

// '*' matches any run of characters (including '.'), '?' matches one character.
bool glob_match(std::string_view pattern, std::string_view text);

}  // namespace vsim

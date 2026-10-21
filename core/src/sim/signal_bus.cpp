#include "vsim/sim/signal_bus.hpp"

#include <stdexcept>

namespace vsim {

SignalId SignalBus::declare(std::string_view name, std::string_view unit,
                            std::string_view description) {
  if (name.empty()) {
    throw std::invalid_argument("signal name must not be empty");
  }
  if (const auto it = index_.find(std::string(name)); it != index_.end()) {
    const SignalInfo& existing = infos_[it->second];
    if (existing.unit != unit) {
      throw std::invalid_argument("signal '" + std::string(name) + "' declared with unit '" +
                                  std::string(unit) + "', but it already has unit '" +
                                  existing.unit + "'");
    }
    return it->second;
  }
  const SignalId id = values_.size();
  infos_.push_back({std::string(name), std::string(unit), std::string(description)});
  values_.push_back(0.0);
  index_.emplace(std::string(name), id);
  return id;
}

std::optional<SignalId> SignalBus::find(std::string_view name) const {
  const auto it = index_.find(std::string(name));
  if (it == index_.end()) {
    return std::nullopt;
  }
  return it->second;
}

SignalId SignalBus::require(std::string_view name) const {
  if (const auto id = find(name)) {
    return *id;
  }
  throw std::out_of_range("unknown signal '" + std::string(name) + "'");
}

std::vector<SignalId> SignalBus::match(std::string_view pattern) const {
  std::vector<SignalId> ids;
  for (SignalId id = 0; id < infos_.size(); ++id) {
    if (glob_match(pattern, infos_[id].name)) {
      ids.push_back(id);
    }
  }
  return ids;
}

bool glob_match(std::string_view pattern, std::string_view text) {
  // Iterative matching that backtracks to the most recent '*'.
  std::size_t p = 0;
  std::size_t t = 0;
  std::size_t star = std::string_view::npos;
  std::size_t star_text = 0;
  while (t < text.size()) {
    if (p < pattern.size() && (pattern[p] == '?' || pattern[p] == text[t])) {
      ++p;
      ++t;
    } else if (p < pattern.size() && pattern[p] == '*') {
      star = p++;
      star_text = t;
    } else if (star != std::string_view::npos) {
      p = star + 1;
      t = ++star_text;
    } else {
      return false;
    }
  }
  while (p < pattern.size() && pattern[p] == '*') {
    ++p;
  }
  return p == pattern.size();
}

}  // namespace vsim

#pragma once

#include <array>
#include <cstddef>
#include <string_view>

namespace vsim {

class ParamTree;

// Non-ideality categories (design doc §10.1). Each one is switched between
// "ideal" and "real" independently so a phenomenon can be traced to its cause.
enum class RealismCategory : std::size_t {
  Environment,
  Sensors,
  Actuators,
  Electrical,
  Comms,
  Variation,
};

inline constexpr std::array<RealismCategory, 6> kRealismCategories = {
    RealismCategory::Environment, RealismCategory::Sensors,    RealismCategory::Actuators,
    RealismCategory::Electrical,  RealismCategory::Comms,      RealismCategory::Variation,
};

std::string_view to_string(RealismCategory category);

class RealismConfig {
 public:
  // Reads "realism.<category>: ideal | real". Missing categories are ideal.
  static RealismConfig from_params(const ParamTree& params);

  bool is_real(RealismCategory category) const {
    return real_[static_cast<std::size_t>(category)];
  }
  void set(RealismCategory category, bool real) { real_[static_cast<std::size_t>(category)] = real; }

 private:
  std::array<bool, kRealismCategories.size()> real_{};
};

}  // namespace vsim

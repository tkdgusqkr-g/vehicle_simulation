#include "vsim/sim/realism.hpp"

#include <stdexcept>
#include <string>

#include "vsim/sim/params.hpp"

namespace vsim {

std::string_view to_string(RealismCategory category) {
  switch (category) {
    case RealismCategory::Environment:
      return "environment";
    case RealismCategory::Sensors:
      return "sensors";
    case RealismCategory::Actuators:
      return "actuators";
    case RealismCategory::Electrical:
      return "electrical";
    case RealismCategory::Comms:
      return "comms";
    case RealismCategory::Variation:
      return "variation";
  }
  return "unknown";
}

RealismConfig RealismConfig::from_params(const ParamTree& params) {
  RealismConfig config;
  const ParamTree realism = params.subtree("realism");
  for (const auto& [key, value] : realism.entries()) {
    bool known = false;
    for (const RealismCategory category : kRealismCategories) {
      if (key != to_string(category)) {
        continue;
      }
      known = true;
      if (value == "real") {
        config.set(category, true);
      } else if (value != "ideal") {
        throw std::invalid_argument("realism." + key + ": expected 'ideal' or 'real', got '" +
                                    value + "'");
      }
    }
    if (!known) {
      throw std::invalid_argument(
          "unknown realism category '" + key +
          "' (expected environment, sensors, actuators, electrical, comms or variation)");
    }
  }
  return config;
}

}  // namespace vsim

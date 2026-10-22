#include <stdexcept>
#include <string>

#include "vsim/models/tire/tire.hpp"

namespace vsim {

std::unique_ptr<TireModel> make_tire(const ParamTree& params, std::string_view axle,
                                     const std::filesystem::path& /*project_dir*/) {
  const std::string model = params.get_string(
      "vehicle.tire." + std::string(axle) + ".model", params.get_string("vehicle.tire.model", "simple"));
  if (model == "simple") {
    return std::make_unique<SimpleTire>(SimpleTire::params_from(params, axle));
  }
  throw std::invalid_argument("vehicle.tire.model '" + model + "' is not available (expected simple)");
}

}  // namespace vsim

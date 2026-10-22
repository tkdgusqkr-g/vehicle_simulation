#include <iostream>
#include <stdexcept>
#include <string>

#include "vsim/models/tire/tire.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {

std::unique_ptr<TireModel> make_tire(const ParamTree& params, std::string_view axle,
                                     const std::filesystem::path& project_dir) {
  const std::string specific = "vehicle.tire." + std::string(axle) + ".";
  const auto get = [&](const char* key, std::string_view fallback) {
    return params.get_string(specific + key, params.get_string(std::string("vehicle.tire.") + key, fallback));
  };
  const std::string model = get("model", "simple");
  if (model == "simple") {
    return std::make_unique<SimpleTire>(SimpleTire::params_from(params, axle));
  }
  if (model == "mf") {
    const std::string file = get("file", "");
    if (file.empty()) {
      throw std::invalid_argument("vehicle.tire.model mf needs vehicle.tire.file (a .tir file)");
    }
    std::filesystem::path path = path_from_utf8(file);
    if (path.is_relative()) {
      path = project_dir / path;
    }
    auto tire = std::make_unique<MagicFormulaTire>(MagicFormulaTire::from_file(path));
    for (const std::string& note : tire->notes()) {
      std::clog << "note: " << path_to_utf8(path) << ": " << note << "\n";
    }
    return tire;
  }
  throw std::invalid_argument("vehicle.tire.model '" + model + "' is not available (expected simple or mf)");
}

}  // namespace vsim

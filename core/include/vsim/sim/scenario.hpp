#pragma once

#include <filesystem>
#include <map>
#include <string>

#include "vsim/sim/params.hpp"

namespace vsim {

// A scenario file plus the vehicle project it references, merged into one
// parameter tree:
//   project.*   project.yaml (name, schema_version)
//   vehicle.*   the project's vehicle file
//   control.*   the project's control file
//   everything else from the scenario file, then its `overrides:` map,
//   then the overrides passed to load().
struct Scenario {
  using Overrides = std::map<std::string, std::string>;

  std::string name;
  std::filesystem::path file;
  std::filesystem::path project_dir;
  ParamTree params;

  static Scenario load(const std::filesystem::path& file, const Overrides& overrides = {});
};

}  // namespace vsim

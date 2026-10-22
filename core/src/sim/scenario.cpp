#include "vsim/sim/scenario.hpp"

#include <stdexcept>
#include <string>

#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

constexpr std::int64_t kProjectSchemaVersion = 1;

bool starts_with(std::string_view text, std::string_view prefix) {
  return text.substr(0, prefix.size()) == prefix;
}

}  // namespace

Scenario Scenario::load(const std::filesystem::path& file, const Overrides& overrides) {
  const std::filesystem::path scenario_file = std::filesystem::absolute(file).lexically_normal();
  const ParamTree scenario = ParamTree::from_yaml_file(scenario_file);
  if (!scenario.has("project")) {
    throw std::invalid_argument("scenario '" + path_to_utf8(scenario_file) +
                                "' has no 'project' (path to the vehicle project directory)");
  }

  std::filesystem::path project_dir =
      (scenario_file.parent_path() / path_from_utf8(scenario.get_string("project")))
          .lexically_normal();
  if (project_dir.filename().empty()) {
    // "a/b/.." normalizes to "a/" with a trailing separator.
    project_dir = project_dir.parent_path();
  }
  const ParamTree project = ParamTree::from_yaml_file(project_dir / "project.yaml");
  const std::int64_t schema = project.get_int("schema_version");
  if (schema != kProjectSchemaVersion) {
    throw std::invalid_argument("project '" + path_to_utf8(project_dir) + "' has schema_version " +
                                std::to_string(schema) + "; this build reads version " +
                                std::to_string(kProjectSchemaVersion));
  }

  ParamTree merged;
  merged.set("project.name", project.get_string("name", path_to_utf8(project_dir.filename())));
  merged.set("project.schema_version", std::to_string(schema));
  // Models resolve file references (tire files, CAD) against this.
  merged.set("project.dir", path_to_utf8(project_dir));
  for (const char* section : {"vehicle", "control"}) {
    if (project.has(section)) {
      merged.merge(
          ParamTree::from_yaml_file(project_dir / path_from_utf8(project.get_string(section))),
          section);
    }
  }
  for (const auto& [key, value] : scenario.entries()) {
    if (key != "project" && !starts_with(key, "overrides.")) {
      merged.set(key, value);
    }
  }
  const ParamTree scenario_overrides = scenario.subtree("overrides");
  for (const auto& [key, value] : scenario_overrides.entries()) {
    merged.set(key, value);
  }
  for (const auto& [key, value] : overrides) {
    merged.set(key, value);
  }

  Scenario result;
  result.name = merged.get_string("name", path_to_utf8(scenario_file.stem()));
  result.file = scenario_file;
  result.project_dir = project_dir;
  result.params = std::move(merged);
  return result;
}

}  // namespace vsim

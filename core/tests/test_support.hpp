#pragma once

#include <string>

#include "vsim/sim/scenario.hpp"
#include "vsim/sim/simulation.hpp"

namespace vsim::test {

inline std::string example_scenario(const std::string& name) {
  return std::string(VSIM_SOURCE_DIR) + "/templates/projects/example_fs_ev/scenarios/" + name +
         ".yaml";
}

inline Scenario load_example(const std::string& name, const Scenario::Overrides& overrides = {}) {
  return Scenario::load(example_scenario(name), overrides);
}

inline double signal(const Simulation& sim, const std::string& name) {
  return sim.bus().get(sim.bus().require(name));
}

}  // namespace vsim::test

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "vsim/sim/params.hpp"

namespace vsim {
namespace {

constexpr const char* kYaml = R"(
vehicle:
  mass_kg: 280
  tire:
    mu: 1.6
  powertrain:
    motors: [fl, fr, rl, rr]
flag: yes
name: test
empty:
)";

TEST(ParamTree, FlattensMapsAndSequences) {
  const ParamTree p = ParamTree::from_yaml_string(kYaml);
  EXPECT_EQ(p.get_double("vehicle.mass_kg"), 280.0);
  EXPECT_EQ(p.get_int("vehicle.mass_kg"), 280);
  EXPECT_DOUBLE_EQ(p.get_double("vehicle.tire.mu"), 1.6);
  EXPECT_EQ(p.get_string_list("vehicle.powertrain.motors"),
            (std::vector<std::string>{"fl", "fr", "rl", "rr"}));
  EXPECT_TRUE(p.get_bool("flag", false));
  EXPECT_EQ(p.get_string("name"), "test");
  EXPECT_FALSE(p.has("empty"));
  EXPECT_TRUE(p.has_prefix("vehicle.tire"));
  EXPECT_FALSE(p.has_prefix("vehicle.ti"));
}

TEST(ParamTree, FallbacksAndErrors) {
  const ParamTree p = ParamTree::from_yaml_string(kYaml);
  EXPECT_EQ(p.get_double("vehicle.missing", 3.5), 3.5);
  EXPECT_THROW(p.get_double("vehicle.missing"), std::out_of_range);
  EXPECT_THROW(p.get_double("name"), std::invalid_argument);
  EXPECT_THROW(p.get_int("vehicle.tire.mu"), std::invalid_argument);
  EXPECT_THROW(p.get_bool("name", false), std::invalid_argument);
}

TEST(ParamTree, SpecialNumbers) {
  const ParamTree p = ParamTree::from_yaml_string("a: .inf\nb: -.inf\nc: +2.5\nd: 1e-3\n");
  EXPECT_TRUE(std::isinf(p.get_double("a")));
  EXPECT_LT(p.get_double("b"), 0.0);
  EXPECT_EQ(p.get_double("c"), 2.5);
  EXPECT_EQ(p.get_double("d"), 1e-3);
}

TEST(ParamTree, SetReplacesSubtreesAndParents) {
  ParamTree p = ParamTree::from_yaml_string(kYaml);
  p.set("vehicle.tire", "none");  // scalar replaces the map below it
  EXPECT_FALSE(p.has("vehicle.tire.mu"));
  EXPECT_EQ(p.get_string("vehicle.tire"), "none");
  p.set("vehicle.tire.mu", "1.4");  // and a key below replaces the scalar again
  EXPECT_FALSE(p.has("vehicle.tire"));
  EXPECT_EQ(p.get_double("vehicle.tire.mu"), 1.4);
}

TEST(ParamTree, MergeAndSubtree) {
  ParamTree p;
  p.merge(ParamTree::from_yaml_string("mu: 1.5\nB: 10\n"), "vehicle.tire");
  EXPECT_EQ(p.get_double("vehicle.tire.B"), 10.0);
  const ParamTree tire = p.subtree("vehicle.tire");
  EXPECT_EQ(tire.entries().size(), 2u);
  EXPECT_EQ(tire.get_double("mu"), 1.5);
}

TEST(ParamTree, YamlRoundTrip) {
  ParamTree p = ParamTree::from_yaml_string(kYaml);
  p.set("logging.signals.0", "veh.*");
  p.set("logging.signals.1", "drv.*");
  const ParamTree again = ParamTree::from_yaml_string(p.to_yaml());
  EXPECT_EQ(again.entries(), p.entries());
}

TEST(ParamTree, ListFromSingleScalar) {
  const ParamTree p = ParamTree::from_yaml_string("signals: veh.*\n");
  EXPECT_EQ(p.get_string_list("signals"), (std::vector<std::string>{"veh.*"}));
  EXPECT_TRUE(p.get_string_list("missing").empty());
}

TEST(ParamTree, RejectsNonMappingRoot) {
  EXPECT_THROW(ParamTree::from_yaml_string("just a scalar"), std::invalid_argument);
  EXPECT_THROW(ParamTree::from_yaml_string("a: [1, 2"), std::runtime_error);
}

}  // namespace
}  // namespace vsim

#include <gtest/gtest.h>

#include <stdexcept>

#include "vsim/sim/signal_bus.hpp"

namespace vsim {
namespace {

TEST(SignalBus, DeclareIsIdempotentAndChecksUnits) {
  SignalBus bus;
  const SignalId a = bus.declare("veh.chassis.vx", "m/s", "speed");
  EXPECT_EQ(bus.declare("veh.chassis.vx", "m/s"), a);
  EXPECT_THROW(bus.declare("veh.chassis.vx", "km/h"), std::invalid_argument);
  EXPECT_THROW(bus.declare("", "m"), std::invalid_argument);
  EXPECT_EQ(bus.size(), 1u);
  EXPECT_EQ(bus.info(a).description, "speed");
}

TEST(SignalBus, GetSetFindRequire) {
  SignalBus bus;
  const SignalId id = bus.declare("drv.cmd.throttle", "-");
  EXPECT_EQ(bus.get(id), 0.0);
  bus.set(id, 0.5);
  EXPECT_EQ(bus.get(id), 0.5);
  EXPECT_EQ(bus.find("drv.cmd.throttle"), id);
  EXPECT_FALSE(bus.find("missing").has_value());
  EXPECT_THROW(bus.require("missing"), std::out_of_range);
}

TEST(SignalBus, MatchKeepsDeclarationOrder) {
  SignalBus bus;
  const SignalId fz_fl = bus.declare("veh.tire.fl.fz", "N");
  bus.declare("veh.tire.fl.fy", "N");
  const SignalId fz_rr = bus.declare("veh.tire.rr.fz", "N");
  bus.declare("drv.cmd.steer", "rad");
  EXPECT_EQ(bus.match("veh.tire.*.fz"), (std::vector<SignalId>{fz_fl, fz_rr}));
  EXPECT_EQ(bus.match("veh.*").size(), 3u);
  EXPECT_EQ(bus.match("*").size(), 4u);
  EXPECT_TRUE(bus.match("pt.*").empty());
}

TEST(GlobMatch, Wildcards) {
  EXPECT_TRUE(glob_match("veh.*", "veh.chassis.vx"));
  EXPECT_TRUE(glob_match("*.fz", "veh.tire.fl.fz"));
  EXPECT_TRUE(glob_match("veh.tire.f?.fz", "veh.tire.fl.fz"));
  EXPECT_FALSE(glob_match("veh.tire.f?.fz", "veh.tire.rl.fz"));
  EXPECT_TRUE(glob_match("a*b*c", "aXXbYYc"));
  EXPECT_FALSE(glob_match("a*b*c", "aXXbYY"));
  EXPECT_TRUE(glob_match("exact", "exact"));
  EXPECT_FALSE(glob_match("exact", "exactly"));
}

}  // namespace
}  // namespace vsim

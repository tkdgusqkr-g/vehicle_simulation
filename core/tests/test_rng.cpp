#include <gtest/gtest.h>

#include <cmath>

#include "vsim/sim/realism.hpp"
#include "vsim/sim/params.hpp"
#include "vsim/sim/rng.hpp"

namespace vsim {
namespace {

TEST(Rng, Fnv1aReferenceValues) {
  EXPECT_EQ(fnv1a64(""), 0xcbf29ce484222325ULL);
  EXPECT_EQ(fnv1a64("a"), 0xaf63dc4c8601ec8cULL);
}

TEST(Rng, StreamsAreReproducibleAndIndependent) {
  EXPECT_EQ(derive_seed(1, "sensors.imu"), derive_seed(1, "sensors.imu"));
  EXPECT_NE(derive_seed(1, "sensors.imu"), derive_seed(2, "sensors.imu"));
  EXPECT_NE(derive_seed(1, "sensors.imu"), derive_seed(1, "environment.road"));

  Rng a(derive_seed(7, "x"));
  Rng b(derive_seed(7, "x"));
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(a.next_u64(), b.next_u64());
  }
}

TEST(Rng, UniformAndNormalStatistics) {
  Rng rng(42);
  constexpr int kSamples = 200000;
  double sum_u = 0.0;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double u = rng.uniform();
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
    sum_u += u;
    const double n = rng.normal();
    sum += n;
    sum_sq += n * n;
  }
  const double mean = sum / kSamples;
  EXPECT_NEAR(sum_u / kSamples, 0.5, 0.005);
  EXPECT_NEAR(mean, 0.0, 0.01);
  EXPECT_NEAR(std::sqrt(sum_sq / kSamples - mean * mean), 1.0, 0.01);
}

TEST(Realism, DefaultsToIdealAndParsesCategories) {
  const RealismConfig defaults = RealismConfig::from_params(ParamTree{});
  for (const RealismCategory category : kRealismCategories) {
    EXPECT_FALSE(defaults.is_real(category));
  }
  const RealismConfig config = RealismConfig::from_params(
      ParamTree::from_yaml_string("realism: {sensors: real, comms: ideal}\n"));
  EXPECT_TRUE(config.is_real(RealismCategory::Sensors));
  EXPECT_FALSE(config.is_real(RealismCategory::Comms));
}

TEST(Realism, RejectsTyposAndBadValues) {
  EXPECT_THROW(RealismConfig::from_params(ParamTree::from_yaml_string("realism: {sensor: real}\n")),
               std::invalid_argument);
  EXPECT_THROW(RealismConfig::from_params(ParamTree::from_yaml_string("realism: {sensors: on}\n")),
               std::invalid_argument);
}

}  // namespace
}  // namespace vsim

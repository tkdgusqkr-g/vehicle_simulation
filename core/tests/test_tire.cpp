#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "vsim/models/tire/tire.hpp"

namespace vsim {
namespace {

constexpr double kLoad = 800.0;

SimpleTire::Params base() {
  SimpleTire::Params p;
  p.nominal_load = 700.0;
  p.mu_x = 1.7;
  p.mu_y = 1.5;
  return p;
}

double peak_of(const SimpleTire& tire, bool longitudinal, double* at = nullptr) {
  double best = 0.0;
  for (int i = 1; i <= 4000; ++i) {
    const double slip = i * 0.0005;
    const TireForces f = longitudinal ? tire.evaluate({kLoad, slip, 0.0})
                                      : tire.evaluate({kLoad, 0.0, slip});
    const double value = longitudinal ? f.fx : f.fy;
    if (value > best) {
      best = value;
      if (at != nullptr) {
        *at = slip;
      }
    }
  }
  return best;
}

TEST(SimpleTire, PureSlipPeaksAtMuTimesLoad) {
  const SimpleTire tire(base());
  EXPECT_NEAR(peak_of(tire, true), 1.7 * kLoad, 1e-3 * kLoad);
  EXPECT_NEAR(peak_of(tire, false), 1.5 * kLoad, 1e-3 * kLoad);
}

TEST(SimpleTire, InitialSlopeIsBcd) {
  const SimpleTire tire(base());
  const SimpleTire::Params& p = tire.params();
  const double small = 1e-6;
  EXPECT_NEAR(tire.evaluate({kLoad, small, 0.0}).fx / small, p.bx * p.cx * p.mu_x * kLoad,
              1e-3 * p.bx * p.cx * p.mu_x * kLoad);
  EXPECT_NEAR(tire.evaluate({kLoad, 0.0, small}).fy / small, p.by * p.cy * p.mu_y * kLoad,
              1e-3 * p.by * p.cy * p.mu_y * kLoad);
  EXPECT_DOUBLE_EQ(tire.longitudinal_stiffness(kLoad), p.bx * p.cx * p.mu_x * kLoad);
  EXPECT_DOUBLE_EQ(tire.cornering_stiffness(kLoad), p.by * p.cy * p.mu_y * kLoad);
  // The linear branch at exactly zero slip matches the curve.
  EXPECT_DOUBLE_EQ(tire.evaluate({kLoad, 0.0, 0.0}).fx, 0.0);
}

TEST(SimpleTire, ForcesAreOddInSlip) {
  const SimpleTire tire(base());
  const TireForces a = tire.evaluate({kLoad, 0.07, 0.05});
  const TireForces b = tire.evaluate({kLoad, -0.07, -0.05});
  EXPECT_DOUBLE_EQ(a.fx, -b.fx);
  EXPECT_DOUBLE_EQ(a.fy, -b.fy);
  EXPECT_DOUBLE_EQ(a.mz, -b.mz);
}

TEST(SimpleTire, CombinedSlipStaysInsideTheFrictionEllipse) {
  const SimpleTire tire(base());
  for (double kappa = -0.5; kappa <= 0.5; kappa += 0.01) {
    for (double alpha = -0.4; alpha <= 0.4; alpha += 0.01) {
      const TireForces f = tire.evaluate({kLoad, kappa, alpha});
      const double ellipse = std::pow(f.fx / (1.7 * kLoad), 2) + std::pow(f.fy / (1.5 * kLoad), 2);
      ASSERT_LE(ellipse, 1.0 + 1e-9) << kappa << " " << alpha;
    }
  }
  // Driving force costs lateral force.
  EXPECT_LT(tire.evaluate({kLoad, 0.1, 0.05}).fy, tire.evaluate({kLoad, 0.0, 0.05}).fy);
}

TEST(SimpleTire, AligningTorqueCentresAndFadesBeforePeakGrip) {
  const SimpleTire tire(base());
  double peak_alpha = 0.0;
  peak_of(tire, false, &peak_alpha);
  const TireForces small = tire.evaluate({kLoad, 0.0, 0.02});
  EXPECT_GT(small.fy, 0.0);
  EXPECT_LT(small.mz, 0.0);

  double mz_peak_alpha = 0.0;
  double mz_peak = 0.0;
  for (double alpha = 0.001; alpha < 0.5; alpha += 0.001) {
    const double mz = -tire.evaluate({kLoad, 0.0, alpha}).mz;
    if (mz > mz_peak) {
      mz_peak = mz;
      mz_peak_alpha = alpha;
    }
  }
  EXPECT_LT(mz_peak_alpha, peak_alpha);
  EXPECT_EQ(tire.evaluate({kLoad, 0.0, 0.5}).mz, 0.0);
}

TEST(SimpleTire, LoadSensitivityLowersFrictionCoefficient) {
  SimpleTire::Params p = base();
  p.load_sensitivity = -0.2;
  const SimpleTire tire(p);
  EXPECT_LT(tire.cornering_stiffness(1400.0) / 1400.0, tire.cornering_stiffness(700.0) / 700.0);
  EXPECT_NEAR(tire.cornering_stiffness(700.0), p.by * p.cy * p.mu_y * 700.0, 1e-9);
}

TEST(SimpleTire, NoLoadNoForce) {
  const SimpleTire tire(base());
  const TireForces f = tire.evaluate({-10.0, 0.2, 0.1});
  EXPECT_EQ(f.fx, 0.0);
  EXPECT_EQ(f.fy, 0.0);
  EXPECT_EQ(f.mz, 0.0);
}

TEST(SimpleTire, RejectsCurvesWithoutAPeak) {
  SimpleTire::Params p = base();
  p.cy = 0.9;
  EXPECT_THROW(SimpleTire{p}, std::invalid_argument);
}

TEST(TireFactory, ReadsAxleOverrides) {
  const ParamTree params = ParamTree::from_yaml_string(R"(
vehicle:
  tire:
    model: simple
    simple: {mu_y: 1.5, By: 11}
    rear:
      simple: {By: 14}
)");
  const auto front = make_tire(params, "front", {});
  const auto rear = make_tire(params, "rear", {});
  EXPECT_DOUBLE_EQ(dynamic_cast<const SimpleTire&>(*front).params().by, 11.0);
  EXPECT_DOUBLE_EQ(dynamic_cast<const SimpleTire&>(*rear).params().by, 14.0);
  EXPECT_DOUBLE_EQ(dynamic_cast<const SimpleTire&>(*rear).params().mu_y, 1.5);

  const ParamTree unknown = ParamTree::from_yaml_string("vehicle: {tire: {model: brush}}");
  EXPECT_THROW(make_tire(unknown, "front", {}), std::invalid_argument);
}

}  // namespace
}  // namespace vsim

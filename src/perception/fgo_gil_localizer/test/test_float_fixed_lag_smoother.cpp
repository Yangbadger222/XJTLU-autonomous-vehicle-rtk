#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "fgo_gil_localizer/float_fixed_lag_smoother.hpp"

namespace fgo_gil_localizer
{
namespace
{

EcefState stateAt(const double stamp_s, const Vec3 & position, const Vec3 & velocity = {})
{
  EcefState state;
  state.stamp_s = stamp_s;
  state.position_ecef_m = position;
  state.velocity_ecef_m_s = velocity;
  state.orientation_ecef_body = {};
  return state;
}

DoubleDifferenceMeasurement syntheticDd(
  const std::uint16_t target_prn,
  const Vec3 & reference_satellite,
  const Vec3 & target_satellite,
  const Vec3 & true_rover,
  const Vec3 & base,
  const double ambiguity_m)
{
  DoubleDifferenceMeasurement measurement;
  measurement.time = {2400, 100.0};
  measurement.group = {GnssConstellation::Gps, 0, false};
  measurement.reference = {GnssConstellation::Gps, 3};
  measurement.target = {GnssConstellation::Gps, target_prn};
  measurement.ambiguity_key = {
    measurement.group, measurement.reference, measurement.target,
    {static_cast<std::uint64_t>(target_prn), 100U + target_prn, 3U, 103U}};
  measurement.reference_position_ecef_m = reference_satellite;
  measurement.target_position_ecef_m = target_satellite;
  measurement.base_position_ecef_m = base;
  measurement.code_dd_m =
    norm(target_satellite - true_rover) - norm(target_satellite - base) -
    norm(reference_satellite - true_rover) + norm(reference_satellite - base);
  measurement.carrier_dd_m = measurement.code_dd_m + ambiguity_m;
  measurement.code_sigma_m = 0.02;
  measurement.carrier_sigma_m = 0.002;
  measurement.code_valid = true;
  measurement.carrier_valid = true;
  return measurement;
}

std::vector<DoubleDifferenceMeasurement> syntheticGnss(
  const Vec3 & true_rover,
  const Vec3 & base)
{
  const Vec3 reference{26500000.0, 0.0, 4000000.0};
  return {
    syntheticDd(7, reference, {21000000.0, 11000000.0, 9000000.0}, true_rover, base, 10.0),
    syntheticDd(8, reference, {22000000.0, -12000000.0, 7000000.0}, true_rover, base, -4.0),
    syntheticDd(9, reference, {20500000.0, 4000000.0, -13000000.0}, true_rover, base, 7.0),
    syntheticDd(10, reference, {28500000.0, -2000000.0, 1000000.0}, true_rover, base, 2.0)};
}

StateFactorNoise loosePositionPrior()
{
  StateFactorNoise noise;
  noise.position_m = 100.0;
  noise.rotation_rad = 1.0e-4;
  noise.velocity_m_s = 1.0e-3;
  noise.accelerometer_bias_m_s2 = 1.0e-4;
  noise.gyroscope_bias_rad_s = 1.0e-5;
  return noise;
}

TEST(FloatFixedLagSmoother, GnssFloatStateAndAmbiguitiesConverge)
{
  const Vec3 truth{6378137.0, 20.0, -10.0};
  const Vec3 base{6378137.0, 0.0, 0.0};
  EcefState initial = stateAt(100.0, truth + Vec3{1.5, -1.0, 0.8});
  FloatFixedLagSmoother smoother;
  ASSERT_TRUE(smoother.addState(1, initial));
  ASSERT_TRUE(smoother.addStatePrior(1, initial, loosePositionPrior()));
  const auto measurements = syntheticGnss(truth, base);
  ASSERT_TRUE(smoother.addGnssFactors(1, measurements));
  ASSERT_TRUE(smoother.optimize());
  ASSERT_NE(smoother.state(1), nullptr);
  EXPECT_LT(norm(smoother.state(1)->position_ecef_m - truth), 0.08);
  const std::array<double, 4> expected{10.0, -4.0, 7.0, 2.0};
  for (std::size_t index = 0; index < measurements.size(); ++index) {
    const auto ambiguity = smoother.ambiguity(measurements[index].ambiguity_key);
    ASSERT_TRUE(ambiguity.has_value());
    EXPECT_NEAR(*ambiguity, expected[index], 0.08);
  }
}

TEST(FloatFixedLagSmoother, JointImuLidarAndGnssFactorsCorrectNextState)
{
  const Vec3 first_position{6378137.0, 0.0, 0.0};
  const Vec3 velocity{0.0, 1.0, 0.0};
  const Vec3 second_truth = first_position + velocity;
  const Vec3 base{6378137.0, -10.0, 0.0};
  FloatFixedLagSmoother smoother;
  const EcefState first = stateAt(0.0, first_position, velocity);
  const EcefState second_initial = stateAt(1.0, second_truth + Vec3{0.5, -0.4, 0.3}, velocity);
  ASSERT_TRUE(smoother.addState(1, first));
  ASSERT_TRUE(smoother.addState(2, second_initial));
  StateFactorNoise strong;
  strong.position_m = 1.0e-4;
  strong.rotation_rad = 1.0e-4;
  strong.velocity_m_s = 1.0e-4;
  strong.accelerometer_bias_m_s2 = 1.0e-4;
  strong.gyroscope_bias_rad_s = 1.0e-5;
  ASSERT_TRUE(smoother.addStatePrior(1, first, strong));

  ImuGraphFactorConfig imu_config;
  imu_config.integration.maximum_step_s = 2.0;
  imu_config.integration.earth_rotation_rad_s = 0.0;
  imu_config.integration.gravitational_parameter_m3_s2 = 0.0;
  imu_config.noise.position_m = 0.05;
  imu_config.noise.velocity_m_s = 0.05;
  std::vector<ImuSample> imu{
    {0.0, {}, {}},
    {1.0, {}, {}}};
  ASSERT_TRUE(smoother.addImuFactor(1, 2, imu, imu_config));

  std::vector<PointToPlaneFactor> planes;
  planes.push_back({{0.0, 0.0, 0.0}, second_truth, {1.0, 0.0, 0.0}});
  planes.push_back({{0.0, 0.0, 0.0}, second_truth, {0.0, 1.0, 0.0}});
  planes.push_back({{0.0, 0.0, 0.0}, second_truth, {0.0, 0.0, 1.0}});
  ASSERT_TRUE(smoother.addLidarFactors(2, {}, planes));
  ASSERT_TRUE(smoother.addGnssFactors(2, syntheticGnss(second_truth, base)));
  ASSERT_TRUE(smoother.optimize());
  ASSERT_NE(smoother.state(2), nullptr);
  EXPECT_LT(norm(smoother.state(2)->position_ecef_m - second_truth), 0.05);
  EXPECT_GT(smoother.diagnostics().last_residual_rows, 20U);
}

TEST(FloatFixedLagSmoother, SchurMarginalizationBoundsWindowAndPreservesPrior)
{
  FloatSmootherConfig config;
  config.duration_s = 100.0;
  config.maximum_states = 2;
  FloatFixedLagSmoother smoother(config);
  const Vec3 origin{6378137.0, 0.0, 0.0};
  const EcefState first = stateAt(0.0, origin);
  ASSERT_TRUE(smoother.addState(1, first));
  StateFactorNoise strong;
  strong.position_m = 0.01;
  strong.rotation_rad = 0.01;
  strong.velocity_m_s = 0.01;
  strong.accelerometer_bias_m_s2 = 0.01;
  strong.gyroscope_bias_rad_s = 0.01;
  ASSERT_TRUE(smoother.addStatePrior(1, first, strong));

  ImuGraphFactorConfig imu_config;
  imu_config.integration.maximum_step_s = 2.0;
  imu_config.integration.earth_rotation_rad_s = 0.0;
  imu_config.integration.gravitational_parameter_m3_s2 = 0.0;
  for (StateId id = 2; id <= 3; ++id) {
    const double stamp = static_cast<double>(id - 1U);
    ASSERT_TRUE(smoother.addState(id, stateAt(stamp, origin + Vec3{0.2 * stamp, 0.0, 0.0})));
    ASSERT_TRUE(
      smoother.addImuFactor(
        id - 1U, id, {{stamp - 1.0, {}, {}}, {stamp, {}, {}}}, imu_config));
  }
  ASSERT_TRUE(smoother.optimize());
  EXPECT_EQ(smoother.stateCount(), 2U);
  EXPECT_EQ(smoother.state(1), nullptr);
  EXPECT_NE(smoother.state(2), nullptr);
  EXPECT_EQ(smoother.diagnostics().marginalizations, 1U);
  EXPECT_TRUE(smoother.diagnostics().last_solve_succeeded);
}

TEST(FloatFixedLagSmoother, GnssOutageAddsNoDuplicateFactor)
{
  FloatFixedLagSmoother smoother;
  ASSERT_TRUE(smoother.addState(1, stateAt(0.0, {6378137.0, 0.0, 0.0})));
  const std::size_t before = smoother.factorCount();
  ASSERT_TRUE(smoother.addGnssFactors(1, {}));
  EXPECT_EQ(smoother.factorCount(), before);
  EXPECT_EQ(smoother.diagnostics().gnss_outages, 1U);
}

TEST(FloatFixedLagSmoother, MarginalPriorDoesNotDoubleCountRetainedFactors)
{
  FloatSmootherConfig bounded_config;
  bounded_config.duration_s = 100.0;
  bounded_config.maximum_states = 2;
  FloatSmootherConfig reference_config = bounded_config;
  reference_config.maximum_states = 3;
  FloatFixedLagSmoother bounded(bounded_config);
  FloatFixedLagSmoother reference(reference_config);
  const Vec3 origin{6378137.0, 0.0, 0.0};

  ImuGraphFactorConfig imu_config;
  imu_config.integration.maximum_step_s = 2.0;
  imu_config.integration.earth_rotation_rad_s = 0.0;
  imu_config.integration.gravitational_parameter_m3_s2 = 0.0;
  imu_config.noise.position_m = 0.2;
  StateFactorNoise anchor_noise;
  anchor_noise.position_m = 0.05;
  anchor_noise.rotation_rad = 0.05;
  anchor_noise.velocity_m_s = 0.05;
  anchor_noise.accelerometer_bias_m_s2 = 0.05;
  anchor_noise.gyroscope_bias_rad_s = 0.05;

  const auto populate = [&](FloatFixedLagSmoother & smoother) {
      EXPECT_TRUE(smoother.addState(1, stateAt(0.0, origin)));
      EXPECT_TRUE(smoother.addState(2, stateAt(1.0, origin + Vec3{0.2, 0.0, 0.0})));
      EXPECT_TRUE(smoother.addState(3, stateAt(2.0, origin + Vec3{0.4, 0.0, 0.0})));
      EXPECT_TRUE(smoother.addStatePrior(1, stateAt(0.0, origin), anchor_noise));
      EXPECT_TRUE(smoother.addImuFactor(1, 2, {{0.0, {}, {}}, {1.0, {}, {}}}, imu_config));
      EXPECT_TRUE(smoother.addImuFactor(2, 3, {{1.0, {}, {}}, {2.0, {}, {}}}, imu_config));
      EXPECT_TRUE(smoother.optimize());
    };
  populate(bounded);
  populate(reference);
  ASSERT_EQ(bounded.state(1), nullptr);
  ASSERT_NE(reference.state(1), nullptr);

  StateFactorNoise conflicting_noise = anchor_noise;
  conflicting_noise.position_m = 0.3;
  const EcefState shifted = stateAt(2.0, origin + Vec3{1.0, 0.0, 0.0});
  ASSERT_TRUE(bounded.addStatePrior(3, shifted, conflicting_noise));
  ASSERT_TRUE(reference.addStatePrior(3, shifted, conflicting_noise));
  ASSERT_TRUE(bounded.optimize());
  ASSERT_TRUE(reference.optimize());
  ASSERT_NE(bounded.state(2), nullptr);
  ASSERT_NE(reference.state(2), nullptr);
  ASSERT_NE(bounded.state(3), nullptr);
  ASSERT_NE(reference.state(3), nullptr);
  EXPECT_NEAR(
    bounded.state(2)->position_ecef_m.x,
    reference.state(2)->position_ecef_m.x, 2.0e-3);
  EXPECT_NEAR(
    bounded.state(3)->position_ecef_m.x,
    reference.state(3)->position_ecef_m.x, 2.0e-3);
}

}  // namespace
}  // namespace fgo_gil_localizer

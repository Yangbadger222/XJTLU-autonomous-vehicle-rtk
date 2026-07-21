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
  measurement.reference_base_position_ecef_m = reference_satellite;
  measurement.target_base_position_ecef_m = target_satellite;
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

TEST(FloatFixedLagSmoother, PreservesSharedReferenceCovarianceBetweenAmbiguities)
{
  const Vec3 truth{6378137.0, 20.0, -10.0};
  const Vec3 base{6378137.0, 0.0, 0.0};
  EcefState initial = stateAt(100.0, truth);
  FloatFixedLagSmoother smoother;
  ASSERT_TRUE(smoother.addState(1, initial));
  StateFactorNoise tight_prior;
  tight_prior.position_m = 1.0e-6;
  tight_prior.rotation_rad = 1.0e-6;
  tight_prior.velocity_m_s = 1.0e-6;
  tight_prior.accelerometer_bias_m_s2 = 1.0e-6;
  tight_prior.gyroscope_bias_rad_s = 1.0e-6;
  ASSERT_TRUE(smoother.addStatePrior(1, initial, tight_prior));
  auto measurements = syntheticGnss(truth, base);
  for (auto & measurement : measurements) {
    measurement.code_valid = false;
    measurement.carrier_target_variance_m2 = 1.0e-4;
    measurement.carrier_reference_variance_m2 = 4.0e-4;
    measurement.carrier_sigma_m = std::sqrt(5.0e-4);
  }
  ASSERT_TRUE(smoother.addGnssFactors(1, measurements));
  ASSERT_TRUE(smoother.optimize());
  const auto estimate = smoother.floatAmbiguityEstimate();
  ASSERT_TRUE(estimate.has_value());
  ASSERT_EQ(estimate->covariance_m2.rows(), 4);
  for (int row = 0; row < 4; ++row) {
    EXPECT_NEAR(estimate->covariance_m2(row, row), 5.0e-4, 2.0e-6);
    for (int column = 0; column < row; ++column) {
      EXPECT_NEAR(estimate->covariance_m2(row, column), 4.0e-4, 2.0e-6);
    }
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
  const auto & diagnostics = smoother.diagnostics();
  EXPECT_GT(diagnostics.last_residual_rows, 20U);
  EXPECT_EQ(diagnostics.state_prior_factors, 1U);
  EXPECT_EQ(diagnostics.imu_factors, 1U);
  EXPECT_EQ(diagnostics.lidar_line_factors, 0U);
  EXPECT_EQ(diagnostics.lidar_plane_factors, 3U);
  EXPECT_EQ(diagnostics.gnss_code_factors, 4U);
  EXPECT_EQ(diagnostics.gnss_carrier_factors, 4U);
  EXPECT_DOUBLE_EQ(diagnostics.window_span_s, 1.0);
}

TEST(FloatFixedLagSmoother, LidarJacobianRemainsConditionedAtEcefScale)
{
  FloatSmootherConfig config;
  config.maximum_iterations = 10;
  FloatFixedLagSmoother smoother(config);
  const Vec3 truth_position{6378137.0, 20.0, -10.0};
  EcefState initial = stateAt(10.0, truth_position + Vec3{0.20, -0.15, 0.10});
  initial.orientation_ecef_body = quaternionFromRotationVector({0.04, -0.03, 0.02});
  ASSERT_TRUE(smoother.addState(1, initial));
  StateFactorNoise prior_noise;
  prior_noise.position_m = 1.0;
  prior_noise.rotation_rad = 0.5;
  prior_noise.velocity_m_s = 1.0e-3;
  prior_noise.accelerometer_bias_m_s2 = 1.0e-3;
  prior_noise.gyroscope_bias_rad_s = 1.0e-4;
  ASSERT_TRUE(smoother.addStatePrior(1, initial, prior_noise));

  const std::array<Vec3, 9> points{
    Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0},
    Vec3{0.0, 2.0, 0.0}, Vec3{0.0, 0.0, 3.0}, Vec3{2.5, 0.0, 0.0},
    Vec3{1.0, -2.0, 0.5}, Vec3{-1.5, 0.5, 2.0}, Vec3{0.5, 1.5, -2.0}};
  const std::array<Vec3, 9> normals{
    Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0},
    Vec3{0.0, 0.0, 1.0}, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0},
    Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}, Vec3{1.0, 0.0, 0.0}};
  std::vector<PointToPlaneFactor> planes;
  for (std::size_t index = 0; index < points.size(); ++index) {
    planes.push_back({points[index], truth_position + points[index], normals[index]});
  }
  ASSERT_TRUE(smoother.addLidarFactors(1, {}, planes));
  ASSERT_TRUE(smoother.optimize());
  ASSERT_NE(smoother.state(1), nullptr);
  EXPECT_LT(norm(smoother.state(1)->position_ecef_m - truth_position), 0.02);
  EXPECT_LT(
    norm(quaternionLog(smoother.state(1)->orientation_ecef_body)), 0.01);
  EXPECT_LT(smoother.diagnostics().last_condition_estimate, 1.0e12);
  EXPECT_TRUE(smoother.diagnostics().last_solve_succeeded);
}

TEST(FloatFixedLagSmoother, BoundsLidarResidualsPerKeyframe)
{
  LidarGraphFactorConfig lidar_config;
  lidar_config.maximum_line_factors_per_keyframe = 8;
  lidar_config.maximum_plane_factors_per_keyframe = 12;
  FloatFixedLagSmoother smoother({}, lidar_config);
  const Vec3 position{6378137.0, 0.0, 0.0};
  ASSERT_TRUE(smoother.addState(1, stateAt(0.0, position)));
  std::vector<PointToLineFactor> lines(40, {{1.0, 0.0, 0.0}, position, {1.0, 0.0, 0.0}});
  std::vector<PointToPlaneFactor> planes(
    60, {{0.0, 1.0, 0.0}, position + Vec3{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}});
  ASSERT_TRUE(smoother.addLidarFactors(1, lines, planes));
  EXPECT_EQ(smoother.diagnostics().lidar_line_factors, 8U);
  EXPECT_EQ(smoother.diagnostics().lidar_plane_factors, 12U);
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

TEST(FloatFixedLagSmoother, ReferenceSwitchUsesExactDoubleDifferenceBasisTransform)
{
  const Vec3 truth{6378137.0, 20.0, -10.0};
  const Vec3 base{6378137.0, 0.0, 0.0};
  const Vec3 reference{26500000.0, 0.0, 4000000.0};
  FloatFixedLagSmoother smoother;
  ASSERT_TRUE(smoother.addState(1, stateAt(100.0, truth)));
  const auto old_target = syntheticDd(
    7, reference, {21000000.0, 11000000.0, 9000000.0}, truth, base, 10.0);
  const auto old_pivot = syntheticDd(
    8, reference, {22000000.0, -12000000.0, 7000000.0}, truth, base, 4.0);
  ASSERT_TRUE(smoother.addGnssFactors(1, {old_target, old_pivot}));

  DoubleDifferenceMeasurement switched_target = old_target;
  switched_target.reference = old_pivot.target;
  switched_target.ambiguity_key.reference = old_pivot.target;
  switched_target.ambiguity_key.receiver_arc_ids[2] = old_pivot.ambiguity_key.receiver_arc_ids[0];
  switched_target.ambiguity_key.receiver_arc_ids[3] = old_pivot.ambiguity_key.receiver_arc_ids[1];
  switched_target.carrier_dd_m = 999.0;
  DoubleDifferenceMeasurement switched_old_reference = old_target;
  switched_old_reference.reference = old_pivot.target;
  switched_old_reference.target = old_target.reference;
  switched_old_reference.ambiguity_key.reference = old_pivot.target;
  switched_old_reference.ambiguity_key.target = old_target.reference;
  switched_old_reference.ambiguity_key.receiver_arc_ids = {
    old_target.ambiguity_key.receiver_arc_ids[2], old_target.ambiguity_key.receiver_arc_ids[3],
    old_pivot.ambiguity_key.receiver_arc_ids[0], old_pivot.ambiguity_key.receiver_arc_ids[1]};
  switched_old_reference.carrier_dd_m = -999.0;
  ASSERT_TRUE(smoother.addGnssFactors(1, {switched_target, switched_old_reference}));

  const auto transformed_target = smoother.ambiguity(switched_target.ambiguity_key);
  const auto transformed_reference = smoother.ambiguity(switched_old_reference.ambiguity_key);
  ASSERT_TRUE(transformed_target.has_value());
  ASSERT_TRUE(transformed_reference.has_value());
  EXPECT_NEAR(*transformed_target, 6.0, 1.0e-9);
  EXPECT_NEAR(*transformed_reference, -4.0, 1.0e-9);
}

TEST(FloatFixedLagSmoother, CovarianceOrderingMatchesAmbiguityKeys)
{
  const Vec3 truth{6378137.0, 20.0, -10.0};
  const Vec3 base{6378137.0, 0.0, 0.0};
  EcefState initial = stateAt(100.0, truth + Vec3{0.5, -0.4, 0.3});
  FloatFixedLagSmoother smoother;
  ASSERT_TRUE(smoother.addState(1, initial));
  ASSERT_TRUE(smoother.addStatePrior(1, initial, loosePositionPrior()));
  const auto measurements = syntheticGnss(truth, base);
  ASSERT_TRUE(smoother.addGnssFactors(1, measurements));
  ASSERT_TRUE(smoother.optimize());
  const auto estimate = smoother.floatAmbiguityEstimate();
  ASSERT_TRUE(estimate.has_value());
  ASSERT_EQ(estimate->keys.size(), measurements.size());
  EXPECT_EQ(estimate->values_m.size(), static_cast<int>(measurements.size()));
  EXPECT_EQ(estimate->covariance_m2.rows(), static_cast<int>(measurements.size()));
  EXPECT_TRUE(estimate->covariance_m2.allFinite());
  EXPECT_LT((estimate->covariance_m2 - estimate->covariance_m2.transpose()).norm(), 1.0e-9);
  for (std::size_t index = 0; index < estimate->keys.size(); ++index) {
    const auto value = smoother.ambiguity(estimate->keys[index]);
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(estimate->values_m(static_cast<int>(index)), *value);
    EXPECT_EQ(estimate->last_observed_state_ids[index], 1U);
    EXPECT_GT(estimate->covariance_m2(static_cast<int>(index), static_cast<int>(index)), 0.0);
  }
}

TEST(FloatFixedLagSmoother, FixedPreviewNeverMutatesFloatWindow)
{
  const Vec3 truth{6378137.0, 20.0, -10.0};
  const Vec3 base{6378137.0, 0.0, 0.0};
  EcefState initial = stateAt(100.0, truth + Vec3{0.5, -0.4, 0.3});
  FloatFixedLagSmoother smoother;
  ASSERT_TRUE(smoother.addState(1, initial));
  ASSERT_TRUE(smoother.addStatePrior(1, initial, loosePositionPrior()));
  const auto measurements = syntheticGnss(truth, base);
  ASSERT_TRUE(smoother.addGnssFactors(1, measurements));
  ASSERT_TRUE(smoother.optimize());
  const EcefState float_state = *smoother.state(1);
  std::vector<double> float_ambiguities;
  for (const auto & measurement : measurements) {
    float_ambiguities.push_back(*smoother.ambiguity(measurement.ambiguity_key));
  }

  Eigen::Vector4d wrong_fixed;
  wrong_fixed << 100.0, 100.0, 100.0, 100.0;
  std::vector<DdAmbiguityKey> keys;
  for (const auto & measurement : measurements) {
    keys.push_back(measurement.ambiguity_key);
  }
  const auto rejected = smoother.previewFixedAmbiguities(keys, wrong_fixed);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_NE(rejected.rejection, FixedBackSubstitutionRejection::None);
  EXPECT_DOUBLE_EQ(smoother.state(1)->position_ecef_m.x, float_state.position_ecef_m.x);
  EXPECT_DOUBLE_EQ(smoother.state(1)->position_ecef_m.y, float_state.position_ecef_m.y);
  EXPECT_DOUBLE_EQ(smoother.state(1)->position_ecef_m.z, float_state.position_ecef_m.z);
  for (std::size_t index = 0; index < measurements.size(); ++index) {
    EXPECT_DOUBLE_EQ(
      *smoother.ambiguity(measurements[index].ambiguity_key),
      float_ambiguities[index]);
  }

  Eigen::Vector4d near_fixed;
  near_fixed << 10.0, -4.0, 7.0, 2.0;
  FixedBackSubstitutionConfig permissive;
  permissive.maximum_cost_increase = 100.0;
  const auto accepted = smoother.previewFixedAmbiguities(keys, near_fixed, permissive);
  EXPECT_TRUE(accepted.accepted) << toString(accepted.rejection);
  EXPECT_DOUBLE_EQ(smoother.state(1)->position_ecef_m.x, float_state.position_ecef_m.x);
  EXPECT_DOUBLE_EQ(smoother.state(1)->position_ecef_m.y, float_state.position_ecef_m.y);
  EXPECT_DOUBLE_EQ(smoother.state(1)->position_ecef_m.z, float_state.position_ecef_m.z);
  for (std::size_t index = 0; index < measurements.size(); ++index) {
    EXPECT_DOUBLE_EQ(
      *smoother.ambiguity(measurements[index].ambiguity_key),
      float_ambiguities[index]);
  }
}

}  // namespace
}  // namespace fgo_gil_localizer

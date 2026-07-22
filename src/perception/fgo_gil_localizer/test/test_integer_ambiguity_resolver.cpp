#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "fgo_gil_localizer/integer_ambiguity_resolver.hpp"
#include "fgo_gil_localizer/rtklib_lambda.hpp"
#include "fgo_gil_localizer/satellite_propagator.hpp"

namespace fgo_gil_localizer
{
namespace
{

DdAmbiguityKey key(
  const std::uint16_t target_prn,
  const std::uint16_t reference_prn = 3,
  const std::uint64_t reference_arc = 300,
  const GnssConstellation constellation = GnssConstellation::Gps)
{
  return {
    {constellation, 0, false},
    {constellation, reference_prn},
    {constellation, target_prn},
    {100U + target_prn, 200U + target_prn, reference_arc, reference_arc + 1U}};
}

FloatAmbiguityEstimate estimateFromCycles(
  const std::vector<DdAmbiguityKey> & keys,
  const Eigen::VectorXd & cycles,
  const Eigen::MatrixXd & covariance_cycles2,
  const std::uint64_t state_id = 10)
{
  FloatAmbiguityEstimate estimate;
  estimate.keys = keys;
  estimate.last_observed_state_ids.assign(keys.size(), state_id);
  estimate.observation_counts.assign(keys.size(), 5U);
  estimate.values_m.resize(cycles.size());
  estimate.covariance_m2.resize(cycles.size(), cycles.size());
  std::vector<double> wavelengths(keys.size());
  for (std::size_t index = 0; index < keys.size(); ++index) {
    const auto & group = keys[index].group;
    const auto wavelength = carrierWavelengthM(
      {group.constellation, group.signal_type, group.l2c_signal, 0});
    EXPECT_TRUE(wavelength.has_value());
    wavelengths[index] = wavelength.value_or(1.0);
    estimate.values_m(static_cast<int>(index)) = cycles(static_cast<int>(index)) *
      wavelengths[index];
  }
  for (int row = 0; row < cycles.size(); ++row) {
    for (int column = 0; column < cycles.size(); ++column) {
      estimate.covariance_m2(row, column) = covariance_cycles2(row, column) *
        wavelengths[static_cast<std::size_t>(row)] *
        wavelengths[static_cast<std::size_t>(column)];
    }
  }
  return estimate;
}

TEST(RtklibLambda, MatchesOfficialSixDimensionalCorrelatedFixture)
{
  Eigen::VectorXd float_ambiguities(6);
  float_ambiguities <<
    1585184.171, -6716599.430, 3915742.905,
    7627233.455, 9565990.879, 989457273.200;
  Eigen::MatrixXd covariance(6, 6);
  covariance <<
    0.227134, 0.112202, 0.112202, 0.112202, 0.112202, 0.103473,
    0.112202, 0.227134, 0.112202, 0.112202, 0.112202, 0.103473,
    0.112202, 0.112202, 0.227134, 0.112202, 0.112202, 0.103473,
    0.112202, 0.112202, 0.112202, 0.227134, 0.112202, 0.103473,
    0.112202, 0.112202, 0.112202, 0.112202, 0.227134, 0.103473,
    0.103473, 0.103473, 0.103473, 0.103473, 0.103473, 0.434339;
  Eigen::Matrix<double, 6, 2> expected;
  expected <<
    1585184.0, 1585184.0,
    -6716599.0, -6716600.0,
    3915743.0, 3915743.0,
    7627234.0, 7627233.0,
    9565991.0, 9565991.0,
    989457273.0, 989457273.0;

  rtklib::LambdaResult result;
  ASSERT_TRUE(rtklib::lambda(float_ambiguities, covariance, 2, result));
  ASSERT_EQ(result.candidates.rows(), 6);
  ASSERT_EQ(result.candidates.cols(), 2);
  EXPECT_LT((result.candidates - expected).cwiseAbs().maxCoeff(), 1.0e-4);
  EXPECT_NEAR(result.squared_norms(0), 3.507984, 1.0e-4);
  EXPECT_NEAR(result.squared_norms(1), 3.708456, 1.0e-4);
}

TEST(IntegerAmbiguityResolver, KnownIntegerFixtureFixesWithInputOrderingPreserved)
{
  const std::vector<DdAmbiguityKey> keys{key(12), key(7), key(19), key(5)};
  Eigen::Vector4d cycles;
  cycles << 12.03, -5.02, 7.01, 31.04;
  Eigen::Matrix4d covariance;
  covariance <<
    0.0004, 0.0001, 0.0, 0.0,
    0.0001, 0.0005, 0.0001, 0.0,
    0.0, 0.0001, 0.0004, 0.0001,
    0.0, 0.0, 0.0001, 0.0005;
  IntegerAmbiguityResolver resolver;
  const IntegerFixResult result = resolver.resolve(estimateFromCycles(keys, cycles, covariance));
  ASSERT_TRUE(result.fixed) << toString(result.rejection_reason);
  ASSERT_EQ(result.keys.size(), keys.size());
  ASSERT_EQ(result.evaluated_keys.size(), keys.size());
  const std::vector<double> expected{12.0, -5.0, 7.0, 31.0};
  for (std::size_t index = 0; index < keys.size(); ++index) {
    EXPECT_TRUE(result.keys[index] == keys[index]);
    EXPECT_TRUE(result.evaluated_keys[index] == keys[index]);
    EXPECT_DOUBLE_EQ(result.integer_cycles(static_cast<int>(index)), expected[index]);
  }
  EXPECT_GT(result.ratio, 3.0);
  EXPECT_GT(result.success_rate, 0.99);
  EXPECT_EQ(result.eligible_ambiguities, 4U);
  EXPECT_EQ(result.evaluated_ambiguities, 4U);
  EXPECT_GT(result.fractional_cycle_rms, 0.0);
  EXPECT_NEAR(result.fractional_cycle_max, 0.04, 1.0e-12);
}

TEST(IntegerAmbiguityResolver, EqualIntegerCandidatesFailRatioTest)
{
  std::vector<DdAmbiguityKey> keys{key(4), key(5), key(6), key(7)};
  Eigen::Vector4d cycles = Eigen::Vector4d::Constant(0.5);
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity() * 0.01;
  IntegerAmbiguityResolverConfig config;
  config.minimum_success_rate = 0.0;
  IntegerAmbiguityResolver resolver(config);
  const IntegerFixResult result = resolver.resolve(estimateFromCycles(keys, cycles, covariance));
  EXPECT_FALSE(result.fixed);
  EXPECT_EQ(result.rejection_reason, IntegerFixRejectionReason::RatioTest);
  EXPECT_EQ(result.evaluated_keys.size(), result.evaluated_ambiguities);
  EXPECT_TRUE(result.keys.empty());
  EXPECT_NEAR(result.ratio, 1.0, 1.0e-12);
  EXPECT_GT(result.best_squared_norm, 0.0);
  EXPECT_GT(result.second_squared_norm, 0.0);
  EXPECT_GT(result.success_rate, 0.0);
  EXPECT_EQ(result.eligible_ambiguities, 4U);
  EXPECT_EQ(result.evaluated_ambiguities, 4U);
  EXPECT_NEAR(result.fractional_cycle_rms, 0.5, 1.0e-12);
  EXPECT_NEAR(result.fractional_cycle_max, 0.5, 1.0e-12);
}

TEST(IntegerCandidateConfirmation, RequiresStableConsecutiveCandidates)
{
  IntegerCandidateConfirmation confirmation(3);
  IntegerFixResult candidate;
  candidate.fixed = true;
  candidate.keys = {key(4), key(5), key(6), key(7)};
  candidate.integer_cycles = Eigen::Vector4d(1.0, 2.0, 3.0, 4.0);
  EXPECT_FALSE(confirmation.update(candidate));
  EXPECT_EQ(confirmation.count(), 1U);
  EXPECT_FALSE(confirmation.update(candidate));
  EXPECT_EQ(confirmation.count(), 2U);

  candidate.integer_cycles(3) = 5.0;
  EXPECT_FALSE(confirmation.update(candidate));
  EXPECT_EQ(confirmation.count(), 1U);
  EXPECT_FALSE(confirmation.update(candidate));
  EXPECT_TRUE(confirmation.update(candidate));
  EXPECT_EQ(confirmation.count(), 3U);

  candidate.fixed = false;
  EXPECT_FALSE(confirmation.update(candidate));
  EXPECT_EQ(confirmation.count(), 0U);
  EXPECT_THROW(IntegerCandidateConfirmation(0), std::invalid_argument);
}

TEST(IntegerAmbiguityResolver, PartialFixDropsWorstVarianceAmbiguity)
{
  std::vector<DdAmbiguityKey> keys{key(4), key(5), key(6), key(7), key(8)};
  Eigen::VectorXd cycles(5);
  cycles << 1.01, 2.02, 3.01, 4.02, 8.5;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(5, 5) * 0.0004;
  covariance(4, 4) = 4.0;
  IntegerAmbiguityResolver resolver;
  const IntegerFixResult result = resolver.resolve(estimateFromCycles(keys, cycles, covariance));
  ASSERT_TRUE(result.fixed) << toString(result.rejection_reason);
  EXPECT_EQ(result.keys.size(), 4U);
  for (const auto & selected : result.keys) {
    EXPECT_FALSE(selected == keys.back());
  }
}

TEST(IntegerAmbiguityResolver, NeverCombinesDifferentReferenceBases)
{
  std::vector<DdAmbiguityKey> keys{
    key(4, 3, 300), key(5, 3, 300), key(6, 8, 800), key(7, 8, 800)};
  Eigen::Vector4d cycles;
  cycles << 1.01, 2.01, 3.01, 4.01;
  const IntegerFixResult result = IntegerAmbiguityResolver().resolve(
    estimateFromCycles(keys, cycles, Eigen::Matrix4d::Identity() * 0.0004));
  EXPECT_FALSE(result.fixed);
  EXPECT_EQ(result.rejection_reason, IntegerFixRejectionReason::AmbiguityBasisConflict);
}

TEST(IntegerAmbiguityResolver, UsesNewestBasisAndCombinesCurrentConstellations)
{
  std::vector<DdAmbiguityKey> keys{
    key(4, 3, 300), key(5, 3, 300), key(6, 3, 300), key(7, 3, 300),
    key(11, 8, 800), key(12, 8, 800),
    key(20, 19, 1900, GnssConstellation::Bds),
    key(21, 19, 1900, GnssConstellation::Bds)};
  Eigen::VectorXd cycles(8);
  cycles << 90.2, -20.3, 11.4, 7.1, 3.01, 4.02, -8.01, 12.02;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(8, 8) * 0.0004;
  auto estimate = estimateFromCycles(keys, cycles, covariance, 20);
  for (std::size_t index = 0; index < 4; ++index) {
    estimate.last_observed_state_ids[index] = 19;
  }
  const IntegerFixResult result = IntegerAmbiguityResolver().resolve(estimate);
  ASSERT_TRUE(result.fixed) << toString(result.rejection_reason);
  ASSERT_EQ(result.keys.size(), 4U);
  EXPECT_TRUE(result.keys[0] == keys[4]);
  EXPECT_TRUE(result.keys[1] == keys[5]);
  EXPECT_TRUE(result.keys[2] == keys[6]);
  EXPECT_TRUE(result.keys[3] == keys[7]);
}

TEST(IntegerAmbiguityResolver, UsesOnlyNewestObservationSetWithinBasis)
{
  std::vector<DdAmbiguityKey> keys{key(4), key(5), key(6), key(7), key(8)};
  Eigen::VectorXd cycles(5);
  cycles << 1.01, 2.01, 3.01, 4.01, 5.01;
  auto estimate = estimateFromCycles(
    keys, cycles, Eigen::MatrixXd::Identity(5, 5) * 0.0004, 20);
  estimate.last_observed_state_ids[0] = 19;
  const IntegerFixResult result = IntegerAmbiguityResolver().resolve(estimate);
  ASSERT_TRUE(result.fixed) << toString(result.rejection_reason);
  EXPECT_EQ(result.keys.size(), 4U);
  EXPECT_FALSE(result.keys.front() == keys.front());
}

TEST(IntegerAmbiguityResolver, ExcludesImmatureAmbiguityArcs)
{
  std::vector<DdAmbiguityKey> keys{key(4), key(5), key(6), key(7)};
  Eigen::Vector4d cycles;
  cycles << 1.01, 2.01, 3.01, 4.01;
  auto estimate = estimateFromCycles(
    keys, cycles, Eigen::Matrix4d::Identity() * 0.0004);
  estimate.observation_counts[2] = 1U;
  const IntegerFixResult result = IntegerAmbiguityResolver().resolve(estimate);
  EXPECT_FALSE(result.fixed);
  EXPECT_EQ(result.rejection_reason, IntegerFixRejectionReason::InsufficientAmbiguities);
}

TEST(IntegerAmbiguityResolver, RejectsNonFiniteAndGlonassInputsClosed)
{
  std::vector<DdAmbiguityKey> keys{key(4), key(5), key(6), key(7)};
  Eigen::Vector4d cycles;
  cycles << 1.0, 2.0, 3.0, 4.0;
  auto estimate = estimateFromCycles(
    keys, cycles, Eigen::Matrix4d::Identity() * 0.001);
  estimate.values_m(2) = std::numeric_limits<double>::quiet_NaN();
  auto result = IntegerAmbiguityResolver().resolve(estimate);
  EXPECT_EQ(result.rejection_reason, IntegerFixRejectionReason::NonFiniteInput);

  keys = {
    key(4, 3, 300, GnssConstellation::Glonass),
    key(5, 3, 300, GnssConstellation::Glonass),
    key(6, 3, 300, GnssConstellation::Glonass),
    key(7, 3, 300, GnssConstellation::Glonass)};
  estimate.keys = keys;
  estimate.last_observed_state_ids.assign(4, 10);
  estimate.values_m = Eigen::Vector4d::Zero();
  estimate.covariance_m2 = Eigen::Matrix4d::Identity();
  result = IntegerAmbiguityResolver().resolve(estimate);
  EXPECT_EQ(result.rejection_reason, IntegerFixRejectionReason::UnsupportedSignal);
}

}  // namespace
}  // namespace fgo_gil_localizer

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "fgo_gil_localizer/gnss_double_difference.hpp"

namespace fgo_gil_localizer
{

struct FloatAmbiguityEstimate
{
  std::vector<DdAmbiguityKey> keys;
  std::vector<std::uint64_t> last_observed_state_ids;
  std::vector<std::size_t> observation_counts;
  Eigen::VectorXd values_m;
  Eigen::MatrixXd covariance_m2;
};

enum class IntegerFixRejectionReason : std::uint8_t
{
  None,
  Disabled,
  NoAmbiguities,
  NoCurrentGnssEpoch,
  InvalidDimensions,
  NonFiniteInput,
  UnsupportedSignal,
  AmbiguityBasisConflict,
  InsufficientAmbiguities,
  CovarianceNotPositiveDefinite,
  LambdaFailure,
  RatioTest,
  SuccessRateTest,
  ResidualTest,
  BackSubstitutionRejected,
  ConfirmationPending,
};

const char * toString(IntegerFixRejectionReason reason) noexcept;

struct IntegerAmbiguityResolverConfig
{
  bool enabled = true;
  bool partial_fixing = true;
  std::size_t minimum_ambiguities = 4;
  std::size_t minimum_observation_epochs = 5;
  double ratio_threshold = 3.0;
  double minimum_success_rate = 0.99;
  double maximum_squared_norm = 25.0;
};

struct IntegerFixResult
{
  bool fixed = false;
  IntegerFixRejectionReason rejection_reason = IntegerFixRejectionReason::NoAmbiguities;
  std::vector<std::size_t> estimate_indices;
  std::vector<DdAmbiguityKey> keys;
  std::vector<DdAmbiguityKey> evaluated_keys;
  Eigen::VectorXd integer_cycles;
  Eigen::VectorXd fixed_values_m;
  double best_squared_norm = 0.0;
  double second_squared_norm = 0.0;
  double ratio = 0.0;
  double success_rate = 0.0;
  std::size_t eligible_ambiguities = 0;
  std::size_t evaluated_ambiguities = 0;
  double fractional_cycle_rms = 0.0;
  double fractional_cycle_max = 0.0;
};

class IntegerCandidateConfirmation
{
public:
  explicit IntegerCandidateConfirmation(std::size_t required_consecutive_epochs = 3);

  bool update(const IntegerFixResult & candidate);
  void reset();
  std::size_t count() const noexcept {return count_;}

private:
  std::size_t required_consecutive_epochs_ = 3;
  std::vector<DdAmbiguityKey> keys_;
  Eigen::VectorXd integer_cycles_;
  std::size_t count_ = 0;
};

class IntegerAmbiguityResolver
{
public:
  explicit IntegerAmbiguityResolver(IntegerAmbiguityResolverConfig config = {});

  IntegerFixResult resolve(const FloatAmbiguityEstimate & estimate) const;
  const IntegerAmbiguityResolverConfig & config() const noexcept {return config_;}

private:
  IntegerAmbiguityResolverConfig config_;
};

}  // namespace fgo_gil_localizer

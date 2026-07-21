#include "fgo_gil_localizer/integer_ambiguity_resolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>

#include "fgo_gil_localizer/rtklib_lambda.hpp"
#include "fgo_gil_localizer/satellite_propagator.hpp"

namespace fgo_gil_localizer
{
namespace
{

struct AmbiguityBasis
{
  SignalGroup group;
  SatelliteId reference;
  std::uint64_t reference_rover_arc = 0;
  std::uint64_t reference_base_arc = 0;

  bool operator<(const AmbiguityBasis & other) const noexcept
  {
    if (group < other.group) {
      return true;
    }
    if (other.group < group) {
      return false;
    }
    if (reference < other.reference) {
      return true;
    }
    if (other.reference < reference) {
      return false;
    }
    if (reference_rover_arc != other.reference_rover_arc) {
      return reference_rover_arc < other.reference_rover_arc;
    }
    return reference_base_arc < other.reference_base_arc;
  }
};

AmbiguityBasis basis(const DdAmbiguityKey & key)
{
  return {key.group, key.reference, key.receiver_arc_ids[2], key.receiver_arc_ids[3]};
}

double bootstrapSuccessRate(const Eigen::VectorXd & conditional_variances)
{
  double probability = 1.0;
  for (int index = 0; index < conditional_variances.size(); ++index) {
    const double variance = conditional_variances(index);
    if (!std::isfinite(variance) || variance <= 0.0) {
      return 0.0;
    }
    const double term = std::erf(0.5 / std::sqrt(2.0 * variance));
    probability *= std::clamp(term, 0.0, 1.0);
  }
  return probability;
}

IntegerFixRejectionReason failedTest(
  const IntegerAmbiguityResolverConfig & config,
  const double ratio,
  const double success_rate,
  const double best_squared_norm)
{
  if (!std::isfinite(ratio) && !std::isinf(ratio)) {
    return IntegerFixRejectionReason::LambdaFailure;
  }
  if (ratio < config.ratio_threshold) {
    return IntegerFixRejectionReason::RatioTest;
  }
  if (success_rate < config.minimum_success_rate) {
    return IntegerFixRejectionReason::SuccessRateTest;
  }
  if (!std::isfinite(best_squared_norm) || best_squared_norm > config.maximum_squared_norm) {
    return IntegerFixRejectionReason::ResidualTest;
  }
  return IntegerFixRejectionReason::None;
}

}  // namespace

const char * toString(const IntegerFixRejectionReason reason) noexcept
{
  switch (reason) {
    case IntegerFixRejectionReason::None: return "NONE";
    case IntegerFixRejectionReason::Disabled: return "DISABLED";
    case IntegerFixRejectionReason::NoAmbiguities: return "NO_AMBIGUITIES";
    case IntegerFixRejectionReason::NoCurrentGnssEpoch: return "NO_CURRENT_GNSS_EPOCH";
    case IntegerFixRejectionReason::InvalidDimensions: return "INVALID_DIMENSIONS";
    case IntegerFixRejectionReason::NonFiniteInput: return "NON_FINITE_INPUT";
    case IntegerFixRejectionReason::UnsupportedSignal: return "UNSUPPORTED_SIGNAL";
    case IntegerFixRejectionReason::AmbiguityBasisConflict: return "AMBIGUITY_BASIS_CONFLICT";
    case IntegerFixRejectionReason::InsufficientAmbiguities: return "INSUFFICIENT_AMBIGUITIES";
    case IntegerFixRejectionReason::CovarianceNotPositiveDefinite:
      return "COVARIANCE_NOT_POSITIVE_DEFINITE";
    case IntegerFixRejectionReason::LambdaFailure: return "LAMBDA_FAILURE";
    case IntegerFixRejectionReason::RatioTest: return "RATIO_TEST";
    case IntegerFixRejectionReason::SuccessRateTest: return "SUCCESS_RATE_TEST";
    case IntegerFixRejectionReason::ResidualTest: return "RESIDUAL_TEST";
    case IntegerFixRejectionReason::BackSubstitutionRejected:
      return "BACK_SUBSTITUTION_REJECTED";
  }
  return "UNKNOWN";
}

IntegerAmbiguityResolver::IntegerAmbiguityResolver(IntegerAmbiguityResolverConfig config)
: config_(config)
{
  if (config_.minimum_ambiguities == 0U || config_.minimum_observation_epochs == 0U ||
    !std::isfinite(config_.ratio_threshold) ||
    config_.ratio_threshold <= 1.0 || !std::isfinite(config_.minimum_success_rate) ||
    config_.minimum_success_rate < 0.0 || config_.minimum_success_rate > 1.0 ||
    !std::isfinite(config_.maximum_squared_norm) || config_.maximum_squared_norm <= 0.0)
  {
    throw std::invalid_argument("integer ambiguity resolver configuration is outside valid bounds");
  }
}

IntegerFixResult IntegerAmbiguityResolver::resolve(const FloatAmbiguityEstimate & estimate) const
{
  IntegerFixResult output;
  if (!config_.enabled) {
    output.rejection_reason = IntegerFixRejectionReason::Disabled;
    return output;
  }
  const std::size_t count = estimate.keys.size();
  if (count == 0U) {
    output.rejection_reason = IntegerFixRejectionReason::NoAmbiguities;
    return output;
  }
  if (estimate.last_observed_state_ids.size() != count ||
    estimate.observation_counts.size() != count ||
    estimate.values_m.size() != static_cast<int>(count) ||
    estimate.covariance_m2.rows() != static_cast<int>(count) ||
    estimate.covariance_m2.cols() != static_cast<int>(count))
  {
    output.rejection_reason = IntegerFixRejectionReason::InvalidDimensions;
    return output;
  }
  if (!estimate.values_m.allFinite() || !estimate.covariance_m2.allFinite()) {
    output.rejection_reason = IntegerFixRejectionReason::NonFiniteInput;
    return output;
  }

  const std::uint64_t newest_state = *std::max_element(
    estimate.last_observed_state_ids.begin(), estimate.last_observed_state_ids.end());
  std::map<SignalGroup, std::map<AmbiguityBasis, std::vector<std::size_t>>> grouped;
  bool had_unsupported_signal = false;
  for (std::size_t index = 0; index < count; ++index) {
    if (estimate.last_observed_state_ids[index] != newest_state) {
      continue;
    }
    if (estimate.observation_counts[index] < config_.minimum_observation_epochs) {
      continue;
    }
    const DdAmbiguityKey & key = estimate.keys[index];
    if (key.group.constellation == GnssConstellation::Glonass) {
      had_unsupported_signal = true;
      continue;
    }
    const SignalKey signal{
      key.group.constellation, key.group.signal_type, key.group.l2c_signal, 0};
    if (!carrierWavelengthM(signal).has_value()) {
      had_unsupported_signal = true;
      continue;
    }
    grouped[key.group][basis(key)].push_back(index);
  }

  std::vector<std::size_t> active;
  for (const auto & signal_group : grouped) {
    if (signal_group.second.size() != 1U) {
      output.rejection_reason = IntegerFixRejectionReason::AmbiguityBasisConflict;
      return output;
    }
    const auto & indices = signal_group.second.begin()->second;
    active.insert(active.end(), indices.begin(), indices.end());
  }
  std::sort(active.begin(), active.end());
  if (active.size() < config_.minimum_ambiguities) {
    output.rejection_reason = active.empty() && had_unsupported_signal ?
      IntegerFixRejectionReason::UnsupportedSignal :
      IntegerFixRejectionReason::InsufficientAmbiguities;
    return output;
  }

  IntegerFixRejectionReason last_rejection = IntegerFixRejectionReason::LambdaFailure;
  while (active.size() >= config_.minimum_ambiguities) {
    const int dimension = static_cast<int>(active.size());
    Eigen::VectorXd float_cycles(dimension);
    Eigen::MatrixXd covariance_cycles2(dimension, dimension);
    std::vector<double> wavelengths(active.size());
    bool valid_wavelengths = true;
    for (int row = 0; row < dimension; ++row) {
      const DdAmbiguityKey & key = estimate.keys[active[static_cast<std::size_t>(row)]];
      const auto wavelength = carrierWavelengthM(
        {key.group.constellation, key.group.signal_type, key.group.l2c_signal, 0});
      if (!wavelength.has_value() || !std::isfinite(*wavelength) || *wavelength <= 0.0) {
        valid_wavelengths = false;
        break;
      }
      wavelengths[static_cast<std::size_t>(row)] = *wavelength;
      float_cycles(row) = estimate.values_m(
        static_cast<int>(active[static_cast<std::size_t>(row)])) / *wavelength;
    }
    if (!valid_wavelengths) {
      last_rejection = IntegerFixRejectionReason::UnsupportedSignal;
      break;
    }
    for (int row = 0; row < dimension; ++row) {
      for (int column = 0; column < dimension; ++column) {
        covariance_cycles2(row, column) = estimate.covariance_m2(
          static_cast<int>(active[static_cast<std::size_t>(row)]),
          static_cast<int>(active[static_cast<std::size_t>(column)])) /
          (wavelengths[static_cast<std::size_t>(row)] *
          wavelengths[static_cast<std::size_t>(column)]);
      }
    }
    covariance_cycles2 = 0.5 * (covariance_cycles2 + covariance_cycles2.transpose());
    Eigen::LDLT<Eigen::MatrixXd> covariance_check(covariance_cycles2);
    if (covariance_check.info() != Eigen::Success ||
      (covariance_check.vectorD().array() <= 0.0).any())
    {
      last_rejection = IntegerFixRejectionReason::CovarianceNotPositiveDefinite;
    } else {
      rtklib::LambdaResult lambda_result;
      if (!rtklib::lambda(float_cycles, covariance_cycles2, 2, lambda_result)) {
        last_rejection = IntegerFixRejectionReason::LambdaFailure;
      } else {
        const double best_norm = lambda_result.squared_norms(0);
        const double second_norm = lambda_result.squared_norms(1);
        const double ratio = best_norm <= std::numeric_limits<double>::epsilon() ?
          std::numeric_limits<double>::infinity() : second_norm / best_norm;
        const double success_rate = bootstrapSuccessRate(
          lambda_result.conditional_variances);
        output.best_squared_norm = best_norm;
        output.second_squared_norm = second_norm;
        output.ratio = ratio;
        output.success_rate = success_rate;
        last_rejection = failedTest(config_, ratio, success_rate, best_norm);
        if (last_rejection == IntegerFixRejectionReason::None) {
          output.fixed = true;
          output.rejection_reason = IntegerFixRejectionReason::None;
          output.estimate_indices = active;
          output.integer_cycles = lambda_result.candidates.col(0);
          output.fixed_values_m.resize(dimension);
          output.keys.reserve(active.size());
          for (int index = 0; index < dimension; ++index) {
            output.keys.push_back(estimate.keys[active[static_cast<std::size_t>(index)]]);
            output.fixed_values_m(index) = output.integer_cycles(index) *
              wavelengths[static_cast<std::size_t>(index)];
          }
          return output;
        }
      }
    }

    if (!config_.partial_fixing || active.size() == config_.minimum_ambiguities) {
      break;
    }
    Eigen::Index worst = 0;
    covariance_cycles2.diagonal().maxCoeff(&worst);
    active.erase(active.begin() + worst);
  }
  output.rejection_reason = last_rejection;
  return output;
}

}  // namespace fgo_gil_localizer

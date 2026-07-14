#include "fgo_gil_localizer/time_sync.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace fgo_gil_localizer
{
namespace
{

double median(std::vector<double> values)
{
  const std::size_t middle = values.size() / 2U;
  std::nth_element(
    values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle),
    values.end());
  double result = values[middle];
  if (values.size() % 2U == 0U) {
    const auto lower = std::max_element(
      values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
    result = 0.5 * (result + *lower);
  }
  return result;
}

}  // namespace

TimeSyncEstimator::TimeSyncEstimator(TimeSyncConfig config)
: config_(config)
{
  if (config_.window_size == 0U || config_.min_coarse_samples == 0U ||
    config_.min_pps_samples == 0U || config_.min_coarse_samples > config_.window_size ||
    config_.min_pps_samples > config_.window_size ||
    !std::isfinite(config_.max_offset_jump_s) || config_.max_offset_jump_s <= 0.0 ||
    !std::isfinite(config_.coarse_timeout_s) || config_.coarse_timeout_s <= 0.0 ||
    !std::isfinite(config_.pps_timeout_s) || config_.pps_timeout_s <= 0.0 ||
    !std::isfinite(config_.coarse_uncertainty_floor_s) ||
    config_.coarse_uncertainty_floor_s < 0.0 ||
    !std::isfinite(config_.pps_uncertainty_floor_s) ||
    config_.pps_uncertainty_floor_s < 0.0)
  {
    throw std::invalid_argument("time-sync configuration is outside valid bounds");
  }
}

ClockObservationResult TimeSyncEstimator::observe(const ClockObservation & observation)
{
  if (!std::isfinite(observation.source_time_s) ||
    !std::isfinite(observation.reference_time_s) ||
    !std::isfinite(observation.uncertainty_s) || observation.uncertainty_s < 0.0)
  {
    ++diagnostics_.nonfinite_rejections;
    return ClockObservationResult::RejectedNonFinite;
  }

  if (!observations_.empty()) {
    const auto & previous = observations_.back();
    if (observation.source_time_s == previous.source_time_s &&
      observation.reference_time_s == previous.reference_time_s)
    {
      ++diagnostics_.duplicate_rejections;
      return ClockObservationResult::RejectedDuplicate;
    }
  }

  bool reset_required = false;
  if (!observations_.empty()) {
    const auto & previous = observations_.back();
    reset_required = observation.source_time_s <= previous.source_time_s ||
      observation.reference_time_s <= previous.reference_time_s;

    if (!reset_required) {
      std::vector<double> offsets;
      offsets.reserve(observations_.size());
      for (const auto & sample : observations_) {
        offsets.push_back(sample.reference_time_s - sample.source_time_s);
      }
      const double new_offset = observation.reference_time_s - observation.source_time_s;
      reset_required = std::abs(new_offset - median(std::move(offsets))) >
        config_.max_offset_jump_s;
    }
    const bool first_pps_after_coarse = observation.pps_locked &&
      std::none_of(
      observations_.begin(), observations_.end(), [](const ClockObservation & sample) {
        return sample.pps_locked;
      });
    reset_required = reset_required || first_pps_after_coarse;
  }

  if (reset_required) {
    observations_.clear();
    ++diagnostics_.resets;
  }
  observations_.push_back(observation);
  while (observations_.size() > config_.window_size) {
    observations_.pop_front();
  }
  ++diagnostics_.accepted;
  return reset_required ? ClockObservationResult::ResetAccepted : ClockObservationResult::Accepted;
}

std::optional<TimeMapping> TimeSyncEstimator::map(
  const double source_time_s,
  const double now_source_time_s) const
{
  if (!std::isfinite(source_time_s) || !std::isfinite(now_source_time_s) || observations_.empty()) {
    return std::nullopt;
  }

  TimeMapping mapping;
  mapping.state = stateFor(now_source_time_s);
  mapping.source_time_s = source_time_s;
  mapping.sample_count = observations_.size();
  mapping.pps_sample_count = static_cast<std::size_t>(std::count_if(
      observations_.begin(), observations_.end(), [](const ClockObservation & sample) {
        return sample.pps_locked;
      }));

  std::vector<double> offsets;
  std::vector<double> uncertainties;
  offsets.reserve(observations_.size());
  uncertainties.reserve(observations_.size());
  const bool pps_only = mapping.state == TimeSyncState::PpsLocked;
  for (const auto & sample : observations_) {
    if (!pps_only || sample.pps_locked) {
      offsets.push_back(sample.reference_time_s - sample.source_time_s);
      uncertainties.push_back(sample.uncertainty_s);
    }
  }
  if (offsets.empty()) {
    return std::nullopt;
  }
  mapping.offset_s = median(offsets);
  double residual_sum_squared = 0.0;
  for (const double offset : offsets) {
    const double residual = offset - mapping.offset_s;
    residual_sum_squared += residual * residual;
  }
  mapping.jitter_s = std::sqrt(residual_sum_squared / static_cast<double>(offsets.size()));
  const double measurement_uncertainty = median(std::move(uncertainties));
  const double floor = mapping.state == TimeSyncState::PpsLocked ?
    config_.pps_uncertainty_floor_s : config_.coarse_uncertainty_floor_s;
  mapping.uncertainty_s = std::max(
    floor, std::hypot(mapping.jitter_s, measurement_uncertainty));
  mapping.mapped_time_s = source_time_s + mapping.offset_s;
  return mapping;
}

TimeSyncState TimeSyncEstimator::state(const double now_source_time_s) const
{
  return stateFor(now_source_time_s);
}

void TimeSyncEstimator::reset()
{
  observations_.clear();
  ++diagnostics_.resets;
}

TimeSyncState TimeSyncEstimator::stateFor(const double now_source_time_s) const
{
  if (!std::isfinite(now_source_time_s) || observations_.empty()) {
    return TimeSyncState::Unsynced;
  }
  if (now_source_time_s < observations_.back().source_time_s ||
    now_source_time_s - observations_.back().source_time_s > config_.coarse_timeout_s)
  {
    return TimeSyncState::Unsynced;
  }
  std::size_t pps_samples = 0;
  std::optional<double> latest_pps_source_time;
  for (const auto & sample : observations_) {
    if (sample.pps_locked) {
      ++pps_samples;
      latest_pps_source_time = sample.source_time_s;
    }
  }
  if (pps_samples >= config_.min_pps_samples && latest_pps_source_time.has_value() &&
    now_source_time_s >= *latest_pps_source_time &&
    now_source_time_s - *latest_pps_source_time <= config_.pps_timeout_s)
  {
    return TimeSyncState::PpsLocked;
  }
  if (observations_.size() >= config_.min_coarse_samples) {
    return TimeSyncState::Coarse;
  }
  return TimeSyncState::Unsynced;
}

const char * toString(const TimeSyncState state) noexcept
{
  switch (state) {
    case TimeSyncState::Unsynced:
      return "UNSYNCED";
    case TimeSyncState::Coarse:
      return "COARSE";
    case TimeSyncState::PpsLocked:
      return "PPS_LOCKED";
  }
  return "UNSYNCED";
}

}  // namespace fgo_gil_localizer

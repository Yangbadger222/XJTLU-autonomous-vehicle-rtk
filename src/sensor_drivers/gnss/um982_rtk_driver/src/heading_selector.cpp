#include "um982_rtk_driver/heading_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace um982_rtk_driver
{
namespace
{

constexpr double kFullCircleDeg = 360.0;
constexpr double kHalfCircleDeg = 180.0;

}  // namespace

std::optional<HeadingSource> parseHeadingSource(const std::string & value)
{
  if (value == "THS") {
    return HeadingSource::THS;
  }
  if (value == "HPR") {
    return HeadingSource::HPR;
  }
  if (value == "UNIHEADING") {
    return HeadingSource::UNIHEADING;
  }
  return std::nullopt;
}

const char * headingSourceName(HeadingSource source)
{
  switch (source) {
    case HeadingSource::THS:
      return "THS";
    case HeadingSource::HPR:
      return "HPR";
    case HeadingSource::UNIHEADING:
      return "UNIHEADING";
  }
  return "UNKNOWN";
}

HeadingSelector::HeadingSelector(HeadingSelectorConfig config)
: config_(config)
{
  const bool finite_config =
    std::isfinite(config_.fallback_timeout_s) &&
    std::isfinite(config_.max_rate_degps) &&
    std::isfinite(config_.max_step_deg) &&
    std::isfinite(config_.rate_slack_deg);
  if (!finite_config || config_.fallback_timeout_s <= 0.0 ||
    config_.switch_min_samples <= 0 || config_.max_rate_degps <= 0.0 ||
    config_.max_step_deg <= 0.0 || config_.rate_slack_deg < 0.0)
  {
    throw std::invalid_argument("invalid UM982 heading selector configuration");
  }
}

HeadingSelection HeadingSelector::observe(
  HeadingSource source, double calibrated_heading_deg, double received_s)
{
  HeadingSelection result;
  result.source = source;
  if (!std::isfinite(calibrated_heading_deg) || !std::isfinite(received_s)) {
    return result;
  }
  if (!first_observation_s_.has_value()) {
    first_observation_s_ = received_s;
  }
  if (!sourceMayTakeControl(source, received_s)) {
    return result;
  }

  const bool switching = active_source_.has_value() && source != *active_source_;
  if (switching) {
    if (pending_source_ != source) {
      pending_source_ = source;
      pending_source_samples_ = 1;
    } else {
      ++pending_source_samples_;
    }
    if (pending_source_samples_ < config_.switch_min_samples) {
      return result;
    }
  } else {
    resetPendingSource();
  }

  const auto index = sourceIndex(source);
  double source_bias_deg = source_bias_deg_[index];
  if (switching && last_published_heading_deg_.has_value()) {
    source_bias_deg = signedHeadingDeltaDeg(
      *last_published_heading_deg_, calibrated_heading_deg);
  }
  const double candidate_heading_deg = normalizeHeadingDeg(
    calibrated_heading_deg + source_bias_deg);
  if (!switching && !continuityAccepts(candidate_heading_deg, received_s)) {
    ++rejected_count_;
    result.continuity_rejected = true;
    return result;
  }

  if (switching) {
    source_bias_deg_[index] = source_bias_deg;
    result.source_switched = true;
  }
  active_source_ = source;
  last_accepted_s_[index] = received_s;
  last_published_heading_deg_ = candidate_heading_deg;
  last_published_s_ = received_s;
  resetPendingSource();

  result.publish = true;
  result.heading_deg = candidate_heading_deg;
  result.source_bias_deg = source_bias_deg_[index];
  return result;
}

std::optional<HeadingSource> HeadingSelector::activeSource() const
{
  return active_source_;
}

int HeadingSelector::rejectedCount() const
{
  return rejected_count_;
}

bool HeadingSelector::sourceMayTakeControl(HeadingSource source, double received_s) const
{
  if (!active_source_.has_value()) {
    return source == config_.primary_source ||
           (source == config_.fallback_source && first_observation_s_.has_value() &&
           received_s - *first_observation_s_ >= config_.fallback_timeout_s);
  }
  if (source == *active_source_ || source == config_.primary_source) {
    return true;
  }
  if (source != config_.fallback_source) {
    return false;
  }
  const auto primary_time = last_accepted_s_[sourceIndex(config_.primary_source)];
  return !primary_time.has_value() ||
         received_s - *primary_time >= config_.fallback_timeout_s;
}

bool HeadingSelector::continuityAccepts(double heading_deg, double received_s) const
{
  if (!last_published_heading_deg_.has_value() || !last_published_s_.has_value()) {
    return true;
  }
  const double elapsed_s = received_s - *last_published_s_;
  if (elapsed_s <= 0.0) {
    return false;
  }
  const double allowed_delta_deg = std::min(
    config_.max_step_deg,
    config_.max_rate_degps * elapsed_s + config_.rate_slack_deg);
  return std::abs(signedHeadingDeltaDeg(heading_deg, *last_published_heading_deg_)) <=
         allowed_delta_deg;
}

void HeadingSelector::resetPendingSource()
{
  pending_source_.reset();
  pending_source_samples_ = 0;
}

double HeadingSelector::normalizeHeadingDeg(double heading_deg)
{
  double normalized = std::fmod(heading_deg, kFullCircleDeg);
  if (normalized < 0.0) {
    normalized += kFullCircleDeg;
  }
  return normalized;
}

double HeadingSelector::signedHeadingDeltaDeg(double target_deg, double reference_deg)
{
  double delta = normalizeHeadingDeg(target_deg) - normalizeHeadingDeg(reference_deg);
  if (delta > kHalfCircleDeg) {
    delta -= kFullCircleDeg;
  }
  if (delta <= -kHalfCircleDeg) {
    delta += kFullCircleDeg;
  }
  return delta;
}

std::size_t HeadingSelector::sourceIndex(HeadingSource source)
{
  return static_cast<std::size_t>(source);
}

}  // namespace um982_rtk_driver

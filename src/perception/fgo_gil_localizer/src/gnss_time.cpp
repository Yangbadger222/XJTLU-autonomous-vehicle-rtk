#include "fgo_gil_localizer/gnss_time.hpp"

#include <cmath>
#include <stdexcept>

namespace fgo_gil_localizer
{

GnssTimeTracker::GnssTimeTracker(const double max_forward_gap_s)
: max_forward_gap_s_(max_forward_gap_s)
{
  if (!std::isfinite(max_forward_gap_s_) || max_forward_gap_s_ <= 0.0) {
    throw std::invalid_argument("GNSS maximum forward gap must be positive and finite");
  }
}

GnssTimeResult GnssTimeTracker::accept(
  const std::uint16_t week,
  const std::uint32_t milliseconds_of_week)
{
  const auto absolute_seconds = gnssAbsoluteSeconds(week, milliseconds_of_week);
  if (!absolute_seconds.has_value()) {
    ++diagnostics_.invalid;
    return GnssTimeResult::Invalid;
  }
  const GnssTimeSample sample{week, milliseconds_of_week, *absolute_seconds};
  if (!latest_.has_value()) {
    latest_ = sample;
    ++diagnostics_.accepted;
    return GnssTimeResult::Accepted;
  }
  if (sample.week == latest_->week &&
    sample.milliseconds_of_week == latest_->milliseconds_of_week)
  {
    ++diagnostics_.duplicates;
    return GnssTimeResult::Duplicate;
  }
  const double delta_s = sample.absolute_seconds - latest_->absolute_seconds;
  latest_ = sample;
  ++diagnostics_.accepted;
  if (delta_s < 0.0) {
    ++diagnostics_.resets;
    return GnssTimeResult::ResetAccepted;
  }
  if (delta_s > max_forward_gap_s_) {
    ++diagnostics_.gaps;
    return GnssTimeResult::GapAccepted;
  }
  return GnssTimeResult::Accepted;
}

void GnssTimeTracker::reset()
{
  latest_.reset();
  ++diagnostics_.resets;
}

std::optional<double> gnssAbsoluteSeconds(
  const std::uint16_t week,
  const std::uint32_t milliseconds_of_week) noexcept
{
  if (week == 0U || milliseconds_of_week >= kGnssWeekMilliseconds) {
    return std::nullopt;
  }
  return static_cast<double>(week) * kGnssWeekSeconds +
         static_cast<double>(milliseconds_of_week) * 1.0e-3;
}

}  // namespace fgo_gil_localizer

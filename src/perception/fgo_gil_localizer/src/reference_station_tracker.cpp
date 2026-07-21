#include "fgo_gil_localizer/reference_station_tracker.hpp"

#include <cmath>
#include <stdexcept>

namespace fgo_gil_localizer
{

ReferenceStationTracker::ReferenceStationTracker(ReferenceStationTrackerConfig config)
: config_(config)
{
  if (!std::isfinite(config_.change_threshold_m) || config_.change_threshold_m <= 0.0 ||
    !std::isfinite(config_.minimum_ecef_radius_m) ||
    !std::isfinite(config_.maximum_ecef_radius_m) || config_.minimum_ecef_radius_m <= 0.0 ||
    config_.maximum_ecef_radius_m <= config_.minimum_ecef_radius_m)
  {
    throw std::invalid_argument("reference-station tracker configuration is invalid");
  }
}

ReferenceStationUpdate ReferenceStationTracker::accept(const ReferenceStationSample & sample)
{
  const double radius = norm(sample.position_ecef_m);
  if ((sample.message_type != 1005U && sample.message_type != 1006U) ||
    !finite(sample.position_ecef_m) || radius < config_.minimum_ecef_radius_m ||
    radius > config_.maximum_ecef_radius_m)
  {
    return ReferenceStationUpdate::Rejected;
  }
  if (!current_.has_value()) {
    current_ = sample;
    return ReferenceStationUpdate::Initial;
  }
  const bool changed = sample.station_id != current_->station_id ||
    sample.itrf_realization != current_->itrf_realization || sample.source != current_->source ||
    norm(sample.position_ecef_m - current_->position_ecef_m) > config_.change_threshold_m;
  if (changed) {
    current_ = sample;
    return ReferenceStationUpdate::Changed;
  }
  current_->message_type = sample.message_type;
  return ReferenceStationUpdate::Unchanged;
}

}  // namespace fgo_gil_localizer

#include "fgo_gil_localizer/imu_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fgo_gil_localizer
{

ImuSegmentBuffer::ImuSegmentBuffer(ImuBufferConfig config)
: config_(config)
{
  if (config_.capacity == 0U || !std::isfinite(config_.max_gap_s) || config_.max_gap_s <= 0.0) {
    throw std::invalid_argument("IMU buffer configuration is outside valid bounds");
  }
}

ImuBufferResult ImuSegmentBuffer::add(const ImuSample & sample)
{
  if (!std::isfinite(sample.stamp_s) || !finite(sample.acceleration_m_s2) ||
    !finite(sample.angular_velocity_rad_s))
  {
    ++diagnostics_.nonfinite;
    return ImuBufferResult::RejectedNonFinite;
  }

  if (!samples_.empty()) {
    const double delta_s = sample.stamp_s - samples_.back().stamp_s;
    if (delta_s == 0.0) {
      ++diagnostics_.duplicates;
      return ImuBufferResult::RejectedDuplicate;
    }
    if (delta_s < 0.0) {
      ++diagnostics_.time_reversals;
      startNewSegment(sample);
      return ImuBufferResult::ResetOnTimeReversal;
    }
    diagnostics_.maximum_observed_gap_s =
      std::max(diagnostics_.maximum_observed_gap_s, delta_s);
    if (delta_s > config_.max_gap_s) {
      ++diagnostics_.gaps;
      startNewSegment(sample);
      return ImuBufferResult::ResetOnGap;
    }
  }

  samples_.push_back(sample);
  ++diagnostics_.accepted;
  if (samples_.size() > config_.capacity) {
    samples_.pop_front();
    ++diagnostics_.evicted;
    return ImuBufferResult::AcceptedWithEviction;
  }
  return ImuBufferResult::Accepted;
}

void ImuSegmentBuffer::reset()
{
  samples_.clear();
  ++diagnostics_.segment_id;
}

void ImuSegmentBuffer::startNewSegment(const ImuSample & sample)
{
  samples_.clear();
  ++diagnostics_.segment_id;
  samples_.push_back(sample);
  ++diagnostics_.accepted;
}

}  // namespace fgo_gil_localizer

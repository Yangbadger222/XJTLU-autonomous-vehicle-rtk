#include "fgo_gil_localizer/imu_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace fgo_gil_localizer
{
namespace
{

std::optional<ImuSample> interpolateImu(
  const std::deque<ImuSample> & samples,
  const double stamp_s)
{
  if (samples.empty() || stamp_s < samples.front().stamp_s || stamp_s > samples.back().stamp_s) {
    return std::nullopt;
  }
  const auto after = std::lower_bound(
    samples.begin(), samples.end(), stamp_s,
    [](const ImuSample & sample, const double value) {return sample.stamp_s < value;});
  if (after == samples.end()) {
    return samples.back();
  }
  if (after->stamp_s == stamp_s || after == samples.begin()) {
    ImuSample output = *after;
    output.stamp_s = stamp_s;
    return output;
  }
  const auto before = std::prev(after);
  const double duration = after->stamp_s - before->stamp_s;
  if (duration <= 0.0) {
    return std::nullopt;
  }
  const double ratio = (stamp_s - before->stamp_s) / duration;
  return ImuSample{
    stamp_s,
    before->acceleration_m_s2 +
    ratio * (after->acceleration_m_s2 - before->acceleration_m_s2),
    before->angular_velocity_rad_s +
    ratio * (after->angular_velocity_rad_s - before->angular_velocity_rad_s)};
}

}  // namespace

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

ImuCoverageResult classifyImuTime(const ImuSegmentBuffer & buffer, const double stamp_s)
{
  if (!std::isfinite(stamp_s)) {
    return ImuCoverageResult::InvalidInterval;
  }
  if (buffer.empty() || stamp_s > buffer.samples().back().stamp_s) {
    return ImuCoverageResult::WaitingForFuture;
  }
  if (stamp_s < buffer.samples().front().stamp_s) {
    return ImuCoverageResult::HistoryUnavailable;
  }
  return ImuCoverageResult::Ready;
}

ImuSegmentSelection selectImuSegment(
  const ImuSegmentBuffer & buffer,
  const double start_s,
  const double end_s)
{
  ImuSegmentSelection output;
  output.segment_id = buffer.diagnostics().segment_id;
  if (!std::isfinite(start_s) || !std::isfinite(end_s) || end_s <= start_s) {
    output.result = ImuCoverageResult::InvalidInterval;
    return output;
  }
  const ImuCoverageResult start = classifyImuTime(buffer, start_s);
  if (start != ImuCoverageResult::Ready) {
    output.result = start;
    return output;
  }
  const ImuCoverageResult end = classifyImuTime(buffer, end_s);
  if (end != ImuCoverageResult::Ready) {
    output.result = end;
    return output;
  }
  const auto start_sample = interpolateImu(buffer.samples(), start_s);
  const auto end_sample = interpolateImu(buffer.samples(), end_s);
  if (!start_sample.has_value() || !end_sample.has_value()) {
    output.result = ImuCoverageResult::InvalidInterval;
    return output;
  }
  output.samples.push_back(*start_sample);
  for (const auto & sample : buffer.samples()) {
    if (sample.stamp_s > start_s && sample.stamp_s < end_s) {
      output.samples.push_back(sample);
    }
  }
  output.samples.push_back(*end_sample);
  output.result = ImuCoverageResult::Ready;
  return output;
}

}  // namespace fgo_gil_localizer

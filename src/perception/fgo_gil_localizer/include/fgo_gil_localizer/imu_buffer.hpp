#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "fgo_gil_localizer/math_types.hpp"

namespace fgo_gil_localizer
{

struct ImuSample
{
  double stamp_s = 0.0;
  Vec3 acceleration_m_s2;
  Vec3 angular_velocity_rad_s;
};

enum class ImuBufferResult : std::uint8_t
{
  Accepted,
  AcceptedWithEviction,
  ResetOnGap,
  ResetOnTimeReversal,
  RejectedDuplicate,
  RejectedNonFinite,
};

struct ImuBufferConfig
{
  std::size_t capacity = 4096;
  double max_gap_s = 0.05;
};

struct ImuBufferDiagnostics
{
  std::uint64_t accepted = 0;
  std::uint64_t evicted = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t time_reversals = 0;
  std::uint64_t gaps = 0;
  std::uint64_t nonfinite = 0;
  std::uint64_t segment_id = 0;
  double maximum_observed_gap_s = 0.0;
};

class ImuSegmentBuffer
{
public:
  explicit ImuSegmentBuffer(ImuBufferConfig config = {});

  ImuBufferResult add(const ImuSample & sample);
  void reset();

  const std::deque<ImuSample> & samples() const noexcept {return samples_;}
  const ImuBufferDiagnostics & diagnostics() const noexcept {return diagnostics_;}
  std::size_t size() const noexcept {return samples_.size();}
  bool empty() const noexcept {return samples_.empty();}

private:
  void startNewSegment(const ImuSample & sample);

  ImuBufferConfig config_;
  std::deque<ImuSample> samples_;
  ImuBufferDiagnostics diagnostics_;
};

}  // namespace fgo_gil_localizer

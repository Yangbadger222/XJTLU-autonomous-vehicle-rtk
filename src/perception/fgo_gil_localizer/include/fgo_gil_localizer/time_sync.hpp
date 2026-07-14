#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace fgo_gil_localizer
{

enum class TimeSyncState : std::uint8_t
{
  Unsynced = 0,
  Coarse = 1,
  PpsLocked = 2,
};

enum class ClockObservationResult : std::uint8_t
{
  Accepted,
  ResetAccepted,
  RejectedDuplicate,
  RejectedNonFinite,
};

struct TimeSyncConfig
{
  std::size_t window_size = 32;
  std::size_t min_coarse_samples = 5;
  std::size_t min_pps_samples = 2;
  double max_offset_jump_s = 0.5;
  double coarse_timeout_s = 5.0;
  double pps_timeout_s = 2.0;
  double coarse_uncertainty_floor_s = 0.02;
  double pps_uncertainty_floor_s = 1.0e-4;
};

struct ClockObservation
{
  double source_time_s = 0.0;
  double reference_time_s = 0.0;
  double uncertainty_s = 0.0;
  bool pps_locked = false;
};

struct TimeMapping
{
  TimeSyncState state = TimeSyncState::Unsynced;
  double source_time_s = 0.0;
  double mapped_time_s = 0.0;
  double offset_s = 0.0;
  double jitter_s = 0.0;
  double uncertainty_s = 0.0;
  std::size_t sample_count = 0;
  std::size_t pps_sample_count = 0;
};

struct TimeSyncDiagnostics
{
  std::uint64_t accepted = 0;
  std::uint64_t duplicate_rejections = 0;
  std::uint64_t nonfinite_rejections = 0;
  std::uint64_t resets = 0;
};

class TimeSyncEstimator
{
public:
  explicit TimeSyncEstimator(TimeSyncConfig config = {});

  ClockObservationResult observe(const ClockObservation & observation);
  std::optional<TimeMapping> map(double source_time_s, double now_source_time_s) const;
  TimeSyncState state(double now_source_time_s) const;
  void reset();

  const TimeSyncDiagnostics & diagnostics() const noexcept {return diagnostics_;}
  std::size_t size() const noexcept {return observations_.size();}

private:
  TimeSyncState stateFor(double now_source_time_s) const;

  TimeSyncConfig config_;
  std::deque<ClockObservation> observations_;
  TimeSyncDiagnostics diagnostics_;
};

const char * toString(TimeSyncState state) noexcept;

}  // namespace fgo_gil_localizer

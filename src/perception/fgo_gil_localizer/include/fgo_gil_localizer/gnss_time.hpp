#pragma once

#include <cstdint>
#include <optional>

namespace fgo_gil_localizer
{

constexpr double kGnssWeekSeconds = 604800.0;
constexpr std::uint32_t kGnssWeekMilliseconds = 604800000U;
constexpr std::uint8_t kGnssClockReferenceReceiver = 1U;

constexpr bool isGnssClockReferenceReceiver(const std::uint8_t receiver) noexcept
{
  return receiver == kGnssClockReferenceReceiver;
}

enum class GnssTimeResult : std::uint8_t
{
  Accepted,
  GapAccepted,
  ResetAccepted,
  Duplicate,
  Invalid,
};

struct GnssTimeSample
{
  std::uint16_t week = 0;
  std::uint32_t milliseconds_of_week = 0;
  double absolute_seconds = 0.0;
};

struct GnssTimeDiagnostics
{
  std::uint64_t accepted = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t gaps = 0;
  std::uint64_t resets = 0;
  std::uint64_t invalid = 0;
};

class GnssTimeTracker
{
public:
  explicit GnssTimeTracker(double max_forward_gap_s = 10.0);

  GnssTimeResult accept(std::uint16_t week, std::uint32_t milliseconds_of_week);
  void reset();

  const std::optional<GnssTimeSample> & latest() const noexcept {return latest_;}
  const GnssTimeDiagnostics & diagnostics() const noexcept {return diagnostics_;}

private:
  double max_forward_gap_s_;
  std::optional<GnssTimeSample> latest_;
  GnssTimeDiagnostics diagnostics_;
};

std::optional<double> gnssAbsoluteSeconds(
  std::uint16_t week,
  std::uint32_t milliseconds_of_week) noexcept;

}  // namespace fgo_gil_localizer

#pragma once

#include <optional>
#include <string>

namespace rtk_fgo_localizer
{

enum class RtkGateMode
{
  Rejected,
  DiagnosticOnly,
  WeakCandidate,
  StrongCandidate,
};

struct RtkQuality
{
  int quality = 0;
  int satellites = 0;
  double hdop = 99.0;
  bool heading_stable = false;

  bool is_fixed() const;
  bool is_float() const;
};

struct RtkGateDecision
{
  RtkGateMode mode = RtkGateMode::Rejected;
  std::string reason;
};

std::optional<RtkQuality> parseGgaQuality(const std::string & sentence);

RtkGateDecision evaluateRtkGate(
  const RtkQuality & quality,
  double position_innovation_m,
  double heading_innovation_rad);

}  // namespace rtk_fgo_localizer

#include "rtk_fgo_localizer/rtk_quality.hpp"

#include <cmath>
#include <sstream>
#include <vector>

namespace rtk_fgo_localizer
{
namespace
{

constexpr int kRtkFixedQuality = 4;
constexpr int kRtkFloatQuality = 5;
constexpr int kMinSatellites = 10;
constexpr double kMaxHdop = 2.0;
constexpr double kMaxStrongInnovationM = 3.0;
constexpr double kMaxWeakInnovationM = 5.0;
constexpr double kMaxHeadingInnovationRad = 0.5;

std::vector<std::string> splitCommaFields(const std::string & sentence)
{
  std::vector<std::string> fields;
  std::stringstream stream(sentence);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::optional<int> parseIntField(const std::string & value)
{
  try {
    size_t parsed = 0;
    const int out = std::stoi(value, &parsed);
    if (parsed != value.size()) {
      return std::nullopt;
    }
    return out;
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<double> parseDoubleField(const std::string & value)
{
  try {
    size_t parsed = 0;
    const double out = std::stod(value, &parsed);
    if (parsed != value.size()) {
      return std::nullopt;
    }
    return out;
  } catch (...) {
    return std::nullopt;
  }
}

bool isGgaSentenceType(const std::string & field)
{
  if (field.size() < 3) {
    return false;
  }
  return field.compare(field.size() - 3, 3, "GGA") == 0;
}

RtkGateDecision reject(const std::string & reason)
{
  return {RtkGateMode::Rejected, reason};
}

}  // namespace

bool RtkQuality::is_fixed() const
{
  return quality == kRtkFixedQuality;
}

bool RtkQuality::is_float() const
{
  return quality == kRtkFloatQuality;
}

std::optional<RtkQuality> parseGgaQuality(const std::string & sentence)
{
  const auto fields = splitCommaFields(sentence);
  if (fields.size() <= 8 || !isGgaSentenceType(fields.front())) {
    return std::nullopt;
  }

  const auto quality = parseIntField(fields[6]);
  const auto satellites = parseIntField(fields[7]);
  const auto hdop = parseDoubleField(fields[8]);
  if (!quality.has_value() || !satellites.has_value() || !hdop.has_value()) {
    return std::nullopt;
  }

  RtkQuality out;
  out.quality = *quality;
  out.satellites = *satellites;
  out.hdop = *hdop;
  return out;
}

RtkGateDecision evaluateRtkGate(
  const RtkQuality & quality,
  double position_innovation_m,
  double heading_innovation_rad)
{
  if (!std::isfinite(position_innovation_m) || !std::isfinite(heading_innovation_rad)) {
    return reject("non-finite innovation");
  }
  const double position_innovation_abs_m = std::abs(position_innovation_m);
  const double heading_innovation_abs_rad = std::abs(heading_innovation_rad);
  if (quality.quality <= 0) {
    return reject("invalid RTK quality");
  }
  if (quality.satellites < kMinSatellites) {
    return reject("too few satellites");
  }
  if (!std::isfinite(quality.hdop) || quality.hdop > kMaxHdop) {
    return reject("HDOP too high");
  }
  if (heading_innovation_abs_rad > kMaxHeadingInnovationRad) {
    return reject("heading innovation too large");
  }
  if (position_innovation_abs_m > kMaxWeakInnovationM) {
    return reject("position innovation too large");
  }
  if (quality.is_fixed()) {
    if (position_innovation_abs_m > kMaxStrongInnovationM) {
      return reject("fixed RTK position innovation too large");
    }
    return {RtkGateMode::StrongCandidate, "RTK fixed quality accepted"};
  }
  if (quality.is_float()) {
    return {RtkGateMode::WeakCandidate, "RTK float quality accepted weakly"};
  }
  return {RtkGateMode::DiagnosticOnly, "non-RTK quality kept for diagnostics"};
}

}  // namespace rtk_fgo_localizer

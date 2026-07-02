#include "rtk_fgo_localizer/rtk_quality.hpp"

#include "rtk_fgo_localizer/gate_params.hpp"

#include <cmath>
#include <sstream>
#include <vector>

namespace rtk_fgo_localizer
{
namespace
{

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
  return quality == RtkGateParams{}.fixed_quality_code;
}

bool RtkQuality::is_float() const
{
  return quality == RtkGateParams{}.float_quality_code;
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

RtkGateDecision evaluateRtkGate(const RtkGateInput & input, const RtkGateParams & params)
{
  const auto & quality = input.quality;
  if (!std::isfinite(input.position_innovation_m) ||
    !std::isfinite(input.heading_innovation_rad))
  {
    return reject("non-finite innovation");
  }
  if (input.implied_speed_mps.has_value() && !std::isfinite(*input.implied_speed_mps)) {
    return reject("non-finite implied speed");
  }
  const double position_innovation_abs_m = std::abs(input.position_innovation_m);
  const double heading_innovation_abs_rad = std::abs(input.heading_innovation_rad);
  if (quality.quality <= 0) {
    return reject("invalid RTK quality");
  }
  if (quality.satellites < params.min_satellites) {
    return reject("too few satellites");
  }
  if (!std::isfinite(quality.hdop) || quality.hdop > params.max_hdop) {
    return reject("HDOP too high");
  }
  if (heading_innovation_abs_rad > params.max_heading_innovation_rad) {
    return reject("heading innovation too large");
  }
  if (input.implied_speed_mps.has_value() &&
    std::abs(*input.implied_speed_mps) > params.max_implied_speed_mps)
  {
    return reject("implied RTK speed too high");
  }
  if (position_innovation_abs_m > params.max_weak_position_innovation_m) {
    return reject("position innovation too large");
  }
  if (quality.quality == params.fixed_quality_code) {
    if (position_innovation_abs_m > params.max_strong_position_innovation_m) {
      return reject("fixed RTK position innovation too large");
    }
    return {RtkGateMode::StrongCandidate, "RTK fixed quality accepted"};
  }
  if (quality.quality == params.float_quality_code) {
    return {RtkGateMode::WeakCandidate, "RTK float quality accepted weakly"};
  }
  return {RtkGateMode::DiagnosticOnly, "non-RTK quality kept for diagnostics"};
}

RtkGateDecision evaluateRtkGate(
  const RtkQuality & quality,
  double position_innovation_m,
  double heading_innovation_rad)
{
  RtkGateInput input;
  input.quality = quality;
  input.position_innovation_m = position_innovation_m;
  input.heading_innovation_rad = heading_innovation_rad;
  return evaluateRtkGate(input, RtkGateParams{});
}

}  // namespace rtk_fgo_localizer

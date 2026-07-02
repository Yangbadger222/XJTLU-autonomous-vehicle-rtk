#include "rtk_fgo_localizer/heading_stability.hpp"

#include "rtk_fgo_localizer/correction_smoother.hpp"

#include <cmath>
#include <limits>

namespace rtk_fgo_localizer
{

double headingSpreadRad(const std::vector<double> & yaw_samples)
{
  if (yaw_samples.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double max_delta = 0.0;
  for (std::size_t i = 0; i < yaw_samples.size(); ++i) {
    if (!std::isfinite(yaw_samples[i])) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    for (std::size_t j = i + 1; j < yaw_samples.size(); ++j) {
      if (!std::isfinite(yaw_samples[j])) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      max_delta = std::max(max_delta, std::abs(normalizeYaw(yaw_samples[i] - yaw_samples[j])));
    }
  }
  return max_delta;
}

bool headingWindowStable(const std::vector<double> & yaw_samples, double max_spread_rad)
{
  if (yaw_samples.empty() || !std::isfinite(max_spread_rad)) {
    return false;
  }
  const double spread = headingSpreadRad(yaw_samples);
  return std::isfinite(spread) && spread <= max_spread_rad;
}

std::optional<double> meanYawRad(const std::vector<double> & yaw_samples)
{
  if (yaw_samples.empty()) {
    return std::nullopt;
  }
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const double yaw : yaw_samples) {
    if (!std::isfinite(yaw)) {
      return std::nullopt;
    }
    sin_sum += std::sin(yaw);
    cos_sum += std::cos(yaw);
  }
  if (sin_sum == 0.0 && cos_sum == 0.0) {
    return std::nullopt;
  }
  return normalizeYaw(std::atan2(sin_sum, cos_sum));
}

}  // namespace rtk_fgo_localizer

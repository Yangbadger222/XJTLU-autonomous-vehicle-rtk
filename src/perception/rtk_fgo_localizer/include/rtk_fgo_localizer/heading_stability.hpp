#pragma once

#include <optional>
#include <vector>

namespace rtk_fgo_localizer
{

double headingSpreadRad(const std::vector<double> & yaw_samples);
bool headingWindowStable(const std::vector<double> & yaw_samples, double max_spread_rad);
std::optional<double> meanYawRad(const std::vector<double> & yaw_samples);

}  // namespace rtk_fgo_localizer

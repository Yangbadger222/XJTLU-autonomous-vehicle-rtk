#pragma once

#include "rtk_fgo_localizer/rtk_quality.hpp"

#include <optional>

namespace rtk_fgo_localizer
{

struct RtkGateInput
{
  RtkQuality quality;
  double position_innovation_m = 0.0;
  double heading_innovation_rad = 0.0;
  std::optional<double> implied_speed_mps;
};

struct RtkGateParams
{
  int fixed_quality_code = 4;
  int float_quality_code = 5;
  int min_satellites = 10;
  double max_hdop = 2.0;
  double max_strong_position_innovation_m = 3.0;
  double max_weak_position_innovation_m = 5.0;
  double max_heading_innovation_rad = 0.5;
  double max_implied_speed_mps = 2.0;
};

}  // namespace rtk_fgo_localizer

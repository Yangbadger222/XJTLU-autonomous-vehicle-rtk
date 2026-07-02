#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <optional>

namespace rtk_fgo_localizer
{

struct DiagnosticSnapshot
{
  std::optional<int> rtk_quality;
  std::optional<int> rtk_satellites;
  std::optional<double> rtk_hdop;
  std::optional<double> position_innovation_m;
  std::optional<double> heading_innovation_rad;
  std::optional<double> implied_fix_speed_mps;
  std::optional<double> fix_age_s;
  std::optional<double> nmea_age_s;
  std::optional<double> heading_age_s;
  std::optional<double> fastlio_age_s;
  std::optional<double> wheel_age_s;
};

void appendDiagnosticSnapshot(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const DiagnosticSnapshot & snapshot);

}  // namespace rtk_fgo_localizer

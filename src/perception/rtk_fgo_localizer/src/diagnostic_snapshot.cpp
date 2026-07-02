#include "rtk_fgo_localizer/diagnostic_snapshot.hpp"

#include <diagnostic_msgs/msg/key_value.hpp>

#include <string>

namespace rtk_fgo_localizer
{
namespace
{

void addValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

void addOptionalInt(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::optional<int> & value)
{
  addValue(status, key, value.has_value() ? std::to_string(*value) : "na");
}

void addOptionalDouble(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::optional<double> & value)
{
  addValue(status, key, value.has_value() ? std::to_string(*value) : "na");
}

}  // namespace

void appendDiagnosticSnapshot(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const DiagnosticSnapshot & snapshot)
{
  addOptionalInt(status, "rtk_gga_quality", snapshot.rtk_quality);
  addOptionalInt(status, "rtk_satellites", snapshot.rtk_satellites);
  addOptionalDouble(status, "rtk_hdop", snapshot.rtk_hdop);
  addValue(
    status, "rtk_is_fixed",
    snapshot.rtk_quality.has_value() && *snapshot.rtk_quality == 4 ? "true" : "false");
  addValue(
    status, "rtk_is_float",
    snapshot.rtk_quality.has_value() && *snapshot.rtk_quality == 5 ? "true" : "false");
  addOptionalDouble(status, "position_innovation_m", snapshot.position_innovation_m);
  addOptionalDouble(status, "heading_innovation_rad", snapshot.heading_innovation_rad);
  addOptionalDouble(status, "implied_fix_speed_mps", snapshot.implied_fix_speed_mps);
  addOptionalDouble(status, "fix_age_s", snapshot.fix_age_s);
  addOptionalDouble(status, "nmea_age_s", snapshot.nmea_age_s);
  addOptionalDouble(status, "heading_age_s", snapshot.heading_age_s);
  addOptionalDouble(status, "fastlio_age_s", snapshot.fastlio_age_s);
  addOptionalDouble(status, "wheel_age_s", snapshot.wheel_age_s);
}

}  // namespace rtk_fgo_localizer

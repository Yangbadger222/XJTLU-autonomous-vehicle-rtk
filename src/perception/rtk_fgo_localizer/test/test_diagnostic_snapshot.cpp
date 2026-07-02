#include "rtk_fgo_localizer/diagnostic_snapshot.hpp"

#include <gtest/gtest.h>

namespace
{

std::string valueFor(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key)
{
  for (const auto & value : status.values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "";
}

}  // namespace

TEST(DiagnosticSnapshot, AddsRtkQualityAndGateValues)
{
  rtk_fgo_localizer::DiagnosticSnapshot snapshot;
  snapshot.rtk_quality = 4;
  snapshot.rtk_satellites = 29;
  snapshot.rtk_hdop = 0.6;
  snapshot.position_innovation_m = 12.5;
  snapshot.heading_innovation_rad = 0.25;
  snapshot.implied_fix_speed_mps = 1.2;
  snapshot.fix_age_s = 0.4;
  snapshot.nmea_age_s = 0.2;
  snapshot.heading_age_s = 0.7;
  snapshot.fastlio_age_s = 0.1;
  snapshot.wheel_age_s = 0.05;

  diagnostic_msgs::msg::DiagnosticStatus status;
  rtk_fgo_localizer::appendDiagnosticSnapshot(status, snapshot);

  EXPECT_EQ(valueFor(status, "rtk_gga_quality"), "4");
  EXPECT_EQ(valueFor(status, "rtk_satellites"), "29");
  EXPECT_EQ(valueFor(status, "rtk_is_fixed"), "true");
  EXPECT_EQ(valueFor(status, "rtk_is_float"), "false");
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "rtk_hdop")), 0.6);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "position_innovation_m")), 12.5);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "heading_innovation_rad")), 0.25);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "implied_fix_speed_mps")), 1.2);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "fix_age_s")), 0.4);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "nmea_age_s")), 0.2);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "heading_age_s")), 0.7);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "fastlio_age_s")), 0.1);
  EXPECT_DOUBLE_EQ(std::stod(valueFor(status, "wheel_age_s")), 0.05);
}

TEST(DiagnosticSnapshot, MarksMissingOptionalValuesAsNa)
{
  rtk_fgo_localizer::DiagnosticSnapshot snapshot;
  diagnostic_msgs::msg::DiagnosticStatus status;
  rtk_fgo_localizer::appendDiagnosticSnapshot(status, snapshot);

  EXPECT_EQ(valueFor(status, "rtk_gga_quality"), "na");
  EXPECT_EQ(valueFor(status, "rtk_satellites"), "na");
  EXPECT_EQ(valueFor(status, "rtk_is_fixed"), "false");
  EXPECT_EQ(valueFor(status, "rtk_hdop"), "na");
  EXPECT_EQ(valueFor(status, "position_innovation_m"), "na");
}

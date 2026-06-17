#include <gtest/gtest.h>

#include "um982_rtk_driver/nmea_parser.hpp"

using um982_rtk_driver::buildNmeaSentence;
using um982_rtk_driver::fixQualityText;
using um982_rtk_driver::hasValidChecksum;
using um982_rtk_driver::parseSentence;

TEST(NmeaParser, ValidatesChecksum)
{
  EXPECT_TRUE(hasValidChecksum(buildNmeaSentence("GNTHS,341.3344,A")));
  EXPECT_FALSE(hasValidChecksum("$GNTHS,341.3344,A*00"));
}

TEST(NmeaParser, ParsesUm982GpggaRtkFixed)
{
  const auto parsed = parseSentence(
    buildNmeaSentence(
      "GPGGA,081212.00,3127.49270000,N,12044.25280000,E,4,28,0.6,"
      "66.2344,M,8.4923,M,0.00,0999"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->gga.has_value());
  EXPECT_EQ(parsed->talker, "GP");
  EXPECT_EQ(parsed->type, "GGA");
  EXPECT_EQ(parsed->gga->fix_quality, 4);
  EXPECT_EQ(parsed->gga->satellites, 28);
  EXPECT_DOUBLE_EQ(parsed->gga->hdop, 0.6);
  EXPECT_NEAR(parsed->gga->latitude, 31.4582116667, 1e-9);
  EXPECT_NEAR(parsed->gga->longitude, 120.7375466667, 1e-9);
  EXPECT_EQ(fixQualityText(parsed->gga->fix_quality), "RTK Fixed");
}

TEST(NmeaParser, ParsesUm982ThsHeading)
{
  const auto parsed = parseSentence(buildNmeaSentence("GNTHS,341.3344,A"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->ths.has_value());
  EXPECT_EQ(parsed->talker, "GN");
  EXPECT_EQ(parsed->type, "THS");
  EXPECT_DOUBLE_EQ(parsed->ths->heading_deg, 341.3344);
  EXPECT_EQ(parsed->ths->mode, "A");
}

TEST(NmeaParser, ParsesUm982HprAttitude)
{
  const auto parsed = parseSentence(
    buildNmeaSentence("GNHPR,081212.00,341.48,-00.64,000.00,4,46,0.00,0999"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->hpr.has_value());
  EXPECT_EQ(parsed->type, "HPR");
  EXPECT_DOUBLE_EQ(parsed->hpr->heading_deg, 341.48);
  EXPECT_DOUBLE_EQ(parsed->hpr->pitch_deg, -0.64);
  EXPECT_DOUBLE_EQ(parsed->hpr->roll_deg, 0.0);
  EXPECT_EQ(parsed->hpr->fix_type, 4);
  EXPECT_EQ(parsed->hpr->satellites, 46);
  EXPECT_DOUBLE_EQ(parsed->hpr->differential_age, 0.0);
  EXPECT_EQ(parsed->hpr->station_id, "0999");
}

TEST(NmeaParser, ParsesUnicoreUniheading)
{
  const auto parsed = parseSentence(
    "#UNIHEADINGA,50,GPS,FINE,2207,282484000,0,0,18,675;"
    "SOL_COMPUTED,NARROW_INT,10736.3838,88.3470,0.0876,0.0000,"
    "0.0001,0.0001,\"201\",52,29,29,29,3,01,3,c3*898773d6");

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->uniheading.has_value());
  EXPECT_EQ(parsed->type, "UNIHEADING");
  EXPECT_EQ(parsed->uniheading->solution_status, "SOL_COMPUTED");
  EXPECT_EQ(parsed->uniheading->position_type, "NARROW_INT");
  EXPECT_DOUBLE_EQ(parsed->uniheading->baseline_length, 10736.3838);
  EXPECT_DOUBLE_EQ(parsed->uniheading->heading_deg, 88.3470);
  EXPECT_DOUBLE_EQ(parsed->uniheading->pitch_deg, 0.0876);
  EXPECT_EQ(parsed->uniheading->satellites, 52);
  EXPECT_EQ(parsed->uniheading->solution_satellites, 29);
}

TEST(NmeaParser, RejectsBadChecksum)
{
  EXPECT_FALSE(parseSentence("$GNHPR,081212.00,341.48,-00.64,000.00,4,46,0.00,0999*00").has_value());
}

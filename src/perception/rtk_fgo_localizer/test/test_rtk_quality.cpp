#include <gtest/gtest.h>

#include "rtk_fgo_localizer/rtk_quality.hpp"

namespace rtk_fgo_localizer
{
namespace
{

TEST(RtkQuality, ParsesGgaQualityFourAsFixed)
{
  auto parsed = parseGgaQuality(
    "$GNGGA,123519,3116.4956,N,12044.2528,E,4,18,0.6,12.3,M,0.0,M,,*00");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->quality, 4);
  EXPECT_TRUE(parsed->is_fixed());
  EXPECT_EQ(parsed->satellites, 18);
  EXPECT_NEAR(parsed->hdop, 0.6, 1e-9);
}

TEST(RtkQuality, GatesFixedSampleAsStrongCandidate)
{
  RtkQuality quality;
  quality.quality = 4;
  quality.hdop = 0.6;
  quality.satellites = 18;

  const auto decision = evaluateRtkGate(quality, 0.4, 0.1);

  EXPECT_EQ(decision.mode, RtkGateMode::StrongCandidate);
}

TEST(RtkQuality, FloatIsWeakCandidate)
{
  RtkQuality quality;
  quality.quality = 5;
  quality.hdop = 0.8;
  quality.satellites = 16;

  const auto decision = evaluateRtkGate(quality, 0.5, 0.2);

  EXPECT_EQ(decision.mode, RtkGateMode::WeakCandidate);
}

TEST(RtkQuality, LargeInnovationRejectsEvenFixed)
{
  RtkQuality quality;
  quality.quality = 4;
  quality.hdop = 0.6;
  quality.satellites = 18;

  const auto decision = evaluateRtkGate(quality, 12.0, 0.1);

  EXPECT_EQ(decision.mode, RtkGateMode::Rejected);
}

}  // namespace
}  // namespace rtk_fgo_localizer

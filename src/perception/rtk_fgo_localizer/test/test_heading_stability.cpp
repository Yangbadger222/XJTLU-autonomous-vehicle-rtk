#include <gtest/gtest.h>

#include "rtk_fgo_localizer/heading_stability.hpp"

#include <cmath>
#include <vector>

namespace rtk_fgo_localizer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double deg(double value)
{
  return value * kPi / 180.0;
}

TEST(HeadingStability, TreatsWraparoundSamplesAsStable)
{
  const std::vector<double> samples{deg(179.0), deg(-179.0), deg(180.0)};

  EXPECT_TRUE(headingWindowStable(samples, deg(3.0)));
  EXPECT_NEAR(headingSpreadRad(samples), deg(2.0), 1e-9);
}

TEST(HeadingStability, RejectsLargeBootstrapJump)
{
  const std::vector<double> samples{deg(-140.0), deg(-101.0)};

  EXPECT_FALSE(headingWindowStable(samples, deg(3.0)));
}

TEST(HeadingStability, AveragesStableWindowAcrossWraparound)
{
  const std::vector<double> samples{deg(358.0), deg(0.0), deg(2.0)};

  const auto mean = meanYawRad(samples);

  ASSERT_TRUE(mean.has_value());
  EXPECT_NEAR(*mean, 0.0, 1e-9);
}

}  // namespace
}  // namespace rtk_fgo_localizer

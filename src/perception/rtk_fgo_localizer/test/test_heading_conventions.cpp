#include <gtest/gtest.h>

#include "rtk_fgo_localizer/heading_conventions.hpp"

#include <cmath>

namespace rtk_fgo_localizer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(HeadingConventions, ConvertsCompassQuaternionYawToEnuYaw)
{
  const double yaw = headingQuaternionYawToEnuYaw(15.0 * kPi / 180.0, true);

  EXPECT_NEAR(yaw, 75.0 * kPi / 180.0, 1e-9);
}

TEST(HeadingConventions, LeavesRosEnuQuaternionYawInRosConvention)
{
  const double yaw = headingQuaternionYawToEnuYaw(-30.0 * kPi / 180.0, false);

  EXPECT_NEAR(yaw, -30.0 * kPi / 180.0, 1e-9);
}

TEST(HeadingConventions, NormalizesCompassWraparound)
{
  const double yaw = headingQuaternionYawToEnuYaw(350.0 * kPi / 180.0, true);

  EXPECT_NEAR(yaw, 100.0 * kPi / 180.0, 1e-9);
}

}  // namespace
}  // namespace rtk_fgo_localizer

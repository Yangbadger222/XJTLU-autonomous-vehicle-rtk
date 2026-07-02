#include <gtest/gtest.h>

#include "rtk_fgo_localizer/correction_smoother.hpp"

#include <cmath>

namespace rtk_fgo_localizer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(CorrectionSmoother, LimitsTranslationStep)
{
  CorrectionSmoother smoother(0.15, 0.3 * kPi / 180.0);
  auto out = smoother.step({1.0, 0.0, 0.0});

  EXPECT_NEAR(out.dx, 0.15, 1e-6);
  EXPECT_NEAR(out.dy, 0.0, 1e-6);
}

TEST(CorrectionSmoother, PreservesTranslationDirection)
{
  CorrectionSmoother smoother(0.5, 0.3 * kPi / 180.0);
  auto out = smoother.step({3.0, 4.0, 0.0});

  EXPECT_NEAR(out.dx, 0.3, 1e-6);
  EXPECT_NEAR(out.dy, 0.4, 1e-6);
}

TEST(CorrectionSmoother, LimitsYawStep)
{
  CorrectionSmoother smoother(0.15, 0.3 * kPi / 180.0);
  auto out = smoother.step({0.0, 0.0, 10.0 * kPi / 180.0});

  EXPECT_NEAR(out.dyaw, 0.3 * kPi / 180.0, 1e-6);
}

TEST(CorrectionSmoother, NormalizesYawBeforeLimiting)
{
  CorrectionSmoother smoother(0.15, 5.0 * kPi / 180.0);
  auto out = smoother.step({0.0, 0.0, 370.0 * kPi / 180.0});

  EXPECT_NEAR(out.dyaw, 5.0 * kPi / 180.0, 1e-6);
}

}  // namespace
}  // namespace rtk_fgo_localizer

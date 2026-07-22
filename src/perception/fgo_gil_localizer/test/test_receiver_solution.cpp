#include <gtest/gtest.h>

#include <Eigen/Core>

#include "fgo_gil_localizer/receiver_solution.hpp"

namespace fgo_gil_localizer
{
namespace
{

TEST(ReceiverSolution, ParsesFixedGgaQualityAndAppliesAcceptanceGate)
{
  const auto quality = parseGgaRtkQuality(
    "$GNGGA,031244.00,3158.0000,N,11847.0000,E,4,16,0.7,15.0,M,0.0,M,,*00");
  ASSERT_TRUE(quality.has_value());
  EXPECT_EQ(quality->quality, 4);
  EXPECT_EQ(quality->satellites, 16);
  EXPECT_DOUBLE_EQ(quality->hdop, 0.7);
  EXPECT_TRUE(isAcceptedFixedRtk(*quality, 4, 10, 2.0));
  EXPECT_FALSE(isAcceptedFixedRtk(*quality, 5, 10, 2.0));
  EXPECT_FALSE(parseGgaRtkQuality("$GNRMC,031244.00,A,,,,,,,,*00").has_value());
}

TEST(ReceiverSolution, ConvertsGeodeticPositionAndEnuCovarianceToEcef)
{
  const auto ecef = geodeticToEcef(0.0, 0.0, 0.0);
  ASSERT_TRUE(ecef.has_value());
  EXPECT_NEAR(ecef->x, 6378137.0, 1.0e-6);
  EXPECT_NEAR(ecef->y, 0.0, 1.0e-9);
  EXPECT_NEAR(ecef->z, 0.0, 1.0e-9);

  Eigen::Matrix3d covariance_enu = Eigen::Matrix3d::Zero();
  covariance_enu(0, 0) = 4.0;
  covariance_enu(1, 1) = 9.0;
  covariance_enu(2, 2) = 16.0;
  const auto covariance_ecef = enuCovarianceToEcef(0.0, 0.0, covariance_enu);
  ASSERT_TRUE(covariance_ecef.has_value());
  EXPECT_NEAR((*covariance_ecef)(0, 0), 16.0, 1.0e-12);
  EXPECT_NEAR((*covariance_ecef)(1, 1), 4.0, 1.0e-12);
  EXPECT_NEAR((*covariance_ecef)(2, 2), 9.0, 1.0e-12);
}

TEST(ReceiverSolution, RejectsInvalidCoordinatesAndCovariance)
{
  EXPECT_FALSE(geodeticToEcef(91.0, 0.0, 0.0).has_value());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  covariance(2, 2) = 0.0;
  EXPECT_FALSE(enuCovarianceToEcef(0.0, 0.0, covariance).has_value());
}

}  // namespace
}  // namespace fgo_gil_localizer

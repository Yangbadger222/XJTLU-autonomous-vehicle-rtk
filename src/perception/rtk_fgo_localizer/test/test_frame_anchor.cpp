#include <gtest/gtest.h>

#include "rtk_fgo_localizer/frame_anchor.hpp"

#include <cmath>
#include <limits>
#include <optional>

namespace rtk_fgo_localizer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(FrameAnchor, FirstFixMapsToReferencePoseTranslation)
{
  FrameAnchor anchor;
  GeoPoint origin{31.0, 121.0, 10.0};
  gtsam::Pose3 fgo_pose(gtsam::Rot3::Yaw(0.0), gtsam::Point3(4.0, 5.0, 0.0));

  ASSERT_TRUE(anchor.initialize(origin, fgo_pose, std::nullopt));
  auto mapped = anchor.geoToMap(origin);

  ASSERT_TRUE(mapped.has_value());
  EXPECT_NEAR(mapped->x(), 4.0, 1e-6);
  EXPECT_NEAR(mapped->y(), 5.0, 1e-6);
}

TEST(FrameAnchor, EastDisplacementRespectsAnchorYaw)
{
  FrameAnchor anchor;
  GeoPoint origin{31.0, 121.0, 10.0};
  gtsam::Pose3 fgo_pose(gtsam::Rot3::Yaw(kPi / 2.0), gtsam::Point3(0.0, 0.0, 0.0));

  ASSERT_TRUE(anchor.initialize(origin, fgo_pose, 0.0));
  auto east = anchor.enuToMap(gtsam::Point3(1.0, 0.0, 0.0));

  EXPECT_NEAR(east.x(), 0.0, 1e-6);
  EXPECT_NEAR(east.y(), 1.0, 1e-6);
}

TEST(FrameAnchor, NorthDisplacementRespectsAnchorYaw)
{
  FrameAnchor anchor;
  GeoPoint origin{31.0, 121.0, 10.0};
  gtsam::Pose3 fgo_pose(gtsam::Rot3::Yaw(0.0), gtsam::Point3(0.0, 0.0, 0.0));

  ASSERT_TRUE(anchor.initialize(origin, fgo_pose, kPi / 2.0));
  auto north = anchor.enuToMap(gtsam::Point3(0.0, 1.0, 0.0));

  EXPECT_NEAR(north.x(), 1.0, 1e-6);
  EXPECT_NEAR(north.y(), 0.0, 1e-6);
}

TEST(FrameAnchor, EnuYawMapsIntoReferenceFrameYaw)
{
  FrameAnchor anchor;
  GeoPoint origin{31.0, 121.0, 10.0};
  gtsam::Pose3 fgo_pose(gtsam::Rot3::Yaw(kPi / 2.0), gtsam::Point3(0.0, 0.0, 0.0));

  ASSERT_TRUE(anchor.initialize(origin, fgo_pose, 0.0));
  const auto mapped_yaw = anchor.enuYawToMapYaw(0.0);

  ASSERT_TRUE(mapped_yaw.has_value());
  EXPECT_NEAR(*mapped_yaw, kPi / 2.0, 1e-6);
}

TEST(FrameAnchor, WraparoundYawIsHandledByRotation)
{
  FrameAnchor anchor;
  GeoPoint origin{31.0, 121.0, 10.0};
  gtsam::Pose3 fgo_pose(gtsam::Rot3::Yaw(-kPi + 0.01), gtsam::Point3(0.0, 0.0, 0.0));

  ASSERT_TRUE(anchor.initialize(origin, fgo_pose, kPi - 0.01));
  auto east = anchor.enuToMap(gtsam::Point3(1.0, 0.0, 0.0));

  EXPECT_NEAR(east.x(), std::cos(0.02), 1e-6);
  EXPECT_NEAR(east.y(), std::sin(0.02), 1e-6);
}

TEST(FrameAnchor, RejectsNonFiniteOrigin)
{
  FrameAnchor anchor;
  GeoPoint origin{std::numeric_limits<double>::quiet_NaN(), 121.0, 10.0};

  EXPECT_FALSE(anchor.initialize(origin, gtsam::Pose3(), std::nullopt));
  EXPECT_FALSE(anchor.initialized());
}

TEST(FrameAnchor, GeoToMapReturnsNulloptBeforeInitialization)
{
  FrameAnchor anchor;

  EXPECT_FALSE(anchor.geoToMap(GeoPoint{31.0, 121.0, 10.0}).has_value());
}

}  // namespace
}  // namespace rtk_fgo_localizer

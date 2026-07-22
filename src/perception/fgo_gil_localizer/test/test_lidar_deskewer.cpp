#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "fgo_gil_localizer/lidar_deskewer.hpp"

namespace fgo_gil_localizer
{
namespace
{

std::vector<ImuSample> constantImu(
  const Vec3 & acceleration,
  const Vec3 & angular_velocity)
{
  std::vector<ImuSample> output;
  for (int index = 0; index <= 20; ++index) {
    output.push_back({index * 0.01, acceleration, angular_velocity});
  }
  return output;
}

TEST(LidarDeskewer, AlignsRotatingMeasurementsToScanEnd)
{
  LidarDeskewConfig config;
  config.gravity_world_m_s2 = {};
  LidarDeskewer deskewer(config);
  const Vec3 fixed_world_point{10.0, 0.0, 0.0};
  std::vector<TimedLidarPoint> points;
  for (int index = 0; index <= 10; ++index) {
    const double offset_s = index * 0.01;
    const Quaternion rotation = quaternionFromRotationVector({0.0, 0.0, offset_s});
    points.push_back(
      {offset_s, rotation.conjugate().rotate(fixed_world_point), 10.0, 0U});
  }

  const auto output = deskewer.deskew(
    points, 0.0, constantImu({}, {0.0, 0.0, 1.0}), DeskewInitialState{});

  ASSERT_EQ(output.result, DeskewResult::Success);
  const Vec3 expected =
    quaternionFromRotationVector({0.0, 0.0, 0.1}).conjugate().rotate(fixed_world_point);
  for (std::size_t index = 0; index < output.points_at_scan_end.size(); ++index) {
    const auto & point = output.points_at_scan_end[index];
    EXPECT_NEAR(point.position.x, expected.x, 1.0e-8);
    EXPECT_NEAR(point.position.y, expected.y, 1.0e-8);
    EXPECT_NEAR(point.position.z, expected.z, 1.0e-8);
    EXPECT_NEAR(point.offset_s, points[index].offset_s, 1.0e-12);
  }
}

TEST(LidarDeskewer, AlignsConstantVelocityMeasurementsToScanEnd)
{
  LidarDeskewConfig config;
  config.gravity_world_m_s2 = {};
  LidarDeskewer deskewer(config);
  std::vector<TimedLidarPoint> points;
  for (int index = 0; index <= 10; ++index) {
    const double offset_s = index * 0.01;
    points.push_back({offset_s, {10.0 - offset_s, 0.0, 0.0}, 10.0, 0U});
  }
  DeskewInitialState initial;
  initial.velocity_world_m_s = {1.0, 0.0, 0.0};

  const auto output = deskewer.deskew(points, 0.0, constantImu({}, {}), initial);

  ASSERT_EQ(output.result, DeskewResult::Success);
  for (const auto & point : output.points_at_scan_end) {
    EXPECT_NEAR(point.position.x, 9.9, 1.0e-10);
  }
}

TEST(LidarDeskewer, RejectsGapAndInsufficientCoverage)
{
  LidarDeskewConfig config;
  config.gravity_world_m_s2 = {};
  config.maximum_imu_gap_s = 0.02;
  LidarDeskewer deskewer(config);
  const std::vector<TimedLidarPoint> points{
    {0.0, {2.0, 0.0, 0.0}, 0.0, 0U},
    {0.1, {2.0, 0.0, 0.0}, 0.0, 0U}};

  EXPECT_EQ(
    deskewer.deskew(
      points, 0.0, {{0.0, {}, {}}, {0.10, {}, {}}}, DeskewInitialState{}).result,
    DeskewResult::ImuGap);
  EXPECT_EQ(
    deskewer.deskew(
      points, 0.0, {{0.0, {}, {}}, {0.05, {}, {}}}, DeskewInitialState{}).result,
    DeskewResult::ImuGap);
}

TEST(LidarDeskewer, RejectsDuplicateReversedAndNonFiniteImu)
{
  LidarDeskewConfig config;
  config.gravity_world_m_s2 = {};
  LidarDeskewer deskewer(config);
  const std::vector<TimedLidarPoint> points{
    {0.0, {2.0, 0.0, 0.0}, 0.0, 0U},
    {0.01, {2.0, 0.0, 0.0}, 0.0, 0U}};

  EXPECT_EQ(
    deskewer.deskew(
      points, 0.0, {{0.0, {}, {}}, {0.0, {}, {}}, {0.01, {}, {}}},
      DeskewInitialState{}).result,
    DeskewResult::ImuDuplicate);
  EXPECT_EQ(
    deskewer.deskew(
      points, 0.0, {{0.0, {}, {}}, {0.02, {}, {}}, {0.01, {}, {}}},
      DeskewInitialState{}).result,
    DeskewResult::ImuTimeReversal);
  auto invalid = ImuSample{0.01, {}, {}};
  invalid.acceleration_m_s2.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    deskewer.deskew(
      points, 0.0, {{0.0, {}, {}}, invalid, {0.02, {}, {}}},
      DeskewInitialState{}).result,
    DeskewResult::InvalidInput);
}

}  // namespace
}  // namespace fgo_gil_localizer

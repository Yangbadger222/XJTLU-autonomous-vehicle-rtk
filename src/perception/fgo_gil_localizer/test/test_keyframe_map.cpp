#include <gtest/gtest.h>

#include "fgo_gil_localizer/keyframe_map.hpp"

namespace fgo_gil_localizer
{
namespace
{

TEST(KeyframePolicy, TriggersOnMotionTimeAndGnssAvailability)
{
  KeyframePolicy policy;
  RigidPose pose;
  EXPECT_EQ(policy.evaluate(1.0, pose, false), KeyframeTrigger::FirstFrame);
  policy.record(1.0, pose, false);
  EXPECT_EQ(policy.evaluate(1.1, pose, false), KeyframeTrigger::None);
  pose.translation.x = 0.6;
  EXPECT_EQ(policy.evaluate(1.1, pose, false), KeyframeTrigger::Translation);
  pose.translation.x = 0.0;
  EXPECT_EQ(policy.evaluate(1.1, pose, true), KeyframeTrigger::GnssAvailabilityChange);
  EXPECT_EQ(policy.evaluate(2.1, pose, false), KeyframeTrigger::ElapsedTime);
  policy.reset();
  EXPECT_EQ(policy.evaluate(3.0, pose, false), KeyframeTrigger::FirstFrame);
}

TEST(KeyframeMap, BoundsStorageAndBuildsRadiusLimitedSubmap)
{
  KeyframeMapConfig config;
  config.maximum_keyframes = 2U;
  config.submap_radius_m = 2.0;
  config.edge_voxel_size_m = 0.01;
  config.plane_voxel_size_m = 0.01;
  KeyframeMap map(config);
  LidarFeatureSet features;
  features.edge_points.push_back({0.0, 0.0, 0.0});
  features.plane_points.push_back({0.0, 0.0, 0.0});
  RigidPose pose;
  map.add(1.0, pose, features);
  pose.translation.x = 1.0;
  map.add(2.0, pose, features);
  pose.translation.x = 10.0;
  map.add(3.0, pose, features);

  EXPECT_EQ(map.size(), 2U);
  const auto near_first_region = map.submap({1.0, 0.0, 0.0});
  EXPECT_EQ(near_first_region.edge_points.size(), 1U);
  EXPECT_EQ(near_first_region.plane_points.size(), 1U);
  const auto near_last = map.submap({10.0, 0.0, 0.0});
  EXPECT_EQ(near_last.edge_points.size(), 1U);
  EXPECT_EQ(near_last.plane_points.size(), 1U);
}

}  // namespace
}  // namespace fgo_gil_localizer

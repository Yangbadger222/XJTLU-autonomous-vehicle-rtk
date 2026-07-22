#include <gtest/gtest.h>

#include <vector>

#include "fgo_gil_localizer/lidar_feature_extractor.hpp"
#include "fgo_gil_localizer/lidar_matcher.hpp"

namespace fgo_gil_localizer
{
namespace
{

TEST(LidarFeatureExtractor, FindsFlatAndDiscontinuousRegionsAcrossLines)
{
  LidarFeatureConfig config;
  config.edge_curvature_threshold = 0.005;
  config.plane_curvature_threshold = 1.0e-6;
  LidarFeatureExtractor extractor(config);
  std::vector<TimedLidarPoint> points;
  for (std::uint8_t line = 0U; line < 4U; ++line) {
    for (int index = 0; index < 40; ++index) {
      const double z = index == 20 ? 1.5 : 0.0;
      points.push_back(
        {0.001 * index, {5.0 + 0.1 * index, 0.2 * line, z}, 20.0, line});
    }
  }

  const auto features = extractor.extract(points);

  EXPECT_GE(features.edge_points.size(), 4U);
  EXPECT_GE(features.plane_points.size(), 20U);
  EXPECT_EQ(extractor.diagnostics().usable_lines, 4U);
}

TEST(LidarMatcher, RejectsSinglePlaneAsDegenerate)
{
  LidarMatcherConfig config;
  config.maximum_neighbor_distance_m = 2.0;
  config.constraint.minimum_line_matches = 1U;
  config.constraint.minimum_plane_matches = 5U;
  LidarMatcher matcher(config);
  LidarFeatureSet source;
  LidarFeatureSet submap;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      const Vec3 point{0.2 * x, 0.2 * y, 0.0};
      submap.plane_points.push_back(point);
      if ((x + y) % 2 == 0) {
        source.plane_points.push_back(point);
      }
    }
  }

  const auto result = matcher.match(source, submap, RigidPose{});

  EXPECT_GE(result.summary.plane_matches, 5U);
  EXPECT_TRUE(result.summary.degenerate);
  EXPECT_FALSE(result.summary.constraint_valid);
}

TEST(LidarMatcher, RejectsCollinearNeighborhoodAsAPlane)
{
  LidarMatcherConfig config;
  config.maximum_neighbor_distance_m = 2.0;
  config.constraint.minimum_line_matches = 1U;
  config.constraint.minimum_plane_matches = 1U;
  LidarMatcher matcher(config);
  LidarFeatureSet source;
  source.plane_points.push_back({0.0, 0.0, 0.0});
  LidarFeatureSet submap;
  for (int index = -3; index <= 3; ++index) {
    submap.plane_points.push_back({0.1 * index, 0.0, 0.0});
  }

  const auto result = matcher.match(source, submap, RigidPose{});

  EXPECT_EQ(result.summary.plane_matches, 0U);
  EXPECT_EQ(result.diagnostics.rejected_plane_geometry, 1U);
  EXPECT_FALSE(result.summary.constraint_valid);
}

}  // namespace
}  // namespace fgo_gil_localizer

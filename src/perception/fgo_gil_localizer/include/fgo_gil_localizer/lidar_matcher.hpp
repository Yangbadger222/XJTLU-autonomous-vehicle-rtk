#pragma once

#include <cstddef>
#include <vector>

#include "fgo_gil_localizer/lidar_factors.hpp"

namespace fgo_gil_localizer
{

struct LidarMatcherConfig
{
  std::size_t nearest_neighbors = 5;
  double maximum_neighbor_distance_m = 1.0;
  double minimum_line_eigen_ratio = 3.0;
  double minimum_line_eigenvalue = 1.0e-4;
  double maximum_plane_eigen_ratio = 0.10;
  double minimum_plane_second_eigenvalue = 1.0e-4;
  double maximum_plane_fit_residual_m = 0.10;
  LidarConstraintConfig constraint;
};

struct LidarMatchDiagnostics
{
  std::size_t edge_queries = 0;
  std::size_t plane_queries = 0;
  std::size_t rejected_neighbor_count = 0;
  std::size_t rejected_line_geometry = 0;
  std::size_t rejected_plane_geometry = 0;
};

struct LidarMatchResult
{
  std::vector<PointToLineFactor> line_factors;
  std::vector<PointToPlaneFactor> plane_factors;
  LidarConstraintSummary summary;
  LidarMatchDiagnostics diagnostics;
};

class LidarMatcher
{
public:
  explicit LidarMatcher(LidarMatcherConfig config = {});

  LidarMatchResult match(
    const LidarFeatureSet & source_lidar,
    const LidarFeatureSet & submap_world,
    const RigidPose & initial_pose_world_lidar) const;

private:
  LidarMatcherConfig config_;
};

}  // namespace fgo_gil_localizer

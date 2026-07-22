#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "fgo_gil_localizer/lidar_types.hpp"

namespace fgo_gil_localizer
{

struct LidarFeatureConfig
{
  std::size_t neighbor_span = 2;
  std::size_t minimum_points_per_line = 9;
  std::size_t maximum_edge_features_per_line = 30;
  std::size_t maximum_plane_features_per_line = 80;
  std::size_t suppression_radius = 2;
  double edge_curvature_threshold = 0.02;
  double plane_curvature_threshold = 0.001;
};

struct LidarFeatureDiagnostics
{
  std::uint64_t frames = 0;
  std::size_t input_points = 0;
  std::size_t usable_lines = 0;
  std::size_t edge_candidates = 0;
  std::size_t plane_candidates = 0;
  std::size_t edge_features = 0;
  std::size_t plane_features = 0;
};

class LidarFeatureExtractor
{
public:
  explicit LidarFeatureExtractor(LidarFeatureConfig config = {});

  LidarFeatureSet extract(const std::vector<TimedLidarPoint> & points);
  const LidarFeatureDiagnostics & diagnostics() const noexcept {return diagnostics_;}

private:
  LidarFeatureConfig config_;
  LidarFeatureDiagnostics diagnostics_;
};

}  // namespace fgo_gil_localizer

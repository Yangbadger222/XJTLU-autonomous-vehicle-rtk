#include "fgo_gil_localizer/lidar_feature_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace fgo_gil_localizer
{
namespace
{

using Candidate = std::pair<double, std::size_t>;

void suppressNeighbors(
  std::vector<bool> & suppressed,
  const std::size_t center,
  const std::size_t radius)
{
  const std::size_t begin = center > radius ? center - radius : 0U;
  const std::size_t end = std::min(suppressed.size() - 1U, center + radius);
  for (std::size_t index = begin; index <= end; ++index) {
    suppressed[index] = true;
  }
}

}  // namespace

LidarFeatureExtractor::LidarFeatureExtractor(LidarFeatureConfig config)
: config_(config)
{
  if (config_.neighbor_span == 0U ||
    config_.minimum_points_per_line < 2U * config_.neighbor_span + 1U ||
    config_.maximum_edge_features_per_line == 0U ||
    config_.maximum_plane_features_per_line == 0U ||
    !std::isfinite(config_.edge_curvature_threshold) ||
    !std::isfinite(config_.plane_curvature_threshold) ||
    config_.edge_curvature_threshold <= config_.plane_curvature_threshold ||
    config_.plane_curvature_threshold < 0.0)
  {
    throw std::invalid_argument("LiDAR feature configuration is outside valid bounds");
  }
}

LidarFeatureSet LidarFeatureExtractor::extract(const std::vector<TimedLidarPoint> & points)
{
  ++diagnostics_.frames;
  diagnostics_.input_points = points.size();
  diagnostics_.usable_lines = 0U;
  diagnostics_.edge_candidates = 0U;
  diagnostics_.plane_candidates = 0U;
  diagnostics_.edge_features = 0U;
  diagnostics_.plane_features = 0U;

  std::map<std::uint8_t, std::vector<TimedLidarPoint>> lines;
  for (const auto & point : points) {
    if (finite(point.position) && std::isfinite(point.offset_s) && point.offset_s >= 0.0) {
      lines[point.line].push_back(point);
    }
  }

  LidarFeatureSet output;
  for (auto & entry : lines) {
    auto & line = entry.second;
    if (line.size() < config_.minimum_points_per_line) {
      continue;
    }
    ++diagnostics_.usable_lines;
    std::stable_sort(
      line.begin(), line.end(), [](const TimedLidarPoint & left, const TimedLidarPoint & right) {
        return left.offset_s < right.offset_s;
      });
    std::vector<Candidate> edge_candidates;
    std::vector<Candidate> plane_candidates;
    for (std::size_t index = config_.neighbor_span;
      index + config_.neighbor_span < line.size(); ++index)
    {
      Vec3 difference = -static_cast<double>(2U * config_.neighbor_span) *
        line[index].position;
      for (std::size_t offset = 1U; offset <= config_.neighbor_span; ++offset) {
        difference = difference + line[index - offset].position + line[index + offset].position;
      }
      const double range_squared = squaredNorm(line[index].position);
      const double normalization = std::max(
        range_squared * static_cast<double>(config_.neighbor_span * config_.neighbor_span),
        1.0e-12);
      const double curvature = squaredNorm(difference) / normalization;
      if (!std::isfinite(curvature)) {
        continue;
      }
      if (curvature >= config_.edge_curvature_threshold) {
        edge_candidates.emplace_back(curvature, index);
      } else if (curvature <= config_.plane_curvature_threshold) {
        plane_candidates.emplace_back(curvature, index);
      }
    }
    diagnostics_.edge_candidates += edge_candidates.size();
    diagnostics_.plane_candidates += plane_candidates.size();

    std::sort(
      edge_candidates.begin(), edge_candidates.end(),
      [](const Candidate & left, const Candidate & right) {return left.first > right.first;});
    std::sort(
      plane_candidates.begin(), plane_candidates.end(),
      [](const Candidate & left, const Candidate & right) {return left.first < right.first;});
    std::vector<bool> suppressed(line.size(), false);
    std::size_t edge_count = 0U;
    for (const auto & candidate : edge_candidates) {
      if (edge_count >= config_.maximum_edge_features_per_line) {
        break;
      }
      if (suppressed[candidate.second]) {
        continue;
      }
      output.edge_points.push_back(line[candidate.second].position);
      ++edge_count;
      suppressNeighbors(suppressed, candidate.second, config_.suppression_radius);
    }
    std::size_t plane_count = 0U;
    for (const auto & candidate : plane_candidates) {
      if (plane_count >= config_.maximum_plane_features_per_line) {
        break;
      }
      if (suppressed[candidate.second]) {
        continue;
      }
      output.plane_points.push_back(line[candidate.second].position);
      ++plane_count;
      suppressNeighbors(suppressed, candidate.second, config_.suppression_radius);
    }
  }
  diagnostics_.edge_features = output.edge_points.size();
  diagnostics_.plane_features = output.plane_points.size();
  return output;
}

}  // namespace fgo_gil_localizer

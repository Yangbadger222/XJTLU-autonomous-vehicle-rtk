#include "fgo_gil_localizer/lidar_matcher.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace fgo_gil_localizer
{
namespace
{

struct LocalGeometry
{
  Vec3 anchor;
  Vec3 direction;
};

std::vector<Vec3> nearestPoints(
  const std::vector<Vec3> & map,
  const Vec3 & query,
  const std::size_t count,
  const double maximum_distance_m)
{
  std::vector<std::pair<double, std::size_t>> distances;
  distances.reserve(map.size());
  const double maximum_distance_squared = maximum_distance_m * maximum_distance_m;
  for (std::size_t index = 0; index < map.size(); ++index) {
    const double distance_squared = squaredNorm(map[index] - query);
    if (std::isfinite(distance_squared) && distance_squared <= maximum_distance_squared) {
      distances.emplace_back(distance_squared, index);
    }
  }
  if (distances.size() < count) {
    return {};
  }
  std::partial_sort(
    distances.begin(), distances.begin() + static_cast<std::ptrdiff_t>(count), distances.end(),
    [](const auto & left, const auto & right) {return left.first < right.first;});
  std::vector<Vec3> output;
  output.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    output.push_back(map[distances[index].second]);
  }
  return output;
}

Eigen::Matrix3d covariance(const std::vector<Vec3> & points, Vec3 & centroid_output)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & point : points) {
    centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  centroid /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
  for (const auto & point : points) {
    const Eigen::Vector3d delta = Eigen::Vector3d(point.x, point.y, point.z) - centroid;
    covariance_matrix.noalias() += delta * delta.transpose();
  }
  covariance_matrix /= static_cast<double>(points.size());
  centroid_output = {centroid.x(), centroid.y(), centroid.z()};
  return covariance_matrix;
}

std::optional<LocalGeometry> fitLine(
  const std::vector<Vec3> & points,
  const double minimum_eigen_ratio,
  const double minimum_eigenvalue)
{
  if (points.size() < 3U) {
    return std::nullopt;
  }
  Vec3 centroid;
  const Eigen::Matrix3d covariance_matrix = covariance(points, centroid);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance_matrix);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }
  const auto eigenvalues = solver.eigenvalues();
  if (eigenvalues(2) < minimum_eigenvalue ||
    eigenvalues(2) <= minimum_eigen_ratio * std::max(eigenvalues(1), 1.0e-12))
  {
    return std::nullopt;
  }
  const auto direction = solver.eigenvectors().col(2);
  return LocalGeometry{centroid, {direction.x(), direction.y(), direction.z()}};
}

std::optional<LocalGeometry> fitPlane(
  const std::vector<Vec3> & points,
  const double maximum_eigen_ratio,
  const double minimum_second_eigenvalue,
  const double maximum_fit_residual_m)
{
  if (points.size() < 3U) {
    return std::nullopt;
  }
  Vec3 centroid;
  const Eigen::Matrix3d covariance_matrix = covariance(points, centroid);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance_matrix);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }
  const auto eigenvalues = solver.eigenvalues();
  if (eigenvalues(1) < minimum_second_eigenvalue ||
    eigenvalues(0) > maximum_eigen_ratio * eigenvalues(1))
  {
    return std::nullopt;
  }
  const auto eigen_normal = solver.eigenvectors().col(0);
  const Vec3 normal{eigen_normal.x(), eigen_normal.y(), eigen_normal.z()};
  for (const auto & point : points) {
    if (std::abs(dot(normal, point - centroid)) > maximum_fit_residual_m) {
      return std::nullopt;
    }
  }
  return LocalGeometry{centroid, normal};
}

}  // namespace

LidarMatcher::LidarMatcher(LidarMatcherConfig config)
: config_(config)
{
  if (config_.nearest_neighbors < 3U ||
    !std::isfinite(config_.maximum_neighbor_distance_m) ||
    config_.maximum_neighbor_distance_m <= 0.0 ||
    !std::isfinite(config_.minimum_line_eigen_ratio) ||
    config_.minimum_line_eigen_ratio <= 1.0 ||
    !std::isfinite(config_.minimum_line_eigenvalue) ||
    config_.minimum_line_eigenvalue <= 0.0 ||
    !std::isfinite(config_.maximum_plane_eigen_ratio) ||
    config_.maximum_plane_eigen_ratio <= 0.0 || config_.maximum_plane_eigen_ratio >= 1.0 ||
    !std::isfinite(config_.minimum_plane_second_eigenvalue) ||
    config_.minimum_plane_second_eigenvalue <= 0.0 ||
    !std::isfinite(config_.maximum_plane_fit_residual_m) ||
    config_.maximum_plane_fit_residual_m <= 0.0)
  {
    throw std::invalid_argument("LiDAR matcher configuration is outside valid bounds");
  }
  analyzeLidarFactors({}, {}, RigidPose{}, config_.constraint);
}

LidarMatchResult LidarMatcher::match(
  const LidarFeatureSet & source_lidar,
  const LidarFeatureSet & submap_world,
  const RigidPose & initial_pose_world_lidar) const
{
  LidarMatchResult output;
  if (!finite(initial_pose_world_lidar)) {
    return output;
  }
  for (const auto & source : source_lidar.edge_points) {
    ++output.diagnostics.edge_queries;
    const Vec3 query_world = transformPoint(initial_pose_world_lidar, source);
    const auto neighbors = nearestPoints(
      submap_world.edge_points, query_world, config_.nearest_neighbors,
      config_.maximum_neighbor_distance_m);
    if (neighbors.empty()) {
      ++output.diagnostics.rejected_neighbor_count;
      continue;
    }
    const auto line = fitLine(
      neighbors, config_.minimum_line_eigen_ratio, config_.minimum_line_eigenvalue);
    if (!line.has_value()) {
      ++output.diagnostics.rejected_line_geometry;
      continue;
    }
    output.line_factors.push_back({source, line->anchor, line->direction});
  }
  for (const auto & source : source_lidar.plane_points) {
    ++output.diagnostics.plane_queries;
    const Vec3 query_world = transformPoint(initial_pose_world_lidar, source);
    const auto neighbors = nearestPoints(
      submap_world.plane_points, query_world, config_.nearest_neighbors,
      config_.maximum_neighbor_distance_m);
    if (neighbors.empty()) {
      ++output.diagnostics.rejected_neighbor_count;
      continue;
    }
    const auto plane = fitPlane(
      neighbors, config_.maximum_plane_eigen_ratio,
      config_.minimum_plane_second_eigenvalue, config_.maximum_plane_fit_residual_m);
    if (!plane.has_value()) {
      ++output.diagnostics.rejected_plane_geometry;
      continue;
    }
    output.plane_factors.push_back({source, plane->anchor, plane->direction});
  }
  output.summary = analyzeLidarFactors(
    output.line_factors, output.plane_factors, initial_pose_world_lidar, config_.constraint);
  return output;
}

}  // namespace fgo_gil_localizer

#include "fgo_gil_localizer/lidar_factors.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fgo_gil_localizer
{
namespace
{

std::optional<Vec3> normalized(const Vec3 & value)
{
  const double magnitude = norm(value);
  if (!std::isfinite(magnitude) || magnitude <= 1.0e-12) {
    return std::nullopt;
  }
  return value / magnitude;
}

PoseJacobianRow pointJacobianRow(const Vec3 & point_world, const Vec3 & direction_world)
{
  const Vec3 rotation = cross(point_world, direction_world);
  return {
    direction_world.x, direction_world.y, direction_world.z,
    rotation.x, rotation.y, rotation.z};
}

double huberWeight(const double residual, const double delta)
{
  const double magnitude = std::abs(residual);
  return magnitude <= delta || magnitude <= 1.0e-15 ? 1.0 : delta / magnitude;
}

void accumulate(
  Eigen::Matrix<double, 6, 6> & information,
  const PoseJacobianRow & jacobian,
  const double weight)
{
  Eigen::Matrix<double, 1, 6> row;
  for (std::size_t index = 0; index < 6U; ++index) {
    row(static_cast<Eigen::Index>(index)) = jacobian[index];
  }
  information.noalias() += weight * row.transpose() * row;
}

}  // namespace

std::optional<PlaneFactorEvaluation> evaluatePointToPlane(
  const PointToPlaneFactor & factor,
  const RigidPose & pose_world_lidar)
{
  if (!finite(factor.point_lidar) || !finite(factor.plane_anchor_world) ||
    !finite(factor.plane_normal_world) || !finite(pose_world_lidar))
  {
    return std::nullopt;
  }
  const auto normal = normalized(factor.plane_normal_world);
  if (!normal.has_value()) {
    return std::nullopt;
  }
  const Vec3 point_world = transformPoint(pose_world_lidar, factor.point_lidar);
  PlaneFactorEvaluation output;
  output.residual = dot(*normal, point_world - factor.plane_anchor_world);
  output.jacobian = pointJacobianRow(point_world, *normal);
  if (!std::isfinite(output.residual)) {
    return std::nullopt;
  }
  return output;
}

std::optional<LineFactorEvaluation> evaluatePointToLine(
  const PointToLineFactor & factor,
  const RigidPose & pose_world_lidar)
{
  if (!finite(factor.point_lidar) || !finite(factor.line_anchor_world) ||
    !finite(factor.line_direction_world) || !finite(pose_world_lidar))
  {
    return std::nullopt;
  }
  const auto direction = normalized(factor.line_direction_world);
  if (!direction.has_value()) {
    return std::nullopt;
  }
  const Vec3 reference = std::abs(direction->x) < 0.9 ? Vec3{1.0, 0.0, 0.0} :
  Vec3{0.0, 1.0, 0.0};
  const auto first_basis = normalized(cross(*direction, reference));
  if (!first_basis.has_value()) {
    return std::nullopt;
  }
  const Vec3 second_basis = cross(*direction, *first_basis);
  const Vec3 point_world = transformPoint(pose_world_lidar, factor.point_lidar);
  const Vec3 difference = point_world - factor.line_anchor_world;
  LineFactorEvaluation output;
  output.residual = {dot(*first_basis, difference), dot(second_basis, difference)};
  output.jacobian = {
    pointJacobianRow(point_world, *first_basis),
    pointJacobianRow(point_world, second_basis)};
  if (!std::isfinite(output.residual[0]) || !std::isfinite(output.residual[1])) {
    return std::nullopt;
  }
  return output;
}

LidarConstraintSummary analyzeLidarFactors(
  const std::vector<PointToLineFactor> & line_factors,
  const std::vector<PointToPlaneFactor> & plane_factors,
  const RigidPose & pose_world_lidar,
  const LidarConstraintConfig & config)
{
  if (config.minimum_line_matches == 0U || config.minimum_plane_matches == 0U ||
    !std::isfinite(config.huber_delta_m) || config.huber_delta_m <= 0.0 ||
    !std::isfinite(config.minimum_information_eigenvalue) ||
    config.minimum_information_eigenvalue <= 0.0 ||
    !std::isfinite(config.maximum_information_condition) ||
    config.maximum_information_condition <= 1.0)
  {
    throw std::invalid_argument("LiDAR constraint configuration is outside valid bounds");
  }

  LidarConstraintSummary output;
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  double residual_sum_squared = 0.0;
  for (const auto & factor : line_factors) {
    const auto evaluation = evaluatePointToLine(factor, pose_world_lidar);
    if (!evaluation.has_value()) {
      continue;
    }
    ++output.line_matches;
    for (std::size_t row = 0; row < 2U; ++row) {
      const double residual = evaluation->residual[row];
      accumulate(
        information, evaluation->jacobian[row],
        huberWeight(residual, config.huber_delta_m));
      residual_sum_squared += residual * residual;
      ++output.residual_rows;
    }
  }
  for (const auto & factor : plane_factors) {
    const auto evaluation = evaluatePointToPlane(factor, pose_world_lidar);
    if (!evaluation.has_value()) {
      continue;
    }
    ++output.plane_matches;
    accumulate(
      information, evaluation->jacobian,
      huberWeight(evaluation->residual, config.huber_delta_m));
    residual_sum_squared += evaluation->residual * evaluation->residual;
    ++output.residual_rows;
  }
  if (output.residual_rows != 0U) {
    output.residual_rms_m =
      std::sqrt(residual_sum_squared / static_cast<double>(output.residual_rows));
  }
  for (std::size_t row = 0; row < 6U; ++row) {
    for (std::size_t column = 0; column < 6U; ++column) {
      output.information[row * 6U + column] = information(
        static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
    }
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
  if (solver.info() != Eigen::Success) {
    return output;
  }
  for (std::size_t index = 0; index < 6U; ++index) {
    output.information_eigenvalues[index] = solver.eigenvalues()(static_cast<Eigen::Index>(index));
  }
  const double minimum_eigenvalue = output.information_eigenvalues.front();
  const double maximum_eigenvalue = output.information_eigenvalues.back();
  output.information_condition = minimum_eigenvalue > 0.0 ?
    maximum_eigenvalue / minimum_eigenvalue : std::numeric_limits<double>::infinity();
  const bool sufficient_matches = output.line_matches >= config.minimum_line_matches &&
    output.plane_matches >= config.minimum_plane_matches;
  output.degenerate = !std::isfinite(output.information_condition) ||
    minimum_eigenvalue<config.minimum_information_eigenvalue ||
      output.information_condition> config.maximum_information_condition;
  output.constraint_valid = sufficient_matches && !output.degenerate;
  return output;
}

}  // namespace fgo_gil_localizer

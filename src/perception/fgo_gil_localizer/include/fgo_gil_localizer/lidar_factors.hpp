#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "fgo_gil_localizer/lidar_types.hpp"

namespace fgo_gil_localizer
{

using PoseJacobianRow = std::array<double, 6>;
using PoseInformationMatrix = std::array<double, 36>;

struct PointToPlaneFactor
{
  Vec3 point_lidar;
  Vec3 plane_anchor_world;
  Vec3 plane_normal_world;
};

struct PointToLineFactor
{
  Vec3 point_lidar;
  Vec3 line_anchor_world;
  Vec3 line_direction_world;
};

struct PlaneFactorEvaluation
{
  double residual = 0.0;
  PoseJacobianRow jacobian{};
};

struct LineFactorEvaluation
{
  std::array<double, 2> residual{};
  std::array<PoseJacobianRow, 2> jacobian{};
};

std::optional<PlaneFactorEvaluation> evaluatePointToPlane(
  const PointToPlaneFactor & factor,
  const RigidPose & pose_world_lidar);
std::optional<LineFactorEvaluation> evaluatePointToLine(
  const PointToLineFactor & factor,
  const RigidPose & pose_world_lidar);

struct LidarConstraintConfig
{
  std::size_t minimum_line_matches = 8;
  std::size_t minimum_plane_matches = 20;
  double huber_delta_m = 0.20;
  double minimum_information_eigenvalue = 1.0e-3;
  double maximum_information_condition = 1.0e8;
};

struct LidarConstraintSummary
{
  std::size_t line_matches = 0;
  std::size_t plane_matches = 0;
  std::size_t residual_rows = 0;
  double residual_rms_m = 0.0;
  std::array<double, 6> information_eigenvalues{};
  double information_condition = 0.0;
  bool degenerate = true;
  bool constraint_valid = false;
  PoseInformationMatrix information{};
};

LidarConstraintSummary analyzeLidarFactors(
  const std::vector<PointToLineFactor> & line_factors,
  const std::vector<PointToPlaneFactor> & plane_factors,
  const RigidPose & pose_world_lidar,
  const LidarConstraintConfig & config = {});

inline double informationAt(
  const PoseInformationMatrix & information,
  const std::size_t row,
  const std::size_t column)
{
  return information[row * 6U + column];
}

}  // namespace fgo_gil_localizer

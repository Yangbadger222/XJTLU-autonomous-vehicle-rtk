#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "fgo_gil_localizer/math_types.hpp"

namespace fgo_gil_localizer
{

struct RigidPose
{
  Quaternion rotation;
  Vec3 translation;
};

bool finite(const RigidPose & pose) noexcept;
Vec3 transformPoint(const RigidPose & pose, const Vec3 & point);
RigidPose compose(const RigidPose & parent_child, const RigidPose & child_object);
RigidPose inverse(const RigidPose & pose);
RigidPose interpolatePose(const RigidPose & first, const RigidPose & second, double ratio);
RigidPose leftPerturbPose(const RigidPose & pose, const std::array<double, 6> & delta);
double rotationDistanceRad(const Quaternion & first, const Quaternion & second);

struct RawLivoxPoint
{
  std::uint32_t offset_time_ns = 0;
  Vec3 position;
  std::uint8_t reflectivity = 0;
  std::uint8_t tag = 0;
  std::uint8_t line = 0;
};

struct TimedLidarPoint
{
  double offset_s = 0.0;
  Vec3 position;
  double reflectivity = 0.0;
  std::uint8_t line = 0;
};

struct LidarFeatureSet
{
  std::vector<Vec3> edge_points;
  std::vector<Vec3> plane_points;
};

struct DeskewTrajectoryState
{
  double stamp_s = 0.0;
  RigidPose pose_world_imu;
  Vec3 velocity_world_m_s;
};

}  // namespace fgo_gil_localizer

#include "fgo_gil_localizer/keyframe_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>

namespace fgo_gil_localizer
{
namespace
{

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    const std::size_t first = std::hash<std::int64_t>{}(key.x);
    const std::size_t second = std::hash<std::int64_t>{}(key.y);
    const std::size_t third = std::hash<std::int64_t>{}(key.z);
    return first ^ (second << 1U) ^ (third << 2U);
  }
};

std::vector<Vec3> transformAndDownsample(
  const std::vector<Vec3> & input,
  const RigidPose & pose,
  const double voxel_size_m,
  const std::size_t maximum_points)
{
  std::vector<Vec3> output;
  output.reserve(std::min(input.size(), maximum_points));
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  for (const auto & point : input) {
    const Vec3 transformed = transformPoint(pose, point);
    if (!finite(transformed)) {
      continue;
    }
    const VoxelKey key{
      static_cast<std::int64_t>(std::floor(transformed.x / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(transformed.y / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(transformed.z / voxel_size_m))};
    if (occupied.insert(key).second) {
      output.push_back(transformed);
      if (output.size() >= maximum_points) {
        break;
      }
    }
  }
  return output;
}

}  // namespace

KeyframePolicy::KeyframePolicy(KeyframePolicyConfig config)
: config_(config)
{
  if (!std::isfinite(config_.translation_threshold_m) ||
    config_.translation_threshold_m <= 0.0 ||
    !std::isfinite(config_.rotation_threshold_rad) || config_.rotation_threshold_rad <= 0.0 ||
    !std::isfinite(config_.elapsed_time_threshold_s) ||
    config_.elapsed_time_threshold_s <= 0.0)
  {
    throw std::invalid_argument("keyframe policy configuration is outside valid bounds");
  }
}

KeyframeTrigger KeyframePolicy::evaluate(
  const double stamp_s,
  const RigidPose & pose,
  const bool gnss_available) const
{
  if (!std::isfinite(stamp_s) || !finite(pose)) {
    return KeyframeTrigger::None;
  }
  if (!initialized_) {
    return KeyframeTrigger::FirstFrame;
  }
  if (gnss_available != last_gnss_available_) {
    return KeyframeTrigger::GnssAvailabilityChange;
  }
  if (norm(pose.translation - last_pose_.translation) >= config_.translation_threshold_m) {
    return KeyframeTrigger::Translation;
  }
  if (rotationDistanceRad(last_pose_.rotation, pose.rotation) >= config_.rotation_threshold_rad) {
    return KeyframeTrigger::Rotation;
  }
  if (stamp_s >= last_stamp_s_ &&
    stamp_s - last_stamp_s_ >= config_.elapsed_time_threshold_s)
  {
    return KeyframeTrigger::ElapsedTime;
  }
  return KeyframeTrigger::None;
}

void KeyframePolicy::record(
  const double stamp_s,
  const RigidPose & pose,
  const bool gnss_available)
{
  if (!std::isfinite(stamp_s) || !finite(pose)) {
    throw std::invalid_argument("cannot record a non-finite keyframe state");
  }
  initialized_ = true;
  last_stamp_s_ = stamp_s;
  last_pose_ = pose;
  last_gnss_available_ = gnss_available;
}

void KeyframePolicy::reset() noexcept
{
  initialized_ = false;
  last_stamp_s_ = 0.0;
  last_pose_ = {};
  last_gnss_available_ = false;
}

KeyframeMap::KeyframeMap(KeyframeMapConfig config)
: config_(config)
{
  if (config_.maximum_keyframes == 0U ||
    config_.maximum_edge_points_per_keyframe == 0U ||
    config_.maximum_plane_points_per_keyframe == 0U ||
    !std::isfinite(config_.submap_radius_m) || config_.submap_radius_m <= 0.0 ||
    !std::isfinite(config_.edge_voxel_size_m) || config_.edge_voxel_size_m <= 0.0 ||
    !std::isfinite(config_.plane_voxel_size_m) || config_.plane_voxel_size_m <= 0.0)
  {
    throw std::invalid_argument("keyframe map configuration is outside valid bounds");
  }
}

std::uint64_t KeyframeMap::add(
  const double stamp_s,
  const RigidPose & pose_world_lidar,
  const LidarFeatureSet & features_lidar)
{
  if (!std::isfinite(stamp_s) || !finite(pose_world_lidar)) {
    throw std::invalid_argument("cannot add a non-finite keyframe");
  }
  LidarKeyframe keyframe;
  keyframe.id = next_id_++;
  keyframe.stamp_s = stamp_s;
  keyframe.pose_world_lidar = pose_world_lidar;
  keyframe.features_world.edge_points = transformAndDownsample(
    features_lidar.edge_points, pose_world_lidar, config_.edge_voxel_size_m,
    config_.maximum_edge_points_per_keyframe);
  keyframe.features_world.plane_points = transformAndDownsample(
    features_lidar.plane_points, pose_world_lidar, config_.plane_voxel_size_m,
    config_.maximum_plane_points_per_keyframe);
  const std::uint64_t id = keyframe.id;
  keyframes_.push_back(std::move(keyframe));
  while (keyframes_.size() > config_.maximum_keyframes) {
    keyframes_.pop_front();
  }
  return id;
}

LidarFeatureSet KeyframeMap::submap(const Vec3 & center_world) const
{
  LidarFeatureSet output;
  if (!finite(center_world)) {
    return output;
  }
  const double radius_squared = config_.submap_radius_m * config_.submap_radius_m;
  for (const auto & keyframe : keyframes_) {
    if (squaredNorm(keyframe.pose_world_lidar.translation - center_world) > radius_squared) {
      continue;
    }
    output.edge_points.insert(
      output.edge_points.end(), keyframe.features_world.edge_points.begin(),
      keyframe.features_world.edge_points.end());
    output.plane_points.insert(
      output.plane_points.end(), keyframe.features_world.plane_points.begin(),
      keyframe.features_world.plane_points.end());
  }
  return output;
}

void KeyframeMap::clear()
{
  keyframes_.clear();
  next_id_ = 0U;
}

const char * toString(const KeyframeTrigger trigger) noexcept
{
  switch (trigger) {
    case KeyframeTrigger::None:
      return "NONE";
    case KeyframeTrigger::FirstFrame:
      return "FIRST_FRAME";
    case KeyframeTrigger::Translation:
      return "TRANSLATION";
    case KeyframeTrigger::Rotation:
      return "ROTATION";
    case KeyframeTrigger::ElapsedTime:
      return "ELAPSED_TIME";
    case KeyframeTrigger::GnssAvailabilityChange:
      return "GNSS_AVAILABILITY_CHANGE";
  }
  return "NONE";
}

}  // namespace fgo_gil_localizer

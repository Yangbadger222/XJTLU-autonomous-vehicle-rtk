#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "fgo_gil_localizer/lidar_types.hpp"

namespace fgo_gil_localizer
{

enum class KeyframeTrigger : std::uint8_t
{
  None,
  FirstFrame,
  Translation,
  Rotation,
  ElapsedTime,
  GnssAvailabilityChange,
};

struct KeyframePolicyConfig
{
  double translation_threshold_m = 0.5;
  double rotation_threshold_rad = 0.17453292519943295;
  double elapsed_time_threshold_s = 1.0;
};

class KeyframePolicy
{
public:
  explicit KeyframePolicy(KeyframePolicyConfig config = {});

  KeyframeTrigger evaluate(double stamp_s, const RigidPose & pose, bool gnss_available) const;
  void record(double stamp_s, const RigidPose & pose, bool gnss_available);
  void reset() noexcept;
  bool initialized() const noexcept {return initialized_;}

private:
  KeyframePolicyConfig config_;
  bool initialized_ = false;
  double last_stamp_s_ = 0.0;
  RigidPose last_pose_;
  bool last_gnss_available_ = false;
};

struct KeyframeMapConfig
{
  std::size_t maximum_keyframes = 20;
  std::size_t maximum_edge_points_per_keyframe = 300;
  std::size_t maximum_plane_points_per_keyframe = 800;
  double submap_radius_m = 20.0;
  double edge_voxel_size_m = 0.15;
  double plane_voxel_size_m = 0.25;
};

struct LidarKeyframe
{
  std::uint64_t id = 0;
  double stamp_s = 0.0;
  RigidPose pose_world_lidar;
  LidarFeatureSet features_world;
};

class KeyframeMap
{
public:
  explicit KeyframeMap(KeyframeMapConfig config = {});

  std::uint64_t add(
    double stamp_s,
    const RigidPose & pose_world_lidar,
    const LidarFeatureSet & features_lidar);
  LidarFeatureSet submap(const Vec3 & center_world) const;
  void clear();

  const std::deque<LidarKeyframe> & keyframes() const noexcept {return keyframes_;}
  std::size_t size() const noexcept {return keyframes_.size();}

private:
  KeyframeMapConfig config_;
  std::deque<LidarKeyframe> keyframes_;
  std::uint64_t next_id_ = 0;
};

const char * toString(KeyframeTrigger trigger) noexcept;

}  // namespace fgo_gil_localizer

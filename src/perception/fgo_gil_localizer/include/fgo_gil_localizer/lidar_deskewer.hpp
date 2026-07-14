#pragma once

#include <cstdint>
#include <vector>

#include "fgo_gil_localizer/imu_buffer.hpp"
#include "fgo_gil_localizer/lidar_types.hpp"

namespace fgo_gil_localizer
{

struct LidarDeskewConfig
{
  double maximum_imu_gap_s = 0.05;
  double maximum_trajectory_duration_s = 1.0;
  Vec3 accelerometer_bias_m_s2;
  Vec3 gyroscope_bias_rad_s;
  Vec3 gravity_world_m_s2{0.0, 0.0, -9.80665};
  RigidPose pose_imu_lidar;
};

struct DeskewInitialState
{
  double stamp_s = 0.0;
  RigidPose pose_world_imu;
  Vec3 velocity_world_m_s;
};

enum class DeskewResult : std::uint8_t
{
  Success,
  EmptyPointCloud,
  InvalidInput,
  InsufficientImuCoverage,
  ImuDuplicate,
  ImuTimeReversal,
  ImuGap,
};

struct DeskewOutput
{
  DeskewResult result = DeskewResult::InvalidInput;
  std::vector<TimedLidarPoint> points_at_scan_end;
  std::vector<DeskewTrajectoryState> trajectory;
  RigidPose pose_world_lidar_at_end;
  double scan_end_s = 0.0;
  double maximum_imu_gap_s = 0.0;
};

class LidarDeskewer
{
public:
  explicit LidarDeskewer(LidarDeskewConfig config = {});

  DeskewOutput deskew(
    const std::vector<TimedLidarPoint> & points,
    double scan_start_s,
    const std::vector<ImuSample> & imu_samples,
    const DeskewInitialState & initial_state) const;

private:
  LidarDeskewConfig config_;
};

const char * toString(DeskewResult result) noexcept;

}  // namespace fgo_gil_localizer

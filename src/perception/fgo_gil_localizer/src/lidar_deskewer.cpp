#include "fgo_gil_localizer/lidar_deskewer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace fgo_gil_localizer
{
namespace
{

std::optional<ImuSample> interpolateImu(
  const std::vector<ImuSample> & samples,
  const double stamp_s)
{
  const auto upper = std::lower_bound(
    samples.begin(), samples.end(), stamp_s,
    [](const ImuSample & sample, const double stamp) {return sample.stamp_s < stamp;});
  if (upper != samples.end() && upper->stamp_s == stamp_s) {
    return *upper;
  }
  if (upper == samples.begin() || upper == samples.end()) {
    return std::nullopt;
  }
  const auto lower = upper - 1;
  const double duration_s = upper->stamp_s - lower->stamp_s;
  if (duration_s <= 0.0 || !std::isfinite(duration_s)) {
    return std::nullopt;
  }
  const double ratio = (stamp_s - lower->stamp_s) / duration_s;
  return ImuSample{
    stamp_s,
    lower->acceleration_m_s2 + ratio *
    (upper->acceleration_m_s2 - lower->acceleration_m_s2),
    lower->angular_velocity_rad_s + ratio *
    (upper->angular_velocity_rad_s - lower->angular_velocity_rad_s)};
}

std::optional<RigidPose> poseAt(
  const std::vector<DeskewTrajectoryState> & trajectory,
  const double stamp_s)
{
  const auto upper = std::lower_bound(
    trajectory.begin(), trajectory.end(), stamp_s,
    [](const DeskewTrajectoryState & state, const double stamp) {return state.stamp_s < stamp;});
  if (upper != trajectory.end() && std::abs(upper->stamp_s - stamp_s) <= 1.0e-12) {
    return upper->pose_world_imu;
  }
  if (upper == trajectory.begin() || upper == trajectory.end()) {
    return std::nullopt;
  }
  const auto lower = upper - 1;
  const double duration_s = upper->stamp_s - lower->stamp_s;
  if (duration_s <= 0.0) {
    return std::nullopt;
  }
  return interpolatePose(
    lower->pose_world_imu, upper->pose_world_imu,
    (stamp_s - lower->stamp_s) / duration_s);
}

}  // namespace

LidarDeskewer::LidarDeskewer(LidarDeskewConfig config)
: config_(config)
{
  if (!std::isfinite(config_.maximum_imu_gap_s) || config_.maximum_imu_gap_s <= 0.0 ||
    !std::isfinite(config_.maximum_trajectory_duration_s) ||
    config_.maximum_trajectory_duration_s <= 0.0 ||
    !finite(config_.accelerometer_bias_m_s2) || !finite(config_.gyroscope_bias_rad_s) ||
    !finite(config_.gravity_world_m_s2) || !finite(config_.pose_imu_lidar))
  {
    throw std::invalid_argument("LiDAR deskew configuration is outside valid bounds");
  }
}

DeskewOutput LidarDeskewer::deskew(
  const std::vector<TimedLidarPoint> & points,
  const double scan_start_s,
  const std::vector<ImuSample> & imu_samples,
  const DeskewInitialState & initial_state) const
{
  DeskewOutput output;
  if (points.empty()) {
    output.result = DeskewResult::EmptyPointCloud;
    return output;
  }
  if (!std::isfinite(scan_start_s) || !std::isfinite(initial_state.stamp_s) ||
    !finite(initial_state.pose_world_imu) || !finite(initial_state.velocity_world_m_s) ||
    initial_state.stamp_s > scan_start_s)
  {
    output.result = DeskewResult::InvalidInput;
    return output;
  }
  double maximum_offset_s = 0.0;
  for (const auto & point : points) {
    if (!std::isfinite(point.offset_s) || point.offset_s < 0.0 || !finite(point.position)) {
      output.result = DeskewResult::InvalidInput;
      return output;
    }
    maximum_offset_s = std::max(maximum_offset_s, point.offset_s);
  }
  output.scan_end_s = scan_start_s + maximum_offset_s;
  if (!std::isfinite(output.scan_end_s) ||
    output.scan_end_s - initial_state.stamp_s > config_.maximum_trajectory_duration_s)
  {
    output.result = DeskewResult::InvalidInput;
    return output;
  }
  if (imu_samples.size() < 2U) {
    output.result = DeskewResult::InsufficientImuCoverage;
    return output;
  }
  for (std::size_t index = 0; index < imu_samples.size(); ++index) {
    const auto & sample = imu_samples[index];
    if (!std::isfinite(sample.stamp_s) || !finite(sample.acceleration_m_s2) ||
      !finite(sample.angular_velocity_rad_s))
    {
      output.result = DeskewResult::InvalidInput;
      return output;
    }
    if (index == 0U) {
      continue;
    }
    const double gap_s = sample.stamp_s - imu_samples[index - 1U].stamp_s;
    if (gap_s == 0.0) {
      output.result = DeskewResult::ImuDuplicate;
      return output;
    }
    if (gap_s < 0.0) {
      output.result = DeskewResult::ImuTimeReversal;
      return output;
    }
    output.maximum_imu_gap_s = std::max(output.maximum_imu_gap_s, gap_s);
    if (gap_s > config_.maximum_imu_gap_s &&
      sample.stamp_s >= initial_state.stamp_s &&
      imu_samples[index - 1U].stamp_s <= output.scan_end_s)
    {
      output.result = DeskewResult::ImuGap;
      return output;
    }
  }
  if (imu_samples.front().stamp_s > initial_state.stamp_s ||
    imu_samples.back().stamp_s < output.scan_end_s)
  {
    output.result = DeskewResult::InsufficientImuCoverage;
    return output;
  }

  std::vector<double> integration_times{initial_state.stamp_s};
  for (const auto & sample : imu_samples) {
    if (sample.stamp_s > initial_state.stamp_s && sample.stamp_s < output.scan_end_s) {
      integration_times.push_back(sample.stamp_s);
    }
  }
  if (integration_times.back() < output.scan_end_s) {
    integration_times.push_back(output.scan_end_s);
  }
  output.trajectory.reserve(integration_times.size());
  output.trajectory.push_back(
    {initial_state.stamp_s, initial_state.pose_world_imu, initial_state.velocity_world_m_s});
  for (std::size_t index = 1; index < integration_times.size(); ++index) {
    const double previous_time_s = integration_times[index - 1U];
    const double current_time_s = integration_times[index];
    const auto previous_imu = interpolateImu(imu_samples, previous_time_s);
    const auto current_imu = interpolateImu(imu_samples, current_time_s);
    if (!previous_imu.has_value() || !current_imu.has_value()) {
      output.result = DeskewResult::InsufficientImuCoverage;
      output.trajectory.clear();
      return output;
    }
    const double delta_s = current_time_s - previous_time_s;
    const Vec3 angular_velocity = 0.5 *
      (previous_imu->angular_velocity_rad_s + current_imu->angular_velocity_rad_s) -
      config_.gyroscope_bias_rad_s;
    const Vec3 acceleration = 0.5 *
      (previous_imu->acceleration_m_s2 + current_imu->acceleration_m_s2) -
      config_.accelerometer_bias_m_s2;
    const auto & previous_state = output.trajectory.back();
    const Quaternion midpoint_rotation =
      (previous_state.pose_world_imu.rotation *
      quaternionFromRotationVector(0.5 * delta_s * angular_velocity)).normalized();
    const Vec3 acceleration_world =
      midpoint_rotation.rotate(acceleration) + config_.gravity_world_m_s2;
    DeskewTrajectoryState current_state;
    current_state.stamp_s = current_time_s;
    current_state.pose_world_imu.translation =
      previous_state.pose_world_imu.translation +
      previous_state.velocity_world_m_s * delta_s +
      0.5 * acceleration_world * delta_s * delta_s;
    current_state.velocity_world_m_s =
      previous_state.velocity_world_m_s + acceleration_world * delta_s;
    current_state.pose_world_imu.rotation =
      (previous_state.pose_world_imu.rotation *
      quaternionFromRotationVector(delta_s * angular_velocity)).normalized();
    if (!finite(current_state.pose_world_imu) || !finite(current_state.velocity_world_m_s)) {
      output.result = DeskewResult::InvalidInput;
      output.trajectory.clear();
      return output;
    }
    output.trajectory.push_back(current_state);
  }

  const RigidPose pose_world_imu_end = output.trajectory.back().pose_world_imu;
  output.pose_world_lidar_at_end = compose(pose_world_imu_end, config_.pose_imu_lidar);
  const RigidPose pose_lidar_end_world = inverse(output.pose_world_lidar_at_end);
  output.points_at_scan_end.reserve(points.size());
  for (const auto & point : points) {
    const auto pose_world_imu_point = poseAt(output.trajectory, scan_start_s + point.offset_s);
    if (!pose_world_imu_point.has_value()) {
      output.result = DeskewResult::InsufficientImuCoverage;
      output.points_at_scan_end.clear();
      return output;
    }
    const RigidPose pose_world_lidar_point =
      compose(*pose_world_imu_point, config_.pose_imu_lidar);
    TimedLidarPoint deskewed = point;
    deskewed.position = transformPoint(
      pose_lidar_end_world, transformPoint(pose_world_lidar_point, point.position));
    output.points_at_scan_end.push_back(deskewed);
  }
  output.result = DeskewResult::Success;
  return output;
}

const char * toString(const DeskewResult result) noexcept
{
  switch (result) {
    case DeskewResult::Success:
      return "SUCCESS";
    case DeskewResult::EmptyPointCloud:
      return "EMPTY_POINT_CLOUD";
    case DeskewResult::InvalidInput:
      return "INVALID_INPUT";
    case DeskewResult::InsufficientImuCoverage:
      return "INSUFFICIENT_IMU_COVERAGE";
    case DeskewResult::ImuDuplicate:
      return "IMU_DUPLICATE";
    case DeskewResult::ImuTimeReversal:
      return "IMU_TIME_REVERSAL";
    case DeskewResult::ImuGap:
      return "IMU_GAP";
  }
  return "INVALID_INPUT";
}

}  // namespace fgo_gil_localizer

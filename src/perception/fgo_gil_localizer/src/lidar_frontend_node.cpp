#include "fgo_gil_localizer/imu_buffer.hpp"
#include "fgo_gil_localizer/keyframe_map.hpp"
#include "fgo_gil_localizer/lidar_deskewer.hpp"
#include "fgo_gil_localizer/lidar_feature_extractor.hpp"
#include "fgo_gil_localizer/lidar_matcher.hpp"
#include "fgo_gil_localizer/livox_preprocessor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "fgo_gil_msgs/msg/lidar_constraint_batch.hpp"
#include "gnss_raw_msgs/msg/observation_epoch.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace fgo_gil_localizer
{
namespace
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

template<typename T>
diagnostic_msgs::msg::KeyValue numericKeyValue(const std::string & key, const T value)
{
  return keyValue(key, std::to_string(value));
}

}  // namespace

class LidarFrontendNode : public rclcpp::Node
{
public:
  LidarFrontendNode()
  : Node("fgo_gil_lidar_frontend")
  {
    declareParameters();
    readParameters();
    createInterfaces();
  }

private:
  struct OdomState
  {
    double stamp_s = 0.0;
    RigidPose pose_world_imu;
    Vec3 velocity_world_m_s;
  };

  struct PendingScan
  {
    livox_ros_driver2::msg::CustomMsg::SharedPtr message;
    double reception_s = 0.0;
    double scan_start_s = 0.0;
    double scan_end_s = 0.0;
  };

  struct FrameDiagnostics
  {
    std::string state = "WAITING_FOR_INPUT";
    std::string keyframe_trigger = "NONE";
    std::size_t raw_points = 0;
    std::size_t preprocessed_points = 0;
    std::size_t deskewed_points = 0;
    std::size_t edge_features = 0;
    std::size_t plane_features = 0;
    std::size_t submap_edge_features = 0;
    std::size_t submap_plane_features = 0;
    std::size_t line_matches = 0;
    std::size_t plane_matches = 0;
    double residual_rms_m = 0.0;
    double information_min_eigenvalue = 0.0;
    double information_condition = 0.0;
    double maximum_imu_gap_s = 0.0;
    double latency_ms = 0.0;
    bool degenerate = true;
    bool constraint_valid = false;
  };

  void declareParameters()
  {
    declare_parameter<std::string>("topics.imu", "/livox/imu");
    declare_parameter<std::string>("topics.lidar", "/livox/lidar");
    declare_parameter<std::string>("topics.initial_odom", "/fastlio2/lio_odom");
    declare_parameter<std::string>(
      "topics.time_sync_diagnostics", "/fgo_gil/time_sync_diagnostics");
    declare_parameter<std::string>("topics.gnss_epoch", "/gnss/raw/observation_epoch");
    declare_parameter<std::string>("topics.diagnostics", "/fgo_gil/lidar_diagnostics");
    declare_parameter<std::string>("topics.constraints", "/fgo_gil/lidar_constraints");
    declare_parameter<std::string>("frames.lidar_world", "odom");
    declare_parameter<int>("buffers.imu_capacity", 8192);
    declare_parameter<int>("buffers.odom_capacity", 100);
    declare_parameter<int>("buffers.pending_scan_capacity", 4);
    declare_parameter<double>("buffers.pending_scan_timeout_s", 0.5);
    declare_parameter<double>("initialization.maximum_odom_age_s", 0.5);
    declare_parameter<double>("initialization.acceleration_scale", 9.80665);
    declare_parameter<bool>("initialization.require_time_sync", true);
    declare_parameter<double>("initialization.time_sync_timeout_s", 2.0);
    declare_parameter<double>("initialization.gnss_availability_timeout_s", 2.0);
    declare_parameter<int>("preprocessing.point_stride", 1);
    declare_parameter<double>("preprocessing.minimum_range_m", 0.5);
    declare_parameter<double>("preprocessing.maximum_range_m", 25.0);
    declare_parameter<int>("preprocessing.maximum_line_exclusive", 4);
    declare_parameter<double>("deskew.maximum_imu_gap_s", 0.05);
    declare_parameter<double>("deskew.maximum_trajectory_duration_s", 1.0);
    declare_parameter<std::vector<double>>(
      "deskew.accelerometer_bias_m_s2", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("deskew.gyroscope_bias_rad_s", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("deskew.gravity_world_m_s2", {0.0, 0.0, -9.80665});
    declare_parameter<std::vector<double>>(
      "deskew.rotation_imu_lidar_wxyz", {1.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
      "deskew.translation_imu_lidar_m", {-0.011, -0.02329, 0.04412});
    declare_parameter<int>("features.neighbor_span", 2);
    declare_parameter<int>("features.minimum_points_per_line", 9);
    declare_parameter<int>("features.maximum_edge_per_line", 30);
    declare_parameter<int>("features.maximum_plane_per_line", 80);
    declare_parameter<int>("features.suppression_radius", 2);
    declare_parameter<double>("features.edge_curvature_threshold", 0.02);
    declare_parameter<double>("features.plane_curvature_threshold", 0.001);
    declare_parameter<int>("features.minimum_edge_per_frame", 8);
    declare_parameter<int>("features.minimum_plane_per_frame", 20);
    declare_parameter<double>("keyframes.translation_threshold_m", 0.5);
    declare_parameter<double>("keyframes.rotation_threshold_rad", 0.17453292519943295);
    declare_parameter<double>("keyframes.elapsed_time_threshold_s", 1.0);
    declare_parameter<int>("keyframes.maximum_keyframes", 20);
    declare_parameter<int>("keyframes.maximum_edge_points", 300);
    declare_parameter<int>("keyframes.maximum_plane_points", 800);
    declare_parameter<double>("keyframes.submap_radius_m", 20.0);
    declare_parameter<double>("keyframes.edge_voxel_size_m", 0.15);
    declare_parameter<double>("keyframes.plane_voxel_size_m", 0.25);
    declare_parameter<int>("matcher.nearest_neighbors", 5);
    declare_parameter<double>("matcher.maximum_neighbor_distance_m", 1.0);
    declare_parameter<double>("matcher.minimum_line_eigen_ratio", 3.0);
    declare_parameter<double>("matcher.minimum_line_eigenvalue", 1.0e-4);
    declare_parameter<double>("matcher.maximum_plane_eigen_ratio", 0.10);
    declare_parameter<double>("matcher.minimum_plane_second_eigenvalue", 1.0e-4);
    declare_parameter<double>("matcher.maximum_plane_fit_residual_m", 0.10);
    declare_parameter<int>("matcher.minimum_line_matches", 8);
    declare_parameter<int>("matcher.minimum_plane_matches", 20);
    declare_parameter<double>("matcher.huber_delta_m", 0.20);
    declare_parameter<double>("matcher.minimum_information_eigenvalue", 1.0e-3);
    declare_parameter<double>("matcher.maximum_information_condition", 1.0e8);
    declare_parameter<double>("diagnostics_period_s", 1.0);
  }

  Vec3 vec3Parameter(const std::string & name) const
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3U) {
      throw std::invalid_argument(name + " must contain exactly 3 values");
    }
    const Vec3 output{values[0], values[1], values[2]};
    if (!finite(output)) {
      throw std::invalid_argument(name + " must be finite");
    }
    return output;
  }

  Quaternion quaternionParameter(const std::string & name) const
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 4U) {
      throw std::invalid_argument(name + " must contain exactly 4 values in wxyz order");
    }
    const Quaternion normalized =
      Quaternion{values[0], values[1], values[2], values[3]}.normalized();
    if (!finite(normalized)) {
      throw std::invalid_argument(name + " must define a finite nonzero quaternion");
    }
    return normalized;
  }

  std::size_t sizeParameter(const std::string & name) const
  {
    const std::int64_t value = get_parameter(name).as_int();
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  void readParameters()
  {
    imu_topic_ = get_parameter("topics.imu").as_string();
    lidar_topic_ = get_parameter("topics.lidar").as_string();
    odom_topic_ = get_parameter("topics.initial_odom").as_string();
    time_sync_topic_ = get_parameter("topics.time_sync_diagnostics").as_string();
    gnss_epoch_topic_ = get_parameter("topics.gnss_epoch").as_string();
    diagnostics_topic_ = get_parameter("topics.diagnostics").as_string();
    constraints_topic_ = get_parameter("topics.constraints").as_string();
    lidar_world_frame_ = get_parameter("frames.lidar_world").as_string();
    odom_capacity_ = sizeParameter("buffers.odom_capacity");
    pending_scan_capacity_ = sizeParameter("buffers.pending_scan_capacity");
    pending_scan_timeout_s_ = get_parameter("buffers.pending_scan_timeout_s").as_double();
    maximum_odom_age_s_ = get_parameter("initialization.maximum_odom_age_s").as_double();
    acceleration_scale_ = get_parameter("initialization.acceleration_scale").as_double();
    require_time_sync_ = get_parameter("initialization.require_time_sync").as_bool();
    time_sync_timeout_s_ = get_parameter("initialization.time_sync_timeout_s").as_double();
    gnss_availability_timeout_s_ =
      get_parameter("initialization.gnss_availability_timeout_s").as_double();
    diagnostics_period_s_ = get_parameter("diagnostics_period_s").as_double();
    if (!std::isfinite(pending_scan_timeout_s_) || pending_scan_timeout_s_ <= 0.0 ||
      !std::isfinite(maximum_odom_age_s_) || maximum_odom_age_s_ <= 0.0 ||
      !std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0.0 ||
      !std::isfinite(time_sync_timeout_s_) || time_sync_timeout_s_ <= 0.0 ||
      !std::isfinite(gnss_availability_timeout_s_) || gnss_availability_timeout_s_ <= 0.0 ||
      !std::isfinite(diagnostics_period_s_) || diagnostics_period_s_ <= 0.0)
    {
      throw std::invalid_argument("LiDAR frontend timing parameters are outside valid bounds");
    }

    LivoxPreprocessorConfig preprocessing;
    preprocessing.point_stride = sizeParameter("preprocessing.point_stride");
    preprocessing.minimum_range_m = get_parameter("preprocessing.minimum_range_m").as_double();
    preprocessing.maximum_range_m = get_parameter("preprocessing.maximum_range_m").as_double();
    const auto maximum_line = sizeParameter("preprocessing.maximum_line_exclusive");
    if (maximum_line > 255U) {
      throw std::invalid_argument("preprocessing.maximum_line_exclusive exceeds uint8 range");
    }
    preprocessing.maximum_line_exclusive = static_cast<std::uint8_t>(maximum_line);
    preprocessor_ = std::make_unique<LivoxPreprocessor>(preprocessing);

    LidarDeskewConfig deskew;
    deskew.maximum_imu_gap_s = get_parameter("deskew.maximum_imu_gap_s").as_double();
    deskew.maximum_trajectory_duration_s =
      get_parameter("deskew.maximum_trajectory_duration_s").as_double();
    deskew.accelerometer_bias_m_s2 = vec3Parameter("deskew.accelerometer_bias_m_s2");
    deskew.gyroscope_bias_rad_s = vec3Parameter("deskew.gyroscope_bias_rad_s");
    deskew.gravity_world_m_s2 = vec3Parameter("deskew.gravity_world_m_s2");
    deskew.pose_imu_lidar.rotation = quaternionParameter("deskew.rotation_imu_lidar_wxyz");
    deskew.pose_imu_lidar.translation = vec3Parameter("deskew.translation_imu_lidar_m");
    deskewer_ = std::make_unique<LidarDeskewer>(deskew);
    imu_buffer_ = std::make_unique<ImuSegmentBuffer>(
      ImuBufferConfig{sizeParameter("buffers.imu_capacity"), deskew.maximum_imu_gap_s});

    LidarFeatureConfig features;
    features.neighbor_span = sizeParameter("features.neighbor_span");
    features.minimum_points_per_line = sizeParameter("features.minimum_points_per_line");
    features.maximum_edge_features_per_line = sizeParameter("features.maximum_edge_per_line");
    features.maximum_plane_features_per_line = sizeParameter("features.maximum_plane_per_line");
    features.suppression_radius = sizeParameter("features.suppression_radius");
    features.edge_curvature_threshold =
      get_parameter("features.edge_curvature_threshold").as_double();
    features.plane_curvature_threshold =
      get_parameter("features.plane_curvature_threshold").as_double();
    feature_extractor_ = std::make_unique<LidarFeatureExtractor>(features);
    minimum_edge_features_ = sizeParameter("features.minimum_edge_per_frame");
    minimum_plane_features_ = sizeParameter("features.minimum_plane_per_frame");

    KeyframePolicyConfig policy;
    policy.translation_threshold_m =
      get_parameter("keyframes.translation_threshold_m").as_double();
    policy.rotation_threshold_rad = get_parameter("keyframes.rotation_threshold_rad").as_double();
    policy.elapsed_time_threshold_s =
      get_parameter("keyframes.elapsed_time_threshold_s").as_double();
    keyframe_policy_ = std::make_unique<KeyframePolicy>(policy);
    KeyframeMapConfig map;
    map.maximum_keyframes = sizeParameter("keyframes.maximum_keyframes");
    map.maximum_edge_points_per_keyframe = sizeParameter("keyframes.maximum_edge_points");
    map.maximum_plane_points_per_keyframe = sizeParameter("keyframes.maximum_plane_points");
    map.submap_radius_m = get_parameter("keyframes.submap_radius_m").as_double();
    map.edge_voxel_size_m = get_parameter("keyframes.edge_voxel_size_m").as_double();
    map.plane_voxel_size_m = get_parameter("keyframes.plane_voxel_size_m").as_double();
    keyframe_map_ = std::make_unique<KeyframeMap>(map);

    LidarMatcherConfig matcher;
    matcher.nearest_neighbors = sizeParameter("matcher.nearest_neighbors");
    matcher.maximum_neighbor_distance_m =
      get_parameter("matcher.maximum_neighbor_distance_m").as_double();
    matcher.minimum_line_eigen_ratio =
      get_parameter("matcher.minimum_line_eigen_ratio").as_double();
    matcher.minimum_line_eigenvalue =
      get_parameter("matcher.minimum_line_eigenvalue").as_double();
    matcher.maximum_plane_eigen_ratio =
      get_parameter("matcher.maximum_plane_eigen_ratio").as_double();
    matcher.minimum_plane_second_eigenvalue =
      get_parameter("matcher.minimum_plane_second_eigenvalue").as_double();
    matcher.maximum_plane_fit_residual_m =
      get_parameter("matcher.maximum_plane_fit_residual_m").as_double();
    matcher.constraint.minimum_line_matches = sizeParameter("matcher.minimum_line_matches");
    matcher.constraint.minimum_plane_matches = sizeParameter("matcher.minimum_plane_matches");
    matcher.constraint.huber_delta_m = get_parameter("matcher.huber_delta_m").as_double();
    matcher.constraint.minimum_information_eigenvalue =
      get_parameter("matcher.minimum_information_eigenvalue").as_double();
    matcher.constraint.maximum_information_condition =
      get_parameter("matcher.maximum_information_condition").as_double();
    matcher_ = std::make_unique<LidarMatcher>(matcher);
  }

  void createInterfaces()
  {
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    constraints_pub_ =
      create_publisher<fgo_gil_msgs::msg::LidarConstraintBatch>(constraints_topic_, 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarFrontendNode::onImu, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarFrontendNode::onLidar, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50,
      std::bind(&LidarFrontendNode::onOdom, this, std::placeholders::_1));
    time_sync_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      time_sync_topic_, 10,
      std::bind(&LidarFrontendNode::onTimeSync, this, std::placeholders::_1));
    gnss_epoch_sub_ = create_subscription<gnss_raw_msgs::msg::ObservationEpoch>(
      gnss_epoch_topic_, 20,
      [this](gnss_raw_msgs::msg::ObservationEpoch::SharedPtr) {
        last_gnss_reception_s_ = now().seconds();
      });
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_s_),
      [this]() {
        drainPendingScans();
        publishDiagnostics();
      });
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    imu_buffer_->add(
      {
        stampSeconds(message->header.stamp),
        acceleration_scale_ * Vec3{
          message->linear_acceleration.x,
          message->linear_acceleration.y,
          message->linear_acceleration.z},
        {message->angular_velocity.x, message->angular_velocity.y,
          message->angular_velocity.z}});
    drainPendingScans();
  }

  void onLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
  {
    const std::size_t usable_count =
      std::min<std::size_t>(message->point_num, message->points.size());
    const double scan_start_s = stampSeconds(message->header.stamp);
    if (!std::isfinite(scan_start_s) || usable_count == 0U) {
      ++invalid_scans_;
      frame_.state = "INVALID_LIDAR_FRAME";
      return;
    }
    if (last_lidar_scan_start_s_.has_value()) {
      if (scan_start_s == *last_lidar_scan_start_s_) {
        ++lidar_duplicates_;
        frame_.state = "LIDAR_DUPLICATE_REJECTED";
        return;
      }
      if (scan_start_s < *last_lidar_scan_start_s_) {
        ++lidar_time_reversals_;
        frame_.state = "LIDAR_TIME_REVERSAL_REJECTED";
        return;
      }
    }
    last_lidar_scan_start_s_ = scan_start_s;
    std::uint32_t maximum_offset_ns = 0U;
    for (std::size_t index = 0; index < usable_count; ++index) {
      maximum_offset_ns = std::max(maximum_offset_ns, message->points[index].offset_time);
    }
    const double scan_end_s = scan_start_s + static_cast<double>(maximum_offset_ns) * 1.0e-9;
    if (pending_scans_.size() >= pending_scan_capacity_) {
      pending_scans_.pop_front();
      ++dropped_pending_scans_;
    }
    pending_scans_.push_back({message, now().seconds(), scan_start_s, scan_end_s});
    drainPendingScans();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    OdomState state;
    state.stamp_s = stampSeconds(message->header.stamp);
    state.pose_world_imu.translation = {
      message->pose.pose.position.x, message->pose.pose.position.y,
      message->pose.pose.position.z};
    state.pose_world_imu.rotation = Quaternion{
      message->pose.pose.orientation.w, message->pose.pose.orientation.x,
      message->pose.pose.orientation.y, message->pose.pose.orientation.z}.normalized();
    const Vec3 velocity_body{
      message->twist.twist.linear.x, message->twist.twist.linear.y,
      message->twist.twist.linear.z};
    state.velocity_world_m_s = state.pose_world_imu.rotation.rotate(velocity_body);
    if (!std::isfinite(state.stamp_s) || !finite(state.pose_world_imu) ||
      !finite(state.velocity_world_m_s))
    {
      ++invalid_odom_;
      return;
    }
    if (!odom_buffer_.empty() && state.stamp_s < odom_buffer_.back().stamp_s) {
      odom_buffer_.clear();
      resetFrontendEpoch();
      ++odom_time_reversals_;
    }
    if (!odom_buffer_.empty() && state.stamp_s == odom_buffer_.back().stamp_s) {
      odom_buffer_.back() = state;
      return;
    }
    odom_buffer_.push_back(state);
    while (odom_buffer_.size() > odom_capacity_) {
      odom_buffer_.pop_front();
    }
    drainPendingScans();
  }

  void onTimeSync(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
  {
    for (const auto & status : message->status) {
      if (status.name != "fgo_gil/time_sync") {
        continue;
      }
      time_sync_state_ = "UNSYNCED";
      for (const auto & value : status.values) {
        if (value.key == "state") {
          time_sync_state_ = value.value;
          break;
        }
      }
      last_time_sync_reception_s_ = now().seconds();
      drainPendingScans();
      return;
    }
  }

  bool timeSyncReady() const
  {
    if (!require_time_sync_) {
      return true;
    }
    if (!last_time_sync_reception_s_.has_value()) {
      return false;
    }
    const double age_s = now().seconds() - *last_time_sync_reception_s_;
    return age_s >= 0.0 && age_s <= time_sync_timeout_s_ &&
           (time_sync_state_ == "COARSE" || time_sync_state_ == "PPS_LOCKED");
  }

  bool gnssAvailable() const
  {
    if (!last_gnss_reception_s_.has_value()) {
      return false;
    }
    const double age_s = now().seconds() - *last_gnss_reception_s_;
    return age_s >= 0.0 && age_s <= gnss_availability_timeout_s_;
  }

  std::optional<OdomState> odomAtOrBefore(const double stamp_s) const
  {
    for (auto iterator = odom_buffer_.rbegin(); iterator != odom_buffer_.rend(); ++iterator) {
      if (iterator->stamp_s <= stamp_s) {
        if (stamp_s - iterator->stamp_s <= maximum_odom_age_s_) {
          return *iterator;
        }
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  void resetFrontendEpoch()
  {
    pending_scans_.clear();
    keyframe_map_->clear();
    keyframe_policy_->reset();
    last_lidar_scan_start_s_.reset();
    frame_ = FrameDiagnostics{};
    frame_.state = "TIME_EPOCH_RESET";
    ++frontend_epoch_resets_;
  }

  void drainPendingScans()
  {
    while (!pending_scans_.empty()) {
      const PendingScan & pending = pending_scans_.front();
      const double pending_age_s = now().seconds() - pending.reception_s;
      if (!timeSyncReady()) {
        frame_.state = "WAITING_FOR_TIME_SYNC";
        if (pending_age_s > pending_scan_timeout_s_) {
          pending_scans_.pop_front();
          ++dropped_unsynced_scans_;
          continue;
        }
        return;
      }
      const auto odom = odomAtOrBefore(pending.scan_start_s);
      if (!odom.has_value()) {
        frame_.state = "WAITING_FOR_LIO_INITIALIZATION";
        if (pending_age_s > pending_scan_timeout_s_) {
          pending_scans_.pop_front();
          ++dropped_uninitialized_scans_;
          continue;
        }
        return;
      }
      if (imu_buffer_->empty() || imu_buffer_->samples().back().stamp_s < pending.scan_end_s) {
        frame_.state = "WAITING_FOR_IMU_COVERAGE";
        if (pending_age_s > pending_scan_timeout_s_) {
          pending_scans_.pop_front();
          ++dropped_imu_scans_;
          continue;
        }
        return;
      }
      processScan(pending, *odom);
      pending_scans_.pop_front();
    }
  }

  void processScan(const PendingScan & pending, const OdomState & odom)
  {
    const auto started = std::chrono::steady_clock::now();
    frame_ = FrameDiagnostics{};
    frame_.raw_points = pending.message->points.size();
    std::vector<RawLivoxPoint> raw_points;
    raw_points.reserve(pending.message->points.size());
    for (const auto & point : pending.message->points) {
      raw_points.push_back(
        {
          point.offset_time,
          {point.x, point.y, point.z},
          point.reflectivity,
          point.tag,
          point.line});
    }
    const auto preprocessed = preprocessor_->process(raw_points, pending.message->point_num);
    frame_.preprocessed_points = preprocessed.size();
    const std::vector<ImuSample> imu_samples(
      imu_buffer_->samples().begin(), imu_buffer_->samples().end());
    const DeskewInitialState initial{
      odom.stamp_s, odom.pose_world_imu, odom.velocity_world_m_s};
    const auto deskew = deskewer_->deskew(
      preprocessed, pending.scan_start_s, imu_samples, initial);
    frame_.maximum_imu_gap_s = deskew.maximum_imu_gap_s;
    if (deskew.result != DeskewResult::Success) {
      frame_.state = std::string("DESKEW_REJECTED_") + toString(deskew.result);
      ++deskew_rejections_;
      finishLatency(started);
      return;
    }
    frame_.deskewed_points = deskew.points_at_scan_end.size();
    const auto features = feature_extractor_->extract(deskew.points_at_scan_end);
    frame_.edge_features = features.edge_points.size();
    frame_.plane_features = features.plane_points.size();
    if (features.edge_points.size() < minimum_edge_features_ ||
      features.plane_points.size() < minimum_plane_features_)
    {
      frame_.state = "FEATURES_INSUFFICIENT";
      ++feature_rejections_;
      finishLatency(started);
      return;
    }

    const bool gnss_available = gnssAvailable();
    const KeyframeTrigger trigger = keyframe_policy_->evaluate(
      deskew.scan_end_s, deskew.pose_world_lidar_at_end, gnss_available);
    frame_.keyframe_trigger = toString(trigger);
    if (keyframe_map_->size() == 0U) {
      keyframe_map_->add(deskew.scan_end_s, deskew.pose_world_lidar_at_end, features);
      keyframe_policy_->record(
        deskew.scan_end_s, deskew.pose_world_lidar_at_end, gnss_available);
      frame_.state = "MAP_INITIALIZED";
      ++accepted_keyframes_;
      publishConstraintBatch(
        deskew.scan_end_s, deskew.pose_world_lidar_at_end, true, {}, {});
      finishLatency(started);
      return;
    }

    const auto submap = keyframe_map_->submap(deskew.pose_world_lidar_at_end.translation);
    frame_.submap_edge_features = submap.edge_points.size();
    frame_.submap_plane_features = submap.plane_points.size();
    const auto match = matcher_->match(features, submap, deskew.pose_world_lidar_at_end);
    frame_.line_matches = match.summary.line_matches;
    frame_.plane_matches = match.summary.plane_matches;
    frame_.residual_rms_m = match.summary.residual_rms_m;
    frame_.information_min_eigenvalue = match.summary.information_eigenvalues.front();
    frame_.information_condition = match.summary.information_condition;
    frame_.degenerate = match.summary.degenerate;
    frame_.constraint_valid = match.summary.constraint_valid;
    if (!match.summary.constraint_valid) {
      frame_.state = match.summary.degenerate ?
        "DEGENERATE_NO_CONSTRAINT" : "MATCHES_INSUFFICIENT";
      ++constraint_rejections_;
      finishLatency(started);
      return;
    }
    frame_.state = "TRACKING";
    ++valid_constraints_;
    if (trigger != KeyframeTrigger::None) {
      publishConstraintBatch(
        deskew.scan_end_s, deskew.pose_world_lidar_at_end, false,
        match.line_factors, match.plane_factors);
      keyframe_map_->add(deskew.scan_end_s, deskew.pose_world_lidar_at_end, features);
      keyframe_policy_->record(
        deskew.scan_end_s, deskew.pose_world_lidar_at_end, gnss_available);
      ++accepted_keyframes_;
    }
    finishLatency(started);
  }

  void publishConstraintBatch(
    const double stamp_s,
    const RigidPose & initial_pose,
    const bool initialization_keyframe,
    const std::vector<PointToLineFactor> & line_factors,
    const std::vector<PointToPlaneFactor> & plane_factors)
  {
    fgo_gil_msgs::msg::LidarConstraintBatch message;
    message.header.stamp = rclcpp::Time(
      static_cast<std::int64_t>(std::llround(stamp_s * 1.0e9)));
    message.header.frame_id = lidar_world_frame_;
    message.frontend_epoch = frontend_epoch_resets_;
    message.sequence = constraint_sequence_++;
    message.initial_pose_world_lidar.position.x = initial_pose.translation.x;
    message.initial_pose_world_lidar.position.y = initial_pose.translation.y;
    message.initial_pose_world_lidar.position.z = initial_pose.translation.z;
    message.initial_pose_world_lidar.orientation.w = initial_pose.rotation.w;
    message.initial_pose_world_lidar.orientation.x = initial_pose.rotation.x;
    message.initial_pose_world_lidar.orientation.y = initial_pose.rotation.y;
    message.initial_pose_world_lidar.orientation.z = initial_pose.rotation.z;
    message.initialization_keyframe = initialization_keyframe;
    message.keyframe = true;
    message.line_factors.reserve(line_factors.size());
    for (const auto & source : line_factors) {
      fgo_gil_msgs::msg::LidarLineFactor factor;
      factor.point_lidar.x = source.point_lidar.x;
      factor.point_lidar.y = source.point_lidar.y;
      factor.point_lidar.z = source.point_lidar.z;
      factor.line_anchor_world.x = source.line_anchor_world.x;
      factor.line_anchor_world.y = source.line_anchor_world.y;
      factor.line_anchor_world.z = source.line_anchor_world.z;
      factor.line_direction_world.x = source.line_direction_world.x;
      factor.line_direction_world.y = source.line_direction_world.y;
      factor.line_direction_world.z = source.line_direction_world.z;
      message.line_factors.push_back(std::move(factor));
    }
    message.plane_factors.reserve(plane_factors.size());
    for (const auto & source : plane_factors) {
      fgo_gil_msgs::msg::LidarPlaneFactor factor;
      factor.point_lidar.x = source.point_lidar.x;
      factor.point_lidar.y = source.point_lidar.y;
      factor.point_lidar.z = source.point_lidar.z;
      factor.plane_anchor_world.x = source.plane_anchor_world.x;
      factor.plane_anchor_world.y = source.plane_anchor_world.y;
      factor.plane_anchor_world.z = source.plane_anchor_world.z;
      factor.plane_normal_world.x = source.plane_normal_world.x;
      factor.plane_normal_world.y = source.plane_normal_world.y;
      factor.plane_normal_world.z = source.plane_normal_world.z;
      message.plane_factors.push_back(std::move(factor));
    }
    constraints_pub_->publish(std::move(message));
  }

  void finishLatency(const std::chrono::steady_clock::time_point & started)
  {
    frame_.latency_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    ++processed_scans_;
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fgo_gil/lidar_frontend";
    status.hardware_id = "livox_mid360";
    status.message = frame_.state;
    if (frame_.state == "TRACKING") {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (frame_.state == "INVALID_LIDAR_FRAME" ||
      frame_.state == "LIDAR_TIME_REVERSAL_REJECTED" ||
      frame_.state.rfind("DESKEW_REJECTED_", 0U) == 0U)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    status.values.push_back(keyValue("time_sync_state", time_sync_state_));
    status.values.push_back(keyValue("keyframe_trigger", frame_.keyframe_trigger));
    status.values.push_back(numericKeyValue("raw_points", frame_.raw_points));
    status.values.push_back(
      numericKeyValue("preprocessed_points", frame_.preprocessed_points));
    status.values.push_back(numericKeyValue("deskewed_points", frame_.deskewed_points));
    status.values.push_back(numericKeyValue("edge_features", frame_.edge_features));
    status.values.push_back(numericKeyValue("plane_features", frame_.plane_features));
    status.values.push_back(
      numericKeyValue("submap_edge_features", frame_.submap_edge_features));
    status.values.push_back(
      numericKeyValue("submap_plane_features", frame_.submap_plane_features));
    status.values.push_back(numericKeyValue("line_matches", frame_.line_matches));
    status.values.push_back(numericKeyValue("plane_matches", frame_.plane_matches));
    status.values.push_back(numericKeyValue("residual_rms_m", frame_.residual_rms_m));
    status.values.push_back(
      numericKeyValue("information_min_eigenvalue", frame_.information_min_eigenvalue));
    status.values.push_back(
      numericKeyValue("information_condition", frame_.information_condition));
    status.values.push_back(numericKeyValue("maximum_imu_gap_s", frame_.maximum_imu_gap_s));
    status.values.push_back(numericKeyValue("latency_ms", frame_.latency_ms));
    status.values.push_back(keyValue("degenerate", frame_.degenerate ? "true" : "false"));
    status.values.push_back(
      keyValue("constraint_valid", frame_.constraint_valid ? "true" : "false"));
    status.values.push_back(numericKeyValue("keyframe_count", keyframe_map_->size()));
    status.values.push_back(numericKeyValue("processed_scans", processed_scans_));
    status.values.push_back(numericKeyValue("valid_constraints", valid_constraints_));
    status.values.push_back(numericKeyValue("accepted_keyframes", accepted_keyframes_));
    status.values.push_back(numericKeyValue("invalid_scans", invalid_scans_));
    status.values.push_back(numericKeyValue("lidar_duplicates", lidar_duplicates_));
    status.values.push_back(
      numericKeyValue("lidar_time_reversals", lidar_time_reversals_));
    status.values.push_back(numericKeyValue("invalid_odom", invalid_odom_));
    status.values.push_back(numericKeyValue("odom_time_reversals", odom_time_reversals_));
    status.values.push_back(
      numericKeyValue("frontend_epoch_resets", frontend_epoch_resets_));
    status.values.push_back(numericKeyValue("deskew_rejections", deskew_rejections_));
    status.values.push_back(numericKeyValue("feature_rejections", feature_rejections_));
    status.values.push_back(numericKeyValue("constraint_rejections", constraint_rejections_));
    status.values.push_back(
      numericKeyValue("dropped_pending_scans", dropped_pending_scans_));
    status.values.push_back(
      numericKeyValue("dropped_unsynced_scans", dropped_unsynced_scans_));
    status.values.push_back(
      numericKeyValue("dropped_uninitialized_scans", dropped_uninitialized_scans_));
    status.values.push_back(numericKeyValue("dropped_imu_scans", dropped_imu_scans_));
    const auto & imu_diagnostics = imu_buffer_->diagnostics();
    status.values.push_back(numericKeyValue("imu_duplicates", imu_diagnostics.duplicates));
    status.values.push_back(
      numericKeyValue("imu_time_reversals", imu_diagnostics.time_reversals));
    status.values.push_back(numericKeyValue("imu_gaps", imu_diagnostics.gaps));
    status.values.push_back(numericKeyValue("imu_nonfinite", imu_diagnostics.nonfinite));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string imu_topic_;
  std::string lidar_topic_;
  std::string odom_topic_;
  std::string time_sync_topic_;
  std::string gnss_epoch_topic_;
  std::string diagnostics_topic_;
  std::string constraints_topic_;
  std::string lidar_world_frame_;
  std::size_t odom_capacity_ = 100U;
  std::size_t pending_scan_capacity_ = 4U;
  std::size_t minimum_edge_features_ = 8U;
  std::size_t minimum_plane_features_ = 20U;
  double pending_scan_timeout_s_ = 0.5;
  double maximum_odom_age_s_ = 0.5;
  double acceleration_scale_ = 9.80665;
  double time_sync_timeout_s_ = 2.0;
  double gnss_availability_timeout_s_ = 2.0;
  double diagnostics_period_s_ = 1.0;
  bool require_time_sync_ = true;

  std::unique_ptr<ImuSegmentBuffer> imu_buffer_;
  std::unique_ptr<LivoxPreprocessor> preprocessor_;
  std::unique_ptr<LidarDeskewer> deskewer_;
  std::unique_ptr<LidarFeatureExtractor> feature_extractor_;
  std::unique_ptr<KeyframePolicy> keyframe_policy_;
  std::unique_ptr<KeyframeMap> keyframe_map_;
  std::unique_ptr<LidarMatcher> matcher_;
  std::deque<OdomState> odom_buffer_;
  std::deque<PendingScan> pending_scans_;
  std::string time_sync_state_ = "UNSYNCED";
  std::optional<double> last_time_sync_reception_s_;
  std::optional<double> last_gnss_reception_s_;
  std::optional<double> last_lidar_scan_start_s_;
  FrameDiagnostics frame_;

  std::uint64_t processed_scans_ = 0;
  std::uint64_t valid_constraints_ = 0;
  std::uint64_t accepted_keyframes_ = 0;
  std::uint64_t invalid_scans_ = 0;
  std::uint64_t lidar_duplicates_ = 0;
  std::uint64_t lidar_time_reversals_ = 0;
  std::uint64_t invalid_odom_ = 0;
  std::uint64_t odom_time_reversals_ = 0;
  std::uint64_t frontend_epoch_resets_ = 0;
  std::uint64_t constraint_sequence_ = 0;
  std::uint64_t deskew_rejections_ = 0;
  std::uint64_t feature_rejections_ = 0;
  std::uint64_t constraint_rejections_ = 0;
  std::uint64_t dropped_pending_scans_ = 0;
  std::uint64_t dropped_unsynced_scans_ = 0;
  std::uint64_t dropped_uninitialized_scans_ = 0;
  std::uint64_t dropped_imu_scans_ = 0;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<fgo_gil_msgs::msg::LidarConstraintBatch>::SharedPtr constraints_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr time_sync_sub_;
  rclcpp::Subscription<gnss_raw_msgs::msg::ObservationEpoch>::SharedPtr gnss_epoch_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace fgo_gil_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fgo_gil_localizer::LidarFrontendNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("fgo_gil_lidar_frontend"), "Node failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

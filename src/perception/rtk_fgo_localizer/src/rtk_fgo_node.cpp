#include "rtk_fgo_localizer/correction_smoother.hpp"
#include "rtk_fgo_localizer/fgo_graph.hpp"
#include "rtk_fgo_localizer/frame_anchor.hpp"
#include "rtk_fgo_localizer/gate_params.hpp"
#include "rtk_fgo_localizer/heading_conventions.hpp"
#include "rtk_fgo_localizer/imu_preintegration_config.hpp"
#include "rtk_fgo_localizer/rtk_quality.hpp"
#include "rtk_fgo_localizer/state_machine.hpp"
#include "rtk_fgo_localizer/topic_buffers.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

namespace rtk_fgo_localizer
{
namespace
{

double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).seconds();
}

bool validFix(const sensor_msgs::msg::NavSatFix & msg)
{
  return msg.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
         std::isfinite(msg.latitude) && std::isfinite(msg.longitude) &&
         std::isfinite(msg.altitude);
}

gtsam::Pose3 poseFromOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
    gtsam::Point3(p.x, p.y, p.z));
}

gtsam::Vector3 velocityFromOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & v = msg.twist.twist.linear;
  return gtsam::Vector3(v.x, v.y, v.z);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

std::string gateModeName(RtkGateMode mode)
{
  switch (mode) {
    case RtkGateMode::Rejected:
      return "REJECTED";
    case RtkGateMode::DiagnosticOnly:
      return "DIAGNOSTIC_ONLY";
    case RtkGateMode::WeakCandidate:
      return "WEAK_CANDIDATE";
    case RtkGateMode::StrongCandidate:
      return "STRONG_CANDIDATE";
  }
  return "UNKNOWN";
}

double fixSigmaXY(const sensor_msgs::msg::NavSatFix & msg)
{
  if (msg.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    return 1.0;
  }
  const double sx = msg.position_covariance[0] > 0.0 ?
    std::sqrt(msg.position_covariance[0]) : 1.0;
  const double sy = msg.position_covariance[4] > 0.0 ?
    std::sqrt(msg.position_covariance[4]) : sx;
  return std::clamp(0.5 * (sx + sy), 0.02, 5.0);
}

double pointDistance(const gtsam::Point3 & a, const gtsam::Point3 & b)
{
  return (a - b).norm();
}

GeoPoint geoPointFromFix(const sensor_msgs::msg::NavSatFix & msg)
{
  return GeoPoint{msg.latitude, msg.longitude, msg.altitude};
}

struct FrameAnchorBootstrapParams
{
  bool require_fixed_for_anchor = true;
  bool use_rtk_heading_for_yaw = true;
  int bootstrap_min_satellites = 10;
  double bootstrap_max_hdop = 2.0;
  std::size_t bootstrap_required_consecutive_samples = 5;
};

bool anchorBootstrapCandidate(
  const GeoPoint & fix,
  const RtkQuality & quality,
  const std::optional<double> & heading_yaw_rad,
  const FrameAnchorBootstrapParams & params,
  int fixed_quality_code,
  std::string * reason)
{
  if (!std::isfinite(fix.latitude_deg) || !std::isfinite(fix.longitude_deg) ||
    !std::isfinite(fix.altitude_m))
  {
    if (reason != nullptr) {
      *reason = "anchor fix is non-finite";
    }
    return false;
  }
  if (params.require_fixed_for_anchor && quality.quality != fixed_quality_code) {
    if (reason != nullptr) {
      *reason = "anchor waiting for RTK fixed quality";
    }
    return false;
  }
  if (quality.satellites < params.bootstrap_min_satellites) {
    if (reason != nullptr) {
      *reason = "anchor waiting for enough satellites";
    }
    return false;
  }
  if (!std::isfinite(quality.hdop) || quality.hdop > params.bootstrap_max_hdop) {
    if (reason != nullptr) {
      *reason = "anchor waiting for HDOP threshold";
    }
    return false;
  }
  if (params.use_rtk_heading_for_yaw && !heading_yaw_rad.has_value()) {
    if (reason != nullptr) {
      *reason = "anchor waiting for RTK heading";
    }
    return false;
  }
  if (reason != nullptr) {
    *reason = "anchor bootstrap candidate accepted";
  }
  return true;
}

}  // namespace

class RtkFgoNode : public rclcpp::Node
{
public:
  RtkFgoNode()
  : Node("rtk_fgo_localizer")
  {
    declareParameters();
    readParameters();
    graph_ = std::make_unique<FgoGraph>(max_states_);
    if (imu_enabled_) {
      graph_->configureImu(imu_config_);
    }
    graph_->setMaxShadowCorrection(max_position_jump_m_);
    if (publish_tf_ || nav2_use_fgo_) {
      RCLCPP_WARN(
        get_logger(),
        "RTK FGO experimental TF/Nav2 flag enabled; this must not be used as a production "
        "map->odom source.");
    }
    smoother_ = std::make_unique<CorrectionSmoother>(
      max_translation_step_m_, max_yaw_step_deg_ * kPi / 180.0);

    subscribeTopics();
    createPublishers();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, keyframe_rate_hz_)),
      [this]() {onTimer();});
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  void declareParameters()
  {
    declare_parameter<std::string>("topics.fastlio_odom", "/fastlio2/lio_odom");
    declare_parameter<std::string>("topics.imu", "/livox/imu");
    declare_parameter<std::string>("topics.wheel_odom", "/odom_CBoar");
    declare_parameter<std::string>("topics.fix", "/fix");
    declare_parameter<std::string>("topics.heading", "/heading");
    declare_parameter<std::string>("topics.rtk_status", "/rtk/status");
    declare_parameter<std::string>("topics.raw_nmea", "/rtk/nmea_sentence");
    declare_parameter<std::string>("frames.map", "map");
    declare_parameter<std::string>("frames.base_link", "base_link");
    declare_parameter<std::string>("frames.odom_fgo", "odom_fgo");
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<bool>("nav2_use_fgo", false);
    declare_parameter<bool>("heading_quaternion_yaw_is_compass", true);
    declare_parameter<double>("window.duration_s", 15.0);
    declare_parameter<double>("window.keyframe_rate_hz", 10.0);
    declare_parameter<int>("window.max_states", 120);
    declare_parameter<bool>("factors.imu_enabled", false);
    declare_parameter<bool>("factors.fastlio_enabled", true);
    declare_parameter<bool>("factors.wheel_enabled", true);
    declare_parameter<bool>("factors.rtk_position_enabled", true);
    declare_parameter<bool>("factors.rtk_heading_enabled", true);
    declare_parameter<int>("rtk_gating.fixed_quality_code", 4);
    declare_parameter<int>("rtk_gating.float_quality_code", 5);
    declare_parameter<int>("rtk_gating.min_satellites", 10);
    declare_parameter<double>("rtk_gating.max_hdop", 2.0);
    declare_parameter<double>("rtk_gating.max_strong_position_innovation_m", 3.0);
    declare_parameter<double>("rtk_gating.max_weak_position_innovation_m", 5.0);
    declare_parameter<double>("rtk_gating.max_heading_innovation_rad", 0.5);
    declare_parameter<double>("rtk_gating.max_implied_speed_mps", 2.0);
    declare_parameter<int>("rtk_gating.recovery_min_samples", 8);
    declare_parameter<double>("rtk_gating.max_position_jump_m", 3.0);
    declare_parameter<double>("rtk_gating.heading_sigma_rad", 0.05);
    declare_parameter<bool>("frame_anchor.require_fixed_for_anchor", true);
    declare_parameter<bool>("frame_anchor.use_rtk_heading_for_yaw", true);
    declare_parameter<double>("frame_anchor.max_anchor_position_innovation_m", 2.0);
    declare_parameter<int>("frame_anchor.bootstrap_min_satellites", 10);
    declare_parameter<double>("frame_anchor.bootstrap_max_hdop", 2.0);
    declare_parameter<int>("frame_anchor.bootstrap_required_consecutive_samples", 5);
    declare_parameter<double>("imu.accelerometer_noise_sigma", 0.1);
    declare_parameter<double>("imu.gyroscope_noise_sigma", 0.01);
    declare_parameter<double>("imu.accelerometer_bias_rw_sigma", 0.001);
    declare_parameter<double>("imu.gyroscope_bias_rw_sigma", 0.0001);
    declare_parameter<double>("imu.integration_error_sigma", 1.0e-8);
    declare_parameter<double>("imu.gravity_mps2", 9.81);
    declare_parameter<double>("correction_smoother.max_translation_step_m", 0.15);
    declare_parameter<double>("correction_smoother.max_yaw_step_deg", 0.3);
  }

  void readParameters()
  {
    fastlio_topic_ = get_parameter("topics.fastlio_odom").as_string();
    imu_topic_ = get_parameter("topics.imu").as_string();
    wheel_topic_ = get_parameter("topics.wheel_odom").as_string();
    fix_topic_ = get_parameter("topics.fix").as_string();
    heading_topic_ = get_parameter("topics.heading").as_string();
    rtk_status_topic_ = get_parameter("topics.rtk_status").as_string();
    raw_nmea_topic_ = get_parameter("topics.raw_nmea").as_string();
    map_frame_ = get_parameter("frames.map").as_string();
    base_frame_ = get_parameter("frames.base_link").as_string();
    odom_fgo_frame_ = get_parameter("frames.odom_fgo").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    nav2_use_fgo_ = get_parameter("nav2_use_fgo").as_bool();
    heading_quaternion_yaw_is_compass_ =
      get_parameter("heading_quaternion_yaw_is_compass").as_bool();
    window_duration_s_ = get_parameter("window.duration_s").as_double();
    keyframe_rate_hz_ = get_parameter("window.keyframe_rate_hz").as_double();
    max_states_ = static_cast<std::size_t>(
      std::max<std::int64_t>(2, get_parameter("window.max_states").as_int()));
    imu_enabled_ = get_parameter("factors.imu_enabled").as_bool();
    fastlio_enabled_ = get_parameter("factors.fastlio_enabled").as_bool();
    wheel_enabled_ = get_parameter("factors.wheel_enabled").as_bool();
    rtk_position_enabled_ = get_parameter("factors.rtk_position_enabled").as_bool();
    rtk_heading_enabled_ = get_parameter("factors.rtk_heading_enabled").as_bool();
    gate_params_.fixed_quality_code =
      static_cast<int>(get_parameter("rtk_gating.fixed_quality_code").as_int());
    gate_params_.float_quality_code =
      static_cast<int>(get_parameter("rtk_gating.float_quality_code").as_int());
    gate_params_.min_satellites =
      static_cast<int>(get_parameter("rtk_gating.min_satellites").as_int());
    gate_params_.max_hdop = get_parameter("rtk_gating.max_hdop").as_double();
    gate_params_.max_strong_position_innovation_m =
      get_parameter("rtk_gating.max_strong_position_innovation_m").as_double();
    gate_params_.max_weak_position_innovation_m =
      get_parameter("rtk_gating.max_weak_position_innovation_m").as_double();
    gate_params_.max_heading_innovation_rad =
      get_parameter("rtk_gating.max_heading_innovation_rad").as_double();
    gate_params_.max_implied_speed_mps =
      get_parameter("rtk_gating.max_implied_speed_mps").as_double();
    recovery_min_samples_ =
      static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("rtk_gating.recovery_min_samples").as_int()));
    max_position_jump_m_ = get_parameter("rtk_gating.max_position_jump_m").as_double();
    heading_sigma_rad_ = get_parameter("rtk_gating.heading_sigma_rad").as_double();
    anchor_params_.require_fixed_for_anchor =
      get_parameter("frame_anchor.require_fixed_for_anchor").as_bool();
    anchor_params_.use_rtk_heading_for_yaw =
      get_parameter("frame_anchor.use_rtk_heading_for_yaw").as_bool();
    anchor_max_position_innovation_m_ =
      get_parameter("frame_anchor.max_anchor_position_innovation_m").as_double();
    anchor_params_.bootstrap_min_satellites =
      static_cast<int>(get_parameter("frame_anchor.bootstrap_min_satellites").as_int());
    anchor_params_.bootstrap_max_hdop =
      get_parameter("frame_anchor.bootstrap_max_hdop").as_double();
    anchor_params_.bootstrap_required_consecutive_samples =
      static_cast<std::size_t>(
      std::max<std::int64_t>(
        1, get_parameter("frame_anchor.bootstrap_required_consecutive_samples").as_int()));
    imu_config_.accelerometer_noise_sigma =
      get_parameter("imu.accelerometer_noise_sigma").as_double();
    imu_config_.gyroscope_noise_sigma =
      get_parameter("imu.gyroscope_noise_sigma").as_double();
    imu_config_.accelerometer_bias_rw_sigma =
      get_parameter("imu.accelerometer_bias_rw_sigma").as_double();
    imu_config_.gyroscope_bias_rw_sigma =
      get_parameter("imu.gyroscope_bias_rw_sigma").as_double();
    imu_config_.integration_error_sigma =
      get_parameter("imu.integration_error_sigma").as_double();
    imu_config_.gravity_mps2 =
      get_parameter("imu.gravity_mps2").as_double();
    max_translation_step_m_ =
      get_parameter("correction_smoother.max_translation_step_m").as_double();
    max_yaw_step_deg_ = get_parameter("correction_smoother.max_yaw_step_deg").as_double();
    state_machine_ = LocalizationStateMachine(recovery_min_samples_);
  }

  void subscribeTopics()
  {
    fastlio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      fastlio_topic_, 50,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        fastlio_buffer_.add(stampToSec(msg->header.stamp), *msg);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 100,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!imu_enabled_) {
          return;
        }
        const double stamp_s = stampToSec(msg->header.stamp);
        graph_->addImuSample(
          stamp_s,
          Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z),
          Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z));
      });
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, 50,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        wheel_buffer_.add(stampToSec(msg->header.stamp), *msg);
      });
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, 20,
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        fix_buffer_.add(stampToSec(msg->header.stamp), *msg);
      });
    heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      heading_topic_, 20,
      [this](geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
        heading_buffer_.add(stampToSec(msg->header.stamp), *msg);
      });
    rtk_status_sub_ = create_subscription<std_msgs::msg::String>(
      rtk_status_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        latest_rtk_status_ = msg->data;
      });
    raw_nmea_sub_ = create_subscription<nmea_msgs::msg::Sentence>(
      raw_nmea_topic_, 50,
      [this](nmea_msgs::msg::Sentence::SharedPtr msg) {
        if (parseGgaQuality(msg->sentence).has_value()) {
          nmea_buffer_.add(stampToSec(msg->header.stamp), *msg);
        }
      });
  }

  void createPublishers()
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rtk_fgo/odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/rtk_fgo/path", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/rtk_fgo/status", 10);
    gate_pub_ = create_publisher<std_msgs::msg::String>("/rtk_fgo/rtk_gate", 10);
    correction_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("/rtk_fgo/correction_status", 10);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/rtk_fgo/factor_diagnostics", 10);
    path_msg_.header.frame_id = map_frame_;
  }

  void onTimer()
  {
    auto latest_lio = fastlio_buffer_.latest();
    if (!latest_lio.has_value()) {
      publishStatus("WAITING_FOR_FASTLIO", RtkGateMode::Rejected, "no FAST-LIO odom");
      return;
    }

    const auto lio_pose = poseFromOdom(latest_lio->value);
    if (!graph_->latestEstimate().has_value()) {
      graph_->addInitialState(latest_lio->stamp_s, lio_pose, velocityFromOdom(latest_lio->value));
      last_lio_stamp_s_ = latest_lio->stamp_s;
      last_lio_pose_ = lio_pose;
    } else if (fastlio_enabled_ && latest_lio->stamp_s > last_lio_stamp_s_ + 1e-4) {
      graph_->addFastLioBetween(latest_lio->stamp_s, last_lio_pose_.between(lio_pose));
      addWheelFactorForLatestTransition(latest_lio->stamp_s);
      last_lio_stamp_s_ = latest_lio->stamp_s;
      last_lio_pose_ = lio_pose;
    }

    auto gate_decision = evaluateLatestRtkGate();
    auto state = state_machine_.update(gate_decision.mode, true);
    ShadowCommitResult commit_result;
    if (rtk_position_enabled_ && gate_decision.mode == RtkGateMode::StrongCandidate &&
      (state == LocalizationState::RtkRecovery || state == LocalizationState::RtkLocked))
    {
      commit_result = tryCommitLatestRtk();
      if (commit_result.committed) {
        state_machine_.markRecoveryCommitted();
        if (rtk_heading_enabled_) {
          addLatestHeadingFactor();
        }
      } else if (state == LocalizationState::RtkRecovery ||
        state == LocalizationState::RtkLocked)
      {
        state_machine_.markRecoveryRejected();
      }
    }

    publishEstimate(commit_result);
    publishStatus(toString(state_machine_.state()), gate_decision.mode, gate_decision.reason);
    publishDiagnostics(gate_decision, commit_result);
  }

  RtkGateDecision evaluateLatestRtkGate()
  {
    auto latest_nmea = nmea_buffer_.latest();
    if (!latest_nmea.has_value()) {
      return {RtkGateMode::Rejected, "no raw NMEA"};
    }
    auto quality = parseGgaQuality(latest_nmea->value.sentence);
    if (!quality.has_value()) {
      return {RtkGateMode::Rejected, "latest NMEA is not valid GGA"};
    }

    const auto estimate = graph_->latestEstimate();
    const auto latest_fix = latestValidFix();
    if (!estimate.has_value() || !latest_fix.has_value()) {
      return {RtkGateMode::Rejected, "missing graph estimate or valid fix"};
    }

    const auto heading_yaw = latestHeadingYaw(estimate->stamp_s);
    if (!frame_anchor_.initialized()) {
      updateFrameAnchorBootstrap(*latest_fix, *quality, heading_yaw, estimate->pose);
      if (!frame_anchor_.initialized()) {
        return {RtkGateMode::Rejected, "waiting for frame anchor bootstrap"};
      }
    }

    const auto fix_point = frame_anchor_.geoToMap(geoPointFromFix(*latest_fix));
    if (!fix_point.has_value()) {
      return {RtkGateMode::Rejected, "frame anchor could not map fix"};
    }
    latest_fix_sigma_m_ = fixSigmaXY(*latest_fix);
    const double position_innovation =
      (estimate->pose.translation() - *fix_point).norm();
    if (position_innovation > anchor_max_position_innovation_m_ &&
      anchor_bootstrap_count_ < anchor_params_.bootstrap_required_consecutive_samples)
    {
      return {RtkGateMode::Rejected, "anchor bootstrap position innovation too large"};
    }

    std::string speed_reason = "speed history not ready";
    const auto fix_stamp_s = stampToSec(latest_fix->header.stamp);
    const auto implied_speed = computeMappedFixSpeed(*fix_point, fix_stamp_s, &speed_reason);

    double heading_innovation = 0.0;
    if (heading_yaw.has_value()) {
      heading_innovation =
        normalizeYaw(
        estimate->pose.rotation().yaw() -
        *heading_yaw);
      quality->heading_stable = true;
    }

    RtkGateInput input;
    input.quality = *quality;
    input.position_innovation_m = position_innovation;
    input.heading_innovation_rad = heading_innovation;
    input.implied_speed_mps = implied_speed;
    auto decision = evaluateRtkGate(input, gate_params_);
    if (!implied_speed.has_value() && decision.mode != RtkGateMode::Rejected) {
      decision.reason += "; " + speed_reason;
    }
    if (decision.mode != RtkGateMode::Rejected) {
      previous_mapped_fix_ = *fix_point;
      previous_mapped_fix_stamp_s_ = fix_stamp_s;
    }
    return decision;
  }

  std::optional<sensor_msgs::msg::NavSatFix> latestValidFix()
  {
    auto latest_fix = fix_buffer_.latest();
    if (!latest_fix.has_value() || !validFix(latest_fix->value)) {
      return std::nullopt;
    }
    return latest_fix->value;
  }

  std::optional<double> latestHeadingYaw(double stamp_s) const
  {
    auto heading = heading_buffer_.closest(stamp_s, 0.5);
    if (!heading.has_value()) {
      return std::nullopt;
    }
    const double yaw = yawFromQuaternion(heading->value.quaternion);
    if (!std::isfinite(yaw)) {
      return std::nullopt;
    }
    const double enu_yaw =
      headingQuaternionYawToEnuYaw(yaw, heading_quaternion_yaw_is_compass_);
    if (!std::isfinite(enu_yaw)) {
      return std::nullopt;
    }
    return enu_yaw;
  }

  void addWheelFactorForLatestTransition(double stamp_s)
  {
    wheel_factor_last_added_ = false;
    if (!wheel_enabled_) {
      wheel_factor_reason_ = "wheel factor disabled";
      return;
    }
    auto wheel_sample = wheel_buffer_.closest(stamp_s, 0.20);
    if (!wheel_sample.has_value()) {
      wheel_factor_reason_ = "no wheel odom sample near FAST-LIO keyframe";
      return;
    }
    const auto wheel_pose = poseFromOdom(wheel_sample->value);
    if (!previous_wheel_pose_.has_value()) {
      previous_wheel_pose_ = wheel_pose;
      previous_wheel_stamp_s_ = wheel_sample->stamp_s;
      wheel_factor_reason_ = "wheel history initialized";
      return;
    }

    const auto relative_wheel_pose = previous_wheel_pose_->between(wheel_pose);
    wheel_factor_last_added_ =
      graph_->addWheelPlanarFactorForLatestTransition(relative_wheel_pose);
    previous_wheel_pose_ = wheel_pose;
    previous_wheel_stamp_s_ = wheel_sample->stamp_s;
    wheel_factor_reason_ = wheel_factor_last_added_ ?
      "wheel factor added to latest transition" :
      "graph has fewer than two states";
  }

  void updateFrameAnchorBootstrap(
    const sensor_msgs::msg::NavSatFix & fix,
    const RtkQuality & quality,
    const std::optional<double> & heading_yaw,
    const gtsam::Pose3 & reference_pose)
  {
    const auto geo = geoPointFromFix(fix);
    std::string reason;
    const bool candidate =
      anchorBootstrapCandidate(
      geo, quality, heading_yaw, anchor_params_, gate_params_.fixed_quality_code, &reason);
    anchor_bootstrap_reason_ = reason;
    if (!candidate) {
      anchor_bootstrap_count_ = 0;
      return;
    }
    ++anchor_bootstrap_count_;
    if (anchor_bootstrap_count_ < anchor_params_.bootstrap_required_consecutive_samples) {
      return;
    }
    const auto yaw_for_anchor =
      anchor_params_.use_rtk_heading_for_yaw ? heading_yaw : std::nullopt;
    if (!frame_anchor_.initialize(geo, reference_pose, yaw_for_anchor)) {
      anchor_bootstrap_reason_ = "frame anchor initialization rejected";
      anchor_bootstrap_count_ = 0;
      return;
    }
    latest_fix_sigma_m_ = fixSigmaXY(fix);
    anchor_bootstrap_reason_ = "frame anchor initialized";
  }

  std::optional<gtsam::Point3> latestFixPoint()
  {
    const auto latest_fix = latestValidFix();
    if (!latest_fix.has_value() || !frame_anchor_.initialized()) {
      return std::nullopt;
    }
    auto mapped = frame_anchor_.geoToMap(geoPointFromFix(*latest_fix));
    if (mapped.has_value()) {
      latest_fix_sigma_m_ = fixSigmaXY(*latest_fix);
    }
    return mapped;
  }

  std::optional<double> computeMappedFixSpeed(
    const gtsam::Point3 & mapped_fix,
    double stamp_s,
    std::string * reason)
  {
    if (!previous_mapped_fix_.has_value() || !previous_mapped_fix_stamp_s_.has_value()) {
      if (reason != nullptr) {
        *reason = "speed history not ready";
      }
      return std::nullopt;
    }
    const double dt = stamp_s - *previous_mapped_fix_stamp_s_;
    if (dt <= 1e-3) {
      if (reason != nullptr) {
        *reason = "speed history timestamp not advanced";
      }
      return std::nullopt;
    }
    const double speed_mps = pointDistance(mapped_fix, *previous_mapped_fix_) / dt;
    if (reason != nullptr) {
      *reason = "speed history ready";
    }
    return speed_mps;
  }

  ShadowCommitResult tryCommitLatestRtk()
  {
    const auto fix_point = latestFixPoint();
    if (!fix_point.has_value()) {
      return {false, 0.0, "no fix point"};
    }
    return graph_->tryShadowRtkCommit(now().seconds(), *fix_point, latest_fix_sigma_m_);
  }

  void addLatestHeadingFactor()
  {
    const auto estimate = graph_->latestEstimate();
    if (!estimate.has_value()) {
      return;
    }
    const auto heading_yaw = latestHeadingYaw(estimate->stamp_s);
    if (!heading_yaw.has_value()) {
      return;
    }
    graph_->addRtkHeading(estimate->stamp_s, *heading_yaw, heading_sigma_rad_);
  }

  void publishEstimate(const ShadowCommitResult & commit_result)
  {
    const auto estimate = graph_->latestEstimate();
    if (!estimate.has_value()) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    const auto output_pose = smoothedOutputPose(estimate->pose);
    const auto translation = output_pose.translation();
    const auto quaternion = output_pose.rotation().toQuaternion();
    odom.pose.pose.position.x = translation.x();
    odom.pose.pose.position.y = translation.y();
    odom.pose.pose.position.z = translation.z();
    odom.pose.pose.orientation.x = quaternion.x();
    odom.pose.pose.orientation.y = quaternion.y();
    odom.pose.pose.orientation.z = quaternion.z();
    odom.pose.pose.orientation.w = quaternion.w();
    odom.twist.twist.linear.x = estimate->velocity.x();
    odom.twist.twist.linear.y = estimate->velocity.y();
    odom.twist.twist.linear.z = estimate->velocity.z();
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_msg_.header.stamp = odom.header.stamp;
    path_msg_.poses.push_back(pose);
    if (path_msg_.poses.size() > 2000) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);

    std_msgs::msg::Float32MultiArray correction;
    correction.data = {
      commit_result.committed ? 1.0F : 0.0F,
      static_cast<float>(commit_result.correction_norm_m),
      static_cast<float>(state_machine_.strong_sample_count()),
    };
    correction_pub_->publish(correction);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = odom_fgo_frame_;
      tf.transform.translation.x = translation.x();
      tf.transform.translation.y = translation.y();
      tf.transform.translation.z = translation.z();
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  gtsam::Pose3 smoothedOutputPose(const gtsam::Pose3 & target_pose)
  {
    if (!has_output_pose_) {
      output_pose_ = target_pose;
      has_output_pose_ = true;
      return output_pose_;
    }

    const auto current_translation = output_pose_.translation();
    const auto target_translation = target_pose.translation();
    const double current_yaw = output_pose_.rotation().yaw();
    const double target_yaw = target_pose.rotation().yaw();
    const auto step = smoother_->step(
      {
        target_translation.x() - current_translation.x(),
        target_translation.y() - current_translation.y(),
        normalizeYaw(target_yaw - current_yaw),
      });

    output_pose_ = gtsam::Pose3(
      gtsam::Rot3::Yaw(normalizeYaw(current_yaw + step.dyaw)),
      gtsam::Point3(
        current_translation.x() + step.dx,
        current_translation.y() + step.dy,
        target_translation.z()));
    return output_pose_;
  }

  void publishStatus(
    const std::string & state,
    RtkGateMode gate_mode,
    const std::string & reason)
  {
    std_msgs::msg::String status;
    std::ostringstream out;
    out << "state=" << state
        << " gate=" << gateModeName(gate_mode)
        << " reason=" << reason
        << " publish_tf=" << (publish_tf_ ? "true" : "false")
        << " nav2_use_fgo=" << (nav2_use_fgo_ ? "true" : "false")
        << " rtk_status=\"" << latest_rtk_status_ << "\"";
    status.data = out.str();
    status_pub_->publish(status);

    std_msgs::msg::String gate;
    gate.data = gateModeName(gate_mode) + ": " + reason;
    gate_pub_->publish(gate);
  }

  void publishDiagnostics(
    const RtkGateDecision & gate_decision,
    const ShadowCommitResult & commit_result)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "rtk_fgo_localizer";
    status.hardware_id = "rtk_fgo_shadow";
    status.level = gate_decision.mode == RtkGateMode::Rejected ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = gate_decision.reason;

    diagnostic_msgs::msg::KeyValue state;
    state.key = "state";
    state.value = toString(state_machine_.state());
    status.values.push_back(state);
    diagnostic_msgs::msg::KeyValue committed;
    committed.key = "last_shadow_commit";
    committed.value = commit_result.committed ? "true" : "false";
    status.values.push_back(committed);
    diagnostic_msgs::msg::KeyValue correction;
    correction.key = "last_correction_norm_m";
    correction.value = std::to_string(commit_result.correction_norm_m);
    status.values.push_back(correction);
    diagnostic_msgs::msg::KeyValue anchor_initialized;
    anchor_initialized.key = "frame_anchor_initialized";
    anchor_initialized.value = frame_anchor_.initialized() ? "true" : "false";
    status.values.push_back(anchor_initialized);
    diagnostic_msgs::msg::KeyValue anchor_status;
    anchor_status.key = "frame_anchor_status";
    anchor_status.value = frame_anchor_.statusString();
    status.values.push_back(anchor_status);
    diagnostic_msgs::msg::KeyValue anchor_count;
    anchor_count.key = "frame_anchor_bootstrap_count";
    anchor_count.value = std::to_string(anchor_bootstrap_count_);
    status.values.push_back(anchor_count);
    diagnostic_msgs::msg::KeyValue anchor_reason;
    anchor_reason.key = "frame_anchor_bootstrap_reason";
    anchor_reason.value = anchor_bootstrap_reason_;
    status.values.push_back(anchor_reason);
    diagnostic_msgs::msg::KeyValue wheel_enabled;
    wheel_enabled.key = "wheel_factor_enabled";
    wheel_enabled.value = wheel_enabled_ ? "true" : "false";
    status.values.push_back(wheel_enabled);
    diagnostic_msgs::msg::KeyValue wheel_added;
    wheel_added.key = "wheel_factor_last_added";
    wheel_added.value = wheel_factor_last_added_ ? "true" : "false";
    status.values.push_back(wheel_added);
    diagnostic_msgs::msg::KeyValue wheel_reason;
    wheel_reason.key = "wheel_factor_reason";
    wheel_reason.value = wheel_factor_reason_;
    status.values.push_back(wheel_reason);
    const auto graph_diagnostics = graph_->diagnostics();
    addDiagnosticValue(status, "graph_state_count", graph_diagnostics.state_count);
    addDiagnosticValue(status, "graph_value_count", graph_diagnostics.value_count);
    addDiagnosticValue(status, "graph_factor_count", graph_diagnostics.factor_count);
    addDiagnosticValue(
      status, "graph_window_rebuild_count",
      graph_diagnostics.window_rebuild_count);
    addDiagnosticValue(status, "graph_latest_state_index", graph_diagnostics.latest_state_index);
    addDiagnosticValue(status, "graph_oldest_state_index", graph_diagnostics.oldest_state_index);
    addDiagnosticValue(
      status, "graph_last_optimization_ok",
      std::string(graph_diagnostics.last_optimization_ok ? "true" : "false"));
    addDiagnosticValue(
      status, "graph_last_optimization_error",
      graph_diagnostics.last_optimization_error);
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  template<typename T>
  void addDiagnosticValue(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key,
    const T & value)
  {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = std::to_string(value);
    status.values.push_back(entry);
  }

  void addDiagnosticValue(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key,
    const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(entry);
  }

  std::string fastlio_topic_;
  std::string imu_topic_;
  std::string wheel_topic_;
  std::string fix_topic_;
  std::string heading_topic_;
  std::string rtk_status_topic_;
  std::string raw_nmea_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_fgo_frame_;
  bool publish_tf_ = false;
  bool nav2_use_fgo_ = false;
  bool heading_quaternion_yaw_is_compass_ = true;
  bool imu_enabled_ = false;
  bool fastlio_enabled_ = true;
  bool wheel_enabled_ = true;
  bool rtk_position_enabled_ = true;
  bool rtk_heading_enabled_ = true;
  double window_duration_s_ = 15.0;
  double keyframe_rate_hz_ = 10.0;
  std::size_t max_states_ = 120;
  RtkGateParams gate_params_;
  std::size_t recovery_min_samples_ = 8;
  double max_position_jump_m_ = 3.0;
  double heading_sigma_rad_ = 0.05;
  FrameAnchorBootstrapParams anchor_params_;
  double anchor_max_position_innovation_m_ = 2.0;
  std::size_t anchor_bootstrap_count_ = 0;
  std::string anchor_bootstrap_reason_ = "not started";
  ImuPreintegrationConfig imu_config_;
  double max_translation_step_m_ = 0.15;
  double max_yaw_step_deg_ = 0.3;
  double latest_fix_sigma_m_ = 1.0;
  double last_lio_stamp_s_ = 0.0;
  gtsam::Pose3 last_lio_pose_;
  bool has_output_pose_ = false;
  gtsam::Pose3 output_pose_;
  std::string latest_rtk_status_ = "-";
  std::optional<gtsam::Point3> previous_mapped_fix_;
  std::optional<double> previous_mapped_fix_stamp_s_;
  std::optional<gtsam::Pose3> previous_wheel_pose_;
  std::optional<double> previous_wheel_stamp_s_;
  bool wheel_factor_last_added_ = false;
  std::string wheel_factor_reason_ = "not started";

  TimestampedBuffer<nav_msgs::msg::Odometry> fastlio_buffer_{2.0};
  TimestampedBuffer<nav_msgs::msg::Odometry> wheel_buffer_{2.0};
  TimestampedBuffer<sensor_msgs::msg::NavSatFix> fix_buffer_{2.0};
  TimestampedBuffer<geometry_msgs::msg::QuaternionStamped> heading_buffer_{2.0};
  TimestampedBuffer<nmea_msgs::msg::Sentence> nmea_buffer_{2.0};

  std::unique_ptr<FgoGraph> graph_;
  LocalizationStateMachine state_machine_;
  std::unique_ptr<CorrectionSmoother> smoother_;
  FrameAnchor frame_anchor_;
  nav_msgs::msg::Path path_msg_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rtk_status_sub_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr raw_nmea_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr correction_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rtk_fgo_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rtk_fgo_localizer::RtkFgoNode>());
  rclcpp::shutdown();
  return 0;
}

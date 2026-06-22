#include "rtk_fgo_localizer/correction_smoother.hpp"
#include "rtk_fgo_localizer/fgo_graph.hpp"
#include "rtk_fgo_localizer/rtk_quality.hpp"
#include "rtk_fgo_localizer/state_machine.hpp"
#include "rtk_fgo_localizer/topic_buffers.hpp"

#include <GeographicLib/LocalCartesian.hpp>
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
    graph_->setMaxShadowCorrection(max_position_jump_m_);
    smoother_ = std::make_unique<CorrectionSmoother>(
      max_translation_step_m_, max_yaw_step_deg_ * kPi / 180.0);

    subscribeTopics();
    createPublishers();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, keyframe_rate_hz_)),
      [this]() { onTimer(); });
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
    declare_parameter<double>("window.duration_s", 15.0);
    declare_parameter<double>("window.keyframe_rate_hz", 10.0);
    declare_parameter<int>("window.max_states", 120);
    declare_parameter<int>("rtk_gating.recovery_min_samples", 8);
    declare_parameter<double>("rtk_gating.max_position_jump_m", 3.0);
    declare_parameter<double>("rtk_gating.heading_sigma_rad", 0.05);
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
    window_duration_s_ = get_parameter("window.duration_s").as_double();
    keyframe_rate_hz_ = get_parameter("window.keyframe_rate_hz").as_double();
    max_states_ = static_cast<std::size_t>(
      std::max<std::int64_t>(2, get_parameter("window.max_states").as_int()));
    recovery_min_samples_ =
      static_cast<std::size_t>(
      std::max<std::int64_t>(1, get_parameter("rtk_gating.recovery_min_samples").as_int()));
    max_position_jump_m_ = get_parameter("rtk_gating.max_position_jump_m").as_double();
    heading_sigma_rad_ = get_parameter("rtk_gating.heading_sigma_rad").as_double();
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
    } else if (latest_lio->stamp_s > last_lio_stamp_s_ + 1e-4) {
      graph_->addFastLioBetween(latest_lio->stamp_s, last_lio_pose_.between(lio_pose));
      last_lio_stamp_s_ = latest_lio->stamp_s;
      last_lio_pose_ = lio_pose;
    }

    auto gate_decision = evaluateLatestRtkGate();
    auto state = state_machine_.update(gate_decision.mode, true);
    ShadowCommitResult commit_result;
    if (gate_decision.mode == RtkGateMode::StrongCandidate &&
      (state == LocalizationState::RtkRecovery || state == LocalizationState::RtkLocked))
    {
      commit_result = tryCommitLatestRtk();
      if (commit_result.committed) {
        state_machine_.markRecoveryCommitted();
      } else if (state == LocalizationState::RtkRecovery) {
        state_machine_.markRecoveryRejected();
      }
      addLatestHeadingFactor();
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
    const auto fix_point = latestFixPoint();
    if (!estimate.has_value() || !fix_point.has_value()) {
      return {RtkGateMode::Rejected, "missing graph estimate or valid fix"};
    }

    const double position_innovation =
      (estimate->pose.translation() - *fix_point).norm();
    double heading_innovation = 0.0;
    if (auto heading = heading_buffer_.closest(estimate->stamp_s, 0.5); heading.has_value()) {
      heading_innovation =
        normalizeYaw(estimate->pose.rotation().yaw() - yawFromQuaternion(heading->value.quaternion));
      quality->heading_stable = true;
    }
    return evaluateRtkGate(*quality, position_innovation, heading_innovation);
  }

  std::optional<gtsam::Point3> latestFixPoint()
  {
    auto latest_fix = fix_buffer_.latest();
    if (!latest_fix.has_value() || !validFix(latest_fix->value)) {
      return std::nullopt;
    }
    const auto & fix = latest_fix->value;
    if (!local_cartesian_) {
      local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
        fix.latitude, fix.longitude, fix.altitude);
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    local_cartesian_->Forward(fix.latitude, fix.longitude, fix.altitude, x, y, z);
    latest_fix_sigma_m_ = fixSigmaXY(fix);
    return gtsam::Point3(x, y, z);
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
    auto heading = heading_buffer_.closest(estimate->stamp_s, 0.5);
    if (!heading.has_value()) {
      return;
    }
    graph_->addRtkHeading(estimate->stamp_s, yawFromQuaternion(heading->value.quaternion), heading_sigma_rad_);
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
    const auto step = smoother_->step({
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
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
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
  double window_duration_s_ = 15.0;
  double keyframe_rate_hz_ = 10.0;
  std::size_t max_states_ = 120;
  std::size_t recovery_min_samples_ = 8;
  double max_position_jump_m_ = 3.0;
  double heading_sigma_rad_ = 0.05;
  double max_translation_step_m_ = 0.15;
  double max_yaw_step_deg_ = 0.3;
  double latest_fix_sigma_m_ = 1.0;
  double last_lio_stamp_s_ = 0.0;
  gtsam::Pose3 last_lio_pose_;
  bool has_output_pose_ = false;
  gtsam::Pose3 output_pose_;
  std::string latest_rtk_status_ = "-";

  TimestampedBuffer<nav_msgs::msg::Odometry> fastlio_buffer_{2.0};
  TimestampedBuffer<nav_msgs::msg::Odometry> wheel_buffer_{2.0};
  TimestampedBuffer<sensor_msgs::msg::NavSatFix> fix_buffer_{2.0};
  TimestampedBuffer<geometry_msgs::msg::QuaternionStamped> heading_buffer_{2.0};
  TimestampedBuffer<nmea_msgs::msg::Sentence> nmea_buffer_{2.0};

  std::unique_ptr<FgoGraph> graph_;
  LocalizationStateMachine state_machine_;
  std::unique_ptr<CorrectionSmoother> smoother_;
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
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

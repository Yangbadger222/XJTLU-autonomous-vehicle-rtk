#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "frc_nodes_cpp/event_marker_core.hpp"

#include "frc_msgs/msg/chassis_status.hpp"
#include "frc_msgs/msg/event_marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace frc_nodes_cpp
{

namespace
{

struct Se2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion yawToQuat(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

std::string getenvOrEmpty(const char * name)
{
  const char * value = std::getenv(name);
  return value == nullptr ? std::string() : std::string(value);
}

std::filesystem::path runtimeRoot()
{
  const auto env_root = getenvOrEmpty("FYP_RUNTIME_ROOT");
  if (!env_root.empty()) {
    return std::filesystem::path(env_root);
  }
  const auto home = getenvOrEmpty("HOME");
  if (!home.empty()) {
    return std::filesystem::path(home) / "XJTLU-autonomous-vehicle" / "runtime-data";
  }
  return std::filesystem::path("runtime-data");
}

std::filesystem::path frcDir(const std::string & subdir)
{
  auto path = runtimeRoot() / "frc" / subdir;
  std::filesystem::create_directories(path);
  return path;
}

std::string makeAdhocSessionId()
{
  std::time_t now = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &now);
#else
  localtime_r(&now, &tm);
#endif
  std::ostringstream out;
  out << "adhoc-" << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
  return out.str();
}

std::string sessionId()
{
  const auto session_dir = getenvOrEmpty("FYP_LOG_SESSION_DIR");
  if (!session_dir.empty()) {
    std::filesystem::path path(session_dir);
    if (path.filename() == "data") {
      path = path.parent_path();
    }
    if (!path.filename().empty()) {
      return path.filename().string();
    }
  }
  return makeAdhocSessionId();
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream out;
  for (const char ch : input) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

}  // namespace

class EventMarkerNode : public rclcpp::Node
{
public:
  EventMarkerNode()
  : Node("frc_event_marker"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    core_(EventMarkerConfig{}),
    session_id_(sessionId()),
    jsonl_path_(frcDir("events") / (session_id_ + ".jsonl"))
  {
    route_id_ = declare_parameter<std::string>("route_id", "");
    layout_id_ = declare_parameter<std::string>("layout_id", "");
    declare_parameter<double>("stuck_odom_speed", 0.05);
    declare_parameter<double>("stuck_cmd_speed", 0.2);
    declare_parameter<double>("stuck_duration_s", 3.0);
    declare_parameter<double>("nearcol_clearance_m", 0.35);
    declare_parameter<double>("nearcol_duration_s", 0.5);
    declare_parameter<int>("jerk_min_samples", 200);
    plan_lookahead_m_ = declare_parameter<double>("plan_lookahead_m", 5.0);
    plan_window_ = declare_parameter<int>("plan_window", 10);
    declare_parameter<double>("cooldown_s", 8.0);
    core_ = EventMarkerCore(makeConfig());

    chassis_sub_ = create_subscription<frc_msgs::msg::ChassisStatus>(
      "/chassis/status", 20,
      [this](const frc_msgs::msg::ChassisStatus::SharedPtr msg) {
        onChassis(*msg);
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 20,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        onCmd(*msg);
      });
    chassis_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_CBoar", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        onChassisOdom(*msg);
      });
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fastlio2/lio_odom", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        onLio(*msg);
      });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        onPlan(*msg);
      });
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      "/local_costmap/costmap_raw", 2,
      [this](const nav2_msgs::msg::Costmap::SharedPtr msg) {
        onCostmap(*msg);
      });
    bt_sub_ = create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      "/behavior_tree_log", 10,
      [this](const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
        onBtLog(*msg);
      });

    pub_ = create_publisher<frc_msgs::msg::EventMarker>("/frc/event_marker", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (const auto event = core_.onTick(nowS()); event.has_value()) {
          emit(event.value());
        }
      });

    RCLCPP_INFO(
      get_logger(), "frc_event_marker_cpp started, session=%s",
      session_id_.c_str());
  }

private:
  EventMarkerConfig makeConfig()
  {
    EventMarkerConfig cfg;
    cfg.stuck_odom_speed = get_parameter_or("stuck_odom_speed", 0.05);
    cfg.stuck_cmd_speed = get_parameter_or("stuck_cmd_speed", 0.2);
    cfg.stuck_duration_s = get_parameter_or("stuck_duration_s", 3.0);
    cfg.nearcol_clearance_m = get_parameter_or("nearcol_clearance_m", 0.35);
    cfg.nearcol_duration_s = get_parameter_or("nearcol_duration_s", 0.5);
    cfg.jerk_min_samples = get_parameter_or("jerk_min_samples", 200);
    cfg.cooldown_s = get_parameter_or("cooldown_s", 8.0);
    return cfg;
  }

  template <typename T>
  T get_parameter_or(const std::string & name, const T & fallback)
  {
    if (has_parameter(name)) {
      return get_parameter(name).get_value<T>();
    }
    return fallback;
  }

  double nowS()
  {
    return now().nanoseconds() * 1e-9;
  }

  void onChassis(const frc_msgs::msg::ChassisStatus & msg)
  {
    for (const auto & event : core_.onChassis(msg.ctrl_mode, msg.ps2_key, nowS())) {
      emit(event);
    }
  }

  void onCmd(const geometry_msgs::msg::Twist & msg)
  {
    if (const auto event = core_.onCmd(msg.linear.x, msg.angular.z, nowS());
      event.has_value())
    {
      emit(event.value());
    }
  }

  void onChassisOdom(const nav_msgs::msg::Odometry & msg)
  {
    core_.onOdomVelocity(msg.twist.twist.linear.x, msg.twist.twist.linear.y);
  }

  void onLio(const nav_msgs::msg::Odometry & msg)
  {
    pose_odom_ = Se2{
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      quatToYaw(msg.pose.pose.orientation)};
  }

  void onCostmap(const nav2_msgs::msg::Costmap & msg)
  {
    if (!pose_odom_.has_value()) {
      return;
    }

    const auto & metadata = msg.metadata;
    const auto width = static_cast<std::size_t>(metadata.size_x);
    const auto height = static_cast<std::size_t>(metadata.size_y);
    if (width == 0 || height == 0 || msg.data.size() < width * height) {
      return;
    }

    const double resolution = metadata.resolution;
    const double origin_x = metadata.origin.position.x;
    const double origin_y = metadata.origin.position.y;
    const double rx = pose_odom_->x;
    const double ry = pose_odom_->y;
    double nearest = std::numeric_limits<double>::infinity();

    for (std::size_t y = 0; y < height; ++y) {
      for (std::size_t x = 0; x < width; ++x) {
        const auto cost = msg.data[y * width + x];
        if (cost < 253U) {
          continue;
        }
        const double cx = origin_x + static_cast<double>(x) * resolution;
        const double cy = origin_y + static_cast<double>(y) * resolution;
        nearest = std::min(nearest, std::hypot(cx - rx, cy - ry));
      }
    }

    const std::optional<double> clearance =
      std::isfinite(nearest) ? std::optional<double>(nearest) : std::nullopt;
    if (const auto event = core_.onCostmapClearance(clearance, nowS());
      event.has_value())
    {
      emit(event.value());
    }
  }

  void onBtLog(const nav2_msgs::msg::BehaviorTreeLog & msg)
  {
    for (const auto & event : msg.event_log) {
      if (const auto decision = core_.onRecovery(
          event.node_name, event.current_status, nowS());
        decision.has_value())
      {
        emit(decision.value());
        return;
      }
    }
  }

  void onPlan(const nav_msgs::msg::Path & msg)
  {
    if (!pose_odom_.has_value() || msg.poses.size() < 2) {
      return;
    }

    std::vector<std::pair<double, double>> points;
    points.reserve(msg.poses.size());
    for (const auto & stamped_pose : msg.poses) {
      points.emplace_back(
        stamped_pose.pose.position.x,
        stamped_pose.pose.position.y);
    }

    std::vector<std::pair<double, double>> ahead;
    ahead.reserve(points.size());
    for (const auto & p : points) {
      const double d = std::hypot(p.first - pose_odom_->x, p.second - pose_odom_->y);
      if (d <= plan_lookahead_m_) {
        ahead.push_back(p);
      }
    }

    if (last_plan_.size() >= 2 && ahead.size() >= 2) {
      double total_min_dist = 0.0;
      for (const auto & p : ahead) {
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto & q : last_plan_) {
          min_dist = std::min(min_dist, std::hypot(p.first - q.first, p.second - q.second));
        }
        total_min_dist += min_dist;
      }
      const double dev = total_min_dist / static_cast<double>(ahead.size());
      plan_devs_.push_back(dev);
      while (static_cast<int>(plan_devs_.size()) > plan_window_) {
        plan_devs_.pop_front();
      }
      double square_sum = 0.0;
      for (const double x : plan_devs_) {
        square_sum += x * x;
      }
      const double rms = std::sqrt(square_sum / static_cast<double>(plan_devs_.size()));
      if (const auto event = core_.onPlanRms(rms, nowS()); event.has_value()) {
        emit(event.value());
      }
    }

    last_plan_ = ahead.size() >= 2 ? ahead : points;
  }

  Se2 poseInMap()
  {
    if (!pose_odom_.has_value()) {
      return {};
    }

    try {
      const auto tf = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      const double tyaw = quatToYaw(q);
      const double tx = tf.transform.translation.x;
      const double ty = tf.transform.translation.y;
      const double x = pose_odom_->x;
      const double y = pose_odom_->y;
      return Se2{
        tx + x * std::cos(tyaw) - y * std::sin(tyaw),
        ty + x * std::sin(tyaw) + y * std::cos(tyaw),
        pose_odom_->yaw + tyaw};
    } catch (const tf2::TransformException &) {
      return pose_odom_.value();
    }
  }

  void emit(const EventDecision & event)
  {
    const auto pose = poseInMap();

    frc_msgs::msg::EventMarker msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.type = event.type;
    msg.severity = event.severity;
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.orientation = yawToQuat(pose.yaw);
    msg.route_id = route_id_;
    msg.layout_id = layout_id_;
    msg.note = event.note;
    msg.session_id = session_id_;
    pub_->publish(msg);

    RCLCPP_WARN(
      get_logger(), "EVENT %s/%s at (%.2f,%.2f) %s",
      event.severity.c_str(), event.type.c_str(), pose.x, pose.y,
      event.note.c_str());

    std::ofstream out(jsonl_path_, std::ios::app);
    if (out) {
      out << "{\"stamp\":" << nowS()
          << ",\"type\":\"" << jsonEscape(event.type)
          << "\",\"severity\":\"" << jsonEscape(event.severity)
          << "\",\"map_x\":" << pose.x
          << ",\"map_y\":" << pose.y
          << ",\"map_yaw\":" << pose.yaw
          << ",\"note\":\"" << jsonEscape(event.note)
          << "\",\"route_id\":\"" << jsonEscape(route_id_)
          << "\",\"layout_id\":\"" << jsonEscape(layout_id_)
          << "\",\"session_id\":\"" << jsonEscape(session_id_)
          << "\"}\n";
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  EventMarkerCore core_;
  std::string route_id_;
  std::string layout_id_;
  std::string session_id_;
  std::filesystem::path jsonl_path_;
  std::optional<Se2> pose_odom_;
  double plan_lookahead_m_{5.0};
  int plan_window_{10};
  std::vector<std::pair<double, double>> last_plan_;
  std::deque<double> plan_devs_;

  rclcpp::Subscription<frc_msgs::msg::ChassisStatus>::SharedPtr chassis_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr chassis_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_sub_;
  rclcpp::Publisher<frc_msgs::msg::EventMarker>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace frc_nodes_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frc_nodes_cpp::EventMarkerNode>());
  rclcpp::shutdown();
  return 0;
}

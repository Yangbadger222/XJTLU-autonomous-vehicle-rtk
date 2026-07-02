#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "frc_nodes_cpp/risk_pipeline_core.hpp"

#include "frc_msgs/msg/anchor_state_array.hpp"
#include "frc_msgs/msg/health.hpp"
#include "frc_msgs/msg/risk_grid.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

template <typename T>
T readLE(const std::vector<uint8_t> & data, const std::size_t offset)
{
  T value{};
  if (offset + sizeof(T) <= data.size()) {
    std::memcpy(&value, data.data() + offset, sizeof(T));
  }
  return value;
}

}  // namespace

class CloudBuffer
{
public:
  explicit CloudBuffer(RiskPipelineConfig config)
  : config_(config)
  {
  }

  void pushCloud(std::vector<Point3> points, const double stamp_s)
  {
    clouds_.emplace_back(stamp_s, std::move(points));
    const double horizon = stamp_s - config_.agg_window_s;
    while (!clouds_.empty() && clouds_.front().first < horizon) {
      clouds_.pop_front();
    }
  }

  std::vector<float> buildMaxZLocal(const Se2 & pose) const
  {
    const int n = config_.gridSize();
    const double half = config_.size_m / 2.0;
    std::vector<float> max_z(static_cast<std::size_t>(n * n), 0.0F);
    std::vector<uint8_t> occupied(static_cast<std::size_t>(n * n), 0U);

    const double cos_y = std::cos(pose.yaw);
    const double sin_y = std::sin(pose.yaw);
    for (const auto & cloud : clouds_) {
      for (const auto & p : cloud.second) {
        const double dx = p.x - pose.x;
        const double dy = p.y - pose.y;
        const double lx = dx * cos_y + dy * sin_y;
        const double ly = -dx * sin_y + dy * cos_y;
        if (p.z < config_.z_min || p.z > config_.z_max ||
          std::abs(lx) >= half || std::abs(ly) >= half)
        {
          continue;
        }
        const int row = std::clamp(
          static_cast<int>((lx + half) / config_.resolution), 0, n - 1);
        const int col = std::clamp(
          static_cast<int>((ly + half) / config_.resolution), 0, n - 1);
        const auto idx = static_cast<std::size_t>(row * n + col);
        if (!occupied[idx] || p.z > max_z[idx]) {
          occupied[idx] = 1U;
          max_z[idx] = static_cast<float>(p.z);
        }
      }
    }
    return max_z;
  }

private:
  RiskPipelineConfig config_;
  std::deque<std::pair<double, std::vector<Point3>>> clouds_;
};

class RiskPipelineNode : public rclcpp::Node
{
public:
  RiskPipelineNode()
  : Node("frc_risk_pipeline"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    cloud_buffer_(config_)
  {
    declare_parameter<double>("tick_rate_hz", 10.0);
    declare_parameter<double>("tick_budget_ms", 80.0);
    declare_parameter<int>("overrun_disable_count", 5);
    declare_parameter<bool>("shadow_mode", true);
    declare_parameter<bool>("debug_tap", false);
    declare_parameter<double>("anchor_sigma_m", 0.75);
    declare_parameter<double>("feature_sigma_m", 1.0);
    declare_parameter<double>("feature_min_score", 0.15);
    declare_parameter<std::string>("prototypes_path", "");
    declare_parameter<std::string>("model_engine_path", "");

    const auto prototypes_path = get_parameter("prototypes_path").as_string();
    const auto model_engine_path = get_parameter("model_engine_path").as_string();
    if (!prototypes_path.empty() && std::filesystem::exists(prototypes_path)) {
      RCLCPP_WARN(
        get_logger(),
        "frc_risk_pipeline_cpp does not yet load prototype NPZ files; "
        "feature-retrieval risk is disabled for %s",
        prototypes_path.c_str());
    }
    if (!model_engine_path.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "frc_risk_pipeline_cpp does not yet run TensorRT models; model risk is disabled");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fastlio2/lio_odom", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        onOdom(*msg);
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/fastlio2/body_cloud", 5,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        onCloud(*msg);
      });
    health_sub_ = create_subscription<frc_msgs::msg::Health>(
      "/frc/health", 10,
      [this](const frc_msgs::msg::Health::SharedPtr msg) {
        onHealth(*msg);
      });
    anchor_sub_ = create_subscription<frc_msgs::msg::AnchorStateArray>(
      "/frc/anchor_states", 10,
      [this](const frc_msgs::msg::AnchorStateArray::SharedPtr msg) {
        onAnchors(*msg);
      });
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      "/local_costmap/costmap_raw", 2,
      [this](const nav2_msgs::msg::Costmap::SharedPtr msg) {
        latest_costmap_stamp_ = stampToSec(msg->header.stamp);
      });

    risk_pub_ = create_publisher<frc_msgs::msg::RiskGrid>("/frc/risk_grid", 5);
    viz_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/frc/risk_grid_viz", 2);
    debug_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/frc/bev_debug", 2);

    const double rate = get_parameter("tick_rate_hz").as_double();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(rate, 1.0)));
    timer_ = create_wall_timer(period, [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "frc_risk_pipeline_cpp started: map-anchor risk enabled, "
      "feature/model risk disabled until C++ artifact loaders are added");
  }

private:
  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    pose_ = Se2{
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      quatToYaw(msg.pose.pose.orientation)};
    pose_stamp_s_ = stampToSec(msg.header.stamp);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!pose_.has_value()) {
      return;
    }

    std::optional<std::size_t> x_offset;
    std::optional<std::size_t> y_offset;
    std::optional<std::size_t> z_offset;
    for (const auto & field : msg.fields) {
      if (field.name == "x") {
        x_offset = field.offset;
      } else if (field.name == "y") {
        y_offset = field.offset;
      } else if (field.name == "z") {
        z_offset = field.offset;
      }
    }
    if (!x_offset.has_value() || !y_offset.has_value() || !z_offset.has_value() ||
      msg.point_step == 0)
    {
      return;
    }

    const std::size_t count = static_cast<std::size_t>(msg.width) *
      static_cast<std::size_t>(msg.height);
    std::vector<Point3> points;
    points.reserve(count);

    const auto pose = pose_.value();
    const double cos_y = std::cos(pose.yaw);
    const double sin_y = std::sin(pose.yaw);
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t base = i * msg.point_step;
      if (base + msg.point_step > msg.data.size()) {
        break;
      }
      const float bx = readLE<float>(msg.data, base + x_offset.value());
      const float by = readLE<float>(msg.data, base + y_offset.value());
      const float bz = readLE<float>(msg.data, base + z_offset.value());
      if (!std::isfinite(bx) || !std::isfinite(by) || !std::isfinite(bz)) {
        continue;
      }
      points.push_back(Point3{
        pose.x + static_cast<double>(bx) * cos_y - static_cast<double>(by) * sin_y,
        pose.y + static_cast<double>(bx) * sin_y + static_cast<double>(by) * cos_y,
        static_cast<double>(bz)});
    }

    cloud_buffer_.pushCloud(std::move(points), stampToSec(msg.header.stamp));
  }

  void onHealth(const frc_msgs::msg::Health & msg)
  {
    health_ = {
      msg.lio_min_eig,
      msg.lio_degenerate ? 1.0F : 0.0F,
      msg.pgo_correcting ? 1.0F : 0.0F,
      static_cast<float>(msg.rtk_status),
      msg.v,
      msg.w};
  }

  void onAnchors(const frc_msgs::msg::AnchorStateArray & msg)
  {
    anchors_.clear();
    anchors_.reserve(msg.anchors.size());
    for (const auto & anchor : msg.anchors) {
      anchors_.push_back(AnchorStateLite{
        anchor.state,
        anchor.p_usable,
        anchor.severity,
        anchor.map_x,
        anchor.map_y});
    }
  }

  std::optional<Se2> mapToOdom()
  {
    try {
      const auto tf = tf_buffer_.lookupTransform("odom", "map", tf2::TimePointZero);
      return Se2{
        tf.transform.translation.x,
        tf.transform.translation.y,
        quatToYaw(tf.transform.rotation)};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void tick()
  {
    if (!pose_.has_value()) {
      return;
    }
    const auto start = std::chrono::steady_clock::now();

    const auto pose = pose_.value();
    const int n = config_.gridSize();
    const double half = config_.size_m / 2.0;
    const double origin_x = pose.x - half;
    const double origin_y = pose.y - half;
    RiskFrame frame(n, config_.resolution, origin_x, origin_y);
    uint8_t source_mask = 0U;

    const auto tf_mo = mapToOdom();
    const float anchor_sigma = static_cast<float>(
      get_parameter("anchor_sigma_m").as_double());
    if (tf_mo.has_value()) {
      const double tx = tf_mo->x;
      const double ty = tf_mo->y;
      const double tyaw = tf_mo->yaw;
      const double cos_t = std::cos(tyaw);
      const double sin_t = std::sin(tyaw);
      for (const auto & anchor : anchors_) {
        const double weight = injectionWeight(anchor);
        if (weight <= 0.0) {
          continue;
        }
        const double ox = tx + anchor.map_x * cos_t - anchor.map_y * sin_t;
        const double oy = ty + anchor.map_x * sin_t + anchor.map_y * cos_t;
        const float confidence = static_cast<float>(
          anchor.p_usable * (anchor.state == "candidate" ? 0.8 : 1.0));
        renderGaussian(
          frame, ox, oy, static_cast<float>(weight), anchor_sigma,
          confidence, kSourceMapAnchor);
        source_mask |= kSourceMapAnchor;
      }
    }

    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    const double budget_ms = get_parameter("tick_budget_ms").as_double();
    if (elapsed_ms > budget_ms) {
      ++overruns_;
      RCLCPP_WARN(
        get_logger(), "tick overrun %.0fms > %.0fms (%d consecutive)",
        elapsed_ms, budget_ms, overruns_);
      if (overruns_ >= get_parameter("overrun_disable_count").as_int()) {
        if (publishing_) {
          RCLCPP_ERROR(
            get_logger(),
            "consecutive overruns: risk_grid publishing disabled, "
            "frc_layer watchdog will bypass");
        }
        publishing_ = false;
        return;
      }
    } else {
      if (!publishing_) {
        RCLCPP_INFO(get_logger(), "tick recovered, publishing resumed");
      }
      overruns_ = 0;
      publishing_ = true;
    }

    if (!publishing_) {
      return;
    }

    publishRisk(frame, source_mask);
    publishDebugIfNeeded(frame);
  }

  nav_msgs::msg::OccupancyGrid makeGrid(
    const std::vector<float> & values,
    const double origin_x,
    const double origin_y)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = "odom";
    grid.info.resolution = static_cast<float>(config_.resolution);
    grid.info.width = static_cast<uint32_t>(config_.gridSize());
    grid.info.height = static_cast<uint32_t>(config_.gridSize());
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data = toOccupancyData(values);
    return grid;
  }

  void publishRisk(const RiskFrame & frame, const uint8_t source_mask)
  {
    frc_msgs::msg::RiskGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.risk = makeGrid(frame.risk, frame.origin_x, frame.origin_y);
    msg.confidence = makeGrid(frame.confidence, frame.origin_x, frame.origin_y);
    msg.source_mask = source_mask;
    risk_pub_->publish(msg);

    if (viz_pub_->get_subscription_count() > 0) {
      viz_pub_->publish(makeGrid(frame.risk, frame.origin_x, frame.origin_y));
    }
  }

  std::vector<float> warpLocalGridToOdom(const std::vector<float> & local_grid)
  {
    const int n = config_.gridSize();
    std::vector<float> out(static_cast<std::size_t>(n * n), 0.0F);
    if (!pose_.has_value()) {
      return out;
    }

    const auto pose = pose_.value();
    const double half = config_.size_m / 2.0;
    const double origin_x = pose.x - half;
    const double origin_y = pose.y - half;
    const double cos_y = std::cos(pose.yaw);
    const double sin_y = std::sin(pose.yaw);

    for (int row = 0; row < n; ++row) {
      const double wy = origin_y + (static_cast<double>(row) + 0.5) * config_.resolution;
      for (int col = 0; col < n; ++col) {
        const double wx = origin_x + (static_cast<double>(col) + 0.5) * config_.resolution;
        const double dx = wx - pose.x;
        const double dy = wy - pose.y;
        const double lx = dx * cos_y + dy * sin_y;
        const double ly = -dx * sin_y + dy * cos_y;
        const int ri = static_cast<int>(std::floor((lx + half) / config_.resolution));
        const int ci = static_cast<int>(std::floor((ly + half) / config_.resolution));
        if (ri >= 0 && ri < n && ci >= 0 && ci < n) {
          out[static_cast<std::size_t>(row * n + col)] =
            local_grid[static_cast<std::size_t>(ri * n + ci)];
        }
      }
    }
    return out;
  }

  void publishDebugIfNeeded(const RiskFrame & frame)
  {
    if (!get_parameter("debug_tap").as_bool() || debug_pub_->get_subscription_count() == 0 ||
      !pose_.has_value())
    {
      return;
    }
    auto max_z = cloud_buffer_.buildMaxZLocal(pose_.value());
    for (auto & value : max_z) {
      value = static_cast<float>(
        std::clamp(
          (static_cast<double>(value) - config_.z_min) /
          (config_.z_max - config_.z_min),
          0.0, 1.0));
    }
    debug_pub_->publish(makeGrid(warpLocalGridToOdom(max_z), frame.origin_x, frame.origin_y));
  }

  RiskPipelineConfig config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  CloudBuffer cloud_buffer_;

  std::optional<Se2> pose_;
  double pose_stamp_s_{0.0};
  double latest_costmap_stamp_{0.0};
  std::vector<float> health_{0.0F, 0.0F, 0.0F, -1.0F, 0.0F, 0.0F};
  std::vector<AnchorStateLite> anchors_;
  int overruns_{0};
  bool publishing_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<frc_msgs::msg::Health>::SharedPtr health_sub_;
  rclcpp::Subscription<frc_msgs::msg::AnchorStateArray>::SharedPtr anchor_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Publisher<frc_msgs::msg::RiskGrid>::SharedPtr risk_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace frc_nodes_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frc_nodes_cpp::RiskPipelineNode>());
  rclcpp::shutdown();
  return 0;
}

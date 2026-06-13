#include <algorithm>
#include <array>
#include <chrono>
#include <memory>

#include "frc_nodes_cpp/health_aggregator_core.hpp"

#include "frc_msgs/msg/health.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace frc_nodes_cpp
{

class HealthAggregatorNode : public rclcpp::Node
{
public:
  HealthAggregatorNode()
  : Node("frc_health_aggregator"),
    core_(
      declare_parameter<double>("stale_timeout_s", 1.0),
      declare_parameter<double>("lio_min_eig_degenerate", 75.0))
  {
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);

    degeneracy_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/fastlio2/degeneracy", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        on_degeneracy(*msg);
      });
    correction_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/pgo/correction_status", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        on_correction(*msg);
      });
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        on_fix(*msg);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_CBoar", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        on_odom(*msg);
      });

    pub_ = create_publisher<frc_msgs::msg::Health>("/frc/health", 10);

    const double period_s = 1.0 / std::max(publish_rate_hz, 0.1);
    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s));
    timer_ = create_wall_timer(period, [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "frc_health_aggregator_cpp started");
  }

private:
  double now_s() const
  {
    return get_clock()->now().nanoseconds() * 1e-9;
  }

  void on_degeneracy(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (msg.data.size() >= 3) {
      inputs_.degeneracy = StampedArray<3>(
        now_s(), std::array<float, 3>{msg.data[0], msg.data[1], msg.data[2]});
    }
  }

  void on_correction(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (msg.data.size() >= 2) {
      inputs_.correction = StampedArray<2>(
        now_s(), std::array<float, 2>{msg.data[0], msg.data[1]});
    }
  }

  void on_fix(const sensor_msgs::msg::NavSatFix & msg)
  {
    inputs_.fix_status = StampedStatus(
      now_s(), static_cast<int8_t>(msg.status.status));
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    inputs_.velocity = StampedArray<2>(
      now_s(),
      std::array<float, 2>{
        static_cast<float>(msg.twist.twist.linear.x),
        static_cast<float>(msg.twist.twist.angular.z)});
  }

  void tick()
  {
    const auto output = core_.build(inputs_, now_s());

    frc_msgs::msg::Health msg;
    msg.header.stamp = get_clock()->now().to_msg();
    msg.lio_min_eig = output.lio_min_eig;
    msg.lio_cond = output.lio_cond;
    msg.lio_degenerate = output.lio_degenerate;
    msg.pgo_correcting = output.pgo_correcting;
    msg.pgo_last_jump = output.pgo_last_jump;
    msg.rtk_status = output.rtk_status;
    msg.v = output.v;
    msg.w = output.w;

    pub_->publish(msg);
  }

  HealthInputs inputs_;
  HealthAggregatorCore core_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr degeneracy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr correction_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<frc_msgs::msg::Health>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace frc_nodes_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frc_nodes_cpp::HealthAggregatorNode>());
  rclcpp::shutdown();
  return 0;
}

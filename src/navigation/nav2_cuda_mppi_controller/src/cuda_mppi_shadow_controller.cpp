#include "nav2_cuda_mppi_controller/cuda_mppi_shadow_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <mutex>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

namespace nav2_cuda_mppi_controller
{
namespace
{

template<typename ValueT>
void declareIfMissing(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, ValueT value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, value);
  }
}

diagnostic_msgs::msg::KeyValue value(const std::string & key, const std::string & content)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = content;
  return item;
}

}  // namespace

void CudaMppiShadowController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_mppi_controller::MPPIController::configure(parent, name, tf, costmap_ros);
  const auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("CUDA MPPI shadow controller lost lifecycle node during configure");
  }

  const auto prefix = name_ + ".";
  declareIfMissing(node, prefix + "cuda_shadow_enabled", true);
  declareIfMissing(node, prefix + "cuda_shadow_batch_size", 4096);
  declareIfMissing(node, prefix + "cuda_shadow_time_steps", 48);
  declareIfMissing(node, prefix + "cuda_shadow_vx_std", 0.28);
  declareIfMissing(node, prefix + "cuda_shadow_wz_std", 0.22);
  declareIfMissing(node, prefix + "cuda_shadow_temperature", 0.45);
  declareIfMissing(node, prefix + "cuda_shadow_gamma", 0.015);
  declareIfMissing(node, prefix + "cuda_shadow_path_weight", 16.0);
  declareIfMissing(node, prefix + "cuda_shadow_goal_weight", 5.0);
  declareIfMissing(node, prefix + "cuda_shadow_lookahead_points", 6);

  shadow_enabled_ = node->get_parameter(prefix + "cuda_shadow_enabled").as_bool();
  shadow_config_.batch_size = static_cast<std::size_t>(
    node->get_parameter(prefix + "cuda_shadow_batch_size").as_int());
  shadow_config_.time_steps = static_cast<std::size_t>(
    node->get_parameter(prefix + "cuda_shadow_time_steps").as_int());
  shadow_config_.vx_std = static_cast<float>(
    node->get_parameter(prefix + "cuda_shadow_vx_std").as_double());
  shadow_config_.wz_std = static_cast<float>(
    node->get_parameter(prefix + "cuda_shadow_wz_std").as_double());
  shadow_config_.temperature = static_cast<float>(
    node->get_parameter(prefix + "cuda_shadow_temperature").as_double());
  shadow_config_.gamma = static_cast<float>(
    node->get_parameter(prefix + "cuda_shadow_gamma").as_double());
  shadow_config_.path_weight = static_cast<float>(
    node->get_parameter(prefix + "cuda_shadow_path_weight").as_double());
  shadow_config_.goal_weight = static_cast<float>(
    node->get_parameter(prefix + "cuda_shadow_goal_weight").as_double());

  if (shadow_enabled_ && mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
    backend_ = std::make_unique<mppi_cuda_backend::CudaMppiBackend>();
  } else if (shadow_enabled_) {
    RCLCPP_WARN(logger_, "CUDA MPPI shadow disabled: no CUDA device available");
    shadow_enabled_ = false;
  }
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    name_ + "/cuda_shadow_diagnostics", rclcpp::QoS(10));
  RCLCPP_INFO(
    logger_, "CUDA MPPI shadow: enabled=%s batch=%zu steps=%zu",
    shadow_enabled_ ? "true" : "false", shadow_config_.batch_size, shadow_config_.time_steps);
}

geometry_msgs::msg::TwistStamped CudaMppiShadowController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  const auto cpu_command = nav2_mppi_controller::MPPIController::computeVelocityCommands(
    robot_pose, robot_speed, goal_checker);
  if (!shadow_enabled_ || !backend_) {
    return cpu_command;
  }

  try {
    const nav_msgs::msg::Path transformed_path = path_handler_.transformPath(robot_pose);
    if (transformed_path.poses.empty()) {
      publishDiagnostic(cpu_command, nullptr, "transformed_path_empty");
      return cpu_command;
    }
    const auto node = parent_.lock();
    if (!node) {
      throw std::runtime_error("CUDA MPPI shadow controller lost lifecycle node");
    }
    const std::size_t lookahead = static_cast<std::size_t>(std::max<std::int64_t>(
      0, node->get_parameter(name_ + ".cuda_shadow_lookahead_points").as_int()));
    const auto & target = transformed_path.poses.at(std::min(
      lookahead, transformed_path.poses.size() - 1U));
    const auto & goal = transformed_path.poses.back();

    auto * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
    mppi_cuda_backend::OptimizerInput input;
    input.robot_x = static_cast<float>(robot_pose.pose.position.x);
    input.robot_y = static_cast<float>(robot_pose.pose.position.y);
    input.robot_yaw = static_cast<float>(tf2::getYaw(robot_pose.pose.orientation));
    input.measured_vx = static_cast<float>(robot_speed.linear.x);
    input.measured_wz = static_cast<float>(robot_speed.angular.z);
    input.path_target_x = static_cast<float>(target.pose.position.x);
    input.path_target_y = static_cast<float>(target.pose.position.y);
    input.goal_x = static_cast<float>(goal.pose.position.x);
    input.goal_y = static_cast<float>(goal.pose.position.y);
    input.nominal_vx.assign(shadow_config_.time_steps, static_cast<float>(cpu_command.twist.linear.x));
    input.nominal_wz.assign(shadow_config_.time_steps, static_cast<float>(cpu_command.twist.angular.z));
    input.costmap = {
      costmap->getCharMap(),
      costmap->getSizeInCellsX(),
      costmap->getSizeInCellsY(),
      static_cast<float>(costmap->getResolution()),
      static_cast<float>(costmap->getOriginX()),
      static_cast<float>(costmap->getOriginY()),
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown(),
    };
    const auto gpu_result = backend_->optimize(shadow_config_, input);
    publishDiagnostic(cpu_command, &gpu_result, "ok");
  } catch (const std::exception & error) {
    publishDiagnostic(cpu_command, nullptr, error.what());
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "CUDA MPPI shadow failed: %s", error.what());
  }
  return cpu_command;
}

void CudaMppiShadowController::publishDiagnostic(
  const geometry_msgs::msg::TwistStamped & cpu_command,
  const mppi_cuda_backend::OptimizationResult * gpu_result,
  const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name_ + "/cuda_mppi_shadow";
  status.hardware_id = "jetson-orin-cuda";
  status.level = gpu_result == nullptr ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    (gpu_result->all_trajectories_collide ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK);
  status.message = message;
  status.values.push_back(value("cpu_vx", std::to_string(cpu_command.twist.linear.x)));
  status.values.push_back(value("cpu_wz", std::to_string(cpu_command.twist.angular.z)));
  status.values.push_back(value("batch_size", std::to_string(shadow_config_.batch_size)));
  status.values.push_back(value("time_steps", std::to_string(shadow_config_.time_steps)));
  if (gpu_result != nullptr) {
    status.values.push_back(value("gpu_elapsed_ms", std::to_string(gpu_result->gpu_elapsed_ms)));
    status.values.push_back(value("gpu_vx", std::to_string(gpu_result->control_vx.front())));
    status.values.push_back(value("gpu_wz", std::to_string(gpu_result->control_wz.front())));
    status.values.push_back(value("all_trajectories_collide", gpu_result->all_trajectories_collide ? "true" : "false"));
  }
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(std::move(array));
}

}  // namespace nav2_cuda_mppi_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_cuda_mppi_controller::CudaMppiShadowController,
  nav2_core::Controller)

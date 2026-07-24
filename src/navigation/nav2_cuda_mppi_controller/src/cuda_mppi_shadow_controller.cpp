#include "nav2_cuda_mppi_controller/cuda_mppi_shadow_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
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

float parameterFloat(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  return static_cast<float>(node->get_parameter(name).as_double());
}

std::size_t parameterSize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  const auto value = node->get_parameter(name).as_int();
  if (value <= 0) {
    throw std::invalid_argument(name + " must be positive for CUDA MPPI shadow");
  }
  return static_cast<std::size_t>(value);
}

float normalizeAngle(float angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool hasFiniteFirstControl(const mppi_cuda_backend::OptimizationResult & result)
{
  return !result.control_vx.empty() && !result.control_wz.empty() &&
         std::isfinite(result.control_vx.front()) && std::isfinite(result.control_wz.front()) &&
         std::isfinite(result.gpu_elapsed_ms);
}

void populatePathAndValidity(
  const nav_msgs::msg::Path & path, nav2_costmap_2d::Costmap2D & costmap,
  bool track_unknown, mppi_cuda_backend::OptimizerInput & input)
{
  if (path.poses.size() < 2U) {
    throw std::invalid_argument("transformed MPPI path must have at least two poses");
  }

  input.path_x.reserve(path.poses.size());
  input.path_y.reserve(path.poses.size());
  input.path_yaw.reserve(path.poses.size());
  input.path_integrated_distance.reserve(path.poses.size());
  input.path_integrated_distance.push_back(0.0F);
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index].pose;
    const float x = static_cast<float>(pose.position.x);
    const float y = static_cast<float>(pose.position.y);
    input.path_x.push_back(x);
    input.path_y.push_back(y);
    input.path_yaw.push_back(static_cast<float>(tf2::getYaw(pose.orientation)));
    if (index > 0U) {
      input.path_integrated_distance.push_back(
        input.path_integrated_distance.back() + std::hypot(
          x - input.path_x[index - 1U], y - input.path_y[index - 1U]));
    }
  }

  input.path_valid.assign(path.poses.size() - 1U, 0U);
  for (std::size_t index = 0U; index + 1U < path.poses.size(); ++index) {
    unsigned int map_x = 0U;
    unsigned int map_y = 0U;
    if (!costmap.worldToMap(input.path_x[index], input.path_y[index], map_x, map_y)) {
      continue;
    }
    const unsigned char cost = costmap.getCost(map_x, map_y);
    input.path_valid[index] = cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
      cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
      (cost != nav2_costmap_2d::NO_INFORMATION || track_unknown) ? 1U : 0U;
  }
}

void copyCpuControlSequence(
  nav2_mppi_controller::Optimizer & optimizer,
  const geometry_msgs::msg::PoseStamped & robot_pose, float model_dt,
  mppi_cuda_backend::OptimizerInput & input)
{
  // MPPIController has already emitted control index one and shifted its
  // sequence. The returned trajectory therefore encodes the exact nominal
  // sequence retained for the next CPU optimization cycle.
  const auto trajectory = optimizer.getOptimizedTrajectory();
  const std::size_t time_steps = trajectory.shape()[0];
  if (time_steps != input.nominal_vx.size()) {
    throw std::runtime_error("CPU MPPI control sequence length does not match CUDA shadow horizon");
  }

  float previous_x = static_cast<float>(robot_pose.pose.position.x);
  float previous_y = static_cast<float>(robot_pose.pose.position.y);
  float previous_yaw = static_cast<float>(tf2::getYaw(robot_pose.pose.orientation));
  for (std::size_t index = 0U; index < time_steps; ++index) {
    const float x = trajectory(index, 0);
    const float y = trajectory(index, 1);
    const float yaw = trajectory(index, 2);
    const float dx = x - previous_x;
    const float dy = y - previous_y;
    input.nominal_vx[index] =
      (dx * std::cos(previous_yaw) + dy * std::sin(previous_yaw)) / model_dt;
    input.nominal_wz[index] = normalizeAngle(yaw - previous_yaw) / model_dt;
    previous_x = x;
    previous_y = y;
    previous_yaw = yaw;
  }
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
  declareIfMissing(node, prefix + "cuda_mppi_authority_enabled", false);
  declareIfMissing(node, prefix + "cuda_mppi_authority_max_gpu_elapsed_ms", 20.0);
  declareIfMissing(node, prefix + "cuda_mppi_authority_max_vx_delta", 0.25);
  declareIfMissing(node, prefix + "cuda_mppi_authority_max_wz_delta", 0.20);

  shadow_enabled_ = node->get_parameter(prefix + "cuda_shadow_enabled").as_bool();
  gpu_authority_enabled_ = node->get_parameter(
    prefix + "cuda_mppi_authority_enabled").as_bool();
  authority_max_gpu_elapsed_ms_ = parameterFloat(
    node, prefix + "cuda_mppi_authority_max_gpu_elapsed_ms");
  authority_max_vx_delta_ = parameterFloat(
    node, prefix + "cuda_mppi_authority_max_vx_delta");
  authority_max_wz_delta_ = parameterFloat(
    node, prefix + "cuda_mppi_authority_max_wz_delta");
  if (gpu_authority_enabled_) {
    shadow_enabled_ = true;
  }
  shadow_config_.batch_size = parameterSize(node, prefix + "cuda_shadow_batch_size");
  // The CUDA shadow may use a larger batch, but every temporal, dynamic, and
  // critic parameter comes from the CPU controller it is checking.
  shadow_config_.time_steps = parameterSize(node, prefix + "time_steps");
  shadow_config_.model_dt = parameterFloat(node, prefix + "model_dt");
  shadow_config_.vx_min = parameterFloat(node, prefix + "vx_min");
  shadow_config_.vx_max = parameterFloat(node, prefix + "vx_max");
  shadow_config_.wz_max = parameterFloat(node, prefix + "wz_max");
  shadow_config_.vx_std = parameterFloat(node, prefix + "vx_std");
  shadow_config_.wz_std = parameterFloat(node, prefix + "wz_std");
  shadow_config_.temperature = parameterFloat(node, prefix + "temperature");
  shadow_config_.gamma = parameterFloat(node, prefix + "gamma");
  auto & critic = shadow_config_.nav2_critics;
  critic.enabled = true;
  critic.consider_footprint = node->get_parameter(
    prefix + "CostCritic.consider_footprint").as_bool();
  critic.constraint_weight = parameterFloat(node, prefix + "ConstraintCritic.cost_weight");
  critic.cost_weight = parameterFloat(node, prefix + "CostCritic.cost_weight");
  critic.cost_critical = parameterFloat(node, prefix + "CostCritic.critical_cost");
  critic.cost_collision = parameterFloat(node, prefix + "CostCritic.collision_cost");
  critic.cost_near_goal_distance = parameterFloat(node, prefix + "CostCritic.near_goal_distance");
  critic.goal_weight = parameterFloat(node, prefix + "GoalCritic.cost_weight");
  critic.goal_threshold = parameterFloat(node, prefix + "GoalCritic.threshold_to_consider");
  critic.goal_angle_weight = parameterFloat(node, prefix + "GoalAngleCritic.cost_weight");
  critic.goal_angle_threshold = parameterFloat(
    node, prefix + "GoalAngleCritic.threshold_to_consider");
  critic.path_align_weight = parameterFloat(node, prefix + "PathAlignCritic.cost_weight");
  critic.path_align_threshold = parameterFloat(
    node, prefix + "PathAlignCritic.threshold_to_consider");
  critic.path_align_max_occupancy_ratio = parameterFloat(
    node, prefix + "PathAlignCritic.max_path_occupancy_ratio");
  critic.path_align_offset = parameterSize(node, prefix + "PathAlignCritic.offset_from_furthest");
  critic.path_align_step = parameterSize(node, prefix + "PathAlignCritic.trajectory_point_step");
  critic.path_follow_weight = parameterFloat(node, prefix + "PathFollowCritic.cost_weight");
  critic.path_follow_threshold = parameterFloat(
    node, prefix + "PathFollowCritic.threshold_to_consider");
  critic.path_follow_offset = parameterSize(node, prefix + "PathFollowCritic.offset_from_furthest");
  critic.path_angle_weight = parameterFloat(node, prefix + "PathAngleCritic.cost_weight");
  critic.path_angle_threshold = parameterFloat(
    node, prefix + "PathAngleCritic.threshold_to_consider");
  critic.path_angle_max_to_furthest = parameterFloat(
    node, prefix + "PathAngleCritic.max_angle_to_furthest");
  critic.path_angle_offset = parameterSize(node, prefix + "PathAngleCritic.offset_from_furthest");
  critic.prefer_forward_weight = parameterFloat(node, prefix + "PreferForwardCritic.cost_weight");
  critic.prefer_forward_threshold = parameterFloat(
    node, prefix + "PreferForwardCritic.threshold_to_consider");

  if (critic.consider_footprint) {
    RCLCPP_WARN(
      logger_, "CUDA MPPI shadow disabled: CostCritic.consider_footprint is not supported");
    shadow_enabled_ = false;
  }

  if (shadow_enabled_ && mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
    backend_ = std::make_unique<mppi_cuda_backend::CudaMppiBackend>();
  } else if (shadow_enabled_) {
    RCLCPP_WARN(logger_, "CUDA MPPI shadow disabled: no CUDA device available");
    shadow_enabled_ = false;
  }
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    name_ + "/cuda_shadow_diagnostics", rclcpp::QoS(10));
  RCLCPP_INFO(
    logger_, "CUDA MPPI: shadow=%s authority=%s batch=%zu CPU-matched-steps=%zu",
    shadow_enabled_ ? "true" : "false", gpu_authority_enabled_ ? "true" : "false",
    shadow_config_.batch_size, shadow_config_.time_steps);
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
    auto * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
    mppi_cuda_backend::OptimizerInput input;
    input.robot_x = static_cast<float>(robot_pose.pose.position.x);
    input.robot_y = static_cast<float>(robot_pose.pose.position.y);
    input.robot_yaw = static_cast<float>(tf2::getYaw(robot_pose.pose.orientation));
    input.measured_vx = static_cast<float>(robot_speed.linear.x);
    input.measured_wz = static_cast<float>(robot_speed.angular.z);
    input.nominal_vx.assign(shadow_config_.time_steps, 0.0F);
    input.nominal_wz.assign(shadow_config_.time_steps, 0.0F);
    input.costmap = {
      costmap->getCharMap(),
      costmap->getSizeInCellsX(),
      costmap->getSizeInCellsY(),
      static_cast<float>(costmap->getResolution()),
      static_cast<float>(costmap->getOriginX()),
      static_cast<float>(costmap->getOriginY()),
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown(),
    };
    populatePathAndValidity(
      transformed_path, *costmap, input.costmap.track_unknown, input);
    input.path_target_x = input.path_x.back();
    input.path_target_y = input.path_y.back();
    input.goal_x = input.path_x.back();
    input.goal_y = input.path_y.back();
    copyCpuControlSequence(optimizer_, robot_pose, shadow_config_.model_dt, input);
    const auto gpu_result = backend_->optimize(shadow_config_, input);
    if (!gpu_authority_enabled_) {
      publishDiagnostic(cpu_command, &gpu_result, "shadow_cpu_authoritative");
      return cpu_command;
    }

    if (gpu_result.all_trajectories_collide) {
      publishDiagnostic(cpu_command, &gpu_result, "cpu_fallback_all_trajectories_collide");
      return cpu_command;
    }
    if (!hasFiniteFirstControl(gpu_result)) {
      publishDiagnostic(cpu_command, &gpu_result, "cpu_fallback_non_finite_gpu_control");
      return cpu_command;
    }
    if (gpu_result.gpu_elapsed_ms > authority_max_gpu_elapsed_ms_) {
      publishDiagnostic(cpu_command, &gpu_result, "cpu_fallback_gpu_runtime_budget");
      return cpu_command;
    }

    const float vx_delta = std::fabs(
      gpu_result.control_vx.front() - static_cast<float>(cpu_command.twist.linear.x));
    const float wz_delta = std::fabs(
      gpu_result.control_wz.front() - static_cast<float>(cpu_command.twist.angular.z));
    if (vx_delta > authority_max_vx_delta_ || wz_delta > authority_max_wz_delta_) {
      publishDiagnostic(cpu_command, &gpu_result, "cpu_fallback_gpu_cpu_disagreement");
      return cpu_command;
    }

    auto gpu_command = cpu_command;
    gpu_command.twist.linear.x = gpu_result.control_vx.front();
    gpu_command.twist.angular.z = gpu_result.control_wz.front();
    publishDiagnostic(cpu_command, &gpu_result, "gpu_authoritative");
    return gpu_command;
  } catch (const std::exception & error) {
    publishDiagnostic(cpu_command, nullptr, "cpu_fallback_" + std::string(error.what()));
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

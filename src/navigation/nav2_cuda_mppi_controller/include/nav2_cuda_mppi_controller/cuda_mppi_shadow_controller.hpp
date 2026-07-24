#ifndef NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_
#define NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

namespace nav2_cuda_mppi_controller
{

// The stock controller remains authoritative. This derived plugin is the
// integration harness that feeds exactly the Nav2 local path and costmap into
// the CUDA backend for performance and safety-parity evidence.
class CudaMppiShadowController : public nav2_mppi_controller::MPPIController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

private:
  void publishDiagnostic(
    const geometry_msgs::msg::TwistStamped & cpu_command,
    const mppi_cuda_backend::OptimizationResult * gpu_result,
    const std::string & message);

  bool shadow_enabled_{true};
  mppi_cuda_backend::SamplingConfig shadow_config_;
  std::unique_ptr<mppi_cuda_backend::CudaMppiBackend> backend_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace nav2_cuda_mppi_controller

#endif  // NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_

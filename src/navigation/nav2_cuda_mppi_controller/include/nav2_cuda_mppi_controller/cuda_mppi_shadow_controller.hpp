#ifndef NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_
#define NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

namespace nav2_cuda_mppi_controller
{

// The stock controller is the default authority. An explicit guarded GPU
// authority mode may return the CUDA command only while its bounded result is
// healthy and remains close to the same-cycle CPU hot fallback.
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
  bool gpu_authority_enabled_{false};
  float authority_max_gpu_elapsed_ms_{20.0F};
  float authority_max_vx_delta_{0.25F};
  float authority_max_wz_delta_{0.20F};
  mppi_cuda_backend::SamplingConfig shadow_config_;
  std::unique_ptr<mppi_cuda_backend::CudaMppiBackend> backend_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace nav2_cuda_mppi_controller

#endif  // NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_

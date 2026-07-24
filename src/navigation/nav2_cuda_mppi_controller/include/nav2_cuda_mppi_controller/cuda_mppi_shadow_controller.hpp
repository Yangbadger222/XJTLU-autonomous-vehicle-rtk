#ifndef NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_
#define NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

namespace nav2_cuda_mppi_controller
{

// The stock controller is the default authority. The explicit CUDA authority
// mode owns the full MPPI optimization cycle and never evaluates CPU MPPI in
// the control loop. CUDA failures are reported as controller failures so Nav2
// sends zero velocity and the route manager can retry through BLOCKED_WAIT.
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

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  geometry_msgs::msg::TwistStamped computeGpuAuthority(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker);

  void resetGpuAuthorityState();

  void applyGpuControlPostprocessing(
    std::vector<float> & control_vx, std::vector<float> & control_wz);

  void publishAuthorityDiagnostic(
    const geometry_msgs::msg::TwistStamped * gpu_command,
    const mppi_cuda_backend::OptimizationResult * gpu_result,
    const std::string & message);

  void publishDiagnostic(
    const geometry_msgs::msg::TwistStamped & cpu_command,
    const mppi_cuda_backend::OptimizationResult * gpu_result,
    const std::string & message);

  bool shadow_enabled_{true};
  bool gpu_authority_enabled_{false};
  float authority_max_gpu_elapsed_ms_{20.0F};
  bool gpu_regenerate_noises_{true};
  bool gpu_shift_control_sequence_{true};
  std::size_t gpu_retry_attempt_limit_{3U};
  float gpu_base_vx_min_{0.0F};
  float gpu_base_vx_max_{0.0F};
  float gpu_base_wz_max_{0.0F};
  std::uint64_t gpu_seed_{0x4d50504943554441ULL};
  std::vector<float> gpu_nominal_vx_;
  std::vector<float> gpu_nominal_wz_;
  std::array<float, 4U> gpu_vx_history_{};
  std::array<float, 4U> gpu_wz_history_{};
  mppi_cuda_backend::SamplingConfig shadow_config_;
  std::unique_ptr<mppi_cuda_backend::CudaMppiBackend> backend_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace nav2_cuda_mppi_controller

#endif  // NAV2_CUDA_MPPI_CONTROLLER__CUDA_MPPI_SHADOW_CONTROLLER_HPP_

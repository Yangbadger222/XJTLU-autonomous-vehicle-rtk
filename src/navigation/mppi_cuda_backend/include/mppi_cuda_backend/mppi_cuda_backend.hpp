#ifndef MPPI_CUDA_BACKEND__MPPI_CUDA_BACKEND_HPP_
#define MPPI_CUDA_BACKEND__MPPI_CUDA_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace mppi_cuda_backend
{

// Mirrors the active DiffDrive critic set in nav2_corridor_rtk.yaml. Full
// footprint collision is deliberately rejected until the GPU has equivalent
// polygon collision support; the current vehicle profile uses centre-point
// collision checks.
struct Nav2CriticConfig
{
  bool enabled{false};
  bool consider_footprint{false};
  float constraint_weight{4.0F};
  float cost_weight{7.0F};
  float cost_critical{300.0F};
  float cost_collision{1000000.0F};
  float cost_near_goal_distance{1.0F};
  float goal_weight{5.0F};
  float goal_threshold{0.6F};
  float goal_angle_weight{3.0F};
  float goal_angle_threshold{0.5F};
  float path_align_weight{12.0F};
  float path_align_threshold{0.5F};
  float path_align_max_occupancy_ratio{0.05F};
  std::size_t path_align_offset{6U};
  std::size_t path_align_step{4U};
  float path_follow_weight{16.0F};
  float path_follow_threshold{1.4F};
  std::size_t path_follow_offset{5U};
  float path_angle_weight{4.0F};
  float path_angle_threshold{0.5F};
  float path_angle_max_to_furthest{1.0F};
  std::size_t path_angle_offset{4U};
  float prefer_forward_weight{5.0F};
  float prefer_forward_threshold{0.5F};
};

struct SamplingConfig
{
  std::size_t batch_size{2048};
  std::size_t time_steps{32};
  float model_dt{0.05F};
  float vx_min{0.0F};
  float vx_max{1.5F};
  float wz_max{0.7F};
  float vx_std{0.28F};
  float wz_std{0.22F};
  float temperature{0.45F};
  float gamma{0.015F};
  float path_weight{16.0F};
  float goal_weight{5.0F};
  float obstacle_weight{3.81F / 254.0F};
  float collision_cost{1000000.0F};
  std::uint64_t seed{0x4d50504943554441ULL};
  Nav2CriticConfig nav2_critics;
};

struct CostmapView
{
  const unsigned char * data{nullptr};
  std::size_t size_x{0};
  std::size_t size_y{0};
  float resolution{0.05F};
  float origin_x{0.0F};
  float origin_y{0.0F};
  // Same meaning as LayeredCostmap::isTrackingUnknown().
  bool track_unknown{false};
};

struct OptimizerInput
{
  float robot_x{0.0F};
  float robot_y{0.0F};
  float robot_yaw{0.0F};
  // Nav2's DiffDrive MotionModel uses measured odometry velocity at rollout
  // index 0, then propagates sampled controls from index 1 onward.
  float measured_vx{0.0F};
  float measured_wz{0.0F};
  float path_target_x{0.0F};
  float path_target_y{0.0F};
  float goal_x{0.0F};
  float goal_y{0.0F};
  // Local MPPI path in the costmap frame. These are required when
  // SamplingConfig::nav2_critics.enabled is true.
  std::vector<float> path_x;
  std::vector<float> path_y;
  std::vector<float> path_yaw;
  // Mirrors Nav2's path_pts_valid: one entry for every path segment except
  // the terminal goal point.
  std::vector<unsigned char> path_valid;
  std::vector<float> path_integrated_distance;
  std::vector<float> nominal_vx;
  std::vector<float> nominal_wz;
  CostmapView costmap;
};

struct OptimizationResult
{
  std::vector<float> control_vx;
  std::vector<float> control_wz;
  float min_cost{0.0F};
  bool all_trajectories_collide{true};
  float gpu_elapsed_ms{0.0F};
};

// The full Nav2 critic mode matches the currently active centre-point
// DiffDrive vehicle profile. Footprint collision remains an explicit
// unsupported mode rather than a silent approximation.
class CudaMppiBackend
{
public:
  CudaMppiBackend();
  ~CudaMppiBackend();
  CudaMppiBackend(const CudaMppiBackend &) = delete;
  CudaMppiBackend & operator=(const CudaMppiBackend &) = delete;

  static bool isAvailable();
  OptimizationResult optimize(const SamplingConfig & config, const OptimizerInput & input);
  void reset();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mppi_cuda_backend

#endif  // MPPI_CUDA_BACKEND__MPPI_CUDA_BACKEND_HPP_

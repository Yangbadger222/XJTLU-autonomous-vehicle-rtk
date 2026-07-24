#ifndef MPPI_CUDA_BACKEND__MPPI_CUDA_BACKEND_HPP_
#define MPPI_CUDA_BACKEND__MPPI_CUDA_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace mppi_cuda_backend
{

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
};

struct CostmapView
{
  const unsigned char * data{nullptr};
  std::size_t size_x{0};
  std::size_t size_y{0};
  float resolution{0.05F};
  float origin_x{0.0F};
  float origin_y{0.0F};
  bool track_unknown{false};
};

struct OptimizerInput
{
  float robot_x{0.0F};
  float robot_y{0.0F};
  float robot_yaw{0.0F};
  float measured_vx{0.0F};
  float measured_wz{0.0F};
  float path_target_x{0.0F};
  float path_target_y{0.0F};
  float goal_x{0.0F};
  float goal_y{0.0F};
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

// This backend deliberately uses the costmap centre point only. It is a
// shadow/benchmark component until the Nav2 plugin adds an equivalent GPU
// footprint check and full critic parity validation.
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

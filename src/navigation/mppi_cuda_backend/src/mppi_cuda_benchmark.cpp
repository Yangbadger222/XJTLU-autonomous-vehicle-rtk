#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

namespace
{

std::vector<unsigned char> makeCostmap(std::size_t width, std::size_t height)
{
  std::vector<unsigned char> costmap(width * height, 0U);
  for (std::size_t y = height / 3U; y < height / 3U + 12U; ++y) {
    for (std::size_t x = width / 2U - 18U; x < width / 2U + 18U; ++x) {
      costmap[y * width + x] = 254U;
    }
  }
  return costmap;
}

float percentile(std::vector<float> values, float quantile)
{
  const std::size_t index = static_cast<std::size_t>(quantile * (values.size() - 1U));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

void runCase(std::size_t batch_size, std::size_t time_steps)
{
  constexpr std::size_t kWidth = 240U;
  constexpr std::size_t kHeight = 240U;
  auto costmap = makeCostmap(kWidth, kHeight);
  mppi_cuda_backend::SamplingConfig config;
  config.batch_size = batch_size;
  config.time_steps = time_steps;
  mppi_cuda_backend::OptimizerInput input;
  input.robot_x = 0.0F;
  input.robot_y = -3.0F;
  input.path_target_x = 0.0F;
  input.path_target_y = 2.0F;
  input.goal_x = 0.0F;
  input.goal_y = 4.0F;
  input.nominal_vx.assign(time_steps, 1.0F);
  input.nominal_wz.assign(time_steps, 0.0F);
  input.costmap = {costmap.data(), kWidth, kHeight, 0.05F, -6.0F, -6.0F, false};

  mppi_cuda_backend::CudaMppiBackend backend;
  for (int iteration = 0; iteration < 10; ++iteration) {
    backend.optimize(config, input);
  }
  std::vector<float> timings;
  timings.reserve(100U);
  mppi_cuda_backend::OptimizationResult result;
  for (int iteration = 0; iteration < 100; ++iteration) {
    result = backend.optimize(config, input);
    timings.push_back(result.gpu_elapsed_ms);
  }
  const float mean = std::accumulate(timings.begin(), timings.end(), 0.0F) /
    static_cast<float>(timings.size());
  std::cout << "batch=" << batch_size << " steps=" << time_steps
            << " mean_ms=" << std::fixed << std::setprecision(3) << mean
            << " p95_ms=" << percentile(timings, 0.95F)
            << " cmd_vx=" << result.control_vx.front()
            << " cmd_wz=" << result.control_wz.front()
            << " all_collide=" << std::boolalpha << result.all_trajectories_collide
            << std::endl;
}

}  // namespace

int main()
{
  if (!mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
    std::cerr << "No CUDA device available" << std::endl;
    return EXIT_FAILURE;
  }
  runCase(1000U, 32U);
  runCase(2048U, 32U);
  runCase(4096U, 32U);
  runCase(4096U, 48U);
  return EXIT_SUCCESS;
}

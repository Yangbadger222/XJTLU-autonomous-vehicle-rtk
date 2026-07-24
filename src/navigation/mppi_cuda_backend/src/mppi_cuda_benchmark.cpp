#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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

std::uint64_t splitmix64(std::uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

float uniform01(std::uint64_t value)
{
  return (static_cast<float>((value >> 40) & 0x00ffffffULL) + 1.0F) /
    16777217.0F;
}

float normalSample(std::uint64_t seed)
{
  constexpr float kTwoPi = 6.28318530717958647692F;
  const float u1 = std::max(uniform01(splitmix64(seed)), 1e-7F);
  const float u2 = uniform01(splitmix64(seed ^ 0xa0761d6478bd642fULL));
  return std::sqrt(-2.0F * std::log(u1)) * std::cos(kTwoPi * u2);
}

// This mirrors the first CUDA kernel and reduction exactly. It intentionally
// excludes the host-only Nav2 footprint and path critics: it measures the
// portion moved to CUDA rather than claiming a whole-controller speedup.
void cpuReference(
  const mppi_cuda_backend::SamplingConfig & config,
  const mppi_cuda_backend::OptimizerInput & input)
{
  std::vector<float> costs(config.batch_size, 0.0F);
  std::vector<float> candidate_vx(config.batch_size * config.time_steps, 0.0F);
  std::vector<float> candidate_wz(config.batch_size * config.time_steps, 0.0F);
  constexpr unsigned char kInscribedObstacle = 253U;
  for (std::size_t candidate = 0; candidate < config.batch_size; ++candidate) {
    float x = input.robot_x;
    float y = input.robot_y;
    float yaw = input.robot_yaw;
    float obstacle_cost = 0.0F;
    float regularization_cost = 0.0F;
    bool collision = false;
    const std::size_t offset = candidate * config.time_steps;
    for (std::size_t step = 0; step < config.time_steps; ++step) {
      const auto sample_id = config.seed ^ (static_cast<std::uint64_t>(candidate) << 32) ^ step;
      const float vx_noise = normalSample(sample_id) * config.vx_std;
      const float wz_noise = normalSample(sample_id ^ 0xe7037ed1a0b428dbULL) * config.wz_std;
      const float vx = std::clamp(input.nominal_vx[step] + vx_noise, config.vx_min, config.vx_max);
      const float wz = std::clamp(input.nominal_wz[step] + wz_noise, -config.wz_max, config.wz_max);
      candidate_vx[offset + step] = vx;
      candidate_wz[offset + step] = wz;
      yaw += wz * config.model_dt;
      x += vx * std::cos(yaw) * config.model_dt;
      y += vx * std::sin(yaw) * config.model_dt;
      regularization_cost += config.gamma * (
        input.nominal_vx[step] * vx_noise / (config.vx_std * config.vx_std) +
        input.nominal_wz[step] * wz_noise / (config.wz_std * config.wz_std));
      const int map_x = static_cast<int>(std::floor((x - input.costmap.origin_x) / input.costmap.resolution));
      const int map_y = static_cast<int>(std::floor((y - input.costmap.origin_y) / input.costmap.resolution));
      if (map_x < 0 || map_y < 0 || map_x >= static_cast<int>(input.costmap.size_x) ||
        map_y >= static_cast<int>(input.costmap.size_y) ||
        input.costmap.data[map_y * input.costmap.size_x + map_x] >= kInscribedObstacle)
      {
        collision = true;
        break;
      }
      obstacle_cost += static_cast<float>(input.costmap.data[map_y * input.costmap.size_x + map_x]) / 254.0F;
    }
    costs[candidate] = collision ? config.collision_cost : regularization_cost +
      config.path_weight * std::hypot(x - input.path_target_x, y - input.path_target_y) +
      config.goal_weight * std::hypot(x - input.goal_x, y - input.goal_y) +
      config.obstacle_weight * obstacle_cost / static_cast<float>(config.time_steps);
  }
  const float minimum = *std::min_element(costs.begin(), costs.end());
  std::vector<float> weights(config.batch_size, 0.0F);
  float normalizer = 0.0F;
  for (std::size_t candidate = 0; candidate < config.batch_size; ++candidate) {
    weights[candidate] = std::exp(-(costs[candidate] - minimum) / config.temperature);
    normalizer += weights[candidate];
  }
  std::vector<float> output_vx(config.time_steps, 0.0F);
  std::vector<float> output_wz(config.time_steps, 0.0F);
  for (std::size_t candidate = 0; candidate < config.batch_size; ++candidate) {
    const std::size_t offset = candidate * config.time_steps;
    for (std::size_t step = 0; step < config.time_steps; ++step) {
      output_vx[step] += weights[candidate] * candidate_vx[offset + step];
      output_wz[step] += weights[candidate] * candidate_wz[offset + step];
    }
  }
  for (std::size_t step = 0; step < config.time_steps; ++step) {
    output_vx[step] /= normalizer;
    output_wz[step] /= normalizer;
  }
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
  std::vector<float> cpu_timings;
  timings.reserve(100U);
  cpu_timings.reserve(100U);
  mppi_cuda_backend::OptimizationResult result;
  for (int iteration = 0; iteration < 100; ++iteration) {
    result = backend.optimize(config, input);
    timings.push_back(result.gpu_elapsed_ms);
    const auto start = std::chrono::steady_clock::now();
    cpuReference(config, input);
    const auto finish = std::chrono::steady_clock::now();
    cpu_timings.push_back(
      std::chrono::duration<float, std::milli>(finish - start).count());
  }
  const float mean = std::accumulate(timings.begin(), timings.end(), 0.0F) /
    static_cast<float>(timings.size());
  const float cpu_mean = std::accumulate(cpu_timings.begin(), cpu_timings.end(), 0.0F) /
    static_cast<float>(cpu_timings.size());
  std::cout << "batch=" << batch_size << " steps=" << time_steps
            << " mean_ms=" << std::fixed << std::setprecision(3) << mean
            << " p95_ms=" << percentile(timings, 0.95F)
            << " cpu_mean_ms=" << cpu_mean
            << " cpu_p95_ms=" << percentile(cpu_timings, 0.95F)
            << " speedup=" << cpu_mean / mean
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

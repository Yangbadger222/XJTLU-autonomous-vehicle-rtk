#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
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

struct Options
{
  bool gpu_only{false};
  bool full_critics{false};
  std::size_t batch_size{0U};
  std::size_t time_steps{0U};
  std::size_t warmup_iterations{10U};
  std::size_t samples{100U};
};

void printUsage(const char * program)
{
  std::cout << "Usage: " << program << " [options]\n"
            << "Without --batch-size/--time-steps, runs the legacy CPU/GPU sweep.\n"
            << "  --gpu-only             Skip the simplified CPU reference.\n"
            << "  --full-critics         Exercise the CUDA Nav2-style critic path.\n"
            << "  --batch-size <count>   Run one selected batch size.\n"
            << "  --time-steps <count>   Run one selected horizon.\n"
            << "  --samples <count>      Timed optimizer calls (default: 100).\n"
            << "  --warmup <count>       Untimed optimizer calls (default: 10).\n";
}

std::size_t parseSize(const std::string & value, const char * flag)
{
  try {
    const auto parsed = std::stoull(value);
    if (parsed == 0U) {
      throw std::invalid_argument("must be positive");
    }
    return static_cast<std::size_t>(parsed);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string(flag) + " requires a positive integer");
  }
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string flag = argv[index];
    const auto next = [&]() {
        if (++index >= argc) {
          throw std::invalid_argument(flag + " requires a value");
        }
        return std::string(argv[index]);
      };
    if (flag == "--gpu-only") {
      options.gpu_only = true;
    } else if (flag == "--full-critics") {
      options.full_critics = true;
      options.gpu_only = true;
    } else if (flag == "--batch-size") {
      options.batch_size = parseSize(next(), "--batch-size");
    } else if (flag == "--time-steps") {
      options.time_steps = parseSize(next(), "--time-steps");
    } else if (flag == "--samples") {
      options.samples = parseSize(next(), "--samples");
    } else if (flag == "--warmup") {
      options.warmup_iterations = parseSize(next(), "--warmup");
    } else if (flag == "--help" || flag == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown option: " + flag);
    }
  }
  if ((options.batch_size == 0U) != (options.time_steps == 0U)) {
    throw std::invalid_argument("--batch-size and --time-steps must be provided together");
  }
  return options;
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
    float propagated_vx = input.measured_vx;
    float propagated_wz = input.measured_wz;
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
      yaw += propagated_wz * config.model_dt;
      x += propagated_vx * std::cos(yaw) * config.model_dt;
      y += propagated_vx * std::sin(yaw) * config.model_dt;
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
      propagated_vx = vx;
      propagated_wz = wz;
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

void populateFullCriticPath(mppi_cuda_backend::SamplingConfig & config,
  mppi_cuda_backend::OptimizerInput & input)
{
  constexpr std::size_t kPathSize = 80U;
  constexpr float kPathSpacing = 0.1F;
  config.nav2_critics.enabled = true;
  input.path_x.reserve(kPathSize);
  input.path_y.reserve(kPathSize);
  input.path_yaw.reserve(kPathSize);
  input.path_integrated_distance.reserve(kPathSize);
  for (std::size_t index = 0U; index < kPathSize; ++index) {
    input.path_x.push_back(static_cast<float>(index) * kPathSpacing);
    input.path_y.push_back(-3.0F);
    input.path_yaw.push_back(0.0F);
    input.path_integrated_distance.push_back(static_cast<float>(index) * kPathSpacing);
  }
  input.path_valid.assign(kPathSize - 1U, 1U);
  input.path_target_x = input.path_x.back();
  input.path_target_y = input.path_y.back();
  input.goal_x = input.path_x.back();
  input.goal_y = input.path_y.back();
}

void runCase(std::size_t batch_size, std::size_t time_steps, const Options & options)
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
  if (options.full_critics) {
    populateFullCriticPath(config, input);
  }

  mppi_cuda_backend::CudaMppiBackend backend;
  for (std::size_t iteration = 0U; iteration < options.warmup_iterations; ++iteration) {
    backend.optimize(config, input);
  }
  std::vector<float> gpu_timings;
  std::vector<float> wall_timings;
  std::vector<float> cpu_timings;
  gpu_timings.reserve(options.samples);
  wall_timings.reserve(options.samples);
  cpu_timings.reserve(options.samples);
  mppi_cuda_backend::OptimizationResult result;
  for (std::size_t iteration = 0U; iteration < options.samples; ++iteration) {
    const auto wall_start = std::chrono::steady_clock::now();
    result = backend.optimize(config, input);
    const auto wall_finish = std::chrono::steady_clock::now();
    gpu_timings.push_back(result.gpu_elapsed_ms);
    wall_timings.push_back(
      std::chrono::duration<float, std::milli>(wall_finish - wall_start).count());
    if (!options.gpu_only) {
      const auto cpu_start = std::chrono::steady_clock::now();
      cpuReference(config, input);
      const auto cpu_finish = std::chrono::steady_clock::now();
      cpu_timings.push_back(
        std::chrono::duration<float, std::milli>(cpu_finish - cpu_start).count());
    }
  }
  const auto mean = [](const std::vector<float> & values) {
      return std::accumulate(values.begin(), values.end(), 0.0F) /
             static_cast<float>(values.size());
    };
  const float gpu_mean = mean(gpu_timings);
  const float wall_mean = mean(wall_timings);
  const float wall_p95 = percentile(wall_timings, 0.95F);
  const float wall_p99 = percentile(wall_timings, 0.99F);
  std::cout << "batch=" << batch_size << " steps=" << time_steps
            << " samples=" << options.samples
            << " full_critics=" << std::boolalpha << options.full_critics
            << " gpu_mean_ms=" << std::fixed << std::setprecision(3) << gpu_mean
            << " gpu_p50_ms=" << percentile(gpu_timings, 0.50F)
            << " gpu_p95_ms=" << percentile(gpu_timings, 0.95F)
            << " gpu_p99_ms=" << percentile(gpu_timings, 0.99F)
            << " wall_mean_ms=" << wall_mean
            << " wall_p95_ms=" << wall_p95
            << " wall_p99_ms=" << wall_p99
            << " backend_p95_hz=" << 1000.0F / wall_p95
            << " backend_p99_hz=" << 1000.0F / wall_p99
            << " cmd_vx=" << result.control_vx.front()
            << " cmd_wz=" << result.control_wz.front()
            << " all_collide=" << result.all_trajectories_collide;
  if (!options.gpu_only) {
    const float cpu_mean = mean(cpu_timings);
    std::cout << " cpu_mean_ms=" << cpu_mean
              << " cpu_p95_ms=" << percentile(cpu_timings, 0.95F)
              << " speedup=" << cpu_mean / gpu_mean;
  }
  std::cout << std::endl;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (!mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
    std::cerr << "No CUDA device available" << std::endl;
    return EXIT_FAILURE;
  }
  try {
    const Options options = parseOptions(argc, argv);
    if (options.batch_size != 0U) {
      runCase(options.batch_size, options.time_steps, options);
      return EXIT_SUCCESS;
    }

    runCase(1000U, 32U, options);
    runCase(2048U, 32U, options);
    runCase(4096U, 32U, options);
    runCase(4096U, 48U, options);
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "mppi_cuda_benchmark failed: " << error.what() << std::endl;
    return EXIT_FAILURE;
  }
}

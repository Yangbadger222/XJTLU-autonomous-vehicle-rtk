#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <limits>
#include <stdexcept>
#include <string>

namespace mppi_cuda_backend
{
namespace
{

constexpr int kThreadsPerBlock = 256;
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr unsigned char kInscribedObstacle = 253U;
constexpr unsigned char kUnknown = 255U;

void checkCuda(cudaError_t status, const char * expression)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(expression) + ": " + cudaGetErrorString(status));
  }
}

#define MPPI_CUDA_CHECK(expression) checkCuda((expression), #expression)

template<typename T>
void release(T *& pointer)
{
  if (pointer != nullptr) {
    cudaFree(pointer);
    pointer = nullptr;
  }
}

__device__ std::uint64_t splitmix64(std::uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

__device__ float uniform01(std::uint64_t value)
{
  constexpr float kScale = 1.0F / 16777217.0F;
  return (static_cast<float>((value >> 40) & 0x00ffffffULL) + 1.0F) * kScale;
}

__device__ float normalSample(std::uint64_t seed)
{
  const float u1 = fmaxf(uniform01(splitmix64(seed)), 1e-7F);
  const float u2 = uniform01(splitmix64(seed ^ 0xa0761d6478bd642fULL));
  return sqrtf(-2.0F * logf(u1)) * cosf(kTwoPi * u2);
}

__device__ float atomicMinFloat(float * address, float value)
{
  auto * address_as_int = reinterpret_cast<int *>(address);
  int old_value = *address_as_int;
  int assumed;
  do {
    assumed = old_value;
    const float current = __int_as_float(assumed);
    old_value = atomicCAS(address_as_int, assumed, __float_as_int(fminf(value, current)));
  } while (assumed != old_value);
  return __int_as_float(old_value);
}

struct DeviceProblem
{
  unsigned int batch_size;
  unsigned int time_steps;
  float model_dt;
  float vx_min;
  float vx_max;
  float wz_max;
  float vx_std;
  float wz_std;
  float gamma;
  float path_weight;
  float goal_weight;
  float obstacle_weight;
  float collision_cost;
  std::uint64_t seed;
  float robot_x;
  float robot_y;
  float robot_yaw;
  float measured_vx;
  float measured_wz;
  float path_target_x;
  float path_target_y;
  float goal_x;
  float goal_y;
  unsigned int map_size_x;
  unsigned int map_size_y;
  float map_resolution;
  float map_origin_x;
  float map_origin_y;
  bool track_unknown;
  Nav2CriticConfig nav2_critics;
  unsigned int path_size;
};

__device__ float normalizeAngle(float angle)
{
  float normalized = fmodf(angle + CUDART_PI_F, 2.0F * CUDART_PI_F);
  if (normalized <= 0.0F) {
    normalized += CUDART_PI_F;
  } else {
    normalized -= CUDART_PI_F;
  }
  return normalized;
}

__device__ float shortestAngularDistance(float from, float to)
{
  return normalizeAngle(to - from);
}

__device__ unsigned int closestPathIndex(
  const float * path_x, const float * path_y, unsigned int path_size, float x, float y)
{
  unsigned int closest = 0U;
  float closest_distance = FLT_MAX;
  for (unsigned int index = 0U; index < path_size; ++index) {
    const float dx = path_x[index] - x;
    const float dy = path_y[index] - y;
    const float distance = dx * dx + dy * dy;
    if (distance < closest_distance) {
      closest_distance = distance;
      closest = index;
    }
  }
  return closest;
}

__device__ unsigned int closestIntegratedPathIndex(
  const float * path_distance, unsigned int upper_bound, float distance, unsigned int start)
{
  if (upper_bound == 0U || start >= upper_bound) {
    return 0U;
  }
  unsigned int candidate = start;
  while (candidate + 1U < upper_bound && path_distance[candidate + 1U] <= distance) {
    ++candidate;
  }
  if (candidate + 1U >= upper_bound) {
    return candidate;
  }
  const float left = fabsf(distance - path_distance[candidate]);
  const float right = fabsf(path_distance[candidate + 1U] - distance);
  return left < right ? candidate : candidate + 1U;
}

__global__ void rolloutAndScore(
  DeviceProblem problem, const unsigned char * costmap,
  const float * nominal_vx, const float * nominal_wz,
  const float * path_x, const float * path_y,
  float * candidate_vx, float * candidate_wz,
  float * trajectory_x, float * trajectory_y, float * trajectory_yaw,
  float * costs, unsigned int * valid_count, unsigned int * furthest_path_point)
{
  const unsigned int candidate = blockIdx.x * blockDim.x + threadIdx.x;
  if (candidate >= problem.batch_size) {
    return;
  }

  float x = problem.robot_x;
  float y = problem.robot_y;
  float yaw = problem.robot_yaw;
  float propagated_vx = problem.measured_vx;
  float propagated_wz = problem.measured_wz;
  float obstacle_cost = 0.0F;
  float regularization_cost = 0.0F;
  bool collision = false;
  const bool full_nav2_critics = problem.nav2_critics.enabled;
  bool near_cost_goal = false;
  if (full_nav2_critics) {
    near_cost_goal = hypotf(
      problem.robot_x - path_x[problem.path_size - 1U],
      problem.robot_y - path_y[problem.path_size - 1U]) <
      problem.nav2_critics.cost_near_goal_distance;
  }
  const std::size_t offset = static_cast<std::size_t>(candidate) * problem.time_steps;

  for (unsigned int step = 0; step < problem.time_steps; ++step) {
    const std::uint64_t sample_id =
      problem.seed ^ (static_cast<std::uint64_t>(candidate) << 32) ^ step;
    // Match Nav2 Humble's MotionModel::predict(): state velocity at index 0
    // is measured odometry, and sampled candidate controls are shifted by one
    // rollout step. Position integrates with the previous yaw while yaw itself
    // advances from the current state angular velocity.
    x += propagated_vx * cosf(yaw) * problem.model_dt;
    y += propagated_vx * sinf(yaw) * problem.model_dt;
    yaw += propagated_wz * problem.model_dt;
    if (full_nav2_critics) {
      trajectory_x[offset + step] = x;
      trajectory_y[offset + step] = y;
      trajectory_yaw[offset + step] = yaw;
    }
    const float vx_noise = normalSample(sample_id) * problem.vx_std;
    const float wz_noise = normalSample(sample_id ^ 0xe7037ed1a0b428dbULL) * problem.wz_std;
    const float vx = fminf(problem.vx_max, fmaxf(problem.vx_min, nominal_vx[step] + vx_noise));
    const float wz = fminf(problem.wz_max, fmaxf(-problem.wz_max, nominal_wz[step] + wz_noise));
    candidate_vx[offset + step] = vx;
    candidate_wz[offset + step] = wz;

    // Nav2 regularizes the bounded candidate control, not the raw Gaussian
    // sample. Clipping at velocity limits therefore changes this term.
    const float bounded_vx_noise = vx - nominal_vx[step];
    const float bounded_wz_noise = wz - nominal_wz[step];
    regularization_cost += problem.gamma * (
      nominal_vx[step] * bounded_vx_noise / (problem.vx_std * problem.vx_std) +
      nominal_wz[step] * bounded_wz_noise / (problem.wz_std * problem.wz_std));

    const int map_x = static_cast<int>(
      floorf((x - problem.map_origin_x) / problem.map_resolution));
    const int map_y = static_cast<int>(
      floorf((y - problem.map_origin_y) / problem.map_resolution));
    const bool outside = map_x < 0 || map_y < 0 ||
      map_x >= static_cast<int>(problem.map_size_x) ||
      map_y >= static_cast<int>(problem.map_size_y);
    const unsigned char map_cost = outside ? kUnknown :
      costmap[map_y * problem.map_size_x + map_x];
    if (full_nav2_critics) {
      const bool pose_collision = map_cost == 254U || map_cost == kInscribedObstacle ||
        (map_cost == kUnknown && !problem.track_unknown);
      if (!collision && pose_collision) {
        collision = true;
      } else if (!collision) {
        if (map_cost >= kInscribedObstacle) {
          obstacle_cost += problem.nav2_critics.cost_critical;
        } else if (!near_cost_goal) {
          obstacle_cost += static_cast<float>(map_cost);
        }
      }
    } else {
      if (outside || map_cost == 254U || map_cost == kInscribedObstacle ||
        (map_cost == kUnknown && !problem.track_unknown))
      {
        collision = true;
        break;
      }
      obstacle_cost += static_cast<float>(map_cost) / 254.0F;
    }
    propagated_vx = vx;
    propagated_wz = wz;
  }

  if (!collision) {
    atomicAdd(valid_count, 1U);
  }
  if (full_nav2_critics) {
    const unsigned int closest = closestPathIndex(
      path_x, path_y, problem.path_size, x, y);
    atomicMax(furthest_path_point, closest);
    costs[candidate] = regularization_cost + (collision ?
      problem.nav2_critics.cost_collision :
      problem.nav2_critics.cost_weight * obstacle_cost /
      (254.0F * static_cast<float>(problem.time_steps)));
  } else {
    const float path_distance = hypotf(x - problem.path_target_x, y - problem.path_target_y);
    const float goal_distance = hypotf(x - problem.goal_x, y - problem.goal_y);
    costs[candidate] = collision ? problem.collision_cost :
      regularization_cost +
      problem.path_weight * path_distance +
      problem.goal_weight * goal_distance +
      problem.obstacle_weight * obstacle_cost / static_cast<float>(problem.time_steps);
  }
}

__global__ void scoreNav2Critics(
  DeviceProblem problem, const float * path_x, const float * path_y, const float * path_yaw,
  const unsigned char * path_valid, const float * path_distance,
  const float * candidate_vx, const float * trajectory_x, const float * trajectory_y,
  const float * trajectory_yaw, const unsigned int * furthest_path_point, float * costs)
{
  const unsigned int candidate = blockIdx.x * blockDim.x + threadIdx.x;
  if (candidate >= problem.batch_size || !problem.nav2_critics.enabled) {
    return;
  }
  const std::size_t offset = static_cast<std::size_t>(candidate) * problem.time_steps;
  const Nav2CriticConfig & critic = problem.nav2_critics;
  const unsigned int path_last = problem.path_size - 1U;
  const unsigned int furthest = *furthest_path_point < path_last ?
    *furthest_path_point : path_last;
  const float robot_goal_distance = hypotf(
    problem.robot_x - path_x[path_last], problem.robot_y - path_y[path_last]);
  float additional_cost = 0.0F;

  // ConstraintCritic: current DiffDrive controls are clipped, but the measured
  // first state can still be beyond the configured velocity limits.
  float constraint_sum = 0.0F;
  for (unsigned int step = 0U; step < problem.time_steps; ++step) {
    const float vx = step == 0U ? problem.measured_vx : candidate_vx[offset + step - 1U];
    constraint_sum += fmaxf(vx - problem.vx_max, 0.0F) +
      fmaxf(problem.vx_min - vx, 0.0F);
  }
  additional_cost += constraint_sum * problem.model_dt * critic.constraint_weight;

  // GoalCritic and GoalAngleCritic only replace path tracking near the local goal.
  if (robot_goal_distance < critic.goal_threshold)
  {
    float distance_sum = 0.0F;
    for (unsigned int step = 0U; step < problem.time_steps; ++step) {
      distance_sum += hypotf(
        trajectory_x[offset + step] - path_x[path_last],
        trajectory_y[offset + step] - path_y[path_last]);
    }
    additional_cost += distance_sum / static_cast<float>(problem.time_steps) * critic.goal_weight;
  }
  if (robot_goal_distance < critic.goal_angle_threshold)
  {
    float angle_sum = 0.0F;
    for (unsigned int step = 0U; step < problem.time_steps; ++step) {
      angle_sum += fabsf(shortestAngularDistance(
        trajectory_yaw[offset + step], path_yaw[path_last]));
    }
    additional_cost += angle_sum / static_cast<float>(problem.time_steps) *
      critic.goal_angle_weight;
  }

  if (robot_goal_distance >= critic.prefer_forward_threshold) {
    // PreferForwardCritic.
    float backward_sum = 0.0F;
    for (unsigned int step = 0U; step < problem.time_steps; ++step) {
      const float vx = step == 0U ? problem.measured_vx : candidate_vx[offset + step - 1U];
      backward_sum += fmaxf(-vx, 0.0F);
    }
    additional_cost += backward_sum * problem.model_dt * critic.prefer_forward_weight;

  }

  // PathFollowCritic.
  if (robot_goal_distance >= critic.path_follow_threshold) {
    unsigned int follow_index = min(
      furthest + static_cast<unsigned int>(critic.path_follow_offset), path_last);
    bool valid = false;
    while (!valid && follow_index < path_last - 1U) {
      valid = path_valid[follow_index] != 0U;
      if (!valid) {
        ++follow_index;
      }
    }
    additional_cost += hypotf(
      trajectory_x[offset + problem.time_steps - 1U] - path_x[follow_index],
      trajectory_y[offset + problem.time_steps - 1U] - path_y[follow_index]) *
      critic.path_follow_weight;

  }

  // PathAngleCritic.
  if (robot_goal_distance >= critic.path_angle_threshold) {
    const unsigned int angle_index = min(
      furthest + static_cast<unsigned int>(critic.path_angle_offset), path_last);
    const float desired_bearing = atan2f(
      path_y[angle_index] - problem.robot_y, path_x[angle_index] - problem.robot_x);
    if (fabsf(shortestAngularDistance(problem.robot_yaw, desired_bearing)) >=
      critic.path_angle_max_to_furthest)
    {
      float yaw_sum = 0.0F;
      for (unsigned int step = 0U; step < problem.time_steps; ++step) {
        const float bearing = atan2f(
          path_y[angle_index] - trajectory_y[offset + step],
          path_x[angle_index] - trajectory_x[offset + step]);
        yaw_sum += fabsf(shortestAngularDistance(trajectory_yaw[offset + step], bearing));
      }
      additional_cost += yaw_sum / static_cast<float>(problem.time_steps) *
        critic.path_angle_weight;
    }

  }

  // PathAlignCritic.
  if (robot_goal_distance >= critic.path_align_threshold) {
    if (furthest >= critic.path_align_offset) {
      const unsigned int initial = closestPathIndex(
        path_x, path_y, problem.path_size, trajectory_x[offset], trajectory_y[offset]);
      const float range = static_cast<float>(furthest) - static_cast<float>(initial);
      unsigned int invalid_count = 0U;
      bool path_blocked = false;
      if (range > 0.0F) {
        for (unsigned int index = initial; index < furthest; ++index) {
          invalid_count += path_valid[index] ? 0U : 1U;
          if (static_cast<float>(invalid_count) / range > critic.path_align_max_occupancy_ratio &&
            invalid_count > 2U)
          {
            path_blocked = true;
            break;
          }
        }
      }
      if (!path_blocked) {
        float trajectory_distance = 0.0F;
        float aligned_distance = 0.0F;
        unsigned int samples = 0U;
        unsigned int path_index = 0U;
        const unsigned int step_size = static_cast<unsigned int>(critic.path_align_step);
        for (unsigned int step = step_size; step < problem.time_steps; step += step_size) {
          trajectory_distance += hypotf(
            trajectory_x[offset + step] - trajectory_x[offset + step - step_size],
            trajectory_y[offset + step] - trajectory_y[offset + step - step_size]);
          path_index = closestIntegratedPathIndex(
            path_distance, furthest, trajectory_distance, path_index);
          if (path_valid[path_index]) {
            aligned_distance += hypotf(
              path_x[path_index] - trajectory_x[offset + step],
              path_y[path_index] - trajectory_y[offset + step]);
            ++samples;
          }
        }
        if (samples > 0U) {
          additional_cost += aligned_distance / static_cast<float>(samples) *
            critic.path_align_weight;
        }
      }
    }
  }
  costs[candidate] += additional_cost;
}

__global__ void reduceMinimum(const float * costs, unsigned int count, float * minimum)
{
  __shared__ float values[kThreadsPerBlock];
  const unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
  values[threadIdx.x] = index < count ? costs[index] : FLT_MAX;
  __syncthreads();
  for (unsigned int stride = blockDim.x / 2; stride > 0; stride /= 2) {
    if (threadIdx.x < stride) {
      values[threadIdx.x] = fminf(values[threadIdx.x], values[threadIdx.x + stride]);
    }
    __syncthreads();
  }
  if (threadIdx.x == 0) {
    atomicMinFloat(minimum, values[0]);
  }
}

__global__ void computeWeights(
  const float * costs, unsigned int count, const float * minimum, float temperature,
  float * weights, float * normalizer)
{
  const unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const float weight = expf(-(costs[index] - *minimum) / temperature);
  weights[index] = weight;
  atomicAdd(normalizer, weight);
}

__global__ void weightedControls(
  const float * candidate_vx, const float * candidate_wz, const float * weights,
  unsigned int batch_size, unsigned int time_steps, float * output_vx, float * output_wz)
{
  const unsigned int candidate = blockIdx.x * blockDim.x + threadIdx.x;
  if (candidate >= batch_size) {
    return;
  }
  const float weight = weights[candidate];
  const std::size_t offset = static_cast<std::size_t>(candidate) * time_steps;
  for (unsigned int step = 0; step < time_steps; ++step) {
    atomicAdd(&output_vx[step], weight * candidate_vx[offset + step]);
    atomicAdd(&output_wz[step], weight * candidate_wz[offset + step]);
  }
}

__global__ void normalizeControls(
  float * output_vx, float * output_wz, const float * normalizer, unsigned int time_steps)
{
  const unsigned int step = blockIdx.x * blockDim.x + threadIdx.x;
  if (step >= time_steps) {
    return;
  }
  const float denominator = fmaxf(*normalizer, 1e-12F);
  output_vx[step] /= denominator;
  output_wz[step] /= denominator;
}

void validate(const SamplingConfig & config, const OptimizerInput & input)
{
  if (config.batch_size == 0U || config.time_steps == 0U || config.model_dt <= 0.0F ||
    config.vx_std <= 0.0F || config.wz_std <= 0.0F || config.temperature <= 0.0F)
  {
    throw std::invalid_argument("MPPI CUDA configuration must be finite and positive");
  }
  if (config.vx_min > config.vx_max || config.wz_max <= 0.0F) {
    throw std::invalid_argument("MPPI CUDA velocity constraints are invalid");
  }
  if (input.nominal_vx.size() != config.time_steps || input.nominal_wz.size() != config.time_steps) {
    throw std::invalid_argument("nominal controls must match time_steps");
  }
  if (config.nav2_critics.enabled) {
    if (config.nav2_critics.consider_footprint) {
      throw std::invalid_argument(
              "CUDA MPPI does not yet support Nav2 footprint collision checking");
    }
    const std::size_t path_size = input.path_x.size();
    if (path_size < 2U || input.path_y.size() != path_size ||
      input.path_yaw.size() != path_size || input.path_valid.size() != path_size - 1U ||
      input.path_integrated_distance.size() != path_size)
    {
      throw std::invalid_argument(
              "full Nav2 CUDA critic mode requires path vectors and path_size - 1 validity data");
    }
  }
  const auto & map = input.costmap;
  if (map.data == nullptr || map.size_x == 0U || map.size_y == 0U || map.resolution <= 0.0F) {
    throw std::invalid_argument("costmap input is invalid");
  }
}

}  // namespace

class CudaMppiBackend::Impl
{
public:
  ~Impl()
  {
    release(device_costmap_);
    release(device_nominal_vx_);
    release(device_nominal_wz_);
    release(device_candidate_vx_);
    release(device_candidate_wz_);
    release(device_trajectory_x_);
    release(device_trajectory_y_);
    release(device_trajectory_yaw_);
    release(device_costs_);
    release(device_weights_);
    release(device_output_vx_);
    release(device_output_wz_);
    release(device_minimum_);
    release(device_normalizer_);
    release(device_valid_count_);
    release(device_furthest_path_point_);
    release(device_path_x_);
    release(device_path_y_);
    release(device_path_yaw_);
    release(device_path_valid_);
    release(device_path_distance_);
    if (start_event_ != nullptr) {
      cudaEventDestroy(start_event_);
    }
    if (end_event_ != nullptr) {
      cudaEventDestroy(end_event_);
    }
  }

  void ensureCapacity(
    const SamplingConfig & config, const CostmapView & map, std::size_t input_path_size)
  {
    const std::size_t candidate_count = config.batch_size * config.time_steps;
    if (candidate_count > candidate_capacity_) {
      release(device_candidate_vx_);
      release(device_candidate_wz_);
      release(device_trajectory_x_);
      release(device_trajectory_y_);
      release(device_trajectory_yaw_);
      MPPI_CUDA_CHECK(cudaMalloc(&device_candidate_vx_, candidate_count * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_candidate_wz_, candidate_count * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_trajectory_x_, candidate_count * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_trajectory_y_, candidate_count * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_trajectory_yaw_, candidate_count * sizeof(float)));
      candidate_capacity_ = candidate_count;
    }
    if (config.time_steps > time_capacity_) {
      release(device_nominal_vx_);
      release(device_nominal_wz_);
      release(device_output_vx_);
      release(device_output_wz_);
      MPPI_CUDA_CHECK(cudaMalloc(&device_nominal_vx_, config.time_steps * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_nominal_wz_, config.time_steps * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_output_vx_, config.time_steps * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_output_wz_, config.time_steps * sizeof(float)));
      time_capacity_ = config.time_steps;
    }
    if (config.batch_size > batch_capacity_) {
      release(device_costs_);
      release(device_weights_);
      MPPI_CUDA_CHECK(cudaMalloc(&device_costs_, config.batch_size * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_weights_, config.batch_size * sizeof(float)));
      batch_capacity_ = config.batch_size;
    }
    const std::size_t map_cells = map.size_x * map.size_y;
    if (map_cells > map_capacity_) {
      release(device_costmap_);
      MPPI_CUDA_CHECK(cudaMalloc(&device_costmap_, map_cells * sizeof(unsigned char)));
      map_capacity_ = map_cells;
    }
    if (config.nav2_critics.enabled && input_path_size > path_capacity_) {
      release(device_path_x_);
      release(device_path_y_);
      release(device_path_yaw_);
      release(device_path_valid_);
      release(device_path_distance_);
      MPPI_CUDA_CHECK(cudaMalloc(&device_path_x_, input_path_size * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_path_y_, input_path_size * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_path_yaw_, input_path_size * sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_path_valid_, input_path_size * sizeof(unsigned char)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_path_distance_, input_path_size * sizeof(float)));
      path_capacity_ = input_path_size;
    }
    if (device_minimum_ == nullptr) {
      MPPI_CUDA_CHECK(cudaMalloc(&device_minimum_, sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_normalizer_, sizeof(float)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_valid_count_, sizeof(unsigned int)));
      MPPI_CUDA_CHECK(cudaMalloc(&device_furthest_path_point_, sizeof(unsigned int)));
      MPPI_CUDA_CHECK(cudaEventCreate(&start_event_));
      MPPI_CUDA_CHECK(cudaEventCreate(&end_event_));
    }
  }

  unsigned char * device_costmap_{nullptr};
  float * device_nominal_vx_{nullptr};
  float * device_nominal_wz_{nullptr};
  float * device_candidate_vx_{nullptr};
  float * device_candidate_wz_{nullptr};
  float * device_trajectory_x_{nullptr};
  float * device_trajectory_y_{nullptr};
  float * device_trajectory_yaw_{nullptr};
  float * device_costs_{nullptr};
  float * device_weights_{nullptr};
  float * device_output_vx_{nullptr};
  float * device_output_wz_{nullptr};
  float * device_minimum_{nullptr};
  float * device_normalizer_{nullptr};
  unsigned int * device_valid_count_{nullptr};
  unsigned int * device_furthest_path_point_{nullptr};
  float * device_path_x_{nullptr};
  float * device_path_y_{nullptr};
  float * device_path_yaw_{nullptr};
  unsigned char * device_path_valid_{nullptr};
  float * device_path_distance_{nullptr};
  std::size_t candidate_capacity_{0U};
  std::size_t time_capacity_{0U};
  std::size_t batch_capacity_{0U};
  std::size_t map_capacity_{0U};
  std::size_t path_capacity_{0U};
  cudaEvent_t start_event_{nullptr};
  cudaEvent_t end_event_{nullptr};
};

CudaMppiBackend::CudaMppiBackend()
: impl_(std::make_unique<Impl>())
{
}

CudaMppiBackend::~CudaMppiBackend() = default;

bool CudaMppiBackend::isAvailable()
{
  int device_count = 0;
  return cudaGetDeviceCount(&device_count) == cudaSuccess && device_count > 0;
}

OptimizationResult CudaMppiBackend::optimize(
  const SamplingConfig & config, const OptimizerInput & input)
{
  validate(config, input);
  if (!isAvailable()) {
    throw std::runtime_error("no CUDA device is available for MPPI backend");
  }
  impl_->ensureCapacity(config, input.costmap, input.path_x.size());

  DeviceProblem problem{
    static_cast<unsigned int>(config.batch_size),
    static_cast<unsigned int>(config.time_steps),
    config.model_dt,
    config.vx_min,
    config.vx_max,
    config.wz_max,
    config.vx_std,
    config.wz_std,
    config.gamma,
    config.path_weight,
    config.goal_weight,
    config.obstacle_weight,
    config.collision_cost,
    config.seed,
    input.robot_x,
    input.robot_y,
    input.robot_yaw,
    input.measured_vx,
    input.measured_wz,
    input.path_target_x,
    input.path_target_y,
    input.goal_x,
    input.goal_y,
    static_cast<unsigned int>(input.costmap.size_x),
    static_cast<unsigned int>(input.costmap.size_y),
    input.costmap.resolution,
    input.costmap.origin_x,
    input.costmap.origin_y,
    input.costmap.track_unknown,
    config.nav2_critics,
    static_cast<unsigned int>(input.path_x.size()),
  };

  const int candidate_blocks = static_cast<int>(
    (config.batch_size + kThreadsPerBlock - 1U) / kThreadsPerBlock);
  const int time_blocks = static_cast<int>(
    (config.time_steps + kThreadsPerBlock - 1U) / kThreadsPerBlock);
  float initial_minimum = FLT_MAX;
  unsigned int valid_count = 0U;
  MPPI_CUDA_CHECK(cudaEventRecord(impl_->start_event_));
  MPPI_CUDA_CHECK(cudaMemcpy(
    impl_->device_costmap_, input.costmap.data,
    input.costmap.size_x * input.costmap.size_y * sizeof(unsigned char), cudaMemcpyHostToDevice));
  MPPI_CUDA_CHECK(cudaMemcpy(
    impl_->device_nominal_vx_, input.nominal_vx.data(),
    config.time_steps * sizeof(float), cudaMemcpyHostToDevice));
  MPPI_CUDA_CHECK(cudaMemcpy(
    impl_->device_nominal_wz_, input.nominal_wz.data(),
    config.time_steps * sizeof(float), cudaMemcpyHostToDevice));
  if (config.nav2_critics.enabled) {
    MPPI_CUDA_CHECK(cudaMemcpy(
      impl_->device_path_x_, input.path_x.data(),
      input.path_x.size() * sizeof(float), cudaMemcpyHostToDevice));
    MPPI_CUDA_CHECK(cudaMemcpy(
      impl_->device_path_y_, input.path_y.data(),
      input.path_y.size() * sizeof(float), cudaMemcpyHostToDevice));
    MPPI_CUDA_CHECK(cudaMemcpy(
      impl_->device_path_yaw_, input.path_yaw.data(),
      input.path_yaw.size() * sizeof(float), cudaMemcpyHostToDevice));
    MPPI_CUDA_CHECK(cudaMemcpy(
      impl_->device_path_valid_, input.path_valid.data(),
      input.path_valid.size() * sizeof(unsigned char), cudaMemcpyHostToDevice));
    MPPI_CUDA_CHECK(cudaMemcpy(
      impl_->device_path_distance_, input.path_integrated_distance.data(),
      input.path_integrated_distance.size() * sizeof(float), cudaMemcpyHostToDevice));
  }
  MPPI_CUDA_CHECK(cudaMemcpy(
    impl_->device_minimum_, &initial_minimum, sizeof(float), cudaMemcpyHostToDevice));
  MPPI_CUDA_CHECK(cudaMemset(impl_->device_normalizer_, 0, sizeof(float)));
  MPPI_CUDA_CHECK(cudaMemset(impl_->device_valid_count_, 0, sizeof(unsigned int)));
  MPPI_CUDA_CHECK(cudaMemset(impl_->device_furthest_path_point_, 0, sizeof(unsigned int)));
  MPPI_CUDA_CHECK(cudaMemset(impl_->device_output_vx_, 0, config.time_steps * sizeof(float)));
  MPPI_CUDA_CHECK(cudaMemset(impl_->device_output_wz_, 0, config.time_steps * sizeof(float)));

  rolloutAndScore<<<candidate_blocks, kThreadsPerBlock>>>(
    problem, impl_->device_costmap_, impl_->device_nominal_vx_, impl_->device_nominal_wz_,
    impl_->device_path_x_, impl_->device_path_y_,
    impl_->device_candidate_vx_, impl_->device_candidate_wz_,
    impl_->device_trajectory_x_, impl_->device_trajectory_y_, impl_->device_trajectory_yaw_,
    impl_->device_costs_, impl_->device_valid_count_, impl_->device_furthest_path_point_);
  MPPI_CUDA_CHECK(cudaGetLastError());
  if (config.nav2_critics.enabled) {
    scoreNav2Critics<<<candidate_blocks, kThreadsPerBlock>>>(
      problem, impl_->device_path_x_, impl_->device_path_y_, impl_->device_path_yaw_,
      impl_->device_path_valid_, impl_->device_path_distance_,
      impl_->device_candidate_vx_, impl_->device_trajectory_x_, impl_->device_trajectory_y_,
      impl_->device_trajectory_yaw_, impl_->device_furthest_path_point_, impl_->device_costs_);
    MPPI_CUDA_CHECK(cudaGetLastError());
  }
  reduceMinimum<<<candidate_blocks, kThreadsPerBlock>>>(
    impl_->device_costs_, static_cast<unsigned int>(config.batch_size), impl_->device_minimum_);
  MPPI_CUDA_CHECK(cudaGetLastError());
  computeWeights<<<candidate_blocks, kThreadsPerBlock>>>(
    impl_->device_costs_, static_cast<unsigned int>(config.batch_size), impl_->device_minimum_,
    config.temperature, impl_->device_weights_, impl_->device_normalizer_);
  MPPI_CUDA_CHECK(cudaGetLastError());
  weightedControls<<<candidate_blocks, kThreadsPerBlock>>>(
    impl_->device_candidate_vx_, impl_->device_candidate_wz_, impl_->device_weights_,
    static_cast<unsigned int>(config.batch_size), static_cast<unsigned int>(config.time_steps),
    impl_->device_output_vx_, impl_->device_output_wz_);
  MPPI_CUDA_CHECK(cudaGetLastError());
  normalizeControls<<<time_blocks, kThreadsPerBlock>>>(
    impl_->device_output_vx_, impl_->device_output_wz_, impl_->device_normalizer_,
    static_cast<unsigned int>(config.time_steps));
  MPPI_CUDA_CHECK(cudaGetLastError());

  OptimizationResult result;
  result.control_vx.resize(config.time_steps);
  result.control_wz.resize(config.time_steps);
  MPPI_CUDA_CHECK(cudaMemcpy(
    result.control_vx.data(), impl_->device_output_vx_, config.time_steps * sizeof(float),
    cudaMemcpyDeviceToHost));
  MPPI_CUDA_CHECK(cudaMemcpy(
    result.control_wz.data(), impl_->device_output_wz_, config.time_steps * sizeof(float),
    cudaMemcpyDeviceToHost));
  MPPI_CUDA_CHECK(cudaMemcpy(
    &result.min_cost, impl_->device_minimum_, sizeof(float), cudaMemcpyDeviceToHost));
  MPPI_CUDA_CHECK(cudaMemcpy(
    &valid_count, impl_->device_valid_count_, sizeof(unsigned int), cudaMemcpyDeviceToHost));
  MPPI_CUDA_CHECK(cudaEventRecord(impl_->end_event_));
  MPPI_CUDA_CHECK(cudaEventSynchronize(impl_->end_event_));
  MPPI_CUDA_CHECK(cudaEventElapsedTime(
    &result.gpu_elapsed_ms, impl_->start_event_, impl_->end_event_));
  result.all_trajectories_collide = valid_count == 0U;
  return result;
}

void CudaMppiBackend::reset()
{
  impl_ = std::make_unique<Impl>();
}

}  // namespace mppi_cuda_backend

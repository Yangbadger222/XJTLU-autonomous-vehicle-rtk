#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace
{

TEST(MppiCudaBackend, ProducesFiniteControlsWhenFreeSpaceExists)
{
  if (!mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
    GTEST_SKIP() << "CUDA device unavailable";
  }
  constexpr std::size_t kWidth = 64U;
  constexpr std::size_t kHeight = 64U;
  std::vector<unsigned char> costmap(kWidth * kHeight, 0U);
  mppi_cuda_backend::SamplingConfig config;
  config.batch_size = 512U;
  config.time_steps = 24U;
  mppi_cuda_backend::OptimizerInput input;
  input.path_target_x = 1.0F;
  input.goal_x = 2.0F;
  input.nominal_vx.assign(config.time_steps, 0.5F);
  input.nominal_wz.assign(config.time_steps, 0.0F);
  input.costmap = {costmap.data(), kWidth, kHeight, 0.1F, -3.2F, -3.2F, false};

  mppi_cuda_backend::CudaMppiBackend backend;
  const auto result = backend.optimize(config, input);

  ASSERT_FALSE(result.all_trajectories_collide);
  ASSERT_EQ(result.control_vx.size(), config.time_steps);
  EXPECT_TRUE(std::isfinite(result.control_vx.front()));
  EXPECT_TRUE(std::isfinite(result.control_wz.front()));
  EXPECT_GE(result.control_vx.front(), config.vx_min);
  EXPECT_LE(result.control_vx.front(), config.vx_max);
}

TEST(MppiCudaBackend, MarksACompletelyLethalCostmapAsBlocked)
{
  if (!mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
    GTEST_SKIP() << "CUDA device unavailable";
  }
  constexpr std::size_t kWidth = 64U;
  constexpr std::size_t kHeight = 64U;
  std::vector<unsigned char> costmap(kWidth * kHeight, 254U);
  mppi_cuda_backend::SamplingConfig config;
  config.batch_size = 512U;
  config.time_steps = 24U;
  mppi_cuda_backend::OptimizerInput input;
  input.nominal_vx.assign(config.time_steps, 0.5F);
  input.nominal_wz.assign(config.time_steps, 0.0F);
  input.costmap = {costmap.data(), kWidth, kHeight, 0.1F, -3.2F, -3.2F, false};

  mppi_cuda_backend::CudaMppiBackend backend;
  const auto result = backend.optimize(config, input);

  EXPECT_TRUE(result.all_trajectories_collide);
}

TEST(MppiCudaBackend, UsesMeasuredSpeedAtFirstRolloutStep)
{
  if (!mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
    GTEST_SKIP() << "CUDA device unavailable";
  }
  constexpr std::size_t kWidth = 64U;
  constexpr std::size_t kHeight = 64U;
  std::vector<unsigned char> costmap(kWidth * kHeight, 0U);
  // The robot starts in cell (32, 32). At 1 m/s and dt=0.05, a measured
  // first-step velocity enters the lethal cell (33, 32).
  costmap[32U * kWidth + 33U] = 254U;
  mppi_cuda_backend::SamplingConfig config;
  config.batch_size = 256U;
  config.time_steps = 1U;
  config.model_dt = 0.05F;
  config.vx_std = 0.01F;
  config.wz_std = 0.01F;
  mppi_cuda_backend::OptimizerInput input;
  input.nominal_vx.assign(config.time_steps, 0.0F);
  input.nominal_wz.assign(config.time_steps, 0.0F);
  input.costmap = {costmap.data(), kWidth, kHeight, 0.05F, -1.6F, -1.6F, false};

  mppi_cuda_backend::CudaMppiBackend backend;
  EXPECT_FALSE(backend.optimize(config, input).all_trajectories_collide);
  input.measured_vx = 1.0F;
  EXPECT_TRUE(backend.optimize(config, input).all_trajectories_collide);
}

}  // namespace

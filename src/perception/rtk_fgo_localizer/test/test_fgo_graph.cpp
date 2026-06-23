#include <gtest/gtest.h>

#include "rtk_fgo_localizer/fgo_graph.hpp"
#include "rtk_fgo_localizer/yaw_factor.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>

namespace rtk_fgo_localizer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(YawFactor, ResidualUsesPoseYawMinusMeasuredYaw)
{
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.1).finished());
  YawFactor factor(0, 12.0 * kPi / 180.0, noise);
  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(10.0 * kPi / 180.0), gtsam::Point3());

  const auto residual = factor.evaluateError(pose);

  EXPECT_NEAR(residual[0], -2.0 * kPi / 180.0, 1e-9);
}

TEST(FgoGraph, AddsInitialStateAndReturnsEstimate)
{
  FgoGraph graph;
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());

  auto estimate = graph.latestEstimate();

  ASSERT_TRUE(estimate.has_value());
}

TEST(FgoGraph, RejectsShadowCommitForHugeRtkCorrection)
{
  FgoGraph graph;
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());
  graph.addFastLioBetween(
    1.0,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));

  auto result = graph.tryShadowRtkCommit(1.0, gtsam::Point3(50.0, 0.0, 0.0), 0.02);

  EXPECT_FALSE(result.committed);
}

TEST(FgoGraph, CommitsSmallRtkRecovery)
{
  FgoGraph graph;
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());
  graph.addFastLioBetween(
    1.0,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));

  auto result = graph.tryShadowRtkCommit(1.0, gtsam::Point3(1.05, 0.0, 0.0), 0.02);

  EXPECT_TRUE(result.committed);
}

TEST(FgoGraph, AddsWheelFactorWithoutCreatingAnotherState)
{
  FgoGraph graph;
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());
  graph.addFastLioBetween(
    1.0,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));

  ASSERT_EQ(graph.stateCount(), 2u);

  const bool added = graph.addWheelPlanarFactorForLatestTransition(
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));

  EXPECT_TRUE(added);
  EXPECT_EQ(graph.stateCount(), 2u);
}

TEST(FgoGraph, ReportsGraphDiagnostics)
{
  FgoGraph graph(3);
  auto diagnostics = graph.diagnostics();

  EXPECT_EQ(diagnostics.state_count, 0u);
  EXPECT_EQ(diagnostics.value_count, 0u);
  EXPECT_EQ(diagnostics.factor_count, 0u);
  EXPECT_EQ(diagnostics.window_rebuild_count, 0u);
}

TEST(FgoGraph, KeepsStateCountAtMaxStatesAfterManyTransitions)
{
  FgoGraph graph(3);
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());

  for (int i = 1; i <= 8; ++i) {
    graph.addFastLioBetween(
      static_cast<double>(i),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, 0.0, 0.0)));
  }

  const auto diagnostics = graph.diagnostics();

  EXPECT_LE(diagnostics.state_count, 3u);
  EXPECT_LE(diagnostics.value_count, 9u);
  EXPECT_GT(diagnostics.window_rebuild_count, 0u);
  EXPECT_EQ(diagnostics.latest_state_index - diagnostics.oldest_state_index, 2u);
  EXPECT_TRUE(graph.latestEstimate().has_value());
}

}  // namespace
}  // namespace rtk_fgo_localizer

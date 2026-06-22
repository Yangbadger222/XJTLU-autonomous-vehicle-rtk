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

}  // namespace
}  // namespace rtk_fgo_localizer

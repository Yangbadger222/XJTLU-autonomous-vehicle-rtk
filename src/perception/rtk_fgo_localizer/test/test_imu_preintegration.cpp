#include <gtest/gtest.h>

#include "rtk_fgo_localizer/fgo_graph.hpp"
#include "rtk_fgo_localizer/imu_preintegration_config.hpp"

namespace rtk_fgo_localizer
{
namespace
{

TEST(FgoGraphImu, AcceptsImuSamplesBeforeNextState)
{
  FgoGraph graph;
  graph.configureImu(ImuPreintegrationConfig{});
  graph.addInitialState(0.0, gtsam::Pose3(), gtsam::Vector3::Zero());
  graph.addImuSample(0.01, Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero());
  graph.addImuSample(0.02, Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero());

  graph.addFastLioBetween(
    0.10,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.0)));

  auto estimate = graph.latestEstimate();

  ASSERT_TRUE(estimate.has_value());
}

}  // namespace
}  // namespace rtk_fgo_localizer

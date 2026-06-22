#pragma once

#include <Eigen/Core>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace rtk_fgo_localizer
{

struct GraphEstimate
{
  double stamp_s = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();
};

struct ShadowCommitResult
{
  bool committed = false;
  double correction_norm_m = 0.0;
  std::string reason;
};

class FgoGraph
{
public:
  explicit FgoGraph(std::size_t max_states = 120);

  void addInitialState(
    double stamp_s,
    const gtsam::Pose3 & pose,
    const gtsam::Vector3 & velocity);

  void addFastLioBetween(double stamp_s, const gtsam::Pose3 & relative_pose);
  void addWheelPlanarBetween(double stamp_s, const gtsam::Pose3 & relative_pose);
  void addRtkHeading(double stamp_s, double measured_yaw_rad, double yaw_sigma_rad);

  void addImuSample(
    double stamp_s,
    const Eigen::Vector3d & acc,
    const Eigen::Vector3d & gyro);
  void closeImuFactorBetween(std::size_t from_index, std::size_t to_index);

  ShadowCommitResult tryShadowRtkCommit(
    double stamp_s,
    const gtsam::Point3 & position_map,
    double position_sigma_m);

  std::optional<GraphEstimate> latestEstimate() const;
  std::size_t stateCount() const;
  void setMaxShadowCorrection(double max_correction_m);

private:
  struct StateRecord
  {
    double stamp_s = 0.0;
    std::size_t index = 0;
  };

  void optimizeActiveGraph();
  void updateLatestStateFromEstimate();
  void pruneIfNeeded();

  std::size_t max_states_ = 120;
  double max_shadow_correction_m_ = 3.0;
  std::vector<StateRecord> states_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  gtsam::Values estimate_;
};

}  // namespace rtk_fgo_localizer

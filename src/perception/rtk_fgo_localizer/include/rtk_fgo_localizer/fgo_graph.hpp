#pragma once

#include "rtk_fgo_localizer/imu_preintegration_config.hpp"

#include <Eigen/Core>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/shared_ptr.hpp>

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

struct FgoGraphDiagnostics
{
  std::size_t state_count = 0;
  std::size_t value_count = 0;
  std::size_t factor_count = 0;
  std::size_t window_rebuild_count = 0;
  std::size_t oldest_state_index = 0;
  std::size_t latest_state_index = 0;
  double last_optimization_error = 0.0;
  bool last_optimization_ok = true;
};

class FgoGraph
{
public:
  explicit FgoGraph(std::size_t max_states = 120);

  void configureImu(const ImuPreintegrationConfig & config);

  void addInitialState(
    double stamp_s,
    const gtsam::Pose3 & pose,
    const gtsam::Vector3 & velocity);

  void addFastLioBetween(double stamp_s, const gtsam::Pose3 & relative_pose);
  void addWheelPlanarBetween(double stamp_s, const gtsam::Pose3 & relative_pose);
  bool addWheelPlanarFactorForLatestTransition(const gtsam::Pose3 & relative_pose);
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
  FgoGraphDiagnostics diagnostics() const;
  void setMaxShadowCorrection(double max_correction_m);

private:
  struct StateRecord
  {
    double stamp_s = 0.0;
    std::size_t index = 0;
  };

  struct TransitionRecord
  {
    std::size_t from_index = 0;
    std::size_t to_index = 0;
    double stamp_s = 0.0;
    gtsam::Pose3 fast_lio_relative_pose;
    std::optional<gtsam::Pose3> wheel_relative_pose;
  };

  struct RtkPositionRecord
  {
    std::size_t state_index = 0;
    gtsam::Point3 position_map;
    double sigma_m = 1.0;
  };

  struct HeadingRecord
  {
    std::size_t state_index = 0;
    double measured_yaw_rad = 0.0;
    double yaw_sigma_rad = 0.05;
  };

  void optimizeActiveGraph();
  void updateLatestStateFromEstimate();
  void pruneIfNeeded();
  void rebuildActiveWindow();
  void resetImuPreintegrator(const gtsam::imuBias::ConstantBias & bias);
  void addPendingImuFactor(const StateRecord & previous, const StateRecord & current);

  std::size_t max_states_ = 120;
  double max_shadow_correction_m_ = 3.0;
  std::vector<StateRecord> states_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  gtsam::Values estimate_;
  std::vector<TransitionRecord> transitions_;
  std::vector<RtkPositionRecord> rtk_position_records_;
  std::vector<HeadingRecord> heading_records_;
  std::size_t window_rebuild_count_ = 0;
  double last_optimization_error_ = 0.0;
  bool last_optimization_ok_ = true;
  bool imu_configured_ = false;
  ImuPreintegrationConfig imu_config_;
  boost::shared_ptr<gtsam::PreintegrationParams> imu_params_;
  std::optional<gtsam::PreintegratedImuMeasurements> imu_preintegrator_;
  std::optional<double> last_imu_stamp_s_;
};

}  // namespace rtk_fgo_localizer

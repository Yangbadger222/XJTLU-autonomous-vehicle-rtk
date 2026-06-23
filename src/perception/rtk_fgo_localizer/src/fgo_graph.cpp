#include "rtk_fgo_localizer/fgo_graph.hpp"

#include "rtk_fgo_localizer/yaw_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <boost/make_shared.hpp>

#include <algorithm>
#include <cmath>
#include <exception>

namespace rtk_fgo_localizer
{
namespace
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

gtsam::noiseModel::Diagonal::shared_ptr posePriorNoise()
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.10, 0.10, 0.10).finished());
}

gtsam::noiseModel::Diagonal::shared_ptr velocityPriorNoise()
{
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.5));
}

gtsam::noiseModel::Diagonal::shared_ptr biasPriorNoise()
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());
}

gtsam::noiseModel::Diagonal::shared_ptr biasBetweenNoise(
  const ImuPreintegrationConfig & config)
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) <<
      config.accelerometer_bias_rw_sigma,
      config.accelerometer_bias_rw_sigma,
      config.accelerometer_bias_rw_sigma,
      config.gyroscope_bias_rw_sigma,
      config.gyroscope_bias_rw_sigma,
      config.gyroscope_bias_rw_sigma).finished());
}

gtsam::noiseModel::Diagonal::shared_ptr fastLioBetweenNoise()
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.03, 0.03, 0.05, 0.10, 0.10, 0.15).finished());
}

gtsam::noiseModel::Diagonal::shared_ptr wheelPlanarNoise()
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.10, 0.10, 10.0, 0.20, 0.20, 10.0).finished());
}

gtsam::noiseModel::Diagonal::shared_ptr rtkPositionNoise(double sigma_m)
{
  const double sigma = std::clamp(sigma_m, 0.02, 10.0);
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sigma, sigma, sigma * 2.0));
}

double translationDistance(const gtsam::Point3 & a, const gtsam::Point3 & b)
{
  return (a - b).norm();
}

}  // namespace

FgoGraph::FgoGraph(std::size_t max_states)
: max_states_(std::max<std::size_t>(2, max_states))
{
}

void FgoGraph::configureImu(const ImuPreintegrationConfig & config)
{
  imu_config_ = config;
  imu_params_ = gtsam::PreintegrationParams::MakeSharedU(config.gravity_mps2);
  imu_params_->accelerometerCovariance =
    gtsam::Matrix33::Identity() *
    config.accelerometer_noise_sigma * config.accelerometer_noise_sigma;
  imu_params_->gyroscopeCovariance =
    gtsam::Matrix33::Identity() *
    config.gyroscope_noise_sigma * config.gyroscope_noise_sigma;
  imu_params_->integrationCovariance =
    gtsam::Matrix33::Identity() *
    config.integration_error_sigma * config.integration_error_sigma;
  imu_configured_ = true;

  gtsam::imuBias::ConstantBias bias;
  if (!states_.empty() && estimate_.exists(B(states_.back().index))) {
    bias = estimate_.at<gtsam::imuBias::ConstantBias>(B(states_.back().index));
  }
  resetImuPreintegrator(bias);
}

void FgoGraph::addInitialState(
  double stamp_s,
  const gtsam::Pose3 & pose,
  const gtsam::Vector3 & velocity)
{
  states_.clear();
  transitions_.clear();
  rtk_position_records_.clear();
  heading_records_.clear();
  graph_ = gtsam::NonlinearFactorGraph();
  initial_ = gtsam::Values();
  estimate_ = gtsam::Values();

  states_.push_back({stamp_s, 0});
  initial_.insert(X(0), pose);
  initial_.insert(V(0), velocity);
  initial_.insert(B(0), gtsam::imuBias::ConstantBias());

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), pose, posePriorNoise()));
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), velocity, velocityPriorNoise()));
  graph_.add(
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(0), gtsam::imuBias::ConstantBias(), biasPriorNoise()));

  estimate_ = initial_;
  if (imu_configured_) {
    resetImuPreintegrator(gtsam::imuBias::ConstantBias());
  }
}

void FgoGraph::addFastLioBetween(double stamp_s, const gtsam::Pose3 & relative_pose)
{
  if (states_.empty()) {
    addInitialState(stamp_s, gtsam::Pose3(), gtsam::Vector3::Zero());
    return;
  }

  const auto previous = states_.back();
  const std::size_t next_index = previous.index + 1;
  const auto previous_pose = estimate_.at<gtsam::Pose3>(X(previous.index));
  const auto next_pose = previous_pose.compose(relative_pose);
  const double dt = std::max(1e-3, stamp_s - previous.stamp_s);
  const auto delta = relative_pose.translation();
  const gtsam::Vector3 velocity(delta.x() / dt, delta.y() / dt, delta.z() / dt);

  states_.push_back({stamp_s, next_index});
  initial_.insert(X(next_index), next_pose);
  initial_.insert(V(next_index), velocity);
  initial_.insert(B(next_index), estimate_.at<gtsam::imuBias::ConstantBias>(B(previous.index)));
  estimate_.insert(X(next_index), next_pose);
  estimate_.insert(V(next_index), velocity);
  estimate_.insert(B(next_index), estimate_.at<gtsam::imuBias::ConstantBias>(B(previous.index)));

  graph_.add(
    gtsam::BetweenFactor<gtsam::Pose3>(
      X(previous.index), X(next_index), relative_pose, fastLioBetweenNoise()));
  transitions_.push_back({previous.index, next_index, stamp_s, relative_pose, std::nullopt});
  addPendingImuFactor(previous, states_.back());
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(next_index), velocity, velocityPriorNoise()));
  graph_.add(
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(next_index),
      estimate_.at<gtsam::imuBias::ConstantBias>(B(previous.index)),
      biasPriorNoise()));
  optimizeActiveGraph();
  if (imu_configured_) {
    const auto latest = states_.back();
    resetImuPreintegrator(estimate_.at<gtsam::imuBias::ConstantBias>(B(latest.index)));
  }
  pruneIfNeeded();
}

void FgoGraph::addWheelPlanarBetween(double stamp_s, const gtsam::Pose3 & relative_pose)
{
  if (states_.empty()) {
    addInitialState(stamp_s, gtsam::Pose3(), gtsam::Vector3::Zero());
    return;
  }
  addFastLioBetween(stamp_s, relative_pose);
  addWheelPlanarFactorForLatestTransition(relative_pose);
}

bool FgoGraph::addWheelPlanarFactorForLatestTransition(const gtsam::Pose3 & relative_pose)
{
  if (states_.size() < 2) {
    return false;
  }
  const auto previous = states_[states_.size() - 2];
  const auto current = states_.back();
  graph_.add(
    gtsam::BetweenFactor<gtsam::Pose3>(
      X(previous.index), X(current.index), relative_pose, wheelPlanarNoise()));
  if (!transitions_.empty() &&
    transitions_.back().from_index == previous.index &&
    transitions_.back().to_index == current.index)
  {
    transitions_.back().wheel_relative_pose = relative_pose;
  }
  optimizeActiveGraph();
  return true;
}

void FgoGraph::addRtkHeading(double, double measured_yaw_rad, double yaw_sigma_rad)
{
  if (states_.empty()) {
    return;
  }
  const double sigma = std::clamp(yaw_sigma_rad, 0.005, 1.0);
  const auto noise =
    gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << sigma).finished());
  graph_.add(boost::make_shared<YawFactor>(X(states_.back().index), measured_yaw_rad, noise));
  heading_records_.push_back({states_.back().index, measured_yaw_rad, sigma});
  optimizeActiveGraph();
}

void FgoGraph::addImuSample(
  double stamp_s,
  const Eigen::Vector3d & acc,
  const Eigen::Vector3d & gyro)
{
  if (!imu_configured_ || !imu_preintegrator_.has_value()) {
    return;
  }
  if (!last_imu_stamp_s_.has_value()) {
    last_imu_stamp_s_ = stamp_s;
    return;
  }
  const double dt = stamp_s - *last_imu_stamp_s_;
  if (dt <= 0.0 || !std::isfinite(dt)) {
    return;
  }
  imu_preintegrator_->integrateMeasurement(acc, gyro, dt);
  last_imu_stamp_s_ = stamp_s;
}

void FgoGraph::closeImuFactorBetween(std::size_t, std::size_t)
{
  // Explicit closing by index is kept as a future integration point; the active path closes
  // pending IMU preintegration automatically when addFastLioBetween() creates a new state.
}

ShadowCommitResult FgoGraph::tryShadowRtkCommit(
  double,
  const gtsam::Point3 & position_map,
  double position_sigma_m)
{
  if (states_.empty()) {
    return {false, 0.0, "graph has no state"};
  }

  const auto latest = states_.back();
  const auto current_pose = estimate_.at<gtsam::Pose3>(X(latest.index));
  const double raw_correction = translationDistance(current_pose.translation(), position_map);
  if (raw_correction > max_shadow_correction_m_) {
    return {false, raw_correction, "raw RTK correction exceeds gate"};
  }

  auto shadow_graph = graph_;
  auto shadow_initial = estimate_;
  shadow_graph.add(
    gtsam::GPSFactor(
      X(latest.index), position_map,
      rtkPositionNoise(position_sigma_m)));

  try {
    const auto shadow_result =
      gtsam::LevenbergMarquardtOptimizer(shadow_graph, shadow_initial).optimize();
    const auto optimized_pose = shadow_result.at<gtsam::Pose3>(X(latest.index));
    const double optimized_correction =
      translationDistance(current_pose.translation(), optimized_pose.translation());
    if (optimized_correction > max_shadow_correction_m_) {
      return {false, optimized_correction, "optimized RTK correction exceeds gate"};
    }
    graph_ = shadow_graph;
    estimate_ = shadow_result;
    initial_ = shadow_result;
    rtk_position_records_.push_back({latest.index, position_map, position_sigma_m});
    return {true, optimized_correction, "RTK shadow commit accepted"};
  } catch (const std::exception & error) {
    return {false, raw_correction, error.what()};
  }
}

std::optional<GraphEstimate> FgoGraph::latestEstimate() const
{
  if (states_.empty() || estimate_.empty()) {
    return std::nullopt;
  }
  const auto latest = states_.back();
  GraphEstimate out;
  out.stamp_s = latest.stamp_s;
  out.pose = estimate_.at<gtsam::Pose3>(X(latest.index));
  out.velocity = estimate_.at<gtsam::Vector3>(V(latest.index));
  return out;
}

std::size_t FgoGraph::stateCount() const
{
  return states_.size();
}

FgoGraphDiagnostics FgoGraph::diagnostics() const
{
  FgoGraphDiagnostics out;
  out.state_count = states_.size();
  out.value_count = estimate_.size();
  out.factor_count = graph_.size();
  out.window_rebuild_count = window_rebuild_count_;
  if (!states_.empty()) {
    out.oldest_state_index = states_.front().index;
    out.latest_state_index = states_.back().index;
  }
  out.last_optimization_error = last_optimization_error_;
  out.last_optimization_ok = last_optimization_ok_;
  return out;
}

void FgoGraph::setMaxShadowCorrection(double max_correction_m)
{
  max_shadow_correction_m_ = std::max(0.0, max_correction_m);
}

void FgoGraph::resetImuPreintegrator(const gtsam::imuBias::ConstantBias & bias)
{
  if (!imu_configured_ || !imu_params_) {
    imu_preintegrator_.reset();
    last_imu_stamp_s_.reset();
    return;
  }
  imu_preintegrator_.emplace(imu_params_, bias);
  last_imu_stamp_s_.reset();
}

void FgoGraph::addPendingImuFactor(const StateRecord & previous, const StateRecord & current)
{
  if (!imu_configured_ || !imu_preintegrator_.has_value()) {
    return;
  }
  if (imu_preintegrator_->deltaTij() <= 0.0) {
    return;
  }

  graph_.add(
    gtsam::ImuFactor(
      X(previous.index), V(previous.index),
      X(current.index), V(current.index),
      B(previous.index), *imu_preintegrator_));
  graph_.add(
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(previous.index), B(current.index),
      gtsam::imuBias::ConstantBias(), biasBetweenNoise(imu_config_)));
}

void FgoGraph::optimizeActiveGraph()
{
  if (states_.empty()) {
    return;
  }
  try {
    estimate_ = gtsam::LevenbergMarquardtOptimizer(graph_, estimate_).optimize();
    initial_ = estimate_;
    last_optimization_error_ = graph_.error(estimate_);
    last_optimization_ok_ = true;
  } catch (const std::exception &) {
    last_optimization_ok_ = false;
    // Keep the last usable estimate; the ROS node will expose graph errors through diagnostics later.
  }
}

void FgoGraph::updateLatestStateFromEstimate()
{
  if (states_.empty() || estimate_.empty()) {
    return;
  }
  const auto latest = states_.back();
  if (!initial_.exists(X(latest.index))) {
    initial_.insert(X(latest.index), estimate_.at<gtsam::Pose3>(X(latest.index)));
  }
}

void FgoGraph::pruneIfNeeded()
{
  if (states_.size() <= max_states_) {
    return;
  }
  rebuildActiveWindow();
  updateLatestStateFromEstimate();
}

void FgoGraph::rebuildActiveWindow()
{
  if (states_.empty()) {
    return;
  }

  const std::size_t keep_count = std::min(max_states_, states_.size());
  const std::size_t drop_count = states_.size() - keep_count;
  std::vector<StateRecord> old_states(
    states_.begin() + static_cast<std::ptrdiff_t>(drop_count), states_.end());
  std::vector<TransitionRecord> old_transitions;
  std::vector<RtkPositionRecord> old_rtk_positions;
  std::vector<HeadingRecord> old_headings;
  old_transitions.reserve(old_states.size() > 0 ? old_states.size() - 1 : 0);
  const std::size_t first_kept_old_index = old_states.front().index;
  for (const auto & transition : transitions_) {
    if (transition.from_index >= first_kept_old_index &&
      transition.to_index >= first_kept_old_index)
    {
      old_transitions.push_back(transition);
    }
  }
  for (const auto & rtk_position : rtk_position_records_) {
    if (rtk_position.state_index >= first_kept_old_index) {
      old_rtk_positions.push_back(rtk_position);
    }
  }
  for (const auto & heading : heading_records_) {
    if (heading.state_index >= first_kept_old_index) {
      old_headings.push_back(heading);
    }
  }

  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_initial;
  gtsam::Values new_estimate;
  std::vector<StateRecord> new_states;
  std::vector<TransitionRecord> new_transitions;
  std::vector<RtkPositionRecord> new_rtk_positions;
  std::vector<HeadingRecord> new_headings;
  new_states.reserve(old_states.size());
  new_transitions.reserve(old_transitions.size());
  new_rtk_positions.reserve(old_rtk_positions.size());
  new_headings.reserve(old_headings.size());

  for (std::size_t i = 0; i < old_states.size(); ++i) {
    const auto old_index = old_states[i].index;
    const auto new_index = i;
    const auto pose = estimate_.at<gtsam::Pose3>(X(old_index));
    const auto velocity = estimate_.at<gtsam::Vector3>(V(old_index));
    const auto bias = estimate_.at<gtsam::imuBias::ConstantBias>(B(old_index));
    new_states.push_back({old_states[i].stamp_s, new_index});
    new_initial.insert(X(new_index), pose);
    new_initial.insert(V(new_index), velocity);
    new_initial.insert(B(new_index), bias);
    new_estimate.insert(X(new_index), pose);
    new_estimate.insert(V(new_index), velocity);
    new_estimate.insert(B(new_index), bias);

    if (i == 0) {
      new_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(new_index), pose, posePriorNoise()));
      new_graph.add(
        gtsam::PriorFactor<gtsam::Vector3>(
          V(new_index), velocity,
          velocityPriorNoise()));
      new_graph.add(
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(new_index), bias, biasPriorNoise()));
    }
  }

  for (const auto & transition : old_transitions) {
    const std::size_t from_new = transition.from_index - first_kept_old_index;
    const std::size_t to_new = transition.to_index - first_kept_old_index;
    if (to_new >= new_states.size() || from_new >= new_states.size()) {
      continue;
    }
    new_graph.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        X(from_new), X(to_new), transition.fast_lio_relative_pose, fastLioBetweenNoise()));
    if (transition.wheel_relative_pose.has_value()) {
      new_graph.add(
        gtsam::BetweenFactor<gtsam::Pose3>(
          X(from_new), X(to_new), *transition.wheel_relative_pose, wheelPlanarNoise()));
    }
    new_transitions.push_back(
      {from_new, to_new, transition.stamp_s, transition.fast_lio_relative_pose,
        transition.wheel_relative_pose});
  }

  for (const auto & rtk_position : old_rtk_positions) {
    const std::size_t state_new = rtk_position.state_index - first_kept_old_index;
    if (state_new >= new_states.size()) {
      continue;
    }
    new_graph.add(
      gtsam::GPSFactor(
        X(state_new), rtk_position.position_map,
        rtkPositionNoise(rtk_position.sigma_m)));
    new_rtk_positions.push_back(
      {state_new, rtk_position.position_map, rtk_position.sigma_m});
  }

  for (const auto & heading : old_headings) {
    const std::size_t state_new = heading.state_index - first_kept_old_index;
    if (state_new >= new_states.size()) {
      continue;
    }
    const auto noise =
      gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(1) << heading.yaw_sigma_rad).finished());
    new_graph.add(boost::make_shared<YawFactor>(X(state_new), heading.measured_yaw_rad, noise));
    new_headings.push_back(
      {state_new, heading.measured_yaw_rad, heading.yaw_sigma_rad});
  }

  states_ = std::move(new_states);
  transitions_ = std::move(new_transitions);
  rtk_position_records_ = std::move(new_rtk_positions);
  heading_records_ = std::move(new_headings);
  graph_ = std::move(new_graph);
  initial_ = std::move(new_initial);
  estimate_ = std::move(new_estimate);
  ++window_rebuild_count_;
  optimizeActiveGraph();
}

}  // namespace rtk_fgo_localizer

#include "rtk_fgo_localizer/fgo_graph.hpp"

#include "rtk_fgo_localizer/yaw_factor.hpp"

#include <gtsam/inference/Symbol.h>
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

void FgoGraph::addInitialState(
  double stamp_s,
  const gtsam::Pose3 & pose,
  const gtsam::Vector3 & velocity)
{
  states_.clear();
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
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(next_index), velocity, velocityPriorNoise()));
  graph_.add(
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(next_index),
      estimate_.at<gtsam::imuBias::ConstantBias>(B(previous.index)),
      biasPriorNoise()));
  optimizeActiveGraph();
  pruneIfNeeded();
}

void FgoGraph::addWheelPlanarBetween(double stamp_s, const gtsam::Pose3 & relative_pose)
{
  if (states_.empty()) {
    addInitialState(stamp_s, gtsam::Pose3(), gtsam::Vector3::Zero());
    return;
  }
  const auto previous = states_.back();
  addFastLioBetween(stamp_s, relative_pose);
  const auto current = states_.back();
  if (current.index != previous.index) {
    graph_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        X(previous.index), X(current.index), relative_pose, wheelPlanarNoise()));
    optimizeActiveGraph();
  }
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
  optimizeActiveGraph();
}

void FgoGraph::addImuSample(
  double,
  const Eigen::Vector3d &,
  const Eigen::Vector3d &)
{
  // The first shadow-mode graph keeps the API but waits for Jetson GTSAM IMU API validation
  // before enabling preintegration factors.
}

void FgoGraph::closeImuFactorBetween(std::size_t, std::size_t)
{
  // See addImuSample(); kept as an integration point for the next IMU chunk.
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

void FgoGraph::setMaxShadowCorrection(double max_correction_m)
{
  max_shadow_correction_m_ = std::max(0.0, max_correction_m);
}

void FgoGraph::optimizeActiveGraph()
{
  if (states_.empty()) {
    return;
  }
  try {
    estimate_ = gtsam::LevenbergMarquardtOptimizer(graph_, estimate_).optimize();
    initial_ = estimate_;
  } catch (const std::exception &) {
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
  // Full dense marginalization is intentionally deferred. The first implementation keeps the
  // fixed-window contract at the bookkeeping level and relies on bounded max_states for runtime.
  while (states_.size() > max_states_) {
    states_.erase(states_.begin());
  }
  updateLatestStateFromEstimate();
}

}  // namespace rtk_fgo_localizer

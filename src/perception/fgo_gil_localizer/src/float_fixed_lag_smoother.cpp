#include "fgo_gil_localizer/float_fixed_lag_smoother.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace fgo_gil_localizer
{
namespace
{

constexpr int kStateDimension = 15;

bool validNoise(const StateFactorNoise & noise)
{
  return std::isfinite(noise.position_m) && noise.position_m > 0.0 &&
         std::isfinite(noise.rotation_rad) && noise.rotation_rad > 0.0 &&
         std::isfinite(noise.velocity_m_s) && noise.velocity_m_s > 0.0 &&
         std::isfinite(noise.accelerometer_bias_m_s2) &&
         noise.accelerometer_bias_m_s2 > 0.0 &&
         std::isfinite(noise.gyroscope_bias_rad_s) && noise.gyroscope_bias_rad_s > 0.0;
}

bool validState(const EcefState & state)
{
  return std::isfinite(state.stamp_s) && finite(state.position_ecef_m) &&
         finite(state.velocity_ecef_m_s) && finite(state.orientation_ecef_body) &&
         finite(state.accelerometer_bias_m_s2) && finite(state.gyroscope_bias_rad_s) &&
         norm(state.position_ecef_m) > 1.0e6;
}

Eigen::VectorXd stateLocalCoordinates(const EcefState & anchor, const EcefState & value)
{
  Eigen::VectorXd delta(kStateDimension);
  const Vec3 position = value.position_ecef_m - anchor.position_ecef_m;
  const Vec3 rotation = quaternionLog(
    value.orientation_ecef_body * anchor.orientation_ecef_body.conjugate());
  const Vec3 velocity = value.velocity_ecef_m_s - anchor.velocity_ecef_m_s;
  const Vec3 accelerometer_bias =
    value.accelerometer_bias_m_s2 - anchor.accelerometer_bias_m_s2;
  const Vec3 gyroscope_bias = value.gyroscope_bias_rad_s - anchor.gyroscope_bias_rad_s;
  for (int axis = 0; axis < 3; ++axis) {
    delta(axis) = position[static_cast<std::size_t>(axis)];
    delta(3 + axis) = rotation[static_cast<std::size_t>(axis)];
    delta(6 + axis) = velocity[static_cast<std::size_t>(axis)];
    delta(9 + axis) = accelerometer_bias[static_cast<std::size_t>(axis)];
    delta(12 + axis) = gyroscope_bias[static_cast<std::size_t>(axis)];
  }
  return delta;
}

EcefState perturbedState(const EcefState & state, const Eigen::VectorXd & delta)
{
  EcefState result = state;
  result.position_ecef_m = result.position_ecef_m + Vec3{delta(0), delta(1), delta(2)};
  result.orientation_ecef_body =
    (quaternionFromRotationVector({delta(3), delta(4), delta(5)}) *
    result.orientation_ecef_body).normalized();
  result.velocity_ecef_m_s = result.velocity_ecef_m_s + Vec3{delta(6), delta(7), delta(8)};
  result.accelerometer_bias_m_s2 =
    result.accelerometer_bias_m_s2 + Vec3{delta(9), delta(10), delta(11)};
  result.gyroscope_bias_rad_s =
    result.gyroscope_bias_rad_s + Vec3{delta(12), delta(13), delta(14)};
  return result;
}

Eigen::VectorXd inverseNoise(const StateFactorNoise & noise)
{
  Eigen::VectorXd inverse(kStateDimension);
  for (int axis = 0; axis < 3; ++axis) {
    inverse(axis) = 1.0 / noise.position_m;
    inverse(3 + axis) = 1.0 / noise.rotation_rad;
    inverse(6 + axis) = 1.0 / noise.velocity_m_s;
    inverse(9 + axis) = 1.0 / noise.accelerometer_bias_m_s2;
    inverse(12 + axis) = 1.0 / noise.gyroscope_bias_rad_s;
  }
  return inverse;
}

PoseJacobianRow independentPoseJacobian(
  const PoseJacobianRow & left_pose_jacobian,
  const Quaternion & orientation_world_body,
  const Vec3 & point_body)
{
  const Vec3 direction_world{
    left_pose_jacobian[0], left_pose_jacobian[1], left_pose_jacobian[2]};
  const Vec3 point_offset_world = orientation_world_body.rotate(point_body);
  const Vec3 rotation = cross(point_offset_world, direction_world);
  return {
    direction_world.x, direction_world.y, direction_world.z,
    rotation.x, rotation.y, rotation.z};
}

std::optional<EcefState> propagateImuState(
  const EcefState & from,
  const std::vector<ImuSample> & samples,
  const EcefImuConfig & config)
{
  if (samples.size() < 2U) {
    return std::nullopt;
  }
  EcefImuPreintegrator preintegrator(config, false);
  if (!preintegrator.reset(from)) {
    return std::nullopt;
  }
  for (const auto & sample : samples) {
    const ImuIntegrationResult result = preintegrator.integrate(sample);
    if (result != ImuIntegrationResult::Initialized &&
      result != ImuIntegrationResult::Integrated)
    {
      return std::nullopt;
    }
  }
  if (!preintegrator.valid()) {
    return std::nullopt;
  }
  return preintegrator.state();
}

std::optional<Eigen::VectorXd> imuResidualFromPrediction(
  const EcefState & from,
  const EcefState & to,
  const EcefState & predicted)
{
  if (std::abs(predicted.stamp_s - to.stamp_s) > 1.0e-6) {
    return std::nullopt;
  }
  Eigen::VectorXd residual(kStateDimension);
  const Vec3 position = to.position_ecef_m - predicted.position_ecef_m;
  const Vec3 rotation = quaternionLog(
    to.orientation_ecef_body * predicted.orientation_ecef_body.conjugate());
  const Vec3 velocity = to.velocity_ecef_m_s - predicted.velocity_ecef_m_s;
  const Vec3 accelerometer_bias =
    to.accelerometer_bias_m_s2 - from.accelerometer_bias_m_s2;
  const Vec3 gyroscope_bias = to.gyroscope_bias_rad_s - from.gyroscope_bias_rad_s;
  for (int axis = 0; axis < 3; ++axis) {
    residual(axis) = position[static_cast<std::size_t>(axis)];
    residual(3 + axis) = rotation[static_cast<std::size_t>(axis)];
    residual(6 + axis) = velocity[static_cast<std::size_t>(axis)];
    residual(9 + axis) = accelerometer_bias[static_cast<std::size_t>(axis)];
    residual(12 + axis) = gyroscope_bias[static_cast<std::size_t>(axis)];
  }
  if (!residual.allFinite()) {
    return std::nullopt;
  }
  return residual;
}

std::optional<Eigen::VectorXd> evaluateImuResidual(
  const EcefState & from,
  const EcefState & to,
  const std::vector<ImuSample> & samples,
  const EcefImuConfig & config)
{
  const auto predicted = propagateImuState(from, samples, config);
  if (!predicted.has_value()) {
    return std::nullopt;
  }
  return imuResidualFromPrediction(from, to, *predicted);
}

double perturbationStep(const int column)
{
  if (column < 3) {
    return 1.0e-3;
  }
  if (column < 6) {
    return 1.0e-6;
  }
  if (column < 9) {
    return 1.0e-4;
  }
  if (column < 12) {
    return 1.0e-5;
  }
  return 1.0e-7;
}

double robustWeight(const double standardized_residual, const double huber_delta)
{
  const double magnitude = std::abs(standardized_residual);
  if (huber_delta <= 0.0 || magnitude <= huber_delta) {
    return 1.0;
  }
  return huber_delta / magnitude;
}

double robustCost(const double standardized_residual, const double huber_delta)
{
  const double magnitude = std::abs(standardized_residual);
  if (huber_delta <= 0.0 || magnitude <= huber_delta) {
    return 0.5 * standardized_residual * standardized_residual;
  }
  return huber_delta * (magnitude - 0.5 * huber_delta);
}

template<typename Factor>
std::vector<Factor> boundedUniformSample(
  const std::vector<Factor> & input,
  const std::size_t maximum_size)
{
  if (input.size() <= maximum_size) {
    return input;
  }
  std::vector<Factor> output;
  output.reserve(maximum_size);
  if (maximum_size == 1U) {
    output.push_back(input.front());
    return output;
  }
  for (std::size_t index = 0; index < maximum_size; ++index) {
    const std::size_t source = index * (input.size() - 1U) / (maximum_size - 1U);
    output.push_back(input[source]);
  }
  return output;
}

}  // namespace

FloatFixedLagSmoother::FloatFixedLagSmoother(
  FloatSmootherConfig config,
  LidarGraphFactorConfig lidar_config,
  GnssGraphFactorConfig gnss_config)
: config_(config), lidar_config_(lidar_config), gnss_config_(gnss_config)
{
  if (!std::isfinite(config_.duration_s) || config_.duration_s <= 0.0 ||
    config_.maximum_states == 0U || config_.maximum_iterations == 0U ||
    !std::isfinite(config_.initial_damping) || config_.initial_damping <= 0.0 ||
    !std::isfinite(config_.convergence_delta_norm) || config_.convergence_delta_norm <= 0.0 ||
    !std::isfinite(lidar_config_.line_sigma_m) || lidar_config_.line_sigma_m <= 0.0 ||
    !std::isfinite(lidar_config_.plane_sigma_m) || lidar_config_.plane_sigma_m <= 0.0 ||
    !std::isfinite(lidar_config_.huber_delta_sigma) ||
    lidar_config_.huber_delta_sigma <= 0.0 ||
    lidar_config_.maximum_line_factors_per_keyframe == 0U ||
    lidar_config_.maximum_plane_factors_per_keyframe == 0U ||
    !std::isfinite(gnss_config_.code_huber_delta_sigma) ||
    gnss_config_.code_huber_delta_sigma <= 0.0 ||
    !std::isfinite(gnss_config_.carrier_huber_delta_sigma) ||
    gnss_config_.carrier_huber_delta_sigma <= 0.0)
  {
    throw std::invalid_argument("float fixed-lag smoother configuration is outside valid bounds");
  }
}

bool FloatFixedLagSmoother::addState(const StateId id, const EcefState & initial_state)
{
  if (states_.find(id) != states_.end() || !validState(initial_state) ||
    (!state_order_.empty() && initial_state.stamp_s <= states_.at(state_order_.back()).stamp_s))
  {
    ++diagnostics_.rejected_factors;
    return false;
  }
  states_[id] = initial_state;
  state_order_.push_back(id);
  invalidateFixLinearization();
  refreshDiagnostics();
  return true;
}

bool FloatFixedLagSmoother::addStatePrior(
  const StateId id,
  const EcefState & mean,
  const StateFactorNoise & noise)
{
  if (states_.find(id) == states_.end() || !validState(mean) || !validNoise(noise)) {
    ++diagnostics_.rejected_factors;
    return false;
  }
  state_priors_.push_back({id, mean, noise});
  invalidateFixLinearization();
  refreshDiagnostics();
  return true;
}

bool FloatFixedLagSmoother::addImuFactor(
  const StateId from,
  const StateId to,
  const std::vector<ImuSample> & samples,
  const ImuGraphFactorConfig & config)
{
  const auto from_state = states_.find(from);
  const auto to_state = states_.find(to);
  if (from_state == states_.end() || to_state == states_.end() || samples.size() < 2U ||
    !validNoise(config.noise) ||
    std::abs(samples.front().stamp_s - from_state->second.stamp_s) > 1.0e-6 ||
    std::abs(samples.back().stamp_s - to_state->second.stamp_s) > 1.0e-6)
  {
    ++diagnostics_.rejected_factors;
    return false;
  }
  if (!evaluateImuResidual(from_state->second, to_state->second, samples, config.integration)) {
    ++diagnostics_.rejected_factors;
    return false;
  }
  imu_factors_.push_back({from, to, samples, config});
  invalidateFixLinearization();
  refreshDiagnostics();
  return true;
}

bool FloatFixedLagSmoother::addLidarFactors(
  const StateId state,
  const std::vector<PointToLineFactor> & line_factors,
  const std::vector<PointToPlaneFactor> & plane_factors,
  const RigidPose & body_lidar)
{
  if (states_.find(state) == states_.end() || !finite(body_lidar) ||
    (line_factors.empty() && plane_factors.empty()))
  {
    ++diagnostics_.rejected_factors;
    return false;
  }
  lidar_factors_.push_back(
    {
      state,
      boundedUniformSample(line_factors, lidar_config_.maximum_line_factors_per_keyframe),
      boundedUniformSample(plane_factors, lidar_config_.maximum_plane_factors_per_keyframe),
      body_lidar});
  invalidateFixLinearization();
  refreshDiagnostics();
  return true;
}

bool FloatFixedLagSmoother::addGnssFactors(
  const StateId state,
  const std::vector<DoubleDifferenceMeasurement> & measurements)
{
  const auto state_iterator = states_.find(state);
  if (state_iterator == states_.end()) {
    ++diagnostics_.rejected_factors;
    return false;
  }
  if (measurements.empty()) {
    recordGnssOutage();
    return true;
  }
  bool any_valid = false;
  for (const auto & measurement : measurements) {
    any_valid = any_valid || measurement.code_valid || measurement.carrier_valid;
    if (measurement.carrier_valid &&
      ambiguities_.find(measurement.ambiguity_key) == ambiguities_.end())
    {
      const auto transformed = transformedAmbiguityInitialization(measurement.ambiguity_key);
      if (transformed.has_value()) {
        ambiguities_[measurement.ambiguity_key] = *transformed;
      } else {
        const auto evaluation = evaluateDdCarrier(measurement, state_iterator->second, 0.0);
        if (evaluation.has_value()) {
          ambiguities_[measurement.ambiguity_key] = -evaluation->residual_m;
        }
      }
      ambiguity_observation_counts_[measurement.ambiguity_key] = 0U;
    }
    if (measurement.carrier_valid &&
      ambiguities_.find(measurement.ambiguity_key) != ambiguities_.end())
    {
      ambiguity_last_state_[measurement.ambiguity_key] = state;
      ++ambiguity_observation_counts_[measurement.ambiguity_key];
    }
  }
  if (!any_valid) {
    recordGnssOutage();
    return true;
  }
  gnss_factors_.push_back({state, measurements});
  invalidateFixLinearization();
  refreshDiagnostics();
  return true;
}

std::optional<double> FloatFixedLagSmoother::transformedAmbiguityInitialization(
  const DdAmbiguityKey & key) const
{
  for (const auto & pivot : ambiguities_) {
    const DdAmbiguityKey & pivot_key = pivot.first;
    if (!(pivot_key.group == key.group) || !(pivot_key.target == key.reference) ||
      pivot_key.receiver_arc_ids[0] != key.receiver_arc_ids[2] ||
      pivot_key.receiver_arc_ids[1] != key.receiver_arc_ids[3])
    {
      continue;
    }
    if (key.target == pivot_key.reference &&
      key.receiver_arc_ids[0] == pivot_key.receiver_arc_ids[2] &&
      key.receiver_arc_ids[1] == pivot_key.receiver_arc_ids[3])
    {
      return -pivot.second;
    }
    for (const auto & target : ambiguities_) {
      const DdAmbiguityKey & target_key = target.first;
      if (!(target_key.group == key.group) || !(target_key.reference == pivot_key.reference) ||
        !(target_key.target == key.target) ||
        target_key.receiver_arc_ids[0] != key.receiver_arc_ids[0] ||
        target_key.receiver_arc_ids[1] != key.receiver_arc_ids[1] ||
        target_key.receiver_arc_ids[2] != pivot_key.receiver_arc_ids[2] ||
        target_key.receiver_arc_ids[3] != pivot_key.receiver_arc_ids[3])
      {
        continue;
      }
      return target.second - pivot.second;
    }
  }
  return std::nullopt;
}

FloatFixedLagSmoother::VariableLayout FloatFixedLagSmoother::createLayout() const
{
  VariableLayout layout;
  for (const StateId id : state_order_) {
    layout.state_offsets[id] = layout.dimension;
    layout.variables.push_back({MarginalVariable::Kind::State, id, {}, kStateDimension});
    layout.dimension += kStateDimension;
  }
  for (const auto & ambiguity : ambiguities_) {
    layout.ambiguity_offsets[ambiguity.first] = layout.dimension;
    layout.variables.push_back({MarginalVariable::Kind::Ambiguity, 0, ambiguity.first, 1});
    ++layout.dimension;
  }
  return layout;
}

FloatFixedLagSmoother::LinearSystem FloatFixedLagSmoother::buildLinearSystem(
  const VariableLayout & layout,
  const std::optional<StateId> marginalize_state)
{
  LinearSystem system;
  system.hessian = Eigen::MatrixXd::Zero(layout.dimension, layout.dimension);
  system.gradient = Eigen::VectorXd::Zero(layout.dimension);

  const auto add_dense_row = [&system](
    const double residual, const double sigma, const double huber_delta,
    const std::vector<std::pair<int, Eigen::VectorXd>> & blocks) {
      if (!std::isfinite(residual) || !std::isfinite(sigma) || sigma <= 0.0) {
        return;
      }
      const double standardized = residual / sigma;
      const double weight = robustWeight(standardized, huber_delta);
      for (const auto & left : blocks) {
        const Eigen::VectorXd left_jacobian = left.second / sigma;
        system.gradient.segment(left.first, left_jacobian.size()) +=
          weight * left_jacobian * standardized;
        for (const auto & right : blocks) {
          const Eigen::VectorXd right_jacobian = right.second / sigma;
          system.hessian.block(
            left.first, right.first, left_jacobian.size(), right_jacobian.size()) +=
            weight * left_jacobian * right_jacobian.transpose();
        }
      }
      system.cost += robustCost(standardized, huber_delta);
      ++system.rows;
    };

  const auto add_correlated_rows = [&system](
    const Eigen::VectorXd & residuals, const Eigen::MatrixXd & covariance,
    const Eigen::MatrixXd & local_jacobian, const std::vector<int> & scalar_offsets,
    const double huber_delta) {
      if (residuals.size() == 0 || covariance.rows() != residuals.size() ||
        covariance.cols() != residuals.size() || local_jacobian.rows() != residuals.size() ||
        local_jacobian.cols() != static_cast<int>(scalar_offsets.size()) ||
        !residuals.allFinite() || !covariance.allFinite() || !local_jacobian.allFinite())
      {
        return;
      }
      Eigen::LLT<Eigen::MatrixXd> covariance_solver(covariance);
      if (covariance_solver.info() != Eigen::Success) {
        return;
      }
      const Eigen::MatrixXd lower = covariance_solver.matrixL();
      const Eigen::VectorXd whitened_residuals =
        lower.triangularView<Eigen::Lower>().solve(residuals);
      const Eigen::MatrixXd whitened_jacobian =
        lower.triangularView<Eigen::Lower>().solve(local_jacobian);
      if (!whitened_residuals.allFinite() || !whitened_jacobian.allFinite()) {
        return;
      }
      const double group_residual = whitened_residuals.norm() /
        std::sqrt(static_cast<double>(whitened_residuals.size()));
      const double weight = robustWeight(group_residual, huber_delta);
      const Eigen::VectorXd local_gradient =
        weight * whitened_jacobian.transpose() * whitened_residuals;
      const Eigen::MatrixXd local_hessian =
        weight * whitened_jacobian.transpose() * whitened_jacobian;
      for (int row = 0; row < local_gradient.size(); ++row) {
        const int global_row = scalar_offsets[static_cast<std::size_t>(row)];
        system.gradient(global_row) += local_gradient(row);
        for (int column = 0; column < local_hessian.cols(); ++column) {
          const int global_column = scalar_offsets[static_cast<std::size_t>(column)];
          system.hessian(global_row, global_column) += local_hessian(row, column);
        }
      }
      system.cost += static_cast<double>(whitened_residuals.size()) *
        robustCost(group_residual, huber_delta);
      system.rows += static_cast<std::size_t>(whitened_residuals.size());
    };

  if (marginal_prior_.has_value()) {
    const DenseMarginalPrior & prior = *marginal_prior_;
    std::vector<int> scalar_offsets;
    Eigen::VectorXd local(prior.hessian.rows());
    int prior_offset = 0;
    bool complete = true;
    for (const MarginalVariable & variable : prior.variables) {
      int current_offset = -1;
      if (variable.kind == MarginalVariable::Kind::State) {
        const auto layout_offset = layout.state_offsets.find(variable.state);
        const auto anchor = prior.state_anchors.find(variable.state);
        const auto current = states_.find(variable.state);
        if (layout_offset == layout.state_offsets.end() || anchor == prior.state_anchors.end() ||
          current == states_.end())
        {
          complete = false;
          break;
        }
        current_offset = layout_offset->second;
        local.segment(prior_offset, variable.dimension) =
          stateLocalCoordinates(anchor->second, current->second);
      } else {
        const auto layout_offset = layout.ambiguity_offsets.find(variable.ambiguity);
        const auto anchor = prior.ambiguity_anchors.find(variable.ambiguity);
        const auto current = ambiguities_.find(variable.ambiguity);
        if (layout_offset == layout.ambiguity_offsets.end() ||
          anchor == prior.ambiguity_anchors.end() || current == ambiguities_.end())
        {
          complete = false;
          break;
        }
        current_offset = layout_offset->second;
        local(prior_offset) = current->second - anchor->second;
      }
      for (int scalar = 0; scalar < variable.dimension; ++scalar) {
        scalar_offsets.push_back(current_offset + scalar);
      }
      prior_offset += variable.dimension;
    }
    if (complete && prior_offset == prior.hessian.rows()) {
      const Eigen::VectorXd shifted_gradient = prior.gradient + prior.hessian * local;
      for (int row = 0; row < prior.hessian.rows(); ++row) {
        system.gradient(scalar_offsets[static_cast<std::size_t>(row)]) += shifted_gradient(row);
        for (int column = 0; column < prior.hessian.cols(); ++column) {
          system.hessian(
            scalar_offsets[static_cast<std::size_t>(row)],
            scalar_offsets[static_cast<std::size_t>(column)]) += prior.hessian(row, column);
        }
      }
      system.cost += 0.5 * local.dot(prior.hessian * local) + prior.gradient.dot(local);
      system.rows += static_cast<std::size_t>(prior.hessian.rows());
    }
  }

  for (const auto & factor : state_priors_) {
    if (marginalize_state.has_value() && factor.state != *marginalize_state) {
      continue;
    }
    const auto state = states_.find(factor.state);
    const auto offset = layout.state_offsets.find(factor.state);
    if (state == states_.end() || offset == layout.state_offsets.end()) {
      continue;
    }
    const Eigen::VectorXd residual = stateLocalCoordinates(factor.mean, state->second);
    const Eigen::VectorXd inverse = inverseNoise(factor.noise);
    for (int row = 0; row < kStateDimension; ++row) {
      Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(kStateDimension);
      jacobian(row) = 1.0;
      add_dense_row(residual(row), 1.0 / inverse(row), 0.0, {{offset->second, jacobian}});
    }
  }

  for (const auto & factor : imu_factors_) {
    if (marginalize_state.has_value() && factor.from != *marginalize_state &&
      factor.to != *marginalize_state)
    {
      continue;
    }
    const auto from = states_.find(factor.from);
    const auto to = states_.find(factor.to);
    const auto from_offset = layout.state_offsets.find(factor.from);
    const auto to_offset = layout.state_offsets.find(factor.to);
    if (from == states_.end() || to == states_.end() ||
      from_offset == layout.state_offsets.end() || to_offset == layout.state_offsets.end())
    {
      continue;
    }
    const auto predicted = propagateImuState(
      from->second, factor.samples, factor.config.integration);
    if (!predicted.has_value()) {
      ++diagnostics_.rejected_factors;
      continue;
    }
    const auto residual = imuResidualFromPrediction(from->second, to->second, *predicted);
    if (!residual.has_value()) {
      ++diagnostics_.rejected_factors;
      continue;
    }
    Eigen::MatrixXd from_jacobian(kStateDimension, kStateDimension);
    Eigen::MatrixXd to_jacobian(kStateDimension, kStateDimension);
    for (int column = 0; column < kStateDimension; ++column) {
      const double epsilon = perturbationStep(column);
      Eigen::VectorXd delta = Eigen::VectorXd::Zero(kStateDimension);
      delta(column) = epsilon;
      const EcefState from_plus_state = perturbedState(from->second, delta);
      const EcefState from_minus_state = perturbedState(from->second, -delta);
      const auto from_plus_prediction = propagateImuState(
        from_plus_state, factor.samples, factor.config.integration);
      const auto from_minus_prediction = propagateImuState(
        from_minus_state, factor.samples, factor.config.integration);
      const auto from_plus = from_plus_prediction.has_value() ?
        imuResidualFromPrediction(from_plus_state, to->second, *from_plus_prediction) :
        std::nullopt;
      const auto from_minus = from_minus_prediction.has_value() ?
        imuResidualFromPrediction(from_minus_state, to->second, *from_minus_prediction) :
        std::nullopt;
      const auto to_plus = imuResidualFromPrediction(
        from->second, perturbedState(to->second, delta), *predicted);
      const auto to_minus = imuResidualFromPrediction(
        from->second, perturbedState(to->second, -delta), *predicted);
      if (!from_plus || !from_minus || !to_plus || !to_minus) {
        from_jacobian.col(column).setZero();
        to_jacobian.col(column).setZero();
      } else {
        from_jacobian.col(column) = (*from_plus - *from_minus) / (2.0 * epsilon);
        to_jacobian.col(column) = (*to_plus - *to_minus) / (2.0 * epsilon);
      }
    }
    const Eigen::VectorXd inverse = inverseNoise(factor.config.noise);
    for (int row = 0; row < kStateDimension; ++row) {
      add_dense_row(
        (*residual)(row), 1.0 / inverse(row), 0.0,
        {{from_offset->second, from_jacobian.row(row).transpose()},
          {to_offset->second, to_jacobian.row(row).transpose()}});
    }
  }

  for (const auto & batch : lidar_factors_) {
    if (marginalize_state.has_value() && batch.state != *marginalize_state) {
      continue;
    }
    const auto state = states_.find(batch.state);
    const auto offset = layout.state_offsets.find(batch.state);
    if (state == states_.end() || offset == layout.state_offsets.end()) {
      continue;
    }
    const RigidPose pose{state->second.orientation_ecef_body, state->second.position_ecef_m};
    for (const auto & source : batch.planes) {
      PointToPlaneFactor factor = source;
      factor.point_lidar = transformPoint(batch.body_lidar, source.point_lidar);
      const auto evaluation = evaluatePointToPlane(factor, pose);
      if (!evaluation.has_value()) {
        continue;
      }
      const PoseJacobianRow pose_jacobian = independentPoseJacobian(
        evaluation->jacobian, pose.rotation, factor.point_lidar);
      Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(kStateDimension);
      for (int column = 0; column < 6; ++column) {
        jacobian(column) = pose_jacobian[static_cast<std::size_t>(column)];
      }
      add_dense_row(
        evaluation->residual, lidar_config_.plane_sigma_m,
        lidar_config_.huber_delta_sigma, {{offset->second, jacobian}});
    }
    for (const auto & source : batch.lines) {
      PointToLineFactor factor = source;
      factor.point_lidar = transformPoint(batch.body_lidar, source.point_lidar);
      const auto evaluation = evaluatePointToLine(factor, pose);
      if (!evaluation.has_value()) {
        continue;
      }
      for (int row = 0; row < 2; ++row) {
        const PoseJacobianRow pose_jacobian = independentPoseJacobian(
          evaluation->jacobian[static_cast<std::size_t>(row)], pose.rotation,
          factor.point_lidar);
        Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(kStateDimension);
        for (int column = 0; column < 6; ++column) {
          jacobian(column) = pose_jacobian[static_cast<std::size_t>(column)];
        }
        add_dense_row(
          evaluation->residual[static_cast<std::size_t>(row)],
          lidar_config_.line_sigma_m, lidar_config_.huber_delta_sigma,
          {{offset->second, jacobian}});
      }
    }
  }

  for (const auto & batch : gnss_factors_) {
    if (marginalize_state.has_value() && batch.state != *marginalize_state) {
      continue;
    }
    const auto state = states_.find(batch.state);
    const auto state_offset = layout.state_offsets.find(batch.state);
    if (state == states_.end() || state_offset == layout.state_offsets.end()) {
      continue;
    }
    using CorrelationGroup = std::pair<SignalGroup, SatelliteId>;
    std::map<CorrelationGroup, std::vector<const DoubleDifferenceMeasurement *>> code_groups;
    std::map<CorrelationGroup, std::vector<const DoubleDifferenceMeasurement *>> carrier_groups;
    for (const auto & measurement : batch.measurements) {
      const CorrelationGroup group{measurement.group, measurement.reference};
      if (measurement.code_valid) {
        code_groups[group].push_back(&measurement);
      }
      if (measurement.carrier_valid) {
        carrier_groups[group].push_back(&measurement);
      }
    }
    const auto add_group = [&](const auto & entries, const bool carrier) {
        const int count = static_cast<int>(entries.size());
        Eigen::VectorXd residuals(count);
        Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(count, count);
        const int ambiguity_columns = carrier ? count : 0;
        Eigen::MatrixXd local_jacobian = Eigen::MatrixXd::Zero(
          count, 6 + ambiguity_columns);
        std::vector<int> scalar_offsets;
        scalar_offsets.reserve(static_cast<std::size_t>(6 + ambiguity_columns));
        for (int column = 0; column < 6; ++column) {
          scalar_offsets.push_back(state_offset->second + column);
        }
        for (int row = 0; row < count; ++row) {
          const DoubleDifferenceMeasurement & measurement =
            *entries[static_cast<std::size_t>(row)];
          std::optional<DdFactorEvaluation> evaluation;
          if (carrier) {
            const auto ambiguity = ambiguities_.find(measurement.ambiguity_key);
            const auto ambiguity_offset = layout.ambiguity_offsets.find(measurement.ambiguity_key);
            if (ambiguity == ambiguities_.end() ||
              ambiguity_offset == layout.ambiguity_offsets.end())
            {
              return;
            }
            evaluation = evaluateDdCarrier(measurement, state->second, ambiguity->second);
            if (evaluation.has_value()) {
              local_jacobian(row, 6 + row) = evaluation->ambiguity_jacobian;
              scalar_offsets.push_back(ambiguity_offset->second);
            }
          } else {
            evaluation = evaluateDdPseudorange(measurement, state->second);
          }
          if (!evaluation.has_value()) {
            return;
          }
          residuals(row) = evaluation->residual_m;
          for (int column = 0; column < 6; ++column) {
            local_jacobian(row, column) =
              evaluation->pose_jacobian[static_cast<std::size_t>(column)];
          }
          double target_variance = carrier ?
            measurement.carrier_target_variance_m2 : measurement.code_target_variance_m2;
          double reference_variance = carrier ?
            measurement.carrier_reference_variance_m2 : measurement.code_reference_variance_m2;
          if (!std::isfinite(target_variance) || !std::isfinite(reference_variance) ||
            target_variance <= 0.0 || reference_variance <= 0.0)
          {
            const double sigma = carrier ? measurement.carrier_sigma_m : measurement.code_sigma_m;
            target_variance = sigma * sigma;
            reference_variance = 0.0;
          }
          covariance(row, row) = target_variance + reference_variance;
          for (int column = 0; column < row; ++column) {
            const DoubleDifferenceMeasurement & other =
              *entries[static_cast<std::size_t>(column)];
            double other_reference_variance = carrier ?
              other.carrier_reference_variance_m2 : other.code_reference_variance_m2;
            if (!std::isfinite(other_reference_variance) || other_reference_variance <= 0.0) {
              other_reference_variance = 0.0;
            }
            const double shared_reference_covariance = std::sqrt(
              std::max(0.0, reference_variance) *
              std::max(0.0, other_reference_variance));
            covariance(row, column) = shared_reference_covariance;
            covariance(column, row) = shared_reference_covariance;
          }
        }
        add_correlated_rows(
          residuals, covariance, local_jacobian, scalar_offsets,
          carrier ? gnss_config_.carrier_huber_delta_sigma :
          gnss_config_.code_huber_delta_sigma);
      };
    for (const auto & group : code_groups) {
      add_group(group.second, false);
    }
    for (const auto & group : carrier_groups) {
      add_group(group.second, true);
    }
  }
  system.hessian = 0.5 * (system.hessian + system.hessian.transpose());
  return system;
}

bool FloatFixedLagSmoother::applyDelta(
  const VariableLayout & layout,
  const Eigen::VectorXd & delta)
{
  if (delta.size() != layout.dimension || !delta.allFinite()) {
    return false;
  }
  auto candidate_states = states_;
  auto candidate_ambiguities = ambiguities_;
  for (const auto & state_offset : layout.state_offsets) {
    EcefState candidate = perturbedState(
      candidate_states.at(state_offset.first),
      delta.segment(state_offset.second, kStateDimension));
    if (!validState(candidate)) {
      return false;
    }
    candidate_states[state_offset.first] = candidate;
  }
  for (const auto & ambiguity_offset : layout.ambiguity_offsets) {
    const double candidate = candidate_ambiguities.at(ambiguity_offset.first) +
      delta(ambiguity_offset.second);
    if (!std::isfinite(candidate)) {
      return false;
    }
    candidate_ambiguities[ambiguity_offset.first] = candidate;
  }
  states_ = std::move(candidate_states);
  ambiguities_ = std::move(candidate_ambiguities);
  return true;
}

bool FloatFixedLagSmoother::optimize()
{
  invalidateFixLinearization();
  ++diagnostics_.optimization_calls;
  diagnostics_.last_solve_succeeded = false;
  if (states_.empty()) {
    return false;
  }
  const VariableLayout layout = createLayout();
  double damping = config_.initial_damping;
  for (std::size_t iteration = 0; iteration < config_.maximum_iterations; ++iteration) {
    LinearSystem system = buildLinearSystem(layout);
    if (system.rows == 0U || !system.hessian.allFinite() || !system.gradient.allFinite()) {
      return false;
    }
    Eigen::MatrixXd damped = system.hessian;
    damped.diagonal().array() += damping;
    Eigen::LDLT<Eigen::MatrixXd> solver(damped);
    if (solver.info() != Eigen::Success) {
      damping *= 10.0;
      continue;
    }
    const Eigen::VectorXd delta = solver.solve(-system.gradient);
    if (solver.info() != Eigen::Success || !delta.allFinite()) {
      damping *= 10.0;
      continue;
    }
    const auto previous_states = states_;
    const auto previous_ambiguities = ambiguities_;
    if (!applyDelta(layout, delta)) {
      return false;
    }
    const LinearSystem candidate_system = buildLinearSystem(layout);
    const double cost_tolerance = 1.0e-9 * std::max(1.0, std::abs(system.cost));
    if (!std::isfinite(candidate_system.cost) ||
      candidate_system.cost > system.cost + cost_tolerance)
    {
      states_ = previous_states;
      ambiguities_ = previous_ambiguities;
      ++diagnostics_.optimization_rollbacks;
      damping *= 10.0;
      continue;
    }
    damping = std::max(config_.initial_damping, 0.3 * damping);
    diagnostics_.last_iterations = iteration + 1U;
    diagnostics_.last_residual_rows = candidate_system.rows;
    diagnostics_.last_cost = candidate_system.cost;
    diagnostics_.last_delta_norm = delta.norm();
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(damped,
      Eigen::EigenvaluesOnly);
    if (eigen_solver.info() == Eigen::Success) {
      const double minimum = std::max(eigen_solver.eigenvalues().minCoeff(), 1.0e-18);
      diagnostics_.last_condition_estimate = eigen_solver.eigenvalues().maxCoeff() / minimum;
    }
    diagnostics_.last_solve_succeeded = true;
    if (delta.norm() < config_.convergence_delta_norm) {
      break;
    }
  }
  if (!diagnostics_.last_solve_succeeded) {
    return false;
  }
  if (!marginalizeOldestIfNeeded()) {
    return false;
  }
  refreshDiagnostics();
  return true;
}

std::optional<Eigen::MatrixXd> FloatFixedLagSmoother::linearizedCovariance(
  const LinearSystem & system) const
{
  if (system.rows == 0U || system.hessian.rows() == 0 ||
    system.hessian.rows() != system.hessian.cols() ||
    !system.hessian.allFinite())
  {
    return std::nullopt;
  }
  Eigen::MatrixXd information = 0.5 * (system.hessian + system.hessian.transpose());
  information.diagonal().array() += std::max(1.0e-12, config_.initial_damping * 1.0e-3);
  Eigen::LDLT<Eigen::MatrixXd> solver(information);
  if (solver.info() != Eigen::Success || (solver.vectorD().array() <= 0.0).any()) {
    return std::nullopt;
  }
  Eigen::MatrixXd covariance = solver.solve(
    Eigen::MatrixXd::Identity(system.hessian.rows(), system.hessian.cols()));
  if (solver.info() != Eigen::Success || !covariance.allFinite()) {
    return std::nullopt;
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  return covariance;
}

bool FloatFixedLagSmoother::prepareFixLinearization()
{
  if (fix_layout_cache_.has_value() && fix_system_cache_.has_value() &&
    fix_covariance_cache_.has_value())
  {
    return true;
  }
  VariableLayout layout = createLayout();
  LinearSystem system = buildLinearSystem(layout);
  const auto covariance = linearizedCovariance(system);
  if (!covariance.has_value()) {
    invalidateFixLinearization();
    return false;
  }
  fix_layout_cache_ = std::move(layout);
  fix_system_cache_ = std::move(system);
  fix_covariance_cache_ = *covariance;
  return true;
}

void FloatFixedLagSmoother::invalidateFixLinearization() noexcept
{
  fix_layout_cache_.reset();
  fix_system_cache_.reset();
  fix_covariance_cache_.reset();
}

bool FloatFixedLagSmoother::marginalizeOldestIfNeeded()
{
  while (state_order_.size() > 1U) {
    const double elapsed = states_.at(state_order_.back()).stamp_s -
      states_.at(state_order_.front()).stamp_s;
    if (state_order_.size() <= config_.maximum_states && elapsed <= config_.duration_s) {
      break;
    }
    if (!marginalizeOldest()) {
      return false;
    }
  }
  return true;
}

bool FloatFixedLagSmoother::marginalizeOldest()
{
  if (state_order_.size() < 2U) {
    return true;
  }
  const StateId oldest = state_order_.front();
  const VariableLayout layout = createLayout();
  const LinearSystem system = buildLinearSystem(layout, oldest);
  std::set<DdAmbiguityKey> retained_ambiguities;
  for (const auto & batch : gnss_factors_) {
    if (batch.state == oldest) {
      continue;
    }
    for (const auto & measurement : batch.measurements) {
      if (measurement.carrier_valid) {
        retained_ambiguities.insert(measurement.ambiguity_key);
      }
    }
  }

  std::vector<int> eliminate;
  std::vector<int> retain;
  const int oldest_offset = layout.state_offsets.at(oldest);
  for (int scalar = 0; scalar < layout.dimension; ++scalar) {
    bool remove = scalar >= oldest_offset && scalar < oldest_offset + kStateDimension;
    if (!remove) {
      for (const auto & ambiguity : layout.ambiguity_offsets) {
        if (scalar == ambiguity.second && retained_ambiguities.find(ambiguity.first) ==
          retained_ambiguities.end())
        {
          remove = true;
          break;
        }
      }
    }
    (remove ? eliminate : retain).push_back(scalar);
  }
  if (eliminate.empty() || retain.empty()) {
    return false;
  }

  Eigen::MatrixXd h_ee(eliminate.size(), eliminate.size());
  Eigen::MatrixXd h_er(eliminate.size(), retain.size());
  Eigen::MatrixXd h_rr(retain.size(), retain.size());
  Eigen::VectorXd g_e(eliminate.size());
  Eigen::VectorXd g_r(retain.size());
  for (std::size_t row = 0; row < eliminate.size(); ++row) {
    g_e(static_cast<int>(row)) = system.gradient(eliminate[row]);
    for (std::size_t column = 0; column < eliminate.size(); ++column) {
      h_ee(static_cast<int>(row), static_cast<int>(column)) =
        system.hessian(eliminate[row], eliminate[column]);
    }
    for (std::size_t column = 0; column < retain.size(); ++column) {
      h_er(static_cast<int>(row), static_cast<int>(column)) =
        system.hessian(eliminate[row], retain[column]);
    }
  }
  for (std::size_t row = 0; row < retain.size(); ++row) {
    g_r(static_cast<int>(row)) = system.gradient(retain[row]);
    for (std::size_t column = 0; column < retain.size(); ++column) {
      h_rr(static_cast<int>(row), static_cast<int>(column)) =
        system.hessian(retain[row], retain[column]);
    }
  }
  h_ee.diagonal().array() += config_.initial_damping;
  Eigen::LDLT<Eigen::MatrixXd> solver(h_ee);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  const Eigen::MatrixXd solved_cross = solver.solve(h_er);
  const Eigen::VectorXd solved_gradient = solver.solve(g_e);
  if (!solved_cross.allFinite() || !solved_gradient.allFinite()) {
    return false;
  }

  DenseMarginalPrior prior;
  prior.hessian = h_rr - h_er.transpose() * solved_cross;
  prior.gradient = g_r - h_er.transpose() * solved_gradient;
  prior.hessian = 0.5 * (prior.hessian + prior.hessian.transpose());
  for (const MarginalVariable & variable : layout.variables) {
    if (variable.kind == MarginalVariable::Kind::State) {
      if (variable.state != oldest) {
        prior.variables.push_back(variable);
        prior.state_anchors[variable.state] = states_.at(variable.state);
      }
    } else if (retained_ambiguities.find(variable.ambiguity) != retained_ambiguities.end()) {
      prior.variables.push_back(variable);
      prior.ambiguity_anchors[variable.ambiguity] = ambiguities_.at(variable.ambiguity);
    }
  }
  marginal_prior_ = std::move(prior);

  states_.erase(oldest);
  state_order_.erase(state_order_.begin());
  state_priors_.erase(
    std::remove_if(
      state_priors_.begin(), state_priors_.end(),
      [oldest](const StatePriorFactor & factor) {return factor.state == oldest;}),
    state_priors_.end());
  imu_factors_.erase(
    std::remove_if(
      imu_factors_.begin(), imu_factors_.end(),
      [oldest](const ImuFactor & factor) {
        return factor.from == oldest || factor.to == oldest;
      }), imu_factors_.end());
  lidar_factors_.erase(
    std::remove_if(
      lidar_factors_.begin(), lidar_factors_.end(),
      [oldest](const LidarFactorBatch & factor) {return factor.state == oldest;}),
    lidar_factors_.end());
  gnss_factors_.erase(
    std::remove_if(
      gnss_factors_.begin(), gnss_factors_.end(),
      [oldest](const GnssFactorBatch & factor) {return factor.state == oldest;}),
    gnss_factors_.end());
  for (auto iterator = ambiguities_.begin(); iterator != ambiguities_.end(); ) {
    if (retained_ambiguities.find(iterator->first) == retained_ambiguities.end()) {
      ambiguity_last_state_.erase(iterator->first);
      ambiguity_observation_counts_.erase(iterator->first);
      iterator = ambiguities_.erase(iterator);
    } else {
      ++iterator;
    }
  }
  ++diagnostics_.marginalizations;
  return true;
}

const EcefState * FloatFixedLagSmoother::state(const StateId id) const noexcept
{
  const auto iterator = states_.find(id);
  return iterator == states_.end() ? nullptr : &iterator->second;
}

std::optional<double> FloatFixedLagSmoother::ambiguity(const DdAmbiguityKey & key) const
{
  const auto iterator = ambiguities_.find(key);
  if (iterator == ambiguities_.end()) {
    return std::nullopt;
  }
  return iterator->second;
}

std::vector<CodeResidualDiagnostics> FloatFixedLagSmoother::codeResidualDiagnostics() const
{
  std::map<SignalGroup, CodeResidualDiagnostics> summaries;
  if (gnss_factors_.empty()) {
    return {};
  }
  const GnssFactorBatch & batch = gnss_factors_.back();
  const auto state_iterator = states_.find(batch.state);
  if (state_iterator == states_.end()) {
    return {};
  }
  for (const DoubleDifferenceMeasurement & measurement : batch.measurements) {
    if (!measurement.code_valid) {
      continue;
    }
    const auto evaluation = evaluateDdPseudorange(measurement, state_iterator->second);
    const double sigma = measurement.code_sigma_m;
    if (!evaluation.has_value() || !std::isfinite(sigma) || sigma <= 0.0) {
      continue;
    }
    const double raw = std::abs(evaluation->residual_m);
    const double normalized = raw / sigma;
    CodeResidualDiagnostics & summary = summaries[measurement.group];
    if (summary.factors == 0U) {
      summary.group = measurement.group;
      summary.sigma_min_m = sigma;
    }
    ++summary.factors;
    summary.raw_rms_m += raw * raw;
    summary.raw_max_m = std::max(summary.raw_max_m, raw);
    summary.normalized_rms += normalized * normalized;
    summary.normalized_max = std::max(summary.normalized_max, normalized);
    summary.sigma_mean_m += sigma;
    summary.sigma_min_m = std::min(summary.sigma_min_m, sigma);
    summary.sigma_max_m = std::max(summary.sigma_max_m, sigma);
  }
  std::vector<CodeResidualDiagnostics> output;
  output.reserve(summaries.size());
  for (auto & entry : summaries) {
    CodeResidualDiagnostics & summary = entry.second;
    const double count = static_cast<double>(summary.factors);
    summary.raw_rms_m = std::sqrt(summary.raw_rms_m / count);
    summary.normalized_rms = std::sqrt(summary.normalized_rms / count);
    summary.sigma_mean_m /= count;
    output.push_back(summary);
  }
  return output;
}

std::vector<CarrierResidualDiagnostics> FloatFixedLagSmoother::carrierResidualDiagnostics(
  const std::size_t minimum_observation_epochs) const
{
  std::map<SignalGroup, CarrierResidualDiagnostics> summaries;
  if (gnss_factors_.empty()) {
    return {};
  }
  const GnssFactorBatch & batch = gnss_factors_.back();
  const auto state_iterator = states_.find(batch.state);
  if (state_iterator == states_.end()) {
    return {};
  }
  for (const DoubleDifferenceMeasurement & measurement : batch.measurements) {
    if (!measurement.carrier_valid) {
      continue;
    }
    const auto ambiguity_iterator = ambiguities_.find(measurement.ambiguity_key);
    if (ambiguity_iterator == ambiguities_.end()) {
      continue;
    }
    const auto evaluation = evaluateDdCarrier(
      measurement, state_iterator->second, ambiguity_iterator->second);
    const double sigma = measurement.carrier_sigma_m;
    if (!evaluation.has_value() || !std::isfinite(sigma) || sigma <= 0.0) {
      continue;
    }
    const double raw = std::abs(evaluation->residual_m);
    const double normalized = raw / sigma;
    const auto observation_iterator =
      ambiguity_observation_counts_.find(measurement.ambiguity_key);
    const std::size_t observations = observation_iterator == ambiguity_observation_counts_.end() ?
      0U : observation_iterator->second;
    CarrierResidualDiagnostics & summary = summaries[measurement.group];
    if (summary.factors == 0U) {
      summary.group = measurement.group;
      summary.minimum_arc_observations = observations;
      summary.sigma_min_m = sigma;
    }
    ++summary.factors;
    summary.fix_eligible_ambiguities +=
      static_cast<std::size_t>(observations >= minimum_observation_epochs);
    summary.minimum_arc_observations = std::min(
      summary.minimum_arc_observations, observations);
    summary.maximum_arc_observations = std::max(
      summary.maximum_arc_observations, observations);
    summary.raw_rms_m += raw * raw;
    summary.raw_max_m = std::max(summary.raw_max_m, raw);
    summary.normalized_rms += normalized * normalized;
    summary.normalized_max = std::max(summary.normalized_max, normalized);
    summary.sigma_mean_m += sigma;
    summary.sigma_min_m = std::min(summary.sigma_min_m, sigma);
    summary.sigma_max_m = std::max(summary.sigma_max_m, sigma);
  }
  std::vector<CarrierResidualDiagnostics> output;
  output.reserve(summaries.size());
  for (auto & entry : summaries) {
    CarrierResidualDiagnostics & summary = entry.second;
    const double count = static_cast<double>(summary.factors);
    summary.raw_rms_m = std::sqrt(summary.raw_rms_m / count);
    summary.normalized_rms = std::sqrt(summary.normalized_rms / count);
    summary.sigma_mean_m /= count;
    output.push_back(summary);
  }
  return output;
}

std::optional<FloatAmbiguityEstimate> FloatFixedLagSmoother::floatAmbiguityEstimate()
{
  if (ambiguities_.empty()) {
    return FloatAmbiguityEstimate{};
  }
  if (!prepareFixLinearization()) {
    return std::nullopt;
  }
  const VariableLayout & layout = *fix_layout_cache_;
  const Eigen::MatrixXd & covariance = *fix_covariance_cache_;
  FloatAmbiguityEstimate estimate;
  const int count = static_cast<int>(ambiguities_.size());
  estimate.values_m.resize(count);
  estimate.covariance_m2.resize(count, count);
  estimate.keys.reserve(ambiguities_.size());
  estimate.last_observed_state_ids.reserve(ambiguities_.size());
  estimate.observation_counts.reserve(ambiguities_.size());
  std::vector<int> scalar_offsets;
  scalar_offsets.reserve(ambiguities_.size());
  for (const auto & ambiguity_value : ambiguities_) {
    estimate.keys.push_back(ambiguity_value.first);
    const auto last_state = ambiguity_last_state_.find(ambiguity_value.first);
    estimate.last_observed_state_ids.push_back(
      last_state == ambiguity_last_state_.end() ? 0U : last_state->second);
    const auto observation_count = ambiguity_observation_counts_.find(ambiguity_value.first);
    estimate.observation_counts.push_back(
      observation_count == ambiguity_observation_counts_.end() ? 0U : observation_count->second);
    estimate.values_m(static_cast<int>(scalar_offsets.size())) = ambiguity_value.second;
    scalar_offsets.push_back(layout.ambiguity_offsets.at(ambiguity_value.first));
  }
  for (int row = 0; row < count; ++row) {
    for (int column = 0; column < count; ++column) {
      estimate.covariance_m2(row, column) =
        covariance(
        scalar_offsets[static_cast<std::size_t>(row)],
        scalar_offsets[static_cast<std::size_t>(column)]);
    }
  }
  if (!estimate.covariance_m2.allFinite()) {
    return std::nullopt;
  }
  return estimate;
}

const char * toString(const FixedBackSubstitutionRejection reason) noexcept
{
  switch (reason) {
    case FixedBackSubstitutionRejection::None: return "NONE";
    case FixedBackSubstitutionRejection::NotEvaluated: return "NOT_EVALUATED";
    case FixedBackSubstitutionRejection::InvalidInput: return "INVALID_INPUT";
    case FixedBackSubstitutionRejection::MissingAmbiguity: return "MISSING_AMBIGUITY";
    case FixedBackSubstitutionRejection::CovarianceUnavailable:
      return "COVARIANCE_UNAVAILABLE";
    case FixedBackSubstitutionRejection::CorrectionLimit: return "CORRECTION_LIMIT";
    case FixedBackSubstitutionRejection::CostIncrease: return "COST_INCREASE";
  }
  return "UNKNOWN";
}

FixedBackSubstitutionResult FloatFixedLagSmoother::previewFixedAmbiguities(
  const std::vector<DdAmbiguityKey> & keys,
  const Eigen::VectorXd & fixed_values_m,
  const FixedBackSubstitutionConfig & config)
{
  FixedBackSubstitutionResult output;
  if (keys.empty() || fixed_values_m.size() != static_cast<int>(keys.size()) ||
    !fixed_values_m.allFinite() ||
    !std::isfinite(config.maximum_position_correction_m) ||
    config.maximum_position_correction_m <= 0.0 ||
    !std::isfinite(config.maximum_rotation_correction_rad) ||
    config.maximum_rotation_correction_rad <= 0.0 ||
    !std::isfinite(config.maximum_velocity_correction_m_s) ||
    config.maximum_velocity_correction_m_s <= 0.0 ||
    !std::isfinite(config.maximum_cost_increase) || config.maximum_cost_increase < 0.0 ||
    state_order_.empty())
  {
    return output;
  }

  if (!prepareFixLinearization()) {
    output.rejection = FixedBackSubstitutionRejection::CovarianceUnavailable;
    return output;
  }
  const VariableLayout & layout = *fix_layout_cache_;
  std::vector<int> selected_offsets;
  selected_offsets.reserve(keys.size());
  for (const auto & key : keys) {
    const auto offset = layout.ambiguity_offsets.find(key);
    if (offset == layout.ambiguity_offsets.end()) {
      output.rejection = FixedBackSubstitutionRejection::MissingAmbiguity;
      return output;
    }
    selected_offsets.push_back(offset->second);
  }
  const LinearSystem & current_system = *fix_system_cache_;
  const Eigen::MatrixXd & covariance = *fix_covariance_cache_;

  const int selected_count = static_cast<int>(keys.size());
  Eigen::MatrixXd ambiguity_covariance(selected_count, selected_count);
  Eigen::MatrixXd covariance_cross(layout.dimension, selected_count);
  Eigen::VectorXd difference(selected_count);
  for (int row = 0; row < selected_count; ++row) {
    difference(row) = fixed_values_m(row) - ambiguities_.at(keys[static_cast<std::size_t>(row)]);
    covariance_cross.col(row) = covariance.col(
      selected_offsets[static_cast<std::size_t>(row)]);
    for (int column = 0; column < selected_count; ++column) {
      ambiguity_covariance(row, column) = covariance(
        selected_offsets[static_cast<std::size_t>(row)],
        selected_offsets[static_cast<std::size_t>(column)]);
    }
  }
  Eigen::LDLT<Eigen::MatrixXd> selected_solver(ambiguity_covariance);
  if (selected_solver.info() != Eigen::Success ||
    (selected_solver.vectorD().array() <= 0.0).any())
  {
    output.rejection = FixedBackSubstitutionRejection::CovarianceUnavailable;
    return output;
  }
  const Eigen::VectorXd correction = covariance_cross * selected_solver.solve(difference);
  if (selected_solver.info() != Eigen::Success || !correction.allFinite()) {
    output.rejection = FixedBackSubstitutionRejection::CovarianceUnavailable;
    return output;
  }

  auto candidate_states = states_;
  auto candidate_ambiguities = ambiguities_;
  for (const auto & state_offset : layout.state_offsets) {
    const Eigen::VectorXd state_correction = correction.segment(
      state_offset.second, kStateDimension);
    output.maximum_position_correction_m = std::max(
      output.maximum_position_correction_m, state_correction.segment(0, 3).norm());
    output.maximum_rotation_correction_rad = std::max(
      output.maximum_rotation_correction_rad, state_correction.segment(3, 3).norm());
    output.maximum_velocity_correction_m_s = std::max(
      output.maximum_velocity_correction_m_s, state_correction.segment(6, 3).norm());
    const EcefState candidate = perturbedState(
      candidate_states.at(state_offset.first), state_correction);
    if (!validState(candidate)) {
      output.rejection = FixedBackSubstitutionRejection::InvalidInput;
      return output;
    }
    candidate_states[state_offset.first] = candidate;
  }
  for (const auto & ambiguity_offset : layout.ambiguity_offsets) {
    candidate_ambiguities[ambiguity_offset.first] += correction(ambiguity_offset.second);
  }
  for (int index = 0; index < selected_count; ++index) {
    candidate_ambiguities[keys[static_cast<std::size_t>(index)]] = fixed_values_m(index);
  }

  output.cost_before = current_system.cost;
  const auto saved_states = states_;
  const auto saved_ambiguities = ambiguities_;
  const auto saved_diagnostics = diagnostics_;
  states_ = candidate_states;
  ambiguities_ = candidate_ambiguities;
  const LinearSystem candidate_system = buildLinearSystem(layout);
  states_ = saved_states;
  ambiguities_ = saved_ambiguities;
  diagnostics_ = saved_diagnostics;
  output.cost_after = candidate_system.cost;

  if (!std::isfinite(output.cost_before) || !std::isfinite(output.cost_after)) {
    output.rejection = FixedBackSubstitutionRejection::InvalidInput;
    return output;
  }
  if (output.maximum_position_correction_m > config.maximum_position_correction_m ||
    output.maximum_rotation_correction_rad > config.maximum_rotation_correction_rad ||
    output.maximum_velocity_correction_m_s > config.maximum_velocity_correction_m_s)
  {
    output.rejection = FixedBackSubstitutionRejection::CorrectionLimit;
    return output;
  }
  if (output.cost_after - output.cost_before > config.maximum_cost_increase) {
    output.rejection = FixedBackSubstitutionRejection::CostIncrease;
    return output;
  }
  output.accepted = true;
  output.rejection = FixedBackSubstitutionRejection::None;
  output.state_id = state_order_.back();
  output.latest_state = candidate_states.at(output.state_id);
  return output;
}

std::size_t FloatFixedLagSmoother::factorCount() const noexcept
{
  std::size_t count = state_priors_.size() + imu_factors_.size();
  for (const auto & batch : lidar_factors_) {
    count += batch.planes.size() + 2U * batch.lines.size();
  }
  for (const auto & batch : gnss_factors_) {
    for (const auto & measurement : batch.measurements) {
      count += static_cast<std::size_t>(measurement.code_valid) +
        static_cast<std::size_t>(measurement.carrier_valid);
    }
  }
  return count;
}

void FloatFixedLagSmoother::refreshDiagnostics()
{
  diagnostics_.states = states_.size();
  diagnostics_.ambiguities = ambiguities_.size();
  diagnostics_.factors = factorCount();
  diagnostics_.state_prior_factors = state_priors_.size();
  diagnostics_.imu_factors = imu_factors_.size();
  diagnostics_.lidar_line_factors = 0U;
  diagnostics_.lidar_plane_factors = 0U;
  diagnostics_.gnss_code_factors = 0U;
  diagnostics_.gnss_carrier_factors = 0U;
  for (const auto & batch : lidar_factors_) {
    diagnostics_.lidar_line_factors += batch.lines.size();
    diagnostics_.lidar_plane_factors += batch.planes.size();
  }
  for (const auto & batch : gnss_factors_) {
    for (const auto & measurement : batch.measurements) {
      diagnostics_.gnss_code_factors += static_cast<std::size_t>(measurement.code_valid);
      diagnostics_.gnss_carrier_factors += static_cast<std::size_t>(measurement.carrier_valid);
    }
  }
  diagnostics_.window_span_s = state_order_.size() < 2U ? 0.0 :
    states_.at(state_order_.back()).stamp_s - states_.at(state_order_.front()).stamp_s;
}

}  // namespace fgo_gil_localizer

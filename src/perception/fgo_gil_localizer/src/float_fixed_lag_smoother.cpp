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

std::optional<Eigen::VectorXd> evaluateImuResidual(
  const EcefState & from,
  const EcefState & to,
  const std::vector<ImuSample> & samples,
  const EcefImuConfig & config)
{
  if (samples.size() < 2U) {
    return std::nullopt;
  }
  EcefImuPreintegrator preintegrator(config);
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
  const EcefState & predicted = preintegrator.state();
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
  lidar_factors_.push_back({state, line_factors, plane_factors, body_lidar});
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
      const auto evaluation = evaluateDdCarrier(measurement, state_iterator->second, 0.0);
      if (evaluation.has_value()) {
        ambiguities_[measurement.ambiguity_key] = -evaluation->residual_m;
      }
    }
  }
  if (!any_valid) {
    recordGnssOutage();
    return true;
  }
  gnss_factors_.push_back({state, measurements});
  refreshDiagnostics();
  return true;
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
    const auto residual = evaluateImuResidual(
      from->second, to->second, factor.samples, factor.config.integration);
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
      const auto from_plus = evaluateImuResidual(
        perturbedState(from->second, delta), to->second, factor.samples,
        factor.config.integration);
      const auto from_minus = evaluateImuResidual(
        perturbedState(from->second, -delta), to->second, factor.samples,
        factor.config.integration);
      const auto to_plus = evaluateImuResidual(
        from->second, perturbedState(to->second, delta), factor.samples,
        factor.config.integration);
      const auto to_minus = evaluateImuResidual(
        from->second, perturbedState(to->second, -delta), factor.samples,
        factor.config.integration);
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
      Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(kStateDimension);
      for (int column = 0; column < 6; ++column) {
        jacobian(column) = evaluation->jacobian[static_cast<std::size_t>(column)];
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
        Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(kStateDimension);
        for (int column = 0; column < 6; ++column) {
          jacobian(column) = evaluation->jacobian[static_cast<std::size_t>(row)]
            [static_cast<std::size_t>(column)];
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
    for (const auto & measurement : batch.measurements) {
      if (measurement.code_valid) {
        const auto evaluation = evaluateDdPseudorange(measurement, state->second);
        if (evaluation.has_value()) {
          Eigen::VectorXd jacobian = Eigen::VectorXd::Zero(kStateDimension);
          for (int column = 0; column < 6; ++column) {
            jacobian(column) = evaluation->pose_jacobian[static_cast<std::size_t>(column)];
          }
          add_dense_row(
            evaluation->residual_m, measurement.code_sigma_m,
            gnss_config_.code_huber_delta_sigma, {{state_offset->second, jacobian}});
        }
      }
      if (measurement.carrier_valid) {
        const auto ambiguity = ambiguities_.find(measurement.ambiguity_key);
        const auto ambiguity_offset = layout.ambiguity_offsets.find(measurement.ambiguity_key);
        if (ambiguity == ambiguities_.end() || ambiguity_offset == layout.ambiguity_offsets.end()) {
          continue;
        }
        const auto evaluation = evaluateDdCarrier(
          measurement, state->second, ambiguity->second);
        if (evaluation.has_value()) {
          Eigen::VectorXd state_jacobian = Eigen::VectorXd::Zero(kStateDimension);
          for (int column = 0; column < 6; ++column) {
            state_jacobian(column) =
              evaluation->pose_jacobian[static_cast<std::size_t>(column)];
          }
          Eigen::VectorXd ambiguity_jacobian(1);
          ambiguity_jacobian(0) = evaluation->ambiguity_jacobian;
          add_dense_row(
            evaluation->residual_m, measurement.carrier_sigma_m,
            gnss_config_.carrier_huber_delta_sigma,
            {{state_offset->second, state_jacobian},
              {ambiguity_offset->second, ambiguity_jacobian}});
        }
      }
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
}

}  // namespace fgo_gil_localizer

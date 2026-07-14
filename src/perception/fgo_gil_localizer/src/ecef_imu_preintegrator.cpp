#include "fgo_gil_localizer/ecef_imu_preintegrator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace fgo_gil_localizer
{

EcefImuPreintegrator::EcefImuPreintegrator(EcefImuConfig config)
: config_(config)
{
  if (!std::isfinite(config_.maximum_step_s) || config_.maximum_step_s <= 0.0 ||
    !std::isfinite(config_.earth_rotation_rad_s) || config_.earth_rotation_rad_s < 0.0 ||
    !std::isfinite(config_.gravitational_parameter_m3_s2) ||
    config_.gravitational_parameter_m3_s2 < 0.0 ||
    !std::isfinite(config_.minimum_radius_m) || config_.minimum_radius_m <= 0.0 ||
    !std::isfinite(config_.accelerometer_bias_perturbation_m_s2) ||
    config_.accelerometer_bias_perturbation_m_s2 <= 0.0 ||
    !std::isfinite(config_.gyroscope_bias_perturbation_rad_s) ||
    config_.gyroscope_bias_perturbation_rad_s <= 0.0)
  {
    throw std::invalid_argument("ECEF IMU configuration is outside valid bounds");
  }
}

bool EcefImuPreintegrator::reset(const EcefState & initial_state)
{
  previous_measurement_.reset();
  reset_ = validState(initial_state);
  valid_ = reset_;
  if (!reset_) {
    ++diagnostics_.invalid_states;
    return false;
  }
  states_.fill(initial_state);
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    states_[1U + axis].accelerometer_bias_m_s2[axis] +=
      config_.accelerometer_bias_perturbation_m_s2;
    states_[4U + axis].gyroscope_bias_rad_s[axis] +=
      config_.gyroscope_bias_perturbation_rad_s;
  }
  return true;
}

ImuIntegrationResult EcefImuPreintegrator::integrate(const ImuSample & measurement)
{
  if (!reset_ || !valid_) {
    return ImuIntegrationResult::RejectedNotReset;
  }
  if (!std::isfinite(measurement.stamp_s) || !finite(measurement.acceleration_m_s2) ||
    !finite(measurement.angular_velocity_rad_s))
  {
    valid_ = false;
    ++diagnostics_.nonfinite;
    return ImuIntegrationResult::RejectedNonFinite;
  }
  if (!previous_measurement_.has_value()) {
    if (measurement.stamp_s < states_[0].stamp_s) {
      valid_ = false;
      ++diagnostics_.time_reversals;
      return ImuIntegrationResult::RejectedTimeReversal;
    }
    previous_measurement_ = measurement;
    for (auto & state : states_) {
      state.stamp_s = measurement.stamp_s;
    }
    return ImuIntegrationResult::Initialized;
  }

  const double delta_s = measurement.stamp_s - previous_measurement_->stamp_s;
  if (delta_s == 0.0) {
    ++diagnostics_.duplicates;
    return ImuIntegrationResult::RejectedDuplicate;
  }
  if (delta_s < 0.0) {
    valid_ = false;
    ++diagnostics_.time_reversals;
    return ImuIntegrationResult::RejectedTimeReversal;
  }
  if (delta_s > config_.maximum_step_s) {
    valid_ = false;
    ++diagnostics_.gaps;
    return ImuIntegrationResult::RejectedGap;
  }

  ImuSample midpoint = measurement;
  midpoint.acceleration_m_s2 =
    0.5 * (previous_measurement_->acceleration_m_s2 + measurement.acceleration_m_s2);
  midpoint.angular_velocity_rad_s =
    0.5 * (previous_measurement_->angular_velocity_rad_s + measurement.angular_velocity_rad_s);
  for (auto & state : states_) {
    if (!propagate(state, midpoint, delta_s)) {
      valid_ = false;
      ++diagnostics_.invalid_states;
      return ImuIntegrationResult::RejectedInvalidState;
    }
    state.stamp_s = measurement.stamp_s;
  }
  previous_measurement_ = measurement;
  ++diagnostics_.integrated_intervals;
  diagnostics_.integrated_duration_s += delta_s;
  return ImuIntegrationResult::Integrated;
}

BiasJacobian EcefImuPreintegrator::biasJacobian() const
{
  BiasJacobian jacobian{};
  if (!reset_ || !valid_) {
    return jacobian;
  }
  const EcefState & nominal = states_[0];
  for (std::size_t column = 0; column < 6U; ++column) {
    const EcefState & perturbed = states_[1U + column];
    const double epsilon = column < 3U ?
      config_.accelerometer_bias_perturbation_m_s2 :
      config_.gyroscope_bias_perturbation_rad_s;
    const Vec3 position_delta = (perturbed.position_ecef_m - nominal.position_ecef_m) / epsilon;
    const Vec3 velocity_delta = (perturbed.velocity_ecef_m_s - nominal.velocity_ecef_m_s) / epsilon;
    const Quaternion orientation_error =
      nominal.orientation_ecef_body.conjugate() * perturbed.orientation_ecef_body;
    const Vec3 orientation_delta = quaternionLog(orientation_error) / epsilon;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      jacobianAt(jacobian, axis, column) = position_delta[axis];
      jacobianAt(jacobian, 3U + axis, column) = velocity_delta[axis];
      jacobianAt(jacobian, 6U + axis, column) = orientation_delta[axis];
    }
  }
  return jacobian;
}

bool EcefImuPreintegrator::propagate(
  EcefState & state,
  const ImuSample & measurement,
  const double delta_s) const
{
  if (!validState(state)) {
    return false;
  }
  const Vec3 corrected_acceleration =
    measurement.acceleration_m_s2 - state.accelerometer_bias_m_s2;
  const Vec3 corrected_angular_velocity =
    measurement.angular_velocity_rad_s - state.gyroscope_bias_rad_s;
  const Vec3 earth_rate_ecef{0.0, 0.0, config_.earth_rotation_rad_s};

  const Quaternion earth_half = quaternionFromRotationVector(-0.5 * delta_s * earth_rate_ecef);
  const Quaternion body_half =
    quaternionFromRotationVector(0.5 * delta_s * corrected_angular_velocity);
  const Quaternion midpoint_orientation =
    (earth_half * state.orientation_ecef_body * body_half).normalized();
  if (!finite(midpoint_orientation)) {
    return false;
  }

  const Vec3 specific_force_ecef = midpoint_orientation.rotate(corrected_acceleration);
  const Vec3 coriolis = -2.0 * cross(earth_rate_ecef, state.velocity_ecef_m_s);
  const Vec3 centrifugal = -cross(earth_rate_ecef, cross(earth_rate_ecef, state.position_ecef_m));
  const Vec3 acceleration_ecef =
    specific_force_ecef + gravityAcceleration(state.position_ecef_m) + coriolis + centrifugal;
  if (!finite(acceleration_ecef)) {
    return false;
  }

  state.position_ecef_m = state.position_ecef_m + state.velocity_ecef_m_s * delta_s +
    0.5 * acceleration_ecef * delta_s * delta_s;
  state.velocity_ecef_m_s = state.velocity_ecef_m_s + acceleration_ecef * delta_s;
  const Quaternion earth_full = quaternionFromRotationVector(-delta_s * earth_rate_ecef);
  const Quaternion body_full = quaternionFromRotationVector(delta_s * corrected_angular_velocity);
  state.orientation_ecef_body =
    (earth_full * state.orientation_ecef_body * body_full).normalized();
  return validState(state);
}

Vec3 EcefImuPreintegrator::gravityAcceleration(const Vec3 & position_ecef_m) const
{
  if (config_.gravitational_parameter_m3_s2 == 0.0) {
    return {};
  }
  const double radius = norm(position_ecef_m);
  if (!std::isfinite(radius) || radius < config_.minimum_radius_m) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid};
  }
  return position_ecef_m *
         (-config_.gravitational_parameter_m3_s2 / (radius * radius * radius));
}

bool EcefImuPreintegrator::validState(const EcefState & state) const
{
  return std::isfinite(state.stamp_s) && finite(state.position_ecef_m) &&
         finite(state.velocity_ecef_m_s) && finite(state.orientation_ecef_body) &&
         finite(state.accelerometer_bias_m_s2) && finite(state.gyroscope_bias_rad_s) &&
         norm(state.position_ecef_m) >= config_.minimum_radius_m;
}

}  // namespace fgo_gil_localizer

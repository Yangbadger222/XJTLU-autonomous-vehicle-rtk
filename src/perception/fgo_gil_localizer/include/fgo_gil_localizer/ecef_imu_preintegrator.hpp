#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "fgo_gil_localizer/imu_buffer.hpp"
#include "fgo_gil_localizer/math_types.hpp"

namespace fgo_gil_localizer
{

struct EcefState
{
  double stamp_s = 0.0;
  Vec3 position_ecef_m;
  Vec3 velocity_ecef_m_s;
  Quaternion orientation_ecef_body;
  Vec3 accelerometer_bias_m_s2;
  Vec3 gyroscope_bias_rad_s;
};

struct EcefImuConfig
{
  double maximum_step_s = 0.05;
  double earth_rotation_rad_s = 7.2921151467e-5;
  double gravitational_parameter_m3_s2 = 3.986004418e14;
  double minimum_radius_m = 1.0e6;
  double accelerometer_bias_perturbation_m_s2 = 1.0e-3;
  double gyroscope_bias_perturbation_rad_s = 1.0e-5;
};

enum class ImuIntegrationResult : std::uint8_t
{
  Initialized,
  Integrated,
  RejectedNotReset,
  RejectedDuplicate,
  RejectedTimeReversal,
  RejectedGap,
  RejectedNonFinite,
  RejectedInvalidState,
};

struct EcefImuDiagnostics
{
  std::uint64_t integrated_intervals = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t time_reversals = 0;
  std::uint64_t gaps = 0;
  std::uint64_t nonfinite = 0;
  std::uint64_t invalid_states = 0;
  double integrated_duration_s = 0.0;
};

class EcefImuPreintegrator
{
public:
  explicit EcefImuPreintegrator(
    EcefImuConfig config = {},
    bool track_bias_jacobian = true);

  bool reset(const EcefState & initial_state);
  ImuIntegrationResult integrate(const ImuSample & measurement);

  bool valid() const noexcept {return valid_;}
  const EcefState & state() const noexcept {return states_[0];}
  BiasJacobian biasJacobian() const;
  const EcefImuDiagnostics & diagnostics() const noexcept {return diagnostics_;}

private:
  bool propagate(EcefState & state, const ImuSample & measurement, double delta_s) const;
  Vec3 gravityAcceleration(const Vec3 & position_ecef_m) const;
  bool validState(const EcefState & state) const;

  EcefImuConfig config_;
  bool track_bias_jacobian_ = true;
  std::array<EcefState, 7> states_{};
  std::optional<ImuSample> previous_measurement_;
  bool reset_ = false;
  bool valid_ = false;
  EcefImuDiagnostics diagnostics_;
};

}  // namespace fgo_gil_localizer

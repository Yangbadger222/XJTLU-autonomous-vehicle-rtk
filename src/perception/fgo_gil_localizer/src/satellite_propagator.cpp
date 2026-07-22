#include "fgo_gil_localizer/satellite_propagator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fgo_gil_localizer
{
namespace
{

constexpr double kGpsMu = 3.986005e14;
constexpr double kBdsMu = 3.986004418e14;
constexpr double kEarthRotationGps = 7.2921151467e-5;
constexpr double kEarthRotationBds = 7.2921150e-5;
constexpr double kRelativityConstant = -4.442807633e-10;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGlonassMu = 3.9860044e14;
constexpr double kGlonassJ2 = 1.0826257e-3;
constexpr double kGlonassEarthRadiusM = 6378136.0;

bool validTime(const GnssTime & time)
{
  return std::isfinite(time.tow_s) && time.tow_s >= 0.0 && time.tow_s < kGnssWeekSeconds;
}

bool finiteEphemeris(const BroadcastEphemeris & ephemeris)
{
  const double values[] = {
    ephemeris.toe_s, ephemeris.toc_s, ephemeris.semi_major_axis_m,
    ephemeris.delta_mean_motion_rad_s, ephemeris.mean_anomaly_rad,
    ephemeris.eccentricity, ephemeris.argument_of_perigee_rad, ephemeris.cuc_rad,
    ephemeris.cus_rad, ephemeris.crc_m, ephemeris.crs_m, ephemeris.cic_rad,
    ephemeris.cis_rad, ephemeris.inclination_rad, ephemeris.inclination_rate_rad_s,
    ephemeris.ascending_node_rad, ephemeris.ascending_node_rate_rad_s,
    ephemeris.clock_bias_s, ephemeris.clock_drift_s_s,
    ephemeris.clock_drift_rate_s_s2, ephemeris.corrected_mean_motion_rad_s,
    ephemeris.glonass_clock_bias_s, ephemeris.glonass_relative_frequency_bias};
  return std::all_of(
    std::begin(values), std::end(values), [](const double value) {
      return std::isfinite(value);
    }) && finite(ephemeris.position_ecef_m) && finite(ephemeris.velocity_ecef_m_s) &&
         finite(ephemeris.acceleration_ecef_m_s2);
}

double earthRotation(const GnssConstellation constellation)
{
  return constellation == GnssConstellation::Bds ? kEarthRotationBds : kEarthRotationGps;
}

double gravitationalParameter(const GnssConstellation constellation)
{
  return constellation == GnssConstellation::Bds ? kBdsMu : kGpsMu;
}

bool isSupportedKeplerian(const GnssConstellation constellation)
{
  return constellation == GnssConstellation::Gps ||
         constellation == GnssConstellation::Galileo ||
         constellation == GnssConstellation::Bds ||
         constellation == GnssConstellation::Qzss;
}

struct PositionClock
{
  Vec3 position;
  double clock_s = 0.0;
};

std::optional<PositionClock> keplerianPositionClock(
  const BroadcastEphemeris & ephemeris,
  const GnssTime & time)
{
  const double tk = wrappedWeekDelta(time, ephemeris.week, ephemeris.toe_s);
  const double mu = gravitationalParameter(ephemeris.satellite.constellation);
  const double omega_e = earthRotation(ephemeris.satellite.constellation);
  const double a = ephemeris.semi_major_axis_m;
  const double n0 = std::sqrt(mu / (a * a * a));
  const double n = ephemeris.corrected_mean_motion_rad_s > 0.0 ?
    ephemeris.corrected_mean_motion_rad_s : n0 + ephemeris.delta_mean_motion_rad_s;
  const double mean_anomaly = ephemeris.mean_anomaly_rad + n * tk;

  double eccentric_anomaly = mean_anomaly;
  bool converged = false;
  for (std::size_t iteration = 0; iteration < 30U; ++iteration) {
    const double next = mean_anomaly + ephemeris.eccentricity * std::sin(eccentric_anomaly);
    if (std::abs(next - eccentric_anomaly) < 1.0e-13) {
      eccentric_anomaly = next;
      converged = true;
      break;
    }
    eccentric_anomaly = next;
  }
  if (!converged) {
    return std::nullopt;
  }

  const double sin_e = std::sin(eccentric_anomaly);
  const double cos_e = std::cos(eccentric_anomaly);
  const double true_anomaly = std::atan2(
    std::sqrt(1.0 - ephemeris.eccentricity * ephemeris.eccentricity) * sin_e,
    cos_e - ephemeris.eccentricity);
  const double phi = true_anomaly + ephemeris.argument_of_perigee_rad;
  const double two_phi = 2.0 * phi;
  const double corrected_argument = phi + ephemeris.cus_rad * std::sin(two_phi) +
    ephemeris.cuc_rad * std::cos(two_phi);
  const double radius = a * (1.0 - ephemeris.eccentricity * cos_e) +
    ephemeris.crs_m * std::sin(two_phi) + ephemeris.crc_m * std::cos(two_phi);
  const double inclination = ephemeris.inclination_rad + ephemeris.inclination_rate_rad_s * tk +
    ephemeris.cis_rad * std::sin(two_phi) + ephemeris.cic_rad * std::cos(two_phi);
  const double x_orbit = radius * std::cos(corrected_argument);
  const double y_orbit = radius * std::sin(corrected_argument);

  Vec3 position;
  const bool bds_geo = ephemeris.satellite.constellation == GnssConstellation::Bds &&
    (ephemeris.satellite.prn <= 5U || ephemeris.satellite.prn >= 59U);
  if (bds_geo) {
    const double ascending_node = ephemeris.ascending_node_rad +
      ephemeris.ascending_node_rate_rad_s * tk - omega_e * ephemeris.toe_s;
    const double x_geo = x_orbit * std::cos(ascending_node) -
      y_orbit * std::cos(inclination) * std::sin(ascending_node);
    const double y_geo = x_orbit * std::sin(ascending_node) +
      y_orbit * std::cos(inclination) * std::cos(ascending_node);
    const double z_geo = y_orbit * std::sin(inclination);
    const double earth_angle = omega_e * tk;
    const double tilt = -5.0 * kPi / 180.0;
    position.x = x_geo * std::cos(earth_angle) +
      y_geo * std::sin(earth_angle) * std::cos(tilt) +
      z_geo * std::sin(earth_angle) * std::sin(tilt);
    position.y = -x_geo * std::sin(earth_angle) +
      y_geo * std::cos(earth_angle) * std::cos(tilt) +
      z_geo * std::cos(earth_angle) * std::sin(tilt);
    position.z = -y_geo * std::sin(tilt) + z_geo * std::cos(tilt);
  } else {
    const double ascending_node = ephemeris.ascending_node_rad +
      (ephemeris.ascending_node_rate_rad_s - omega_e) * tk -
      omega_e * ephemeris.toe_s;
    position.x = x_orbit * std::cos(ascending_node) -
      y_orbit * std::cos(inclination) * std::sin(ascending_node);
    position.y = x_orbit * std::sin(ascending_node) +
      y_orbit * std::cos(inclination) * std::cos(ascending_node);
    position.z = y_orbit * std::sin(inclination);
  }

  const double clock_dt = wrappedWeekDelta(time, ephemeris.week, ephemeris.toc_s);
  const double relativistic = kRelativityConstant * ephemeris.eccentricity *
    std::sqrt(a) * sin_e;
  const double clock = ephemeris.clock_bias_s + ephemeris.clock_drift_s_s * clock_dt +
    ephemeris.clock_drift_rate_s_s2 * clock_dt * clock_dt + relativistic;
  if (!finite(position) || !std::isfinite(clock)) {
    return std::nullopt;
  }
  return PositionClock{position, clock};
}

Vec3 glonassAcceleration(const Vec3 & position, const Vec3 & velocity, const Vec3 & broadcast)
{
  const double radius2 = squaredNorm(position);
  const double radius = std::sqrt(radius2);
  const double z2 = position.z * position.z;
  const double gravity_scale = -kGlonassMu / (radius2 * radius);
  const double j2_scale = 1.5 * kGlonassJ2 * kGlonassMu *
    kGlonassEarthRadiusM * kGlonassEarthRadiusM / (radius2 * radius2 * radius);
  const double common = 5.0 * z2 / radius2;
  const double omega2 = kEarthRotationGps * kEarthRotationGps;
  return {
    gravity_scale * position.x + j2_scale * position.x * (1.0 - common) +
    omega2 * position.x + 2.0 * kEarthRotationGps * velocity.y + broadcast.x,
    gravity_scale * position.y + j2_scale * position.y * (1.0 - common) +
    omega2 * position.y - 2.0 * kEarthRotationGps * velocity.x + broadcast.y,
    gravity_scale * position.z + j2_scale * position.z * (3.0 - common) + broadcast.z};
}

void glonassRk4(Vec3 & position, Vec3 & velocity, const Vec3 & broadcast, const double step_s)
{
  const Vec3 k1_position = velocity;
  const Vec3 k1_velocity = glonassAcceleration(position, velocity, broadcast);
  const Vec3 k2_position = velocity + 0.5 * step_s * k1_velocity;
  const Vec3 k2_velocity = glonassAcceleration(
    position + 0.5 * step_s * k1_position,
    velocity + 0.5 * step_s * k1_velocity,
    broadcast);
  const Vec3 k3_position = velocity + 0.5 * step_s * k2_velocity;
  const Vec3 k3_velocity = glonassAcceleration(
    position + 0.5 * step_s * k2_position,
    velocity + 0.5 * step_s * k2_velocity,
    broadcast);
  const Vec3 k4_position = velocity + step_s * k3_velocity;
  const Vec3 k4_velocity = glonassAcceleration(
    position + step_s * k3_position,
    velocity + step_s * k3_velocity,
    broadcast);
  position = position + step_s / 6.0 *
    (k1_position + 2.0 * k2_position + 2.0 * k3_position + k4_position);
  velocity = velocity + step_s / 6.0 *
    (k1_velocity + 2.0 * k2_velocity + 2.0 * k3_velocity + k4_velocity);
}

SatellitePropagationResult failure(const SatellitePropagationError error)
{
  SatellitePropagationResult result;
  result.error = error;
  return result;
}

}  // namespace

std::optional<double> carrierFrequencyHz(const SignalKey & signal) noexcept
{
  switch (signal.constellation) {
    case GnssConstellation::Gps:
    case GnssConstellation::Qzss:
      if (signal.signal_type == 0U || signal.signal_type == 3U || signal.signal_type == 11U) {
        return 1575.42e6;
      }
      if (signal.signal_type == 9U || signal.signal_type == 17U) {
        return 1227.60e6;
      }
      if (signal.signal_type == 6U || signal.signal_type == 14U) {
        return 1176.45e6;
      }
      break;
    case GnssConstellation::Glonass:
      if (signal.glonass_frequency_channel < -7 || signal.glonass_frequency_channel > 13) {
        return std::nullopt;
      }
      if (signal.signal_type == 0U) {
        return 1602.0e6 + static_cast<double>(signal.glonass_frequency_channel) * 0.5625e6;
      }
      if (signal.signal_type == 5U) {
        return 1246.0e6 + static_cast<double>(signal.glonass_frequency_channel) * 0.4375e6;
      }
      break;
    case GnssConstellation::Sbas:
      if (signal.signal_type == 0U) {
        return 1575.42e6;
      }
      if (signal.signal_type == 6U) {
        return 1176.45e6;
      }
      break;
    case GnssConstellation::Galileo:
      if (signal.signal_type == 1U || signal.signal_type == 2U) {
        return 1575.42e6;
      }
      if (signal.signal_type == 12U) {
        return 1176.45e6;
      }
      if (signal.signal_type == 17U) {
        return 1207.14e6;
      }
      if (signal.signal_type == 18U || signal.signal_type == 22U) {
        return 1278.75e6;
      }
      break;
    case GnssConstellation::Bds:
      if (signal.signal_type == 0U || signal.signal_type == 4U) {
        return 1561.098e6;
      }
      if (signal.signal_type == 8U || signal.signal_type == 23U) {
        return 1575.42e6;
      }
      if (signal.signal_type == 5U || signal.signal_type == 13U || signal.signal_type == 17U) {
        return 1207.14e6;
      }
      if (signal.signal_type == 12U || signal.signal_type == 28U) {
        return 1176.45e6;
      }
      if (signal.signal_type == 6U || signal.signal_type == 21U) {
        return 1268.52e6;
      }
      break;
    case GnssConstellation::Irnss:
      if (signal.signal_type == 6U || signal.signal_type == 14U) {
        return 1176.45e6;
      }
      break;
  }
  return std::nullopt;
}

std::optional<double> carrierWavelengthM(const SignalKey & signal) noexcept
{
  const auto frequency = carrierFrequencyHz(signal);
  if (!frequency.has_value() || *frequency <= 0.0) {
    return std::nullopt;
  }
  return kSpeedOfLightMps / *frequency;
}

double wrappedWeekDelta(
  const GnssTime & time,
  const std::uint32_t reference_week,
  const double reference_tow_s)
{
  double delta = (static_cast<double>(time.week) - static_cast<double>(reference_week)) *
    kGnssWeekSeconds + time.tow_s - reference_tow_s;
  while (delta > 0.5 * kGnssWeekSeconds) {
    delta -= kGnssWeekSeconds;
  }
  while (delta < -0.5 * kGnssWeekSeconds) {
    delta += kGnssWeekSeconds;
  }
  return delta;
}

GnssTime addGnssSeconds(const GnssTime & time, const double delta_s)
{
  GnssTime result = time;
  result.tow_s += delta_s;
  while (result.tow_s < 0.0) {
    if (result.week == 0U) {
      result.tow_s = std::numeric_limits<double>::quiet_NaN();
      return result;
    }
    --result.week;
    result.tow_s += kGnssWeekSeconds;
  }
  while (result.tow_s >= kGnssWeekSeconds) {
    ++result.week;
    result.tow_s -= kGnssWeekSeconds;
  }
  return result;
}

SatellitePropagator::SatellitePropagator(SatellitePropagationConfig config)
: config_(config)
{
  if (!std::isfinite(config_.maximum_kepler_age_s) || config_.maximum_kepler_age_s <= 0.0 ||
    !std::isfinite(config_.maximum_glonass_age_s) || config_.maximum_glonass_age_s <= 0.0 ||
    !std::isfinite(config_.glonass_integration_step_s) ||
    config_.glonass_integration_step_s <= 0.0)
  {
    throw std::invalid_argument("satellite propagation configuration is outside valid bounds");
  }
}

SatellitePropagationResult SatellitePropagator::propagate(
  const BroadcastEphemeris & ephemeris,
  const GnssTime & query_time) const
{
  if (!validTime(query_time)) {
    return failure(SatellitePropagationError::InvalidTime);
  }
  if (!finiteEphemeris(ephemeris) || ephemeris.satellite.prn == 0U ||
    !std::isfinite(ephemeris.toe_s) || ephemeris.toe_s < 0.0 ||
    ephemeris.toe_s >= kGnssWeekSeconds)
  {
    return failure(SatellitePropagationError::InvalidEphemeris);
  }
  if (config_.reject_unhealthy && ephemeris.health != 0U) {
    return failure(SatellitePropagationError::UnhealthySatellite);
  }
  if (ephemeris.model == EphemerisModel::Keplerian) {
    return propagateKeplerian(ephemeris, query_time);
  }
  if (ephemeris.model == EphemerisModel::Glonass) {
    return propagateGlonass(ephemeris, query_time);
  }
  return failure(SatellitePropagationError::UnsupportedConstellation);
}

SatellitePropagationResult SatellitePropagator::propagateToReceiveFrame(
  const BroadcastEphemeris & ephemeris,
  const GnssTime & receive_time,
  const double pseudorange_m) const
{
  if (!std::isfinite(pseudorange_m) || pseudorange_m <= 0.0 ||
    pseudorange_m > 1.0e8)
  {
    return failure(SatellitePropagationError::InvalidTime);
  }
  const double transit_s = pseudorange_m / kSpeedOfLightMps;
  const GnssTime transmit_time = addGnssSeconds(receive_time, -transit_s);
  SatellitePropagationResult result = propagate(ephemeris, transmit_time);
  if (!result.ok()) {
    return result;
  }
  const double angle = earthRotation(ephemeris.satellite.constellation) * transit_s;
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  const Vec3 position = result.state->position_ecef_m;
  const Vec3 velocity = result.state->velocity_ecef_m_s;
  result.state->position_ecef_m = {
    cosine * position.x + sine * position.y,
    -sine * position.x + cosine * position.y,
    position.z};
  result.state->velocity_ecef_m_s = {
    cosine * velocity.x + sine * velocity.y,
    -sine * velocity.x + cosine * velocity.y,
    velocity.z};
  return result;
}

SatellitePropagationResult SatellitePropagator::propagateKeplerian(
  const BroadcastEphemeris & ephemeris,
  const GnssTime & query_time) const
{
  if (!isSupportedKeplerian(ephemeris.satellite.constellation)) {
    return failure(SatellitePropagationError::UnsupportedConstellation);
  }
  if (ephemeris.semi_major_axis_m < 1.0e6 || ephemeris.semi_major_axis_m > 1.0e8 ||
    ephemeris.eccentricity < 0.0 || ephemeris.eccentricity >= 1.0)
  {
    return failure(SatellitePropagationError::InvalidEphemeris);
  }
  const double age_s = wrappedWeekDelta(query_time, ephemeris.week, ephemeris.toe_s);
  if (std::abs(age_s) > config_.maximum_kepler_age_s) {
    return failure(SatellitePropagationError::EphemerisTooOld);
  }
  const auto center = keplerianPositionClock(ephemeris, query_time);
  const auto before = keplerianPositionClock(ephemeris, addGnssSeconds(query_time, -0.01));
  const auto after = keplerianPositionClock(ephemeris, addGnssSeconds(query_time, 0.01));
  if (!center.has_value() || !before.has_value() || !after.has_value()) {
    return failure(SatellitePropagationError::KeplerDidNotConverge);
  }

  SatelliteState state;
  state.satellite = ephemeris.satellite;
  state.transmit_time = query_time;
  state.position_ecef_m = center->position;
  state.velocity_ecef_m_s = (after->position - before->position) / 0.02;
  state.clock_offset_s = center->clock_s;
  state.clock_drift_s_s = (after->clock_s - before->clock_s) / 0.02;
  if (!finite(state.position_ecef_m) || !finite(state.velocity_ecef_m_s) ||
    !std::isfinite(state.clock_offset_s) || !std::isfinite(state.clock_drift_s_s))
  {
    return failure(SatellitePropagationError::NumericalFailure);
  }
  SatellitePropagationResult result;
  result.state = state;
  return result;
}

SatellitePropagationResult SatellitePropagator::propagateGlonass(
  const BroadcastEphemeris & ephemeris,
  const GnssTime & query_time) const
{
  if (ephemeris.satellite.constellation != GnssConstellation::Glonass ||
    norm(ephemeris.position_ecef_m) < 1.0e6)
  {
    return failure(SatellitePropagationError::InvalidEphemeris);
  }
  double remaining_s = wrappedWeekDelta(query_time, ephemeris.week, ephemeris.toe_s);
  if (std::abs(remaining_s) > config_.maximum_glonass_age_s) {
    return failure(SatellitePropagationError::EphemerisTooOld);
  }
  const double clock_dt = remaining_s;
  Vec3 position = ephemeris.position_ecef_m;
  Vec3 velocity = ephemeris.velocity_ecef_m_s;
  while (std::abs(remaining_s) > 1.0e-9) {
    const double step = std::copysign(
      std::min(std::abs(remaining_s), config_.glonass_integration_step_s), remaining_s);
    glonassRk4(position, velocity, ephemeris.acceleration_ecef_m_s2, step);
    remaining_s -= step;
  }
  SatelliteState state;
  state.satellite = ephemeris.satellite;
  state.transmit_time = query_time;
  state.position_ecef_m = position;
  state.velocity_ecef_m_s = velocity;
  state.clock_offset_s = -ephemeris.glonass_clock_bias_s +
    ephemeris.glonass_relative_frequency_bias * clock_dt;
  state.clock_drift_s_s = ephemeris.glonass_relative_frequency_bias;
  if (!finite(state.position_ecef_m) || !finite(state.velocity_ecef_m_s) ||
    !std::isfinite(state.clock_offset_s))
  {
    return failure(SatellitePropagationError::NumericalFailure);
  }
  SatellitePropagationResult result;
  result.state = state;
  return result;
}

}  // namespace fgo_gil_localizer

#pragma once

#include <cstdint>
#include <vector>

#include "fgo_gil_localizer/math_types.hpp"

namespace fgo_gil_localizer
{

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kGnssWeekSeconds = 604800.0;

enum class GnssConstellation : std::uint8_t
{
  Gps = 0,
  Glonass = 1,
  Sbas = 2,
  Galileo = 3,
  Bds = 4,
  Qzss = 5,
  Irnss = 6,
};

struct SatelliteId
{
  GnssConstellation constellation = GnssConstellation::Gps;
  std::uint16_t prn = 0;

  bool operator==(const SatelliteId & other) const noexcept
  {
    return constellation == other.constellation && prn == other.prn;
  }

  bool operator<(const SatelliteId & other) const noexcept
  {
    if (constellation != other.constellation) {
      return constellation < other.constellation;
    }
    return prn < other.prn;
  }
};

struct SignalKey
{
  GnssConstellation constellation = GnssConstellation::Gps;
  std::uint8_t signal_type = 0;
  bool l2c_signal = false;
  std::int16_t glonass_frequency_channel = 0;

  bool operator==(const SignalKey & other) const noexcept
  {
    return constellation == other.constellation && signal_type == other.signal_type &&
           l2c_signal == other.l2c_signal &&
           glonass_frequency_channel == other.glonass_frequency_channel;
  }

  bool operator<(const SignalKey & other) const noexcept
  {
    if (constellation != other.constellation) {
      return constellation < other.constellation;
    }
    if (signal_type != other.signal_type) {
      return signal_type < other.signal_type;
    }
    if (l2c_signal != other.l2c_signal) {
      return l2c_signal < other.l2c_signal;
    }
    return glonass_frequency_channel < other.glonass_frequency_channel;
  }
};

struct GnssTime
{
  std::uint32_t week = 0;
  double tow_s = 0.0;
};

struct GnssObservation
{
  SatelliteId satellite;
  SignalKey signal;
  std::uint8_t channel_number = 0;
  double pseudorange_m = 0.0;
  double carrier_phase_cycles = 0.0;
  double doppler_hz = 0.0;
  double pseudorange_std_m = 0.0;
  double carrier_phase_std_cycles = 0.0;
  double cn0_db_hz = 0.0;
  double lock_time_s = 0.0;
  bool pseudorange_valid = false;
  bool carrier_phase_valid = false;
  std::uint32_t tracking_status = 0;
};

enum class GnssReceiver : std::uint8_t
{
  Master = 1,
  Secondary = 2,
  Base = 3,
};

struct GnssObservationEpoch
{
  GnssReceiver receiver = GnssReceiver::Master;
  GnssTime time;
  std::vector<GnssObservation> observations;
};

enum class EphemerisModel : std::uint8_t
{
  Keplerian = 1,
  Glonass = 2,
};

struct BroadcastEphemeris
{
  SatelliteId satellite;
  EphemerisModel model = EphemerisModel::Keplerian;
  std::uint32_t health = 0;
  std::uint32_t week = 0;
  double toe_s = 0.0;
  double toc_s = 0.0;

  double semi_major_axis_m = 0.0;
  double delta_mean_motion_rad_s = 0.0;
  double mean_anomaly_rad = 0.0;
  double eccentricity = 0.0;
  double argument_of_perigee_rad = 0.0;
  double cuc_rad = 0.0;
  double cus_rad = 0.0;
  double crc_m = 0.0;
  double crs_m = 0.0;
  double cic_rad = 0.0;
  double cis_rad = 0.0;
  double inclination_rad = 0.0;
  double inclination_rate_rad_s = 0.0;
  double ascending_node_rad = 0.0;
  double ascending_node_rate_rad_s = 0.0;
  double clock_bias_s = 0.0;
  double clock_drift_s_s = 0.0;
  double clock_drift_rate_s_s2 = 0.0;
  double corrected_mean_motion_rad_s = 0.0;

  std::int16_t glonass_frequency_channel = 0;
  Vec3 position_ecef_m;
  Vec3 velocity_ecef_m_s;
  Vec3 acceleration_ecef_m_s2;
  double glonass_clock_bias_s = 0.0;
  double glonass_relative_frequency_bias = 0.0;
};

struct SatelliteState
{
  SatelliteId satellite;
  GnssTime transmit_time;
  Vec3 position_ecef_m;
  Vec3 velocity_ecef_m_s;
  double clock_offset_s = 0.0;
  double clock_drift_s_s = 0.0;
};

}  // namespace fgo_gil_localizer

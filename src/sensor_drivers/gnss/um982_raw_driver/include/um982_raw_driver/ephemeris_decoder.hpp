#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "um982_raw_driver/binary_framer.hpp"
#include "um982_raw_driver/observation_decoder.hpp"

namespace um982_raw_driver
{

constexpr std::uint16_t kGpsEphemerisMessageId = 106;
constexpr std::uint16_t kGlonassEphemerisMessageId = 107;
constexpr std::uint16_t kBdsEphemerisMessageId = 108;
constexpr std::uint16_t kGalileoEphemerisMessageId = 109;
constexpr std::uint16_t kQzssEphemerisMessageId = 110;

enum class EphemerisModel : std::uint8_t
{
  Keplerian = 1,
  Glonass = 2,
};

enum class EphemerisReferenceFrame : std::uint8_t
{
  Wgs84 = 1,
  Pz90_02 = 2,
  Cgcs2000 = 3,
  Gtrf = 4,
  Jgs = 5,
};

struct Ephemeris
{
  BinaryHeader header;
  GnssConstellation constellation = GnssConstellation::Gps;
  std::uint16_t prn = 0;
  EphemerisModel model = EphemerisModel::Keplerian;
  EphemerisReferenceFrame reference_frame = EphemerisReferenceFrame::Wgs84;
  std::uint32_t health = 0;
  std::uint32_t issue_of_data_ephemeris = 0;
  std::uint32_t issue_of_data_clock = 0;
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
  double group_delay_1_s = 0.0;
  double group_delay_2_s = 0.0;
  bool group_delay_1_valid = false;
  bool group_delay_2_valid = false;
  double corrected_mean_motion_rad_s = 0.0;
  double ura_variance_m2 = 0.0;
  bool ura_variance_valid = false;
  std::uint32_t accuracy_index = 0;

  std::int16_t glonass_frequency_channel = 0;
  std::array<double, 3> position_ecef_m{};
  std::array<double, 3> velocity_ecef_m_s{};
  std::array<double, 3> acceleration_ecef_m_s2{};
  double glonass_clock_bias_s = 0.0;
  double glonass_relative_frequency_bias = 0.0;
  double glonass_l1_l2_delay_s = 0.0;
  std::uint32_t glonass_frame_time_s = 0;
  std::uint32_t glonass_flags = 0;
};

enum class EphemerisDecodeError
{
  None,
  UnsupportedMessage,
  FrameSizeMismatch,
  PayloadLayoutMismatch,
  NonFiniteField,
  InvalidField,
  InvalidPrn,
};

struct EphemerisDecodeResult
{
  EphemerisDecodeError error = EphemerisDecodeError::None;
  std::string reason;
  std::optional<Ephemeris> ephemeris;

  bool ok() const noexcept {return ephemeris.has_value();}
};

bool isEphemerisMessage(std::uint16_t message_id) noexcept;
EphemerisDecodeResult decodeEphemerisFrame(const BinaryFrame & frame);

}  // namespace um982_raw_driver

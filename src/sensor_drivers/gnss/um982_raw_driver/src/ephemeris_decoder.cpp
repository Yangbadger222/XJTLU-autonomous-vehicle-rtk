#include "um982_raw_driver/ephemeris_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <utility>

namespace um982_raw_driver
{
namespace
{

constexpr std::size_t kGpsPayloadSize = 224;
constexpr std::size_t kGlonassPayloadSize = 144;
constexpr std::size_t kBdsPayloadSize = 232;
constexpr std::size_t kGalileoPayloadSize = 220;
constexpr double kSecondsPerWeek = 604800.0;

std::uint16_t readLe16(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

std::uint32_t readLe32(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

std::uint64_t readLe64(const std::uint8_t * data)
{
  std::uint64_t value = 0;
  for (std::size_t byte = 0; byte < 8U; ++byte) {
    value |= static_cast<std::uint64_t>(data[byte]) << (8U * byte);
  }
  return value;
}

double readLeDouble(const std::uint8_t * data)
{
  const std::uint64_t bits = readLe64(data);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

EphemerisDecodeResult failure(
  const EphemerisDecodeError error,
  const std::string & reason)
{
  EphemerisDecodeResult result;
  result.error = error;
  result.reason = reason;
  return result;
}

bool finiteKeplerian(const Ephemeris & ephemeris)
{
  const double values[] = {
    ephemeris.toe_s, ephemeris.toc_s, ephemeris.semi_major_axis_m,
    ephemeris.delta_mean_motion_rad_s, ephemeris.mean_anomaly_rad,
    ephemeris.eccentricity, ephemeris.argument_of_perigee_rad, ephemeris.cuc_rad,
    ephemeris.cus_rad, ephemeris.crc_m, ephemeris.crs_m, ephemeris.cic_rad,
    ephemeris.cis_rad, ephemeris.inclination_rad, ephemeris.inclination_rate_rad_s,
    ephemeris.ascending_node_rad, ephemeris.ascending_node_rate_rad_s,
    ephemeris.clock_bias_s, ephemeris.clock_drift_s_s,
    ephemeris.clock_drift_rate_s_s2, ephemeris.group_delay_1_s,
    ephemeris.group_delay_2_s, ephemeris.corrected_mean_motion_rad_s,
    ephemeris.ura_variance_m2};
  return std::all_of(
    std::begin(values), std::end(values), [](const double value) {
      return std::isfinite(value);
    });
}

bool validWeekSecond(const double value)
{
  return value >= 0.0 && value <= kSecondsPerWeek;
}

EphemerisDecodeResult validateKeplerian(Ephemeris ephemeris)
{
  if (!finiteKeplerian(ephemeris)) {
    return failure(EphemerisDecodeError::NonFiniteField, "non-finite Keplerian field");
  }
  if (!validWeekSecond(ephemeris.toe_s) || !validWeekSecond(ephemeris.toc_s)) {
    return failure(EphemerisDecodeError::InvalidField, "Toe/Toc outside one GNSS week");
  }
  if (ephemeris.semi_major_axis_m < 1.0e6 || ephemeris.semi_major_axis_m > 1.0e8) {
    return failure(EphemerisDecodeError::InvalidField, "semi-major axis outside GNSS range");
  }
  if (ephemeris.eccentricity < 0.0 || ephemeris.eccentricity >= 1.0) {
    return failure(EphemerisDecodeError::InvalidField, "eccentricity outside [0, 1)");
  }
  if (ephemeris.ura_variance_m2 < 0.0) {
    return failure(EphemerisDecodeError::InvalidField, "negative URA variance");
  }

  EphemerisDecodeResult result;
  result.ephemeris = std::move(ephemeris);
  return result;
}

void decodeSharedKeplerianOrbit(const std::uint8_t * payload, Ephemeris & ephemeris)
{
  ephemeris.toe_s = readLeDouble(payload + 32);
  ephemeris.semi_major_axis_m = readLeDouble(payload + 40);
  ephemeris.delta_mean_motion_rad_s = readLeDouble(payload + 48);
  ephemeris.mean_anomaly_rad = readLeDouble(payload + 56);
  ephemeris.eccentricity = readLeDouble(payload + 64);
  ephemeris.argument_of_perigee_rad = readLeDouble(payload + 72);
  ephemeris.cuc_rad = readLeDouble(payload + 80);
  ephemeris.cus_rad = readLeDouble(payload + 88);
  ephemeris.crc_m = readLeDouble(payload + 96);
  ephemeris.crs_m = readLeDouble(payload + 104);
  ephemeris.cic_rad = readLeDouble(payload + 112);
  ephemeris.cis_rad = readLeDouble(payload + 120);
  ephemeris.inclination_rad = readLeDouble(payload + 128);
  ephemeris.inclination_rate_rad_s = readLeDouble(payload + 136);
  ephemeris.ascending_node_rad = readLeDouble(payload + 144);
  ephemeris.ascending_node_rate_rad_s = readLeDouble(payload + 152);
}

EphemerisDecodeResult decodeGpsLike(
  const BinaryFrame & frame,
  const bool dedicated_qzss_message)
{
  const std::uint8_t * payload = frame.bytes.data() + kBinaryHeaderSize;
  Ephemeris ephemeris;
  ephemeris.header = frame.header;
  ephemeris.model = EphemerisModel::Keplerian;
  const std::uint32_t source_prn = readLe32(payload);
  if (!dedicated_qzss_message) {
    if (source_prn >= 1U && source_prn <= 32U) {
      ephemeris.constellation = GnssConstellation::Gps;
      ephemeris.reference_frame = EphemerisReferenceFrame::Wgs84;
      ephemeris.prn = static_cast<std::uint16_t>(source_prn);
    } else if (source_prn >= 33U && source_prn <= 42U) {
      ephemeris.constellation = GnssConstellation::Qzss;
      ephemeris.reference_frame = EphemerisReferenceFrame::Jgs;
      ephemeris.prn = static_cast<std::uint16_t>(source_prn + 160U);
    } else {
      return failure(EphemerisDecodeError::InvalidPrn, "GPSEPH PRN outside GPS/QZSS range");
    }
  } else {
    if (source_prn < 1U || source_prn > 10U) {
      return failure(EphemerisDecodeError::InvalidPrn, "QZSS PRN outside 1..10");
    }
    ephemeris.constellation = GnssConstellation::Qzss;
    ephemeris.reference_frame = EphemerisReferenceFrame::Jgs;
    ephemeris.prn = static_cast<std::uint16_t>(source_prn + 192U);
  }
  ephemeris.health = readLe32(payload + 12);
  ephemeris.issue_of_data_ephemeris = readLe32(payload + 16);
  ephemeris.issue_of_data_clock = readLe32(payload + 160);
  ephemeris.week = readLe32(payload + 24);
  decodeSharedKeplerianOrbit(payload, ephemeris);
  ephemeris.toc_s = readLeDouble(payload + 164);
  ephemeris.group_delay_1_s = readLeDouble(payload + 172);
  ephemeris.group_delay_1_valid = true;
  ephemeris.clock_bias_s = readLeDouble(payload + 180);
  ephemeris.clock_drift_s_s = readLeDouble(payload + 188);
  ephemeris.clock_drift_rate_s_s2 = readLeDouble(payload + 196);
  ephemeris.corrected_mean_motion_rad_s = readLeDouble(payload + 208);
  ephemeris.ura_variance_m2 = readLeDouble(payload + 216);
  ephemeris.ura_variance_valid = true;
  return validateKeplerian(std::move(ephemeris));
}

EphemerisDecodeResult decodeBds(const BinaryFrame & frame)
{
  const std::uint8_t * payload = frame.bytes.data() + kBinaryHeaderSize;
  const std::uint32_t prn = readLe32(payload);
  if (prn < 1U || prn > 63U) {
    return failure(EphemerisDecodeError::InvalidPrn, "BDS PRN outside 1..63");
  }
  Ephemeris ephemeris;
  ephemeris.header = frame.header;
  ephemeris.constellation = GnssConstellation::Bds;
  ephemeris.prn = static_cast<std::uint16_t>(prn);
  ephemeris.model = EphemerisModel::Keplerian;
  ephemeris.reference_frame = EphemerisReferenceFrame::Cgcs2000;
  ephemeris.health = readLe32(payload + 12);
  ephemeris.issue_of_data_ephemeris = readLe32(payload + 16);
  ephemeris.issue_of_data_clock = readLe32(payload + 160);
  ephemeris.week = readLe32(payload + 24);
  decodeSharedKeplerianOrbit(payload, ephemeris);
  ephemeris.toc_s = readLeDouble(payload + 164);
  ephemeris.group_delay_1_s = readLeDouble(payload + 172);
  ephemeris.group_delay_2_s = readLeDouble(payload + 180);
  ephemeris.group_delay_1_valid = true;
  ephemeris.group_delay_2_valid = true;
  ephemeris.clock_bias_s = readLeDouble(payload + 188);
  ephemeris.clock_drift_s_s = readLeDouble(payload + 196);
  ephemeris.clock_drift_rate_s_s2 = readLeDouble(payload + 204);
  ephemeris.corrected_mean_motion_rad_s = readLeDouble(payload + 216);
  ephemeris.ura_variance_m2 = readLeDouble(payload + 224);
  ephemeris.ura_variance_valid = true;
  return validateKeplerian(std::move(ephemeris));
}

EphemerisDecodeResult decodeGalileo(const BinaryFrame & frame)
{
  const std::uint8_t * payload = frame.bytes.data() + kBinaryHeaderSize;
  const std::uint32_t prn = readLe32(payload);
  if (prn < 1U || prn > 36U) {
    return failure(EphemerisDecodeError::InvalidPrn, "Galileo PRN outside 1..36");
  }
  const std::uint32_t fnav_received = readLe32(payload + 4);
  const std::uint32_t inav_received = readLe32(payload + 8);
  if (fnav_received > 1U || inav_received > 1U || (fnav_received | inav_received) == 0U) {
    return failure(EphemerisDecodeError::InvalidField, "invalid Galileo navigation flags");
  }

  Ephemeris ephemeris;
  ephemeris.header = frame.header;
  ephemeris.constellation = GnssConstellation::Galileo;
  ephemeris.prn = static_cast<std::uint16_t>(prn);
  ephemeris.model = EphemerisModel::Keplerian;
  ephemeris.reference_frame = EphemerisReferenceFrame::Gtrf;
  ephemeris.health = std::any_of(
    payload + 12, payload + 18, [](const std::uint8_t value) {
      return value != 0U;
    }) ? 1U : 0U;
  ephemeris.issue_of_data_ephemeris = readLe32(payload + 20);
  ephemeris.issue_of_data_clock = ephemeris.issue_of_data_ephemeris;
  ephemeris.week = frame.header.week;
  ephemeris.toe_s = static_cast<double>(readLe32(payload + 24));
  const double root_a = readLeDouble(payload + 28);
  ephemeris.semi_major_axis_m = root_a * root_a;
  ephemeris.delta_mean_motion_rad_s = readLeDouble(payload + 36);
  ephemeris.mean_anomaly_rad = readLeDouble(payload + 44);
  ephemeris.eccentricity = readLeDouble(payload + 52);
  ephemeris.argument_of_perigee_rad = readLeDouble(payload + 60);
  ephemeris.cuc_rad = readLeDouble(payload + 68);
  ephemeris.cus_rad = readLeDouble(payload + 76);
  ephemeris.crc_m = readLeDouble(payload + 84);
  ephemeris.crs_m = readLeDouble(payload + 92);
  ephemeris.cic_rad = readLeDouble(payload + 100);
  ephemeris.cis_rad = readLeDouble(payload + 108);
  ephemeris.inclination_rad = readLeDouble(payload + 116);
  ephemeris.inclination_rate_rad_s = readLeDouble(payload + 124);
  ephemeris.ascending_node_rad = readLeDouble(payload + 132);
  ephemeris.ascending_node_rate_rad_s = readLeDouble(payload + 140);
  const std::size_t clock_offset = inav_received != 0U ? 176U : 148U;
  ephemeris.toc_s = static_cast<double>(readLe32(payload + clock_offset));
  ephemeris.clock_bias_s = readLeDouble(payload + clock_offset + 4U);
  ephemeris.clock_drift_s_s = readLeDouble(payload + clock_offset + 12U);
  ephemeris.clock_drift_rate_s_s2 = readLeDouble(payload + clock_offset + 20U);
  ephemeris.group_delay_1_s = readLeDouble(payload + 204);
  ephemeris.group_delay_2_s = readLeDouble(payload + 212);
  ephemeris.group_delay_1_valid = fnav_received != 0U;
  ephemeris.group_delay_2_valid = inav_received != 0U;
  ephemeris.accuracy_index = payload[18];
  return validateKeplerian(std::move(ephemeris));
}

EphemerisDecodeResult decodeGlonass(const BinaryFrame & frame)
{
  const std::uint8_t * payload = frame.bytes.data() + kBinaryHeaderSize;
  const std::uint16_t prn = readLe16(payload);
  const std::uint16_t frequency = readLe16(payload + 2);
  if (prn < 38U || prn > 61U) {
    return failure(EphemerisDecodeError::InvalidPrn, "GLONASS slot PRN outside 38..61");
  }
  if (frequency > 20U) {
    return failure(EphemerisDecodeError::InvalidField, "GLONASS frequency field exceeds 20");
  }

  Ephemeris ephemeris;
  ephemeris.header = frame.header;
  ephemeris.constellation = GnssConstellation::Glonass;
  ephemeris.prn = prn;
  ephemeris.model = EphemerisModel::Glonass;
  ephemeris.reference_frame = EphemerisReferenceFrame::Pz90_02;
  ephemeris.glonass_frequency_channel = static_cast<std::int16_t>(frequency) - 7;
  ephemeris.week = readLe16(payload + 6);
  ephemeris.toe_s = static_cast<double>(readLe32(payload + 8)) / 1000.0;
  ephemeris.toc_s = ephemeris.toe_s;
  ephemeris.issue_of_data_ephemeris = readLe32(payload + 20);
  ephemeris.issue_of_data_clock = ephemeris.issue_of_data_ephemeris;
  ephemeris.health = readLe32(payload + 24);
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    ephemeris.position_ecef_m[axis] = readLeDouble(payload + 28U + axis * 8U);
    ephemeris.velocity_ecef_m_s[axis] = readLeDouble(payload + 52U + axis * 8U);
    ephemeris.acceleration_ecef_m_s2[axis] = readLeDouble(payload + 76U + axis * 8U);
  }
  ephemeris.glonass_clock_bias_s = readLeDouble(payload + 100);
  ephemeris.glonass_l1_l2_delay_s = readLeDouble(payload + 108);
  ephemeris.glonass_relative_frequency_bias = readLeDouble(payload + 116);
  ephemeris.glonass_frame_time_s = readLe32(payload + 124);
  ephemeris.accuracy_index = readLe32(payload + 132);
  ephemeris.glonass_flags = readLe32(payload + 140);

  const auto finite_array = [](const std::array<double, 3> & values) {
      return std::all_of(
        values.begin(), values.end(), [](const double value) {
          return std::isfinite(value);
        });
    };
  if (!std::isfinite(ephemeris.toe_s) || !finite_array(ephemeris.position_ecef_m) ||
    !finite_array(ephemeris.velocity_ecef_m_s) ||
    !finite_array(ephemeris.acceleration_ecef_m_s2) ||
    !std::isfinite(ephemeris.glonass_clock_bias_s) ||
    !std::isfinite(ephemeris.glonass_l1_l2_delay_s) ||
    !std::isfinite(ephemeris.glonass_relative_frequency_bias))
  {
    return failure(EphemerisDecodeError::NonFiniteField, "non-finite GLONASS field");
  }
  const double position_norm_squared =
    ephemeris.position_ecef_m[0] * ephemeris.position_ecef_m[0] +
    ephemeris.position_ecef_m[1] * ephemeris.position_ecef_m[1] +
    ephemeris.position_ecef_m[2] * ephemeris.position_ecef_m[2];
  if (!validWeekSecond(ephemeris.toe_s) || position_norm_squared < 1.0e12 ||
    ephemeris.glonass_frame_time_s > 86400U)
  {
    return failure(EphemerisDecodeError::InvalidField, "GLONASS time or position outside range");
  }

  EphemerisDecodeResult result;
  result.ephemeris = std::move(ephemeris);
  return result;
}

}  // namespace

bool isEphemerisMessage(const std::uint16_t message_id) noexcept
{
  return message_id >= kGpsEphemerisMessageId && message_id <= kQzssEphemerisMessageId;
}

EphemerisDecodeResult decodeEphemerisFrame(const BinaryFrame & frame)
{
  std::size_t expected_payload_size = 0;
  switch (frame.header.message_id) {
    case kGpsEphemerisMessageId:
    case kQzssEphemerisMessageId:
      expected_payload_size = kGpsPayloadSize;
      break;
    case kGlonassEphemerisMessageId:
      expected_payload_size = kGlonassPayloadSize;
      break;
    case kBdsEphemerisMessageId:
      expected_payload_size = kBdsPayloadSize;
      break;
    case kGalileoEphemerisMessageId:
      expected_payload_size = kGalileoPayloadSize;
      break;
    default:
      return failure(EphemerisDecodeError::UnsupportedMessage, "unsupported message ID");
  }

  const std::size_t expected_frame_size =
    kBinaryHeaderSize + static_cast<std::size_t>(frame.header.payload_length) + kBinaryCrcSize;
  if (frame.bytes.size() != expected_frame_size) {
    return failure(
      EphemerisDecodeError::FrameSizeMismatch,
      "frame byte count disagrees with header");
  }
  if (frame.header.payload_length != expected_payload_size) {
    return failure(
      EphemerisDecodeError::PayloadLayoutMismatch,
      "unexpected ephemeris payload length");
  }

  switch (frame.header.message_id) {
    case kGpsEphemerisMessageId:
      return decodeGpsLike(frame, false);
    case kGlonassEphemerisMessageId:
      return decodeGlonass(frame);
    case kBdsEphemerisMessageId:
      return decodeBds(frame);
    case kGalileoEphemerisMessageId:
      return decodeGalileo(frame);
    case kQzssEphemerisMessageId:
      return decodeGpsLike(frame, true);
    default:
      break;
  }
  return failure(EphemerisDecodeError::UnsupportedMessage, "unsupported message ID");
}

}  // namespace um982_raw_driver

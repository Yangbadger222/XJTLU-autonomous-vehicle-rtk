#include "um982_raw_driver/observation_decoder.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <sstream>
#include <utility>

namespace um982_raw_driver
{
namespace
{

constexpr std::uint32_t kCarrierPhaseValidMask = 1U << 10U;
constexpr std::uint32_t kPseudorangeValidMask = 1U << 12U;
constexpr std::uint32_t kConstellationMask = 0x7U;
constexpr std::uint32_t kSignalTypeMask = 0x1FU;

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

float readLeFloat(const std::uint8_t * data)
{
  const std::uint32_t bits = readLe32(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double readLeDouble(const std::uint8_t * data)
{
  const std::uint64_t bits = readLe64(data);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

ObservationDecodeResult failure(
  const ObservationDecodeError error,
  const std::string & reason)
{
  ObservationDecodeResult result;
  result.error = error;
  result.reason = reason;
  return result;
}

std::optional<ObservationReceiver> receiverForMessage(const std::uint16_t message_id)
{
  switch (message_id) {
    case kObsVmMessageId:
      return ObservationReceiver::Master;
    case kObsVhMessageId:
      return ObservationReceiver::Secondary;
    case kObsVBaseMessageId:
      return ObservationReceiver::Base;
    default:
      return std::nullopt;
  }
}

bool validPrn(const GnssConstellation constellation, const std::uint16_t prn)
{
  switch (constellation) {
    case GnssConstellation::Gps:
      return prn >= 1U && prn <= 32U;
    case GnssConstellation::Glonass:
      return prn >= 38U && prn <= 61U;
    case GnssConstellation::Sbas:
      return prn >= 120U && prn <= 158U;
    case GnssConstellation::Galileo:
      return prn >= 1U && prn <= 36U;
    case GnssConstellation::Bds:
      return prn >= 1U && prn <= 63U;
    case GnssConstellation::Qzss:
      return prn >= 193U && prn <= 202U;
    case GnssConstellation::Irnss:
      return prn >= 1U && prn <= 15U;
  }
  return false;
}

std::string recordReason(const std::size_t record_index, const std::string & reason)
{
  std::ostringstream stream;
  stream << "observation[" << record_index << "]: " << reason;
  return stream.str();
}

}  // namespace

bool isObservationMessage(const std::uint16_t message_id) noexcept
{
  return message_id == kObsVmMessageId || message_id == kObsVhMessageId ||
         message_id == kObsVBaseMessageId;
}

ObservationDecodeResult decodeObservationFrame(const BinaryFrame & frame)
{
  const auto receiver = receiverForMessage(frame.header.message_id);
  if (!receiver.has_value()) {
    return failure(ObservationDecodeError::UnsupportedMessage, "unsupported message ID");
  }

  const std::size_t expected_frame_size =
    kBinaryHeaderSize + static_cast<std::size_t>(frame.header.payload_length) + kBinaryCrcSize;
  if (frame.bytes.size() != expected_frame_size) {
    return failure(
      ObservationDecodeError::FrameSizeMismatch,
      "frame byte count disagrees with header");
  }
  if (frame.header.payload_length < sizeof(std::uint32_t)) {
    return failure(
      ObservationDecodeError::PayloadLayoutMismatch,
      "payload lacks observation count");
  }

  const std::uint8_t * payload = frame.bytes.data() + kBinaryHeaderSize;
  const std::uint32_t observation_count = readLe32(payload);
  const std::size_t records_bytes =
    static_cast<std::size_t>(frame.header.payload_length) - sizeof(std::uint32_t);
  if (records_bytes % kObservationRecordSize != 0U ||
    observation_count != records_bytes / kObservationRecordSize)
  {
    return failure(
      ObservationDecodeError::PayloadLayoutMismatch,
      "observation count disagrees with 40-byte payload records");
  }

  ObservationEpoch epoch;
  epoch.receiver = *receiver;
  epoch.header = frame.header;
  epoch.observations.reserve(observation_count);

  for (std::size_t index = 0; index < observation_count; ++index) {
    const std::uint8_t * record =
      payload + sizeof(std::uint32_t) + index * kObservationRecordSize;
    Observation observation;
    observation.system_frequency = readLe16(record);
    observation.prn = readLe16(record + 2);
    observation.pseudorange_m = readLeDouble(record + 4);
    observation.carrier_phase_cycles = readLeDouble(record + 12);
    observation.pseudorange_std_m = static_cast<double>(readLe16(record + 20)) / 100.0;
    observation.carrier_phase_std_cycles =
      static_cast<double>(readLe16(record + 22)) / 10000.0;
    observation.doppler_hz = static_cast<double>(readLeFloat(record + 24));
    observation.cn0_db_hz = static_cast<double>(readLe16(record + 28)) / 100.0;
    observation.lock_time_s = static_cast<double>(readLeFloat(record + 32));
    observation.tracking_status = readLe32(record + 36);

    if (!std::isfinite(observation.pseudorange_m) ||
      !std::isfinite(observation.carrier_phase_cycles) ||
      !std::isfinite(observation.doppler_hz) || !std::isfinite(observation.lock_time_s))
    {
      return failure(
        ObservationDecodeError::NonFiniteMeasurement,
        recordReason(index, "non-finite measurement"));
    }
    if (observation.lock_time_s < 0.0) {
      return failure(
        ObservationDecodeError::InvalidMeasurement,
        recordReason(index, "negative lock time"));
    }

    const std::uint8_t constellation_value = static_cast<std::uint8_t>(
      (observation.tracking_status >> 16U) & kConstellationMask);
    if (constellation_value > static_cast<std::uint8_t>(GnssConstellation::Irnss)) {
      return failure(
        ObservationDecodeError::InvalidConstellation,
        recordReason(index, "reserved constellation value"));
    }
    observation.constellation = static_cast<GnssConstellation>(constellation_value);
    if (!validPrn(observation.constellation, observation.prn)) {
      return failure(
        ObservationDecodeError::InvalidPrn,
        recordReason(index, "PRN outside the constellation range"));
    }

    if (observation.constellation == GnssConstellation::Glonass) {
      if (observation.system_frequency > 13U) {
        return failure(
          ObservationDecodeError::InvalidSystemFrequency,
          recordReason(index, "GLONASS frequency field exceeds channel range"));
      }
      observation.glonass_frequency_channel =
        static_cast<std::int16_t>(observation.system_frequency) - 7;
    } else if (observation.system_frequency != 0U) {
      return failure(
        ObservationDecodeError::InvalidSystemFrequency,
        recordReason(index, "non-GLONASS frequency field is not zero"));
    }

    observation.channel_number = static_cast<std::uint8_t>(
      (observation.tracking_status >> 5U) & 0x1FU);
    observation.signal_type = static_cast<std::uint8_t>(
      (observation.tracking_status >> 21U) & kSignalTypeMask);
    observation.carrier_phase_valid =
      (observation.tracking_status & kCarrierPhaseValidMask) != 0U;
    observation.pseudorange_valid =
      (observation.tracking_status & kPseudorangeValidMask) != 0U;
    observation.l2c_signal = (observation.tracking_status & (1U << 26U)) != 0U;

    if (observation.pseudorange_valid && observation.pseudorange_m <= 0.0) {
      return failure(
        ObservationDecodeError::InvalidMeasurement,
        recordReason(index, "valid pseudorange is not positive"));
    }
    epoch.observations.push_back(observation);
  }

  ObservationDecodeResult result;
  result.epoch = std::move(epoch);
  return result;
}

}  // namespace um982_raw_driver

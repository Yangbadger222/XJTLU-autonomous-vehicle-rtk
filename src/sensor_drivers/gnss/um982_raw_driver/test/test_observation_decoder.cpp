#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "um982_raw_driver/binary_framer.hpp"
#include "um982_raw_driver/observation_decoder.hpp"

namespace
{

struct RecordSpec
{
  std::uint16_t system_frequency = 0;
  std::uint16_t prn = 1;
  double pseudorange_m = 21720097.812;
  double carrier_phase_cycles = -114139892.254585;
  std::uint16_t pseudorange_std_scaled = 52;
  std::uint16_t carrier_phase_std_scaled = 181;
  float doppler_hz = -2263.222F;
  std::uint16_t cn0_scaled = 4270;
  float lock_time_s = 6262.010F;
  std::uint32_t tracking_status = 0;
};

void writeLe16(
  std::vector<std::uint8_t> & data, const std::size_t offset,
  const std::uint16_t value)
{
  data.at(offset) = static_cast<std::uint8_t>(value & 0xFFU);
  data.at(offset + 1U) = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void writeLe32(
  std::vector<std::uint8_t> & data, const std::size_t offset,
  const std::uint32_t value)
{
  data.at(offset) = static_cast<std::uint8_t>(value & 0xFFU);
  data.at(offset + 1U) = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  data.at(offset + 2U) = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  data.at(offset + 3U) = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void writeLe64(
  std::vector<std::uint8_t> & data, const std::size_t offset,
  const std::uint64_t value)
{
  for (std::size_t byte = 0; byte < 8U; ++byte) {
    data.at(offset + byte) = static_cast<std::uint8_t>(value >> (8U * byte));
  }
}

void writeFloat(std::vector<std::uint8_t> & data, const std::size_t offset, const float value)
{
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe32(data, offset, bits);
}

void writeDouble(std::vector<std::uint8_t> & data, const std::size_t offset, const double value)
{
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe64(data, offset, bits);
}

std::uint32_t trackingStatus(
  const std::uint8_t constellation,
  const std::uint8_t signal_type,
  const std::uint8_t channel,
  const bool pseudorange_valid = true,
  const bool carrier_phase_valid = true,
  const bool l2c_signal = false)
{
  return (static_cast<std::uint32_t>(channel & 0x1FU) << 5U) |
         (carrier_phase_valid ? (1U << 10U) : 0U) |
         (pseudorange_valid ? (1U << 12U) : 0U) |
         (static_cast<std::uint32_t>(constellation & 0x7U) << 16U) |
         (static_cast<std::uint32_t>(signal_type & 0x1FU) << 21U) |
         (l2c_signal ? (1U << 26U) : 0U);
}

std::vector<std::uint8_t> makePayload(
  const std::vector<RecordSpec> & records,
  const std::uint32_t declared_count = std::numeric_limits<std::uint32_t>::max())
{
  std::vector<std::uint8_t> payload(
    4U + records.size() * um982_raw_driver::kObservationRecordSize, 0);
  writeLe32(
    payload, 0,
    declared_count == std::numeric_limits<std::uint32_t>::max() ?
    static_cast<std::uint32_t>(records.size()) : declared_count);

  for (std::size_t index = 0; index < records.size(); ++index) {
    const std::size_t offset = 4U + index * um982_raw_driver::kObservationRecordSize;
    const auto & record = records[index];
    writeLe16(payload, offset, record.system_frequency);
    writeLe16(payload, offset + 2U, record.prn);
    writeDouble(payload, offset + 4U, record.pseudorange_m);
    writeDouble(payload, offset + 12U, record.carrier_phase_cycles);
    writeLe16(payload, offset + 20U, record.pseudorange_std_scaled);
    writeLe16(payload, offset + 22U, record.carrier_phase_std_scaled);
    writeFloat(payload, offset + 24U, record.doppler_hz);
    writeLe16(payload, offset + 28U, record.cn0_scaled);
    writeFloat(payload, offset + 32U, record.lock_time_s);
    writeLe32(payload, offset + 36U, record.tracking_status);
  }
  return payload;
}

um982_raw_driver::BinaryFrame makeFrame(
  const std::uint16_t message_id,
  const std::vector<std::uint8_t> & payload)
{
  std::vector<std::uint8_t> bytes(
    um982_raw_driver::kBinaryHeaderSize + payload.size() +
    um982_raw_driver::kBinaryCrcSize, 0);
  bytes[0] = 0xAA;
  bytes[1] = 0x44;
  bytes[2] = 0xB5;
  bytes[3] = 75;
  writeLe16(bytes, 4, message_id);
  writeLe16(bytes, 6, static_cast<std::uint16_t>(payload.size()));
  bytes[8] = 0;
  bytes[9] = 160;
  writeLe16(bytes, 10, 2427);
  writeLe32(bytes, 12, 345678000);
  writeLe32(bytes, 16, 18);
  bytes[21] = 18;
  writeLe16(bytes, 22, 2);
  std::copy(payload.begin(), payload.end(), bytes.begin() + 24);
  const std::uint32_t crc = um982_raw_driver::calculateCrc32(
    bytes.data(), bytes.size() - um982_raw_driver::kBinaryCrcSize);
  writeLe32(bytes, bytes.size() - um982_raw_driver::kBinaryCrcSize, crc);

  um982_raw_driver::BinaryFramer framer;
  auto frames = framer.consume(bytes);
  if (frames.size() != 1U) {
    throw std::runtime_error("test frame did not pass binary framing");
  }
  return std::move(frames.front());
}

}  // namespace

TEST(ObservationDecoder, DecodesAndScalesMasterObservation)
{
  RecordSpec record;
  record.tracking_status = trackingStatus(0, 6, 4, true, true, true);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  ASSERT_TRUE(result.ok()) << result.reason;
  ASSERT_EQ(result.epoch->observations.size(), 1U);
  EXPECT_EQ(result.epoch->receiver, um982_raw_driver::ObservationReceiver::Master);
  EXPECT_EQ(result.epoch->header.week, 2427);
  EXPECT_EQ(result.epoch->header.milliseconds_of_week, 345678000U);
  const auto & observation = result.epoch->observations.front();
  EXPECT_EQ(observation.constellation, um982_raw_driver::GnssConstellation::Gps);
  EXPECT_EQ(observation.prn, 1);
  EXPECT_EQ(observation.signal_type, 6);
  EXPECT_EQ(observation.channel_number, 4);
  EXPECT_NEAR(observation.pseudorange_m, record.pseudorange_m, 1e-9);
  EXPECT_NEAR(observation.carrier_phase_cycles, record.carrier_phase_cycles, 1e-9);
  EXPECT_NEAR(observation.pseudorange_std_m, 0.52, 1e-12);
  EXPECT_NEAR(observation.carrier_phase_std_cycles, 0.0181, 1e-12);
  EXPECT_NEAR(observation.doppler_hz, record.doppler_hz, 1e-6);
  EXPECT_NEAR(observation.cn0_db_hz, 42.70, 1e-12);
  EXPECT_NEAR(observation.lock_time_s, record.lock_time_s, 1e-3);
  EXPECT_TRUE(observation.pseudorange_valid);
  EXPECT_TRUE(observation.carrier_phase_valid);
  EXPECT_TRUE(observation.l2c_signal);
}

TEST(ObservationDecoder, MapsSecondaryAndBaseReceivers)
{
  const auto empty_payload = makePayload({});
  const auto secondary = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVhMessageId, empty_payload));
  const auto base = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVBaseMessageId, empty_payload));

  ASSERT_TRUE(secondary.ok());
  ASSERT_TRUE(base.ok());
  EXPECT_EQ(secondary.epoch->receiver, um982_raw_driver::ObservationReceiver::Secondary);
  EXPECT_EQ(base.epoch->receiver, um982_raw_driver::ObservationReceiver::Base);
}

TEST(ObservationDecoder, DecodesMultipleRecordsAtFortyByteStride)
{
  RecordSpec gps;
  gps.prn = 3;
  gps.pseudorange_m = 21000003.0;
  gps.tracking_status = trackingStatus(0, 0, 1);
  RecordSpec bds;
  bds.prn = 21;
  bds.pseudorange_m = 22000021.0;
  bds.tracking_status = trackingStatus(4, 8, 2);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({gps, bds})));

  ASSERT_TRUE(result.ok()) << result.reason;
  ASSERT_EQ(result.epoch->observations.size(), 2U);
  EXPECT_EQ(result.epoch->observations[0].prn, 3);
  EXPECT_EQ(result.epoch->observations[1].prn, 21);
  EXPECT_EQ(result.epoch->observations[1].constellation, um982_raw_driver::GnssConstellation::Bds);
  EXPECT_EQ(result.epoch->observations[1].signal_type, 8);
  EXPECT_DOUBLE_EQ(result.epoch->observations[1].pseudorange_m, 22000021.0);
}

TEST(ObservationDecoder, ConvertsGlonassFrequencyChannel)
{
  RecordSpec record;
  record.system_frequency = 1;
  record.prn = 38;
  record.tracking_status = trackingStatus(1, 0, 2);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  ASSERT_TRUE(result.ok()) << result.reason;
  const auto & observation = result.epoch->observations.front();
  EXPECT_EQ(observation.constellation, um982_raw_driver::GnssConstellation::Glonass);
  EXPECT_EQ(observation.glonass_frequency_channel, -6);
}

TEST(ObservationDecoder, KeepsInvalidMeasurementFlagsWithoutInventingValidity)
{
  RecordSpec record;
  record.pseudorange_m = 0.0;
  record.carrier_phase_cycles = 0.0;
  record.tracking_status = trackingStatus(0, 0, 0, false, false);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_FALSE(result.epoch->observations.front().pseudorange_valid);
  EXPECT_FALSE(result.epoch->observations.front().carrier_phase_valid);
}

TEST(ObservationDecoder, RejectsUnsupportedMessage)
{
  um982_raw_driver::BinaryFrame frame;
  frame.header.message_id = 37;

  const auto result = um982_raw_driver::decodeObservationFrame(frame);

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::UnsupportedMessage);
}

TEST(ObservationDecoder, RejectsObservationCountMismatch)
{
  RecordSpec record;
  record.tracking_status = trackingStatus(0, 0, 0);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record}, 2)));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::PayloadLayoutMismatch);
}

TEST(ObservationDecoder, RejectsFrameByteCountMismatch)
{
  RecordSpec record;
  record.tracking_status = trackingStatus(0, 0, 0);
  auto frame = makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record}));
  frame.bytes.pop_back();

  const auto result = um982_raw_driver::decodeObservationFrame(frame);

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::FrameSizeMismatch);
}

TEST(ObservationDecoder, RejectsNonFiniteMeasurement)
{
  RecordSpec record;
  record.pseudorange_m = std::numeric_limits<double>::quiet_NaN();
  record.tracking_status = trackingStatus(0, 0, 0);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::NonFiniteMeasurement);
}

TEST(ObservationDecoder, RejectsReservedConstellation)
{
  RecordSpec record;
  record.tracking_status = trackingStatus(7, 0, 0);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::InvalidConstellation);
}

TEST(ObservationDecoder, RejectsPrnOutsideConstellationRange)
{
  RecordSpec record;
  record.prn = 99;
  record.tracking_status = trackingStatus(0, 0, 0);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::InvalidPrn);
}

TEST(ObservationDecoder, RejectsFrequencyOnNonGlonassObservation)
{
  RecordSpec record;
  record.system_frequency = 7;
  record.tracking_status = trackingStatus(0, 0, 0);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::InvalidSystemFrequency);
}

TEST(ObservationDecoder, RejectsNonPositiveValidPseudorange)
{
  RecordSpec record;
  record.pseudorange_m = 0.0;
  record.tracking_status = trackingStatus(0, 0, 0, true, false);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::InvalidMeasurement);
}

TEST(ObservationDecoder, RejectsNegativeLockTime)
{
  RecordSpec record;
  record.lock_time_s = -0.5F;
  record.tracking_status = trackingStatus(0, 0, 0);

  const auto result = um982_raw_driver::decodeObservationFrame(
    makeFrame(um982_raw_driver::kObsVmMessageId, makePayload({record})));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, um982_raw_driver::ObservationDecodeError::InvalidMeasurement);
}

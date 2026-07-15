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
#include "um982_raw_driver/ephemeris_decoder.hpp"

namespace
{

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
  for (std::size_t byte = 0; byte < 4U; ++byte) {
    data.at(offset + byte) = static_cast<std::uint8_t>(value >> (8U * byte));
  }
}

void writeDouble(std::vector<std::uint8_t> & data, const std::size_t offset, const double value)
{
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t byte = 0; byte < 8U; ++byte) {
    data.at(offset + byte) = static_cast<std::uint8_t>(bits >> (8U * byte));
  }
}

um982_raw_driver::BinaryFrame makeFrame(
  const std::uint16_t message_id,
  const std::vector<std::uint8_t> & payload)
{
  std::vector<std::uint8_t> bytes(
    um982_raw_driver::kBinaryHeaderSize + payload.size() + um982_raw_driver::kBinaryCrcSize,
    0);
  bytes[0] = 0xAA;
  bytes[1] = 0x44;
  bytes[2] = 0xB5;
  bytes[3] = 77;
  writeLe16(bytes, 4, message_id);
  writeLe16(bytes, 6, static_cast<std::uint16_t>(payload.size()));
  bytes[9] = 160;
  writeLe16(bytes, 10, 2427);
  writeLe32(bytes, 12, 345678000);
  bytes[21] = 18;
  std::copy(payload.begin(), payload.end(), bytes.begin() + um982_raw_driver::kBinaryHeaderSize);
  writeLe32(
    bytes, bytes.size() - um982_raw_driver::kBinaryCrcSize,
    um982_raw_driver::calculateCrc32(
      bytes.data(), bytes.size() - um982_raw_driver::kBinaryCrcSize));
  um982_raw_driver::BinaryFramer framer;
  auto frames = framer.consume(bytes);
  if (frames.size() != 1U) {
    throw std::runtime_error("test frame did not pass binary framing");
  }
  return std::move(frames.front());
}

void writeSharedOrbit(std::vector<std::uint8_t> & payload)
{
  writeDouble(payload, 32, 345600.0);
  writeDouble(payload, 40, 26560000.0);
  writeDouble(payload, 48, 4.4e-9);
  writeDouble(payload, 56, 0.46);
  writeDouble(payload, 64, 0.0074);
  writeDouble(payload, 72, -2.54);
  writeDouble(payload, 80, 1.0e-7);
  writeDouble(payload, 88, 9.0e-6);
  writeDouble(payload, 96, 207.0);
  writeDouble(payload, 104, -1.7);
  writeDouble(payload, 112, -2.0e-8);
  writeDouble(payload, 120, 1.1e-7);
  writeDouble(payload, 128, 0.97);
  writeDouble(payload, 136, 4.0e-10);
  writeDouble(payload, 144, -0.003);
  writeDouble(payload, 152, -8.0e-9);
}

std::vector<std::uint8_t> makeGpsLikePayload(const std::uint32_t prn)
{
  std::vector<std::uint8_t> payload(224, 0);
  writeLe32(payload, 0, prn);
  writeDouble(payload, 4, 345000.0);
  writeLe32(payload, 16, 30);
  writeLe32(payload, 20, 30);
  writeLe32(payload, 24, 2427);
  writeLe32(payload, 28, 379);
  writeSharedOrbit(payload);
  writeLe32(payload, 160, 30);
  writeDouble(payload, 164, 345600.0);
  writeDouble(payload, 172, 2.3e-9);
  writeDouble(payload, 180, -2.8e-4);
  writeDouble(payload, 188, -9.3e-12);
  writeDouble(payload, 196, 0.0);
  writeDouble(payload, 208, 1.458e-4);
  writeDouble(payload, 216, 4.0);
  return payload;
}

std::vector<std::uint8_t> makeBdsPayload()
{
  std::vector<std::uint8_t> payload(232, 0);
  writeLe32(payload, 0, 19);
  writeDouble(payload, 4, 345000.0);
  writeLe32(payload, 16, 7);
  writeLe32(payload, 20, 7);
  writeLe32(payload, 24, 2427);
  writeSharedOrbit(payload);
  writeLe32(payload, 160, 8);
  writeDouble(payload, 164, 345600.0);
  writeDouble(payload, 172, 4.9e-8);
  writeDouble(payload, 180, -1.4e-7);
  writeDouble(payload, 188, 8.2e-4);
  writeDouble(payload, 196, 8.2e-14);
  writeDouble(payload, 204, 0.0);
  writeDouble(payload, 216, 7.29e-5);
  writeDouble(payload, 224, 4.0);
  return payload;
}

std::vector<std::uint8_t> makeGalileoPayload()
{
  std::vector<std::uint8_t> payload(220, 0);
  writeLe32(payload, 0, 36);
  writeLe32(payload, 4, 1);
  writeLe32(payload, 8, 1);
  writeLe32(payload, 20, 107);
  writeLe32(payload, 24, 356400);
  writeDouble(payload, 28, 5440.61113);
  writeDouble(payload, 36, 2.47e-9);
  writeDouble(payload, 44, -1.46);
  writeDouble(payload, 52, 2.84e-4);
  writeDouble(payload, 60, -1.32);
  writeDouble(payload, 68, -8.5e-6);
  writeDouble(payload, 76, 9.0e-6);
  writeDouble(payload, 84, 159.0);
  writeDouble(payload, 92, -183.9);
  writeDouble(payload, 100, 9.3e-9);
  writeDouble(payload, 108, -3.9e-8);
  writeDouble(payload, 116, 0.996);
  writeDouble(payload, 124, -2.6e-10);
  writeDouble(payload, 132, -1.2);
  writeDouble(payload, 140, -5.4e-9);
  writeLe32(payload, 148, 356400);
  writeDouble(payload, 152, -3.1e-4);
  writeDouble(payload, 160, -5.3e-12);
  writeDouble(payload, 168, 0.0);
  writeLe32(payload, 176, 356400);
  writeDouble(payload, 180, -3.0e-4);
  writeDouble(payload, 188, -5.2e-12);
  writeDouble(payload, 196, 0.0);
  writeDouble(payload, 204, 5.8e-9);
  writeDouble(payload, 212, 6.7e-9);
  return payload;
}

std::vector<std::uint8_t> makeGlonassPayload()
{
  std::vector<std::uint8_t> payload(144, 0);
  writeLe16(payload, 0, 40);
  writeLe16(payload, 2, 12);
  writeLe16(payload, 6, 2427);
  writeLe32(payload, 8, 114318000);
  writeLe32(payload, 20, 43);
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    writeDouble(payload, 28 + axis * 8U, 1.8e7 + static_cast<double>(axis) * 1.0e6);
    writeDouble(payload, 52 + axis * 8U, 300.0 + static_cast<double>(axis));
    writeDouble(payload, 76 + axis * 8U, -1.0e-6 * static_cast<double>(axis + 1U));
  }
  writeDouble(payload, 100, -9.6e-5);
  writeDouble(payload, 108, -2.7e-9);
  writeDouble(payload, 116, 9.0e-13);
  writeLe32(payload, 124, 38040);
  writeLe32(payload, 140, 12);
  return payload;
}

}  // namespace

TEST(EphemerisDecoder, DecodesGpsAndQzssKeplerianEphemerides)
{
  const auto gps = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGpsEphemerisMessageId, makeGpsLikePayload(10)));
  ASSERT_TRUE(gps.ok()) << gps.reason;
  EXPECT_EQ(gps.ephemeris->constellation, um982_raw_driver::GnssConstellation::Gps);
  EXPECT_EQ(gps.ephemeris->reference_frame, um982_raw_driver::EphemerisReferenceFrame::Wgs84);
  EXPECT_EQ(gps.ephemeris->prn, 10);
  EXPECT_NEAR(gps.ephemeris->semi_major_axis_m, 26560000.0, 1e-6);
  EXPECT_NEAR(gps.ephemeris->clock_bias_s, -2.8e-4, 1e-15);
  EXPECT_TRUE(gps.ephemeris->group_delay_1_valid);
  EXPECT_TRUE(gps.ephemeris->ura_variance_valid);

  const auto qzss_from_gps = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGpsEphemerisMessageId, makeGpsLikePayload(36)));
  ASSERT_TRUE(qzss_from_gps.ok()) << qzss_from_gps.reason;
  EXPECT_EQ(qzss_from_gps.ephemeris->constellation, um982_raw_driver::GnssConstellation::Qzss);
  EXPECT_EQ(qzss_from_gps.ephemeris->prn, 196);

  const auto qzss = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kQzssEphemerisMessageId, makeGpsLikePayload(4)));
  ASSERT_TRUE(qzss.ok()) << qzss.reason;
  EXPECT_EQ(qzss.ephemeris->constellation, um982_raw_driver::GnssConstellation::Qzss);
  EXPECT_EQ(qzss.ephemeris->reference_frame, um982_raw_driver::EphemerisReferenceFrame::Jgs);
  EXPECT_EQ(qzss.ephemeris->prn, 196);
}

TEST(EphemerisDecoder, DecodesBdsOffsets)
{
  const auto result = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kBdsEphemerisMessageId, makeBdsPayload()));
  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.ephemeris->constellation, um982_raw_driver::GnssConstellation::Bds);
  EXPECT_EQ(
    result.ephemeris->reference_frame, um982_raw_driver::EphemerisReferenceFrame::Cgcs2000);
  EXPECT_EQ(result.ephemeris->prn, 19);
  EXPECT_NEAR(result.ephemeris->group_delay_2_s, -1.4e-7, 1e-18);
  EXPECT_TRUE(result.ephemeris->group_delay_2_valid);
  EXPECT_NEAR(result.ephemeris->clock_bias_s, 8.2e-4, 1e-15);
}

TEST(EphemerisDecoder, DecodesGalileoRootAAndInavClock)
{
  const auto result = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGalileoEphemerisMessageId, makeGalileoPayload()));
  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.ephemeris->week, 2427U);
  EXPECT_EQ(result.ephemeris->reference_frame, um982_raw_driver::EphemerisReferenceFrame::Gtrf);
  EXPECT_NEAR(result.ephemeris->semi_major_axis_m, 5440.61113 * 5440.61113, 1e-5);
  EXPECT_NEAR(result.ephemeris->clock_bias_s, -3.0e-4, 1e-15);
  EXPECT_FALSE(result.ephemeris->ura_variance_valid);
}

TEST(EphemerisDecoder, DecodesGlonassStateVector)
{
  const auto result = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGlonassEphemerisMessageId, makeGlonassPayload()));
  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.ephemeris->model, um982_raw_driver::EphemerisModel::Glonass);
  EXPECT_EQ(
    result.ephemeris->reference_frame, um982_raw_driver::EphemerisReferenceFrame::Pz90_02);
  EXPECT_EQ(result.ephemeris->prn, 40);
  EXPECT_EQ(result.ephemeris->glonass_frequency_channel, 5);
  EXPECT_NEAR(result.ephemeris->toe_s, 114318.0, 1e-12);
  EXPECT_NEAR(result.ephemeris->position_ecef_m[2], 2.0e7, 1e-6);
}

TEST(EphemerisDecoder, RejectsUnsupportedAndWrongPayloadLength)
{
  const auto unsupported = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(999, std::vector<std::uint8_t>(16, 0)));
  EXPECT_EQ(unsupported.error, um982_raw_driver::EphemerisDecodeError::UnsupportedMessage);

  const auto wrong_size = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGpsEphemerisMessageId, std::vector<std::uint8_t>(223, 0)));
  EXPECT_EQ(wrong_size.error, um982_raw_driver::EphemerisDecodeError::PayloadLayoutMismatch);

  auto truncated_frame = makeFrame(
    um982_raw_driver::kGpsEphemerisMessageId, makeGpsLikePayload(10));
  truncated_frame.bytes.pop_back();
  const auto truncated = um982_raw_driver::decodeEphemerisFrame(truncated_frame);
  EXPECT_EQ(truncated.error, um982_raw_driver::EphemerisDecodeError::FrameSizeMismatch);
}

TEST(EphemerisDecoder, RejectsInvalidPrnAndNonFiniteOrbit)
{
  auto bad_prn = makeGpsLikePayload(43);
  const auto prn_result = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGpsEphemerisMessageId, bad_prn));
  EXPECT_EQ(prn_result.error, um982_raw_driver::EphemerisDecodeError::InvalidPrn);

  auto non_finite = makeGpsLikePayload(10);
  writeDouble(non_finite, 64, std::numeric_limits<double>::quiet_NaN());
  const auto finite_result = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGpsEphemerisMessageId, non_finite));
  EXPECT_EQ(finite_result.error, um982_raw_driver::EphemerisDecodeError::NonFiniteField);
}

TEST(EphemerisDecoder, RejectsInvalidGlonassTimeAndGalileoFlags)
{
  auto glonass = makeGlonassPayload();
  writeLe32(glonass, 124, 86401);
  const auto glonass_result = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGlonassEphemerisMessageId, glonass));
  EXPECT_EQ(glonass_result.error, um982_raw_driver::EphemerisDecodeError::InvalidField);

  auto galileo = makeGalileoPayload();
  writeLe32(galileo, 4, 0);
  writeLe32(galileo, 8, 0);
  const auto galileo_result = um982_raw_driver::decodeEphemerisFrame(
    makeFrame(um982_raw_driver::kGalileoEphemerisMessageId, galileo));
  EXPECT_EQ(galileo_result.error, um982_raw_driver::EphemerisDecodeError::InvalidField);
}

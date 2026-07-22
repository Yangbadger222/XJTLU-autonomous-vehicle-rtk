#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "um982_raw_driver/binary_framer.hpp"

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
  data.at(offset) = static_cast<std::uint8_t>(value & 0xFFU);
  data.at(offset + 1U) = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  data.at(offset + 2U) = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  data.at(offset + 3U) = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

std::vector<std::uint8_t> makeFrame(
  const std::uint16_t message_id,
  const std::vector<std::uint8_t> & payload)
{
  std::vector<std::uint8_t> frame(
    um982_raw_driver::kBinaryHeaderSize + payload.size() +
    um982_raw_driver::kBinaryCrcSize, 0);
  frame[0] = 0xAA;
  frame[1] = 0x44;
  frame[2] = 0xB5;
  frame[3] = 73;
  writeLe16(frame, 4, message_id);
  writeLe16(frame, 6, static_cast<std::uint16_t>(payload.size()));
  frame[8] = 0;
  frame[9] = 160;
  writeLe16(frame, 10, 2427);
  writeLe32(frame, 12, 345678000);
  writeLe32(frame, 16, 18);
  frame[20] = 0;
  frame[21] = 18;
  writeLe16(frame, 22, 3);
  std::copy(payload.begin(), payload.end(), frame.begin() + 24);
  const std::uint32_t crc = um982_raw_driver::calculateCrc32(
    frame.data(), frame.size() - um982_raw_driver::kBinaryCrcSize);
  writeLe32(frame, frame.size() - um982_raw_driver::kBinaryCrcSize, crc);
  return frame;
}

}  // namespace

TEST(BinaryFramer, MatchesOfficialCrcAlgorithmCheckValue)
{
  const std::string input = "123456789";
  EXPECT_EQ(
    um982_raw_driver::calculateCrc32(
      reinterpret_cast<const std::uint8_t *>(input.data()), input.size()),
    0x2DFD2D88U);
}

TEST(BinaryFramer, MatchesOfficialUm982VersionAsciiFixture)
{
  const std::string payload =
    "VERSIONA,94,GPS,FINE,2190,117325000,0,0,18,160;\"UM982\","
    "\"R4.10Build5251\",\"HRPT00-S10C-P\",\"-\",\"ffff48ffff0fffff\",\"2021/11/26\"";

  EXPECT_EQ(
    um982_raw_driver::calculateCrc32(
      reinterpret_cast<const std::uint8_t *>(payload.data()), payload.size()),
    0xE195B254U);
}

TEST(BinaryFramer, ParsesHeaderAndPayload)
{
  um982_raw_driver::BinaryFramer framer;
  const auto bytes = makeFrame(12, {1, 2, 3, 4});

  const auto frames = framer.consume(bytes);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].header.cpu_idle_percent, 73);
  EXPECT_EQ(frames[0].header.message_id, 12);
  EXPECT_EQ(frames[0].header.payload_length, 4);
  EXPECT_EQ(frames[0].header.time_status, 160);
  EXPECT_EQ(frames[0].header.week, 2427);
  EXPECT_EQ(frames[0].header.milliseconds_of_week, 345678000U);
  EXPECT_EQ(frames[0].header.format_version, 18U);
  EXPECT_EQ(frames[0].header.leap_seconds, 18);
  EXPECT_EQ(frames[0].header.output_delay_ms, 3);
  EXPECT_EQ(frames[0].bytes, bytes);
  EXPECT_EQ(framer.bufferedBytes(), 0U);
}

TEST(BinaryFramer, AcceptsOneByteFragments)
{
  um982_raw_driver::BinaryFramer framer;
  const auto bytes = makeFrame(13, {9, 8, 7});
  std::vector<um982_raw_driver::BinaryFrame> output;

  for (const std::uint8_t byte : bytes) {
    const auto frames = framer.consume(&byte, 1);
    output.insert(output.end(), frames.begin(), frames.end());
  }

  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].header.message_id, 13);
}

TEST(BinaryFramer, ExtractsCoalescedFrames)
{
  um982_raw_driver::BinaryFramer framer;
  const auto first = makeFrame(12, {1});
  const auto second = makeFrame(284, {2, 3});
  std::vector<std::uint8_t> bytes = first;
  bytes.insert(bytes.end(), second.begin(), second.end());

  const auto frames = framer.consume(bytes);

  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].header.message_id, 12);
  EXPECT_EQ(frames[1].header.message_id, 284);
  EXPECT_EQ(frames[1].stream_offset, first.size());
}

TEST(BinaryFramer, PreservesSplitSyncAfterNoise)
{
  um982_raw_driver::BinaryFramer framer;
  const auto frame = makeFrame(138, {4, 5});
  std::vector<std::uint8_t> first_chunk{0x10, 0x20, 0xAA, 0x44};

  EXPECT_TRUE(framer.consume(first_chunk).empty());
  const auto frames = framer.consume(frame.data() + 2, frame.size() - 2);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].header.message_id, 138);
  EXPECT_EQ(frames[0].stream_offset, 2U);
  EXPECT_EQ(framer.stats().bytes_discarded, 2U);
}

TEST(BinaryFramer, RejectsBadCrcAndResynchronizes)
{
  um982_raw_driver::BinaryFramer framer;
  auto bad = makeFrame(12, {1, 2});
  bad.back() ^= 0x80U;
  const auto good = makeFrame(13, {3, 4});
  bad.insert(bad.end(), good.begin(), good.end());

  const auto frames = framer.consume(bad);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].header.message_id, 13);
  EXPECT_EQ(framer.stats().crc_failures, 1U);
}

TEST(BinaryFramer, RejectsOversizedLengthAndResynchronizes)
{
  um982_raw_driver::BinaryFramer framer(64, 128);
  std::vector<std::uint8_t> oversized(um982_raw_driver::kBinaryHeaderSize, 0);
  oversized[0] = 0xAA;
  oversized[1] = 0x44;
  oversized[2] = 0xB5;
  writeLe16(oversized, 6, 65);
  const auto good = makeFrame(12, {1});
  oversized.insert(oversized.end(), good.begin(), good.end());

  const auto frames = framer.consume(oversized);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].header.message_id, 12);
  EXPECT_EQ(framer.stats().length_failures, 1U);
}

TEST(BinaryFramer, BoundsUnframedInput)
{
  um982_raw_driver::BinaryFramer framer(64, 128);
  const std::vector<std::uint8_t> noise(1000, 0x55);

  EXPECT_TRUE(framer.consume(noise).empty());

  EXPECT_LE(framer.bufferedBytes(), 128U);
  EXPECT_EQ(framer.stats().buffer_overflows, 1U);
  EXPECT_EQ(framer.stats().bytes_received, 1000U);
  EXPECT_EQ(framer.stats().bytes_discarded, 1000U);
}

TEST(BinaryFramer, StaysBoundedUnderDeterministicRandomNoiseAndRecovers)
{
  um982_raw_driver::BinaryFramer framer(256, 512);
  std::mt19937 generator(982);
  std::uniform_int_distribution<int> length_distribution(0, 80);
  std::uniform_int_distribution<int> byte_distribution(0, 255);

  for (int iteration = 0; iteration < 2000; ++iteration) {
    std::vector<std::uint8_t> noise(
      static_cast<std::size_t>(length_distribution(generator)));
    std::generate(
      noise.begin(), noise.end(), [&]() {
        return static_cast<std::uint8_t>(byte_distribution(generator));
      });
    EXPECT_NO_THROW(framer.consume(noise));
    EXPECT_LE(framer.bufferedBytes(), 512U);
  }

  const std::vector<std::uint8_t> flush(600, 0);
  EXPECT_TRUE(framer.consume(flush).empty());
  const auto frames = framer.consume(makeFrame(284, {1, 2, 3}));
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].header.message_id, 284);
}

TEST(BinaryFramer, ValidatesConfiguredBounds)
{
  EXPECT_THROW(um982_raw_driver::BinaryFramer(65, 92), std::invalid_argument);
  EXPECT_THROW(um982_raw_driver::BinaryFramer(65536, 131072), std::invalid_argument);
}

TEST(BinaryFramer, ResetDiscardsPartialFrameAcrossReconnect)
{
  um982_raw_driver::BinaryFramer framer;
  const auto frame = makeFrame(12, {1, 2, 3});
  ASSERT_TRUE(framer.consume(frame.data(), 10).empty());

  framer.reset();
  const auto frames = framer.consume(frame);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].stream_offset, 10U);
  EXPECT_EQ(framer.stats().bytes_discarded, 10U);
}

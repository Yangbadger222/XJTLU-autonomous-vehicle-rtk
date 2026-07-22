#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "um982_raw_driver/mixed_stream_framer.hpp"

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
    um982_raw_driver::kBinaryCrcSize, 0U);
  frame[0] = 0xAA;
  frame[1] = 0x44;
  frame[2] = 0xB5;
  frame[3] = 73;
  writeLe16(frame, 4, message_id);
  writeLe16(frame, 6, static_cast<std::uint16_t>(payload.size()));
  frame[9] = 160;
  writeLe16(frame, 10, 2427);
  writeLe32(frame, 12, 345678000);
  writeLe32(frame, 16, 18);
  frame[21] = 18;
  writeLe16(frame, 22, 3);
  std::copy(payload.begin(), payload.end(), frame.begin() + 24);
  const std::uint32_t crc = um982_raw_driver::calculateCrc32(
    frame.data(), frame.size() - um982_raw_driver::kBinaryCrcSize);
  writeLe32(frame, frame.size() - um982_raw_driver::kBinaryCrcSize, crc);
  return frame;
}

std::vector<std::uint8_t> bytes(const std::string & text)
{
  return std::vector<std::uint8_t>(text.begin(), text.end());
}

}  // namespace

TEST(MixedStreamFramer, AcceptsAsciiAndBinaryOneByteFragments)
{
  um982_raw_driver::MixedStreamFramer framer;
  const std::string first = "$GNGGA,one*00\r\n";
  const auto binary = makeFrame(12, {1U, 2U, 3U});
  const std::string second = "#UNIHEADINGA,two*00000000\r\n";
  std::vector<std::uint8_t> stream = bytes(first);
  stream.insert(stream.end(), binary.begin(), binary.end());
  const auto second_bytes = bytes(second);
  stream.insert(stream.end(), second_bytes.begin(), second_bytes.end());

  std::vector<um982_raw_driver::MixedStreamItem> output;
  for (const auto byte : stream) {
    const auto items = framer.consume(&byte, 1U);
    output.insert(output.end(), items.begin(), items.end());
  }

  ASSERT_EQ(output.size(), 3U);
  EXPECT_EQ(output[0].kind, um982_raw_driver::MixedStreamItemKind::AsciiLine);
  EXPECT_EQ(output[0].ascii_line, first);
  EXPECT_EQ(output[1].kind, um982_raw_driver::MixedStreamItemKind::BinaryFrame);
  EXPECT_EQ(output[1].binary_frame.header.message_id, 12U);
  EXPECT_EQ(output[1].binary_frame.bytes, binary);
  EXPECT_EQ(output[2].ascii_line, second);
  EXPECT_EQ(framer.bufferedBytes(), 0U);
}

TEST(MixedStreamFramer, ExtractsArbitrarilyChunkedCoalescedItems)
{
  um982_raw_driver::MixedStreamFramer framer;
  const std::string first = "$GNRMC,first*00\n";
  const auto binary = makeFrame(284, {4U, 5U, 6U, 7U});
  const std::string second = "$GPTHS,second*00\r\n";
  std::vector<std::uint8_t> stream = bytes(first);
  stream.insert(stream.end(), binary.begin(), binary.end());
  const auto second_bytes = bytes(second);
  stream.insert(stream.end(), second_bytes.begin(), second_bytes.end());

  std::mt19937 random(982U);
  std::uniform_int_distribution<std::size_t> chunk_size(1U, 11U);
  std::vector<um982_raw_driver::MixedStreamItem> output;
  for (std::size_t offset = 0; offset < stream.size(); ) {
    const std::size_t count = std::min(chunk_size(random), stream.size() - offset);
    const auto items = framer.consume(stream.data() + offset, count);
    output.insert(output.end(), items.begin(), items.end());
    offset += count;
  }

  ASSERT_EQ(output.size(), 3U);
  EXPECT_EQ(output[0].ascii_line, first);
  EXPECT_EQ(output[1].binary_frame.header.message_id, 284U);
  EXPECT_EQ(output[2].ascii_line, second);
  EXPECT_EQ(output[0].stream_offset, 0U);
  EXPECT_EQ(output[1].stream_offset, first.size());
  EXPECT_EQ(output[2].stream_offset, first.size() + binary.size());
}

TEST(MixedStreamFramer, BinaryPayloadNeverLeaksAsciiMarkersOrNewlines)
{
  um982_raw_driver::MixedStreamFramer framer;
  const std::vector<std::uint8_t> payload{
    '$', 'G', 'N', 'G', 'G', 'A', ',', '\r', '\n', '#', 'U', 'N', 'I',
    0xAA, 0x44, 0xB5, '$', 'G', 'P', 'T', 'H', 'S', '\n'};
  const auto binary = makeFrame(13, payload);
  const std::string line = "$GNGGA,after*00\r\n";
  std::vector<std::uint8_t> stream = binary;
  const auto line_bytes = bytes(line);
  stream.insert(stream.end(), line_bytes.begin(), line_bytes.end());

  const auto output = framer.consume(stream);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_EQ(output[0].kind, um982_raw_driver::MixedStreamItemKind::BinaryFrame);
  EXPECT_EQ(output[0].binary_frame.bytes, binary);
  EXPECT_EQ(output[1].kind, um982_raw_driver::MixedStreamItemKind::AsciiLine);
  EXPECT_EQ(output[1].ascii_line, line);
  EXPECT_EQ(framer.stats().ascii_lines_emitted, 1U);
  EXPECT_EQ(framer.stats().binary_frames_emitted, 1U);
}

TEST(MixedStreamFramer, RejectsBadCrcWithoutLeakingItsPayload)
{
  um982_raw_driver::MixedStreamFramer framer;
  auto bad = makeFrame(12, {'$', 'B', 'A', 'D', '\n', '#', 'B', 'A', 'D', '\n'});
  bad.back() ^= 0x80U;
  const auto good = makeFrame(13, {8U, 9U});
  const std::string line = "$GNRMC,good*00\r\n";
  std::vector<std::uint8_t> stream = bad;
  stream.insert(stream.end(), good.begin(), good.end());
  const auto line_bytes = bytes(line);
  stream.insert(stream.end(), line_bytes.begin(), line_bytes.end());

  const auto output = framer.consume(stream);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_EQ(output[0].binary_frame.header.message_id, 13U);
  EXPECT_EQ(output[1].ascii_line, line);
  EXPECT_EQ(framer.stats().crc_failures, 1U);
  EXPECT_EQ(framer.stats().ascii_lines_emitted, 1U);
  EXPECT_EQ(framer.stats().bytes_discarded, bad.size());
}

TEST(MixedStreamFramer, RecoversFromNoiseOversizeAsciiAndSplitSync)
{
  um982_raw_driver::MixedStreamFramer framer(64U, 128U, 24U);
  const auto good = makeFrame(12, {1U});
  std::vector<std::uint8_t> first = bytes("noise$012345678901234567890123456789\nmore");
  first.push_back(0xAA);
  first.push_back(0x44);

  EXPECT_TRUE(framer.consume(first).empty());
  const auto output = framer.consume(good.data() + 2U, good.size() - 2U);

  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].binary_frame.header.message_id, 12U);
  EXPECT_EQ(framer.stats().ascii_overflows, 1U);
  EXPECT_GT(framer.stats().bytes_discarded, 0U);
}

TEST(MixedStreamFramer, CorruptAsciiCannotConsumeFollowingBinary)
{
  um982_raw_driver::MixedStreamFramer framer;
  const auto good = makeFrame(106, {2U, 3U});
  std::vector<std::uint8_t> stream = bytes("$GNGGA,incomplete");
  stream.insert(stream.end(), good.begin(), good.end());

  const auto output = framer.consume(stream);

  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output[0].binary_frame.header.message_id, 106U);
  EXPECT_EQ(framer.stats().invalid_ascii_lines, 1U);
}

TEST(MixedStreamFramer, BoundsUnframedInput)
{
  um982_raw_driver::MixedStreamFramer framer(32U, 64U, 24U);
  std::vector<std::uint8_t> noise(256U, 0x10U);

  EXPECT_TRUE(framer.consume(noise).empty());
  EXPECT_LE(framer.bufferedBytes(), 2U);
  EXPECT_EQ(framer.stats().buffer_overflows, 1U);
}

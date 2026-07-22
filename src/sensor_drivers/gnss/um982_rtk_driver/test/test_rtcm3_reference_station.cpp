#include "um982_rtk_driver/rtcm3_reference_station.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace
{

void setUnsignedBits(
  std::vector<std::uint8_t> & data, const std::size_t position,
  const std::size_t width, const std::uint64_t value)
{
  for (std::size_t index = 0; index < width; ++index) {
    const std::size_t bit = position + index;
    const std::uint8_t mask = static_cast<std::uint8_t>(1U << (7U - bit % 8U));
    if (((value >> (width - index - 1U)) & 1U) != 0U) {
      data[bit / 8U] |= mask;
    }
  }
}

void setSigned38Bits(
  std::vector<std::uint8_t> & data, const std::size_t position,
  const std::int64_t value)
{
  constexpr std::uint64_t mask = (std::uint64_t{1} << 38U) - 1U;
  setUnsignedBits(data, position, 38U, static_cast<std::uint64_t>(value) & mask);
}

std::vector<std::uint8_t> referenceFrame(
  const std::uint16_t type, const std::int64_t x, const std::int64_t y,
  const std::int64_t z, const std::uint16_t height = 0U)
{
  const std::size_t payload_size = type == 1006U ? 21U : 19U;
  std::vector<std::uint8_t> frame(3U + payload_size, 0U);
  frame[0] = 0xD3U;
  frame[1] = static_cast<std::uint8_t>((payload_size >> 8U) & 0x03U);
  frame[2] = static_cast<std::uint8_t>(payload_size & 0xFFU);
  setUnsignedBits(frame, 24U, 12U, type);
  setUnsignedBits(frame, 36U, 12U, 42U);
  setUnsignedBits(frame, 48U, 6U, 20U);
  setUnsignedBits(frame, 54U, 1U, 1U);
  setUnsignedBits(frame, 55U, 1U, 1U);
  setUnsignedBits(frame, 56U, 1U, 1U);
  setUnsignedBits(frame, 57U, 1U, 1U);
  setSigned38Bits(frame, 58U, x);
  setSigned38Bits(frame, 98U, y);
  setSigned38Bits(frame, 138U, z);
  if (type == 1006U) {
    setUnsignedBits(frame, 176U, 16U, height);
  }
  const std::uint32_t crc = um982_rtk_driver::crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<std::uint8_t>((crc >> 16U) & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>((crc >> 8U) & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  return frame;
}

}  // namespace

TEST(Rtcm3ReferenceStation, Crc24QMatchesPublishedCheckValue)
{
  const std::vector<std::uint8_t> text{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(um982_rtk_driver::crc24q(text.data(), text.size()), 0xCDE703U);
}

TEST(Rtcm3ReferenceStation, DecodesSigned1005Coordinates)
{
  const auto result = um982_rtk_driver::decodeRtcm3ReferenceStation(
    referenceFrame(1005U, 11111111111LL, -22222222222LL, 33333333333LL));

  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.station->station_id, 42U);
  EXPECT_EQ(result.station->itrf_realization, 20U);
  EXPECT_TRUE(result.station->gps_indicator);
  EXPECT_TRUE(result.station->glonass_indicator);
  EXPECT_TRUE(result.station->galileo_indicator);
  EXPECT_TRUE(result.station->reference_station_indicator);
  EXPECT_DOUBLE_EQ(result.station->position_ecef_m[0], 1111111.1111);
  EXPECT_DOUBLE_EQ(result.station->position_ecef_m[1], -2222222.2222);
  EXPECT_DOUBLE_EQ(result.station->position_ecef_m[2], 3333333.3333);
  EXPECT_FALSE(result.station->antenna_height_valid);
}

TEST(Rtcm3ReferenceStation, Decodes1006AntennaHeight)
{
  const auto result = um982_rtk_driver::decodeRtcm3ReferenceStation(
    referenceFrame(1006U, -27000000000LL, 43000000000LL, 38000000000LL, 12345U));

  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.station->message_type, 1006U);
  EXPECT_TRUE(result.station->antenna_height_valid);
  EXPECT_DOUBLE_EQ(result.station->antenna_height_m, 1.2345);
}

TEST(Rtcm3ReferenceStation, HandlesArbitraryFragmentsAndStickyFrames)
{
  const auto first = referenceFrame(1005U, 1, 2, 3);
  const auto second = referenceFrame(1006U, 4, 5, 6, 7U);
  um982_rtk_driver::Rtcm3Framer fragmented;
  std::vector<std::vector<std::uint8_t>> output;
  for (const auto byte : first) {
    auto frames = fragmented.consume(&byte, 1U);
    output.insert(output.end(), frames.begin(), frames.end());
  }
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front(), first);

  std::vector<std::uint8_t> sticky = second;
  sticky.insert(sticky.end(), first.begin(), first.end());
  const auto frames = fragmented.consume(sticky);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0], second);
  EXPECT_EQ(frames[1], first);
}

TEST(Rtcm3ReferenceStation, RejectsBadCrcAndResynchronizes)
{
  auto bad = referenceFrame(1005U, 1, 2, 3);
  bad[10] ^= 0x40U;
  const auto good = referenceFrame(1006U, 4, 5, 6, 7U);
  bad.insert(bad.end(), good.begin(), good.end());

  um982_rtk_driver::Rtcm3Framer framer;
  const auto frames = framer.consume(bad);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front(), good);
  EXPECT_EQ(framer.stats().crc_failures, 1U);
  EXPECT_GT(framer.stats().bytes_discarded, 0U);
}

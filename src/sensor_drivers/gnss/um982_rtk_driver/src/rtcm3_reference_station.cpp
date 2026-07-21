#include "um982_rtk_driver/rtcm3_reference_station.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace um982_rtk_driver
{
namespace
{

constexpr std::uint8_t kPreamble = 0xD3U;
constexpr std::size_t kHeaderBytes = 3U;
constexpr std::size_t kCrcBytes = 3U;
constexpr std::size_t kMaximumPayloadBytes = 1023U;
constexpr std::uint32_t kCrc24QPolynomial = 0x1864CFBU;

std::uint64_t unsignedBits(
  const std::vector<std::uint8_t> & data, const std::size_t position,
  const std::size_t width)
{
  if (width == 0U || width > 64U || position + width > data.size() * 8U) {
    throw std::out_of_range("RTCM bit field is outside the frame");
  }
  std::uint64_t value = 0U;
  for (std::size_t index = 0; index < width; ++index) {
    const std::size_t bit = position + index;
    value = (value << 1U) |
      static_cast<std::uint64_t>((data[bit / 8U] >> (7U - bit % 8U)) & 1U);
  }
  return value;
}

std::int64_t signed38Bits(
  const std::vector<std::uint8_t> & data, const std::size_t position)
{
  constexpr std::uint64_t sign_bit = std::uint64_t{1} << 37U;
  constexpr std::uint64_t modulus = std::uint64_t{1} << 38U;
  const std::uint64_t value = unsignedBits(data, position, 38U);
  return value & sign_bit ?
         static_cast<std::int64_t>(value - modulus) : static_cast<std::int64_t>(value);
}

std::uint32_t expectedCrc(const std::vector<std::uint8_t> & frame) noexcept
{
  const std::size_t size = frame.size();
  return (static_cast<std::uint32_t>(frame[size - 3U]) << 16U) |
         (static_cast<std::uint32_t>(frame[size - 2U]) << 8U) |
         static_cast<std::uint32_t>(frame[size - 1U]);
}

}  // namespace

Rtcm3Framer::Rtcm3Framer(const std::size_t maximum_buffer_bytes)
: maximum_buffer_bytes_(maximum_buffer_bytes)
{
  if (maximum_buffer_bytes_ < kHeaderBytes + kMaximumPayloadBytes + kCrcBytes) {
    throw std::invalid_argument("RTCM buffer must hold one maximum-size frame");
  }
  buffer_.reserve(maximum_buffer_bytes_);
}

std::vector<std::vector<std::uint8_t>> Rtcm3Framer::consume(
  const std::uint8_t * data, const std::size_t size)
{
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("RTCM input pointer is null");
  }
  stats_.bytes_received += size;
  if (size != 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<std::vector<std::uint8_t>> frames;
  while (!buffer_.empty()) {
    const auto preamble = std::find(buffer_.begin(), buffer_.end(), kPreamble);
    if (preamble != buffer_.begin()) {
      const std::size_t discarded = static_cast<std::size_t>(preamble - buffer_.begin());
      stats_.bytes_discarded += discarded;
      buffer_.erase(buffer_.begin(), preamble);
      if (buffer_.empty()) {
        break;
      }
    }
    if (buffer_.size() < kHeaderBytes) {
      break;
    }
    if ((buffer_[1] & 0xFCU) != 0U) {
      ++stats_.header_failures;
      ++stats_.bytes_discarded;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t payload_size =
      (static_cast<std::size_t>(buffer_[1] & 0x03U) << 8U) |
      static_cast<std::size_t>(buffer_[2]);
    const std::size_t frame_size = kHeaderBytes + payload_size + kCrcBytes;
    if (buffer_.size() < frame_size) {
      break;
    }
    const std::uint32_t actual_crc = crc24q(buffer_.data(), kHeaderBytes + payload_size);
    const std::uint32_t frame_crc =
      (static_cast<std::uint32_t>(buffer_[frame_size - 3U]) << 16U) |
      (static_cast<std::uint32_t>(buffer_[frame_size - 2U]) << 8U) |
      static_cast<std::uint32_t>(buffer_[frame_size - 1U]);
    if (actual_crc != frame_crc) {
      ++stats_.crc_failures;
      ++stats_.bytes_discarded;
      buffer_.erase(buffer_.begin());
      continue;
    }
    frames.emplace_back(buffer_.begin(), buffer_.begin() + frame_size);
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
    ++stats_.frames_emitted;
  }

  if (buffer_.size() > maximum_buffer_bytes_) {
    const std::size_t discarded = buffer_.size() - maximum_buffer_bytes_;
    stats_.bytes_discarded += discarded;
    ++stats_.buffer_overflows;
    buffer_.erase(buffer_.begin(), buffer_.begin() + discarded);
  }
  return frames;
}

void Rtcm3Framer::reset() noexcept
{
  buffer_.clear();
}

std::uint32_t crc24q(const std::uint8_t * data, const std::size_t size) noexcept
{
  std::uint32_t crc = 0U;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<std::uint32_t>(data[index]) << 16U;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1U;
      if ((crc & 0x1000000U) != 0U) {
        crc ^= kCrc24QPolynomial;
      }
    }
  }
  return crc & 0xFFFFFFU;
}

std::optional<std::uint16_t> rtcm3MessageType(
  const std::vector<std::uint8_t> & frame) noexcept
{
  if (frame.size() < kHeaderBytes + 2U + kCrcBytes || frame[0] != kPreamble ||
    (frame[1] & 0xFCU) != 0U)
  {
    return std::nullopt;
  }
  const std::size_t payload_size =
    (static_cast<std::size_t>(frame[1] & 0x03U) << 8U) |
    static_cast<std::size_t>(frame[2]);
  if (frame.size() != kHeaderBytes + payload_size + kCrcBytes ||
    crc24q(frame.data(), kHeaderBytes + payload_size) != expectedCrc(frame))
  {
    return std::nullopt;
  }
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(frame[3]) << 4U) |
    (static_cast<std::uint16_t>(frame[4]) >> 4U));
}

Rtcm3ReferenceStationResult decodeRtcm3ReferenceStation(
  const std::vector<std::uint8_t> & frame)
{
  const auto message_type = rtcm3MessageType(frame);
  if (!message_type.has_value()) {
    return {std::nullopt, "INVALID_RTCM_FRAME"};
  }
  if (*message_type != 1005U && *message_type != 1006U) {
    return {std::nullopt, "UNSUPPORTED_MESSAGE_TYPE"};
  }
  const std::size_t payload_size = frame.size() - kHeaderBytes - kCrcBytes;
  const std::size_t expected_payload_size = *message_type == 1005U ? 19U : 21U;
  if (payload_size != expected_payload_size) {
    return {std::nullopt, "INVALID_REFERENCE_STATION_LENGTH"};
  }

  Rtcm3ReferenceStation station;
  station.message_type = *message_type;
  station.station_id = static_cast<std::uint16_t>(unsignedBits(frame, 36U, 12U));
  station.itrf_realization = static_cast<std::uint8_t>(unsignedBits(frame, 48U, 6U));
  station.gps_indicator = unsignedBits(frame, 54U, 1U) != 0U;
  station.glonass_indicator = unsignedBits(frame, 55U, 1U) != 0U;
  station.galileo_indicator = unsignedBits(frame, 56U, 1U) != 0U;
  station.reference_station_indicator = unsignedBits(frame, 57U, 1U) != 0U;
  station.position_ecef_m = {
    static_cast<double>(signed38Bits(frame, 58U)) * 0.0001,
    static_cast<double>(signed38Bits(frame, 98U)) * 0.0001,
    static_cast<double>(signed38Bits(frame, 138U)) * 0.0001};
  if (*message_type == 1006U) {
    station.antenna_height_valid = true;
    station.antenna_height_m = static_cast<double>(unsignedBits(frame, 176U, 16U)) * 0.0001;
  }
  if (!std::all_of(
      station.position_ecef_m.begin(), station.position_ecef_m.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return {std::nullopt, "NONFINITE_REFERENCE_STATION"};
  }
  return {station, ""};
}

}  // namespace um982_rtk_driver

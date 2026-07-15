#include "um982_raw_driver/binary_framer.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace um982_raw_driver
{
namespace
{

constexpr std::array<std::uint8_t, 3> kSync{{0xAA, 0x44, 0xB5}};
constexpr std::uint32_t kCrcPolynomial = 0xEDB88320U;

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

}  // namespace

std::uint32_t calculateCrc32(const std::uint8_t * data, const std::size_t size)
{
  std::uint32_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ kCrcPolynomial : crc >> 1U;
    }
  }
  return crc;
}

BinaryHeader parseBinaryHeader(const std::uint8_t * data, const std::size_t size)
{
  if (data == nullptr || size < kBinaryHeaderSize) {
    throw std::invalid_argument("a complete UM982 binary header is required");
  }
  BinaryHeader header;
  header.cpu_idle_percent = data[3];
  header.message_id = readLe16(data + 4);
  header.payload_length = readLe16(data + 6);
  header.time_reference = data[8];
  header.time_status = data[9];
  header.week = readLe16(data + 10);
  header.milliseconds_of_week = readLe32(data + 12);
  header.format_version = readLe32(data + 16);
  header.reserved = data[20];
  header.leap_seconds = data[21];
  header.output_delay_ms = readLe16(data + 22);
  return header;
}

BinaryFramer::BinaryFramer(
  const std::size_t max_payload_bytes,
  const std::size_t max_buffer_bytes)
: max_payload_bytes_(max_payload_bytes), max_buffer_bytes_(max_buffer_bytes)
{
  if (max_payload_bytes_ > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("max_payload_bytes exceeds the Unicore uint16 length field");
  }
  const std::size_t largest_frame = kBinaryHeaderSize + max_payload_bytes_ + kBinaryCrcSize;
  if (max_buffer_bytes_ < largest_frame) {
    throw std::invalid_argument("max_buffer_bytes cannot hold the configured largest frame");
  }
  buffer_.reserve(max_buffer_bytes_);
}

std::vector<BinaryFrame> BinaryFramer::consume(
  const std::uint8_t * data,
  const std::size_t size)
{
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("non-zero input size requires a data pointer");
  }

  appendBounded(data, size);
  std::vector<BinaryFrame> frames;

  while (true) {
    const std::size_t sync_offset = findSync();
    if (sync_offset == buffer_.size()) {
      const std::size_t keep = trailingSyncPrefixSize();
      discardPrefix(buffer_.size() - keep, true);
      break;
    }
    if (sync_offset != 0U) {
      discardPrefix(sync_offset, true);
    }
    if (buffer_.size() < kBinaryHeaderSize) {
      break;
    }

    const BinaryHeader header = parseBinaryHeader(buffer_.data(), buffer_.size());
    if (header.payload_length > max_payload_bytes_) {
      ++stats_.length_failures;
      discardPrefix(1, true);
      continue;
    }

    const std::size_t frame_size =
      kBinaryHeaderSize + static_cast<std::size_t>(header.payload_length) + kBinaryCrcSize;
    if (buffer_.size() < frame_size) {
      break;
    }

    const std::size_t crc_offset = frame_size - kBinaryCrcSize;
    const std::uint32_t expected_crc = readLe32(buffer_.data() + crc_offset);
    const std::uint32_t calculated_crc = calculateCrc32(buffer_.data(), crc_offset);
    if (expected_crc != calculated_crc) {
      ++stats_.crc_failures;
      discardPrefix(1, true);
      continue;
    }

    BinaryFrame frame;
    frame.header = header;
    frame.stream_offset = buffer_start_offset_;
    frame.bytes.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    frames.push_back(std::move(frame));
    ++stats_.frames_emitted;
    discardPrefix(frame_size, false);
  }

  return frames;
}

std::vector<BinaryFrame> BinaryFramer::consume(const std::vector<std::uint8_t> & data)
{
  return consume(data.data(), data.size());
}

const FramerStats & BinaryFramer::stats() const noexcept
{
  return stats_;
}

std::size_t BinaryFramer::bufferedBytes() const noexcept
{
  return buffer_.size();
}

void BinaryFramer::reset()
{
  stats_.bytes_discarded += buffer_.size();
  buffer_.clear();
  buffer_start_offset_ = stats_.bytes_received;
}

void BinaryFramer::appendBounded(const std::uint8_t * data, const std::size_t size)
{
  if (size == 0U) {
    return;
  }

  stats_.bytes_received += size;
  if (size >= max_buffer_bytes_) {
    const std::size_t dropped = buffer_.size() + size - max_buffer_bytes_;
    stats_.bytes_discarded += dropped;
    ++stats_.buffer_overflows;
    buffer_.assign(data + (size - max_buffer_bytes_), data + size);
    buffer_start_offset_ = stats_.bytes_received - max_buffer_bytes_;
    return;
  }

  if (buffer_.size() + size > max_buffer_bytes_) {
    const std::size_t dropped = buffer_.size() + size - max_buffer_bytes_;
    discardPrefix(dropped, true);
    ++stats_.buffer_overflows;
  }
  buffer_.insert(buffer_.end(), data, data + size);
}

void BinaryFramer::discardPrefix(const std::size_t size, const bool count_as_discarded)
{
  const std::size_t actual = std::min(size, buffer_.size());
  if (count_as_discarded) {
    stats_.bytes_discarded += actual;
  }
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(actual));
  buffer_start_offset_ += actual;
}

std::size_t BinaryFramer::findSync() const
{
  const auto it = std::search(buffer_.begin(), buffer_.end(), kSync.begin(), kSync.end());
  return static_cast<std::size_t>(std::distance(buffer_.begin(), it));
}

std::size_t BinaryFramer::trailingSyncPrefixSize() const
{
  if (buffer_.size() >= 2U &&
    buffer_[buffer_.size() - 2U] == kSync[0] && buffer_.back() == kSync[1])
  {
    return 2;
  }
  if (!buffer_.empty() && buffer_.back() == kSync[0]) {
    return 1;
  }
  return 0;
}

}  // namespace um982_raw_driver

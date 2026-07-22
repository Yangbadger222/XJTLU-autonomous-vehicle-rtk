#include "um982_raw_driver/mixed_stream_framer.hpp"

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

constexpr std::array<std::uint8_t, 3> kBinarySync{{0xAA, 0x44, 0xB5}};

std::uint32_t readLe32(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

}  // namespace

MixedStreamFramer::MixedStreamFramer(
  const std::size_t max_payload_bytes,
  const std::size_t max_buffer_bytes,
  const std::size_t max_ascii_line_bytes)
: max_payload_bytes_(max_payload_bytes),
  max_buffer_bytes_(max_buffer_bytes),
  max_ascii_line_bytes_(max_ascii_line_bytes)
{
  if (max_payload_bytes_ > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("max_payload_bytes exceeds the Unicore uint16 length field");
  }
  const std::size_t largest_frame = kBinaryHeaderSize + max_payload_bytes_ + kBinaryCrcSize;
  if (max_buffer_bytes_ < largest_frame) {
    throw std::invalid_argument("max_buffer_bytes cannot hold the configured largest frame");
  }
  if (max_ascii_line_bytes_ == 0U || max_ascii_line_bytes_ > max_buffer_bytes_) {
    throw std::invalid_argument("max_ascii_line_bytes must fit in the mixed-stream buffer");
  }
  buffer_.reserve(max_buffer_bytes_);
}

std::vector<MixedStreamItem> MixedStreamFramer::consume(
  const std::uint8_t * data,
  const std::size_t size)
{
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("non-zero input size requires a data pointer");
  }

  appendBounded(data, size);
  std::vector<MixedStreamItem> items;

  while (!buffer_.empty()) {
    if (startsWithBinarySync()) {
      if (buffer_.size() < kBinaryHeaderSize) {
        break;
      }
      const BinaryHeader header = parseBinaryHeader(buffer_.data(), buffer_.size());
      if (header.payload_length > max_payload_bytes_) {
        ++stats_.length_failures;
        discardPrefix(1U, true);
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
        discardPrefix(frame_size, true);
        continue;
      }

      MixedStreamItem item;
      item.kind = MixedStreamItemKind::BinaryFrame;
      item.stream_offset = buffer_start_offset_;
      item.binary_frame.header = header;
      item.binary_frame.stream_offset = buffer_start_offset_;
      item.binary_frame.bytes.assign(
        buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
      items.push_back(std::move(item));
      ++stats_.binary_frames_emitted;
      discardPrefix(frame_size, false);
      continue;
    }

    if (isAsciiLineStart(buffer_.front())) {
      const auto newline = std::find(buffer_.begin(), buffer_.end(), '\n');
      const std::size_t newline_offset =
        newline == buffer_.end() ? buffer_.size() :
        static_cast<std::size_t>(std::distance(buffer_.begin(), newline));
      const std::size_t binary_offset = findBinarySync(1U);
      if (binary_offset < newline_offset) {
        ++stats_.invalid_ascii_lines;
        discardPrefix(binary_offset, true);
        continue;
      }
      if (newline == buffer_.end()) {
        if (buffer_.size() > max_ascii_line_bytes_) {
          ++stats_.ascii_overflows;
          const std::size_t next_start = findNextItemStart(1U);
          if (next_start < buffer_.size()) {
            discardPrefix(next_start, true);
          } else {
            const std::size_t keep = trailingBinarySyncPrefixSize();
            discardPrefix(buffer_.size() - keep, true);
          }
          continue;
        }
        break;
      }

      const std::size_t line_size = newline_offset + 1U;
      if (line_size > max_ascii_line_bytes_) {
        ++stats_.ascii_overflows;
        discardPrefix(line_size, true);
        continue;
      }
      if (!isValidAsciiLine(buffer_.data(), line_size)) {
        ++stats_.invalid_ascii_lines;
        discardPrefix(line_size, true);
        continue;
      }

      MixedStreamItem item;
      item.kind = MixedStreamItemKind::AsciiLine;
      item.stream_offset = buffer_start_offset_;
      item.ascii_line.assign(
        buffer_.begin(),
        buffer_.begin() + static_cast<std::ptrdiff_t>(line_size));
      items.push_back(std::move(item));
      ++stats_.ascii_lines_emitted;
      discardPrefix(line_size, false);
      continue;
    }

    const std::size_t next_start = findNextItemStart();
    if (next_start < buffer_.size()) {
      discardPrefix(next_start, true);
      continue;
    }
    const std::size_t keep = trailingBinarySyncPrefixSize();
    discardPrefix(buffer_.size() - keep, true);
    break;
  }

  return items;
}

std::vector<MixedStreamItem> MixedStreamFramer::consume(
  const std::vector<std::uint8_t> & data)
{
  return consume(data.data(), data.size());
}

const MixedStreamFramerStats & MixedStreamFramer::stats() const noexcept
{
  return stats_;
}

std::size_t MixedStreamFramer::bufferedBytes() const noexcept
{
  return buffer_.size();
}

void MixedStreamFramer::reset()
{
  stats_.bytes_discarded += buffer_.size();
  buffer_.clear();
  buffer_start_offset_ = stats_.bytes_received;
}

void MixedStreamFramer::appendBounded(const std::uint8_t * data, const std::size_t size)
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

void MixedStreamFramer::discardPrefix(const std::size_t size, const bool count_as_discarded)
{
  const std::size_t actual = std::min(size, buffer_.size());
  if (count_as_discarded) {
    stats_.bytes_discarded += actual;
  }
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(actual));
  buffer_start_offset_ += actual;
}

std::size_t MixedStreamFramer::findBinarySync(const std::size_t begin) const
{
  if (begin >= buffer_.size()) {
    return buffer_.size();
  }
  const auto first = buffer_.begin() + static_cast<std::ptrdiff_t>(begin);
  const auto it = std::search(first, buffer_.end(), kBinarySync.begin(), kBinarySync.end());
  return static_cast<std::size_t>(std::distance(buffer_.begin(), it));
}

std::size_t MixedStreamFramer::findNextItemStart(const std::size_t begin) const
{
  std::size_t result = findBinarySync(begin);
  for (std::size_t index = begin; index < buffer_.size(); ++index) {
    if (isAsciiLineStart(buffer_[index])) {
      result = std::min(result, index);
      break;
    }
  }
  return result;
}

std::size_t MixedStreamFramer::trailingBinarySyncPrefixSize() const
{
  if (buffer_.size() >= 2U &&
    buffer_[buffer_.size() - 2U] == kBinarySync[0] && buffer_.back() == kBinarySync[1])
  {
    return 2U;
  }
  if (!buffer_.empty() && buffer_.back() == kBinarySync[0]) {
    return 1U;
  }
  return 0U;
}

bool MixedStreamFramer::startsWithBinarySync() const
{
  return buffer_.size() >= kBinarySync.size() &&
         std::equal(kBinarySync.begin(), kBinarySync.end(), buffer_.begin());
}

bool MixedStreamFramer::isAsciiLineStart(const std::uint8_t byte) noexcept
{
  return byte == static_cast<std::uint8_t>('$') || byte == static_cast<std::uint8_t>('#');
}

bool MixedStreamFramer::isValidAsciiLine(
  const std::uint8_t * data,
  const std::size_t size) noexcept
{
  for (std::size_t index = 0; index < size; ++index) {
    const std::uint8_t byte = data[index];
    if (byte == static_cast<std::uint8_t>('\r') ||
      byte == static_cast<std::uint8_t>('\n') || byte == static_cast<std::uint8_t>('\t'))
    {
      continue;
    }
    if (byte < 0x20U || byte > 0x7EU) {
      return false;
    }
  }
  return true;
}

}  // namespace um982_raw_driver

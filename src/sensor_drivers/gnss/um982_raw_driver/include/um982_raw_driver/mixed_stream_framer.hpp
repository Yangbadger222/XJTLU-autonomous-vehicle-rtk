#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "um982_raw_driver/binary_framer.hpp"

namespace um982_raw_driver
{

enum class MixedStreamItemKind
{
  AsciiLine,
  BinaryFrame,
};

struct MixedStreamItem
{
  MixedStreamItemKind kind = MixedStreamItemKind::AsciiLine;
  std::uint64_t stream_offset = 0;
  std::string ascii_line;
  BinaryFrame binary_frame;
};

struct MixedStreamFramerStats
{
  std::uint64_t bytes_received = 0;
  std::uint64_t bytes_discarded = 0;
  std::uint64_t ascii_lines_emitted = 0;
  std::uint64_t binary_frames_emitted = 0;
  std::uint64_t crc_failures = 0;
  std::uint64_t length_failures = 0;
  std::uint64_t ascii_overflows = 0;
  std::uint64_t invalid_ascii_lines = 0;
  std::uint64_t buffer_overflows = 0;
};

class MixedStreamFramer
{
public:
  explicit MixedStreamFramer(
    std::size_t max_payload_bytes = 65535,
    std::size_t max_buffer_bytes = 131072,
    std::size_t max_ascii_line_bytes = 1024);

  std::vector<MixedStreamItem> consume(const std::uint8_t * data, std::size_t size);
  std::vector<MixedStreamItem> consume(const std::vector<std::uint8_t> & data);

  const MixedStreamFramerStats & stats() const noexcept;
  std::size_t bufferedBytes() const noexcept;
  void reset();

private:
  void appendBounded(const std::uint8_t * data, std::size_t size);
  void discardPrefix(std::size_t size, bool count_as_discarded);
  std::size_t findBinarySync(std::size_t begin = 0) const;
  std::size_t findNextItemStart(std::size_t begin = 0) const;
  std::size_t trailingBinarySyncPrefixSize() const;
  bool startsWithBinarySync() const;
  static bool isAsciiLineStart(std::uint8_t byte) noexcept;
  static bool isValidAsciiLine(const std::uint8_t * data, std::size_t size) noexcept;

  std::size_t max_payload_bytes_;
  std::size_t max_buffer_bytes_;
  std::size_t max_ascii_line_bytes_;
  std::uint64_t buffer_start_offset_ = 0;
  std::vector<std::uint8_t> buffer_;
  MixedStreamFramerStats stats_;
};

}  // namespace um982_raw_driver

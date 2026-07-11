#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace um982_raw_driver
{

constexpr std::size_t kBinaryHeaderSize = 24;
constexpr std::size_t kBinaryCrcSize = 4;

struct BinaryHeader
{
  std::uint8_t cpu_idle_percent = 0;
  std::uint16_t message_id = 0;
  std::uint16_t payload_length = 0;
  std::uint8_t time_reference = 0;
  std::uint8_t time_status = 0;
  std::uint16_t week = 0;
  std::uint32_t milliseconds_of_week = 0;
  std::uint32_t format_version = 0;
  std::uint8_t reserved = 0;
  std::uint8_t leap_seconds = 0;
  std::uint16_t output_delay_ms = 0;
};

struct BinaryFrame
{
  BinaryHeader header;
  std::uint64_t stream_offset = 0;
  std::vector<std::uint8_t> bytes;
};

struct FramerStats
{
  std::uint64_t bytes_received = 0;
  std::uint64_t bytes_discarded = 0;
  std::uint64_t frames_emitted = 0;
  std::uint64_t crc_failures = 0;
  std::uint64_t length_failures = 0;
  std::uint64_t buffer_overflows = 0;
};

std::uint32_t calculateCrc32(const std::uint8_t * data, std::size_t size);

class BinaryFramer
{
public:
  explicit BinaryFramer(
    std::size_t max_payload_bytes = 65535,
    std::size_t max_buffer_bytes = 131072);

  std::vector<BinaryFrame> consume(const std::uint8_t * data, std::size_t size);
  std::vector<BinaryFrame> consume(const std::vector<std::uint8_t> & data);

  const FramerStats & stats() const noexcept;
  std::size_t bufferedBytes() const noexcept;
  void reset();

private:
  void appendBounded(const std::uint8_t * data, std::size_t size);
  void discardPrefix(std::size_t size, bool count_as_discarded);
  std::size_t findSync() const;
  std::size_t trailingSyncPrefixSize() const;

  std::size_t max_payload_bytes_;
  std::size_t max_buffer_bytes_;
  std::uint64_t buffer_start_offset_ = 0;
  std::vector<std::uint8_t> buffer_;
  FramerStats stats_;
};

}  // namespace um982_raw_driver

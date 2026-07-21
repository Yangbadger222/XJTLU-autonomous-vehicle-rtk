#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace um982_rtk_driver
{

struct Rtcm3FramerStats
{
  std::uint64_t bytes_received = 0;
  std::uint64_t bytes_discarded = 0;
  std::uint64_t frames_emitted = 0;
  std::uint64_t crc_failures = 0;
  std::uint64_t header_failures = 0;
  std::uint64_t buffer_overflows = 0;
};

class Rtcm3Framer
{
public:
  explicit Rtcm3Framer(std::size_t maximum_buffer_bytes = 8192);

  std::vector<std::vector<std::uint8_t>> consume(
    const std::uint8_t * data, std::size_t size);
  std::vector<std::vector<std::uint8_t>> consume(
    const std::vector<std::uint8_t> & data)
  {
    return consume(data.data(), data.size());
  }
  void reset() noexcept;

  const Rtcm3FramerStats & stats() const noexcept {return stats_;}
  std::size_t bufferedBytes() const noexcept {return buffer_.size();}

private:
  std::size_t maximum_buffer_bytes_;
  std::vector<std::uint8_t> buffer_;
  Rtcm3FramerStats stats_;
};

struct Rtcm3ReferenceStation
{
  std::uint16_t message_type = 0;
  std::uint16_t station_id = 0;
  std::uint8_t itrf_realization = 0;
  bool gps_indicator = false;
  bool glonass_indicator = false;
  bool galileo_indicator = false;
  bool reference_station_indicator = false;
  std::array<double, 3> position_ecef_m{};
  bool antenna_height_valid = false;
  double antenna_height_m = 0.0;
};

struct Rtcm3ReferenceStationResult
{
  std::optional<Rtcm3ReferenceStation> station;
  std::string reason;

  bool ok() const noexcept {return station.has_value();}
};

std::uint32_t crc24q(const std::uint8_t * data, std::size_t size) noexcept;
std::optional<std::uint16_t> rtcm3MessageType(
  const std::vector<std::uint8_t> & frame) noexcept;
Rtcm3ReferenceStationResult decodeRtcm3ReferenceStation(
  const std::vector<std::uint8_t> & frame);

}  // namespace um982_rtk_driver

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "um982_raw_driver/binary_framer.hpp"

namespace um982_raw_driver
{

constexpr std::uint16_t kObsVmMessageId = 12;
constexpr std::uint16_t kObsVhMessageId = 13;
constexpr std::uint16_t kObsVBaseMessageId = 284;
constexpr std::size_t kObservationRecordSize = 40;

enum class ObservationReceiver : std::uint8_t
{
  Master = 1,
  Secondary = 2,
  Base = 3,
};

enum class GnssConstellation : std::uint8_t
{
  Gps = 0,
  Glonass = 1,
  Sbas = 2,
  Galileo = 3,
  Bds = 4,
  Qzss = 5,
  Irnss = 6,
};

struct Observation
{
  GnssConstellation constellation = GnssConstellation::Gps;
  std::uint16_t prn = 0;
  std::uint8_t signal_type = 0;
  std::uint8_t channel_number = 0;
  std::uint16_t system_frequency = 0;
  std::int16_t glonass_frequency_channel = 0;
  double pseudorange_m = 0.0;
  double carrier_phase_cycles = 0.0;
  double doppler_hz = 0.0;
  double pseudorange_std_m = 0.0;
  double carrier_phase_std_cycles = 0.0;
  double cn0_db_hz = 0.0;
  double lock_time_s = 0.0;
  bool pseudorange_valid = false;
  bool carrier_phase_valid = false;
  bool l2c_signal = false;
  std::uint32_t tracking_status = 0;
};

struct ObservationEpoch
{
  ObservationReceiver receiver = ObservationReceiver::Master;
  BinaryHeader header;
  std::vector<Observation> observations;
};

enum class ObservationDecodeError
{
  None,
  UnsupportedMessage,
  FrameSizeMismatch,
  PayloadLayoutMismatch,
  NonFiniteMeasurement,
  InvalidMeasurement,
  InvalidConstellation,
  InvalidPrn,
  InvalidSystemFrequency,
};

struct ObservationDecodeResult
{
  ObservationDecodeError error = ObservationDecodeError::None;
  std::string reason;
  std::optional<ObservationEpoch> epoch;

  bool ok() const noexcept {return epoch.has_value();}
};

bool isObservationMessage(std::uint16_t message_id) noexcept;
ObservationDecodeResult decodeObservationFrame(const BinaryFrame & frame);

}  // namespace um982_raw_driver

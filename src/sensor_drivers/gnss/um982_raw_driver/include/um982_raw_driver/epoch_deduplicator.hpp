#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_set>

#include "um982_raw_driver/observation_decoder.hpp"

namespace um982_raw_driver
{

struct ObservationEpochKey
{
  ObservationReceiver receiver = ObservationReceiver::Master;
  std::uint8_t time_reference = 0;
  std::uint16_t week = 0;
  std::uint32_t milliseconds_of_week = 0;

  bool operator==(const ObservationEpochKey & other) const noexcept;
};

struct ObservationEpochKeyHash
{
  std::size_t operator()(const ObservationEpochKey & key) const noexcept;
};

class EpochDeduplicator
{
public:
  explicit EpochDeduplicator(std::size_t capacity = 256);

  bool accept(const ObservationEpochKey & key);
  void reset();
  std::size_t size() const noexcept;

private:
  std::size_t capacity_;
  std::deque<ObservationEpochKey> insertion_order_;
  std::unordered_set<ObservationEpochKey, ObservationEpochKeyHash> keys_;
};

}  // namespace um982_raw_driver

#include "um982_raw_driver/epoch_deduplicator.hpp"

#include <functional>
#include <stdexcept>

namespace um982_raw_driver
{

bool ObservationEpochKey::operator==(const ObservationEpochKey & other) const noexcept
{
  return receiver == other.receiver && time_reference == other.time_reference &&
         week == other.week && milliseconds_of_week == other.milliseconds_of_week;
}

std::size_t ObservationEpochKeyHash::operator()(const ObservationEpochKey & key) const noexcept
{
  std::size_t seed = std::hash<std::uint8_t>{}(static_cast<std::uint8_t>(key.receiver));
  seed ^= std::hash<std::uint8_t>{}(key.time_reference) + 0x9E3779B9U + (seed << 6U) +
    (seed >> 2U);
  seed ^= std::hash<std::uint16_t>{}(key.week) + 0x9E3779B9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<std::uint32_t>{}(key.milliseconds_of_week) + 0x9E3779B9U +
    (seed << 6U) + (seed >> 2U);
  return seed;
}

EpochDeduplicator::EpochDeduplicator(const std::size_t capacity)
: capacity_(capacity)
{
  if (capacity_ == 0U) {
    throw std::invalid_argument("epoch deduplication capacity must be positive");
  }
  keys_.reserve(capacity_);
}

bool EpochDeduplicator::accept(const ObservationEpochKey & key)
{
  if (keys_.find(key) != keys_.end()) {
    return false;
  }

  if (insertion_order_.size() == capacity_) {
    keys_.erase(insertion_order_.front());
    insertion_order_.pop_front();
  }
  insertion_order_.push_back(key);
  keys_.insert(key);
  return true;
}

void EpochDeduplicator::reset()
{
  insertion_order_.clear();
  keys_.clear();
}

std::size_t EpochDeduplicator::size() const noexcept
{
  return insertion_order_.size();
}

}  // namespace um982_raw_driver

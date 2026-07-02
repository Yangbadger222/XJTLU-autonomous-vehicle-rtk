#pragma once

#include <deque>
#include <cmath>
#include <cstddef>
#include <optional>

namespace rtk_fgo_localizer
{

template<typename T>
struct TimestampedSample
{
  double stamp_s = 0.0;
  T value;
};

template<typename T>
class TimestampedBuffer
{
public:
  explicit TimestampedBuffer(double max_age_s = 2.0)
  : max_age_s_(max_age_s)
  {
  }

  void add(double stamp_s, const T & value)
  {
    samples_.push_back({stamp_s, value});
    prune(stamp_s);
  }

  std::optional<TimestampedSample<T>> latest() const
  {
    if (samples_.empty()) {
      return std::nullopt;
    }
    return samples_.back();
  }

  std::optional<TimestampedSample<T>> closest(double stamp_s, double tolerance_s) const
  {
    if (samples_.empty()) {
      return std::nullopt;
    }

    std::optional<TimestampedSample<T>> best;
    double best_dt = tolerance_s;
    for (const auto & sample : samples_) {
      const double dt = std::abs(sample.stamp_s - stamp_s);
      if (dt <= best_dt) {
        best = sample;
        best_dt = dt;
      }
    }
    return best;
  }

  std::size_t size() const
  {
    return samples_.size();
  }

  void clear()
  {
    samples_.clear();
  }

private:
  void prune(double newest_stamp_s)
  {
    while (!samples_.empty() && newest_stamp_s - samples_.front().stamp_s > max_age_s_) {
      samples_.pop_front();
    }
  }

  double max_age_s_ = 2.0;
  std::deque<TimestampedSample<T>> samples_;
};

}  // namespace rtk_fgo_localizer

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace um982_rtk_driver
{

struct NtripReconnectConfig
{
  double initial_backoff_s = 1.0;
  double max_backoff_s = 10.0;
  double jitter_s = 0.2;
  bool reconnect_without_valid_gga = true;
  double max_cached_valid_gga_age_s = 30.0;
};

class NtripReconnectPolicy
{
public:
  explicit NtripReconnectPolicy(NtripReconnectConfig config)
  : config_(config)
  {
    if (!std::isfinite(config_.initial_backoff_s) || config_.initial_backoff_s <= 0.0 ||
      !std::isfinite(config_.max_backoff_s) ||
      config_.max_backoff_s < config_.initial_backoff_s ||
      !std::isfinite(config_.jitter_s) || config_.jitter_s < 0.0 ||
      !std::isfinite(config_.max_cached_valid_gga_age_s) ||
      config_.max_cached_valid_gga_age_s < 0.0)
    {
      throw std::invalid_argument("invalid NTRIP reconnect configuration");
    }
  }

  std::chrono::milliseconds retryDelay(unsigned int consecutive_failures, double jitter_unit) const
  {
    const unsigned int exponent = std::min(
      consecutive_failures > 0 ? consecutive_failures - 1 : 0, 30U);
    const double base_s = std::min(
      config_.max_backoff_s,
      config_.initial_backoff_s * static_cast<double>(uint64_t{1} << exponent));
    const double bounded_jitter = std::clamp(jitter_unit, 0.0, 1.0) * config_.jitter_s;
    return std::chrono::milliseconds(static_cast<int64_t>((base_s + bounded_jitter) * 1000.0));
  }

  std::optional<std::string> initialGga(
    const std::string & latest_gga,
    bool latest_gga_valid,
    const std::string & cached_valid_gga,
    double cached_valid_gga_age_s) const
  {
    if (latest_gga_valid && !latest_gga.empty()) {
      return latest_gga;
    }
    if (!cached_valid_gga.empty() && std::isfinite(cached_valid_gga_age_s) &&
      cached_valid_gga_age_s <= config_.max_cached_valid_gga_age_s)
    {
      return cached_valid_gga;
    }
    if (config_.reconnect_without_valid_gga) {
      return std::string{};
    }
    return std::nullopt;
  }

private:
  NtripReconnectConfig config_;
};

}  // namespace um982_rtk_driver

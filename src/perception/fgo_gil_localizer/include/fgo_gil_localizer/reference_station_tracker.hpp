#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "fgo_gil_localizer/math_types.hpp"

namespace fgo_gil_localizer
{

struct ReferenceStationTrackerConfig
{
  double change_threshold_m = 0.01;
  double minimum_ecef_radius_m = 5.0e6;
  double maximum_ecef_radius_m = 7.0e6;
};

struct ReferenceStationSample
{
  std::uint16_t station_id = 0;
  std::uint16_t message_type = 0;
  std::uint8_t itrf_realization = 0;
  std::string source;
  Vec3 position_ecef_m;
};

enum class ReferenceStationUpdate : std::uint8_t
{
  Rejected,
  Initial,
  Unchanged,
  Changed,
};

class ReferenceStationTracker
{
public:
  explicit ReferenceStationTracker(ReferenceStationTrackerConfig config = {});

  ReferenceStationUpdate accept(const ReferenceStationSample & sample);
  const std::optional<ReferenceStationSample> & current() const noexcept {return current_;}

private:
  ReferenceStationTrackerConfig config_;
  std::optional<ReferenceStationSample> current_;
};

}  // namespace fgo_gil_localizer

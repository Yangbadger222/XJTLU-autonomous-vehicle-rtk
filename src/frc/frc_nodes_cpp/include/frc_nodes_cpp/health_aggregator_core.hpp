#pragma once

#include <array>
#include <cstdint>

namespace frc_nodes_cpp
{

template <std::size_t N>
struct StampedArray
{
  bool valid{false};
  double stamp_s{0.0};
  std::array<float, N> values{};

  StampedArray() = default;

  StampedArray(const double stamp, const std::array<float, N> & data)
  : valid(true), stamp_s(stamp), values(data)
  {
  }
};

struct StampedStatus
{
  bool valid{false};
  double stamp_s{0.0};
  int8_t value{-1};

  StampedStatus() = default;

  StampedStatus(const double stamp, const int8_t status)
  : valid(true), stamp_s(stamp), value(status)
  {
  }
};

struct HealthInputs
{
  StampedArray<3> degeneracy;
  StampedArray<2> correction;
  StampedStatus fix_status;
  StampedArray<2> velocity;
};

struct HealthOutput
{
  float lio_min_eig{0.0F};
  float lio_cond{0.0F};
  bool lio_degenerate{false};
  bool pgo_correcting{false};
  float pgo_last_jump{0.0F};
  int8_t rtk_status{-1};
  float v{0.0F};
  float w{0.0F};
};

class HealthAggregatorCore
{
public:
  HealthAggregatorCore(const double stale_timeout_s, const double lio_min_eig_degenerate)
  : stale_timeout_s_(stale_timeout_s),
    lio_min_eig_degenerate_(static_cast<float>(lio_min_eig_degenerate))
  {
  }

  HealthOutput build(const HealthInputs & inputs, const double now_s) const
  {
    HealthOutput out;

    if (fresh(inputs.degeneracy, now_s)) {
      const auto & data = inputs.degeneracy.values;
      out.lio_min_eig = data[0];
      out.lio_cond = data[1];
      out.lio_degenerate = data[2] > 0.5F || data[0] < lio_min_eig_degenerate_;
    }

    if (fresh(inputs.correction, now_s)) {
      const auto & data = inputs.correction.values;
      out.pgo_correcting = data[0] > 0.5F;
      out.pgo_last_jump = data[1];
    }

    if (fresh(inputs.fix_status, now_s)) {
      out.rtk_status = inputs.fix_status.value;
    }

    if (fresh(inputs.velocity, now_s)) {
      const auto & data = inputs.velocity.values;
      out.v = data[0];
      out.w = data[1];
    }

    return out;
  }

private:
  template <typename SampleT>
  bool fresh(const SampleT & sample, const double now_s) const
  {
    return sample.valid && (now_s - sample.stamp_s) < stale_timeout_s_;
  }

  double stale_timeout_s_;
  float lio_min_eig_degenerate_;
};

}  // namespace frc_nodes_cpp

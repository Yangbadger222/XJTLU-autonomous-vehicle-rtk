#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace um982_rtk_driver
{

enum class HeadingSource : std::size_t
{
  THS = 0,
  HPR = 1,
  UNIHEADING = 2,
};

std::optional<HeadingSource> parseHeadingSource(const std::string & value);
const char * headingSourceName(HeadingSource source);

struct HeadingSelectorConfig
{
  HeadingSource primary_source = HeadingSource::THS;
  HeadingSource fallback_source = HeadingSource::UNIHEADING;
  double fallback_timeout_s = 1.5;
  int switch_min_samples = 3;
  double max_rate_degps = 75.0;
  double max_step_deg = 15.0;
  double rate_slack_deg = 2.0;
};

struct HeadingSelection
{
  bool publish = false;
  bool source_switched = false;
  bool continuity_rejected = false;
  HeadingSource source = HeadingSource::THS;
  double heading_deg = 0.0;
  double source_bias_deg = 0.0;
};

// Select one continuous vehicle-heading stream from multiple UM982 sentences.
class HeadingSelector
{
public:
  explicit HeadingSelector(HeadingSelectorConfig config = {});

  HeadingSelection observe(
    HeadingSource source, double calibrated_heading_deg, double received_s);

  std::optional<HeadingSource> activeSource() const;
  int rejectedCount() const;

private:
  static constexpr std::size_t kSourceCount = 3;

  bool sourceMayTakeControl(HeadingSource source, double received_s) const;
  bool continuityAccepts(double heading_deg, double received_s) const;
  void resetPendingSource();
  static double normalizeHeadingDeg(double heading_deg);
  static double signedHeadingDeltaDeg(double target_deg, double reference_deg);
  static std::size_t sourceIndex(HeadingSource source);

  HeadingSelectorConfig config_;
  std::optional<double> first_observation_s_;
  std::optional<HeadingSource> active_source_;
  std::optional<HeadingSource> pending_source_;
  int pending_source_samples_ = 0;
  std::optional<double> last_published_heading_deg_;
  std::optional<double> last_published_s_;
  std::array<std::optional<double>, kSourceCount> last_accepted_s_{};
  std::array<double, kSourceCount> source_bias_deg_{};
  int rejected_count_ = 0;
};

}  // namespace um982_rtk_driver

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "fgo_gil_localizer/gnss_time.hpp"
#include "fgo_gil_localizer/time_sync.hpp"

namespace fgo_gil_localizer
{
namespace
{

TEST(TimeSyncEstimator, PromotesCoarseAndPpsStates)
{
  TimeSyncEstimator estimator;
  for (int index = 0; index < 5; ++index) {
    EXPECT_EQ(
      estimator.observe({100.0 + index, 200.01 + index, 0.01, false}),
      ClockObservationResult::Accepted);
  }
  const auto coarse = estimator.map(106.0, 106.0);
  ASSERT_TRUE(coarse.has_value());
  EXPECT_EQ(coarse->state, TimeSyncState::Coarse);
  EXPECT_NEAR(coarse->mapped_time_s, 206.01, 1.0e-12);
  EXPECT_GE(coarse->uncertainty_s, 0.02);

  EXPECT_EQ(
    estimator.observe({107.0, 207.0, 2.0e-5, true}),
    ClockObservationResult::ResetAccepted);
  EXPECT_EQ(
    estimator.observe({108.0, 208.0, 2.0e-5, true}),
    ClockObservationResult::Accepted);
  const auto pps = estimator.map(108.1, 108.1);
  ASSERT_TRUE(pps.has_value());
  EXPECT_EQ(pps->state, TimeSyncState::PpsLocked);
  EXPECT_NEAR(pps->offset_s, 100.0, 1.0e-12);
  EXPECT_NEAR(pps->uncertainty_s, 1.0e-4, 1.0e-12);
}

TEST(TimeSyncEstimator, RejectsDuplicateAndResetsOnTimeOrOffsetJump)
{
  TimeSyncEstimator estimator;
  EXPECT_EQ(estimator.observe({1.0, 2.0, 0.01, false}), ClockObservationResult::Accepted);
  EXPECT_EQ(
    estimator.observe({1.0, 2.0, 0.01, false}),
    ClockObservationResult::RejectedDuplicate);
  EXPECT_EQ(
    estimator.observe({0.5, 1.5, 0.01, false}),
    ClockObservationResult::ResetAccepted);
  EXPECT_EQ(
    estimator.observe({1.5, 20.0, 0.01, false}),
    ClockObservationResult::ResetAccepted);
  EXPECT_EQ(estimator.diagnostics().resets, 2U);
}

TEST(TimeSyncEstimator, RejectsNonFiniteObservation)
{
  TimeSyncEstimator estimator;
  EXPECT_EQ(
    estimator.observe({std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0, false}),
    ClockObservationResult::RejectedNonFinite);
  EXPECT_FALSE(estimator.map(1.0, 1.0).has_value());
}

TEST(TimeSyncEstimator, RejectsNonFiniteConfiguration)
{
  TimeSyncConfig config;
  config.coarse_timeout_s = std::numeric_limits<double>::infinity();
  EXPECT_THROW(TimeSyncEstimator{config}, std::invalid_argument);

  config = TimeSyncConfig{};
  config.pps_uncertainty_floor_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(TimeSyncEstimator{config}, std::invalid_argument);
}

TEST(TimeSyncEstimator, ExpiresStaleCoarseMapping)
{
  TimeSyncEstimator estimator;
  for (int index = 0; index < 5; ++index) {
    estimator.observe({10.0 + index, 20.0 + index, 0.01, false});
  }
  EXPECT_EQ(estimator.state(14.1), TimeSyncState::Coarse);
  EXPECT_EQ(estimator.state(20.0), TimeSyncState::Unsynced);
}

TEST(GnssTimeTracker, HandlesRolloverGapDuplicateAndReset)
{
  GnssTimeTracker tracker(2.0);
  EXPECT_EQ(tracker.accept(2427, 604799500), GnssTimeResult::Accepted);
  EXPECT_EQ(tracker.accept(2427, 604799500), GnssTimeResult::Duplicate);
  EXPECT_EQ(tracker.accept(2428, 500), GnssTimeResult::Accepted);
  EXPECT_EQ(tracker.accept(2428, 5000), GnssTimeResult::GapAccepted);
  EXPECT_EQ(tracker.accept(2427, 1000), GnssTimeResult::ResetAccepted);
  EXPECT_EQ(tracker.diagnostics().duplicates, 1U);
  EXPECT_EQ(tracker.diagnostics().gaps, 1U);
  EXPECT_EQ(tracker.diagnostics().resets, 1U);
}

TEST(GnssTimeTracker, RejectsInvalidWeekAndTow)
{
  GnssTimeTracker tracker;
  EXPECT_EQ(tracker.accept(0, 0), GnssTimeResult::Invalid);
  EXPECT_EQ(tracker.accept(2427, kGnssWeekMilliseconds), GnssTimeResult::Invalid);
}

TEST(GnssTimeTracker, InterleavedBaseEpochsDoNotResetMasterClockTracking)
{
  constexpr std::uint8_t secondary_receiver = 2U;
  constexpr std::uint8_t base_receiver = 3U;

  GnssTimeTracker tracker;
  TimeSyncEstimator estimator;
  const auto consume = [&tracker, &estimator](
    const std::uint8_t receiver, const std::uint32_t tow_ms,
    const double reception_time_s) {
      if (!isGnssClockReferenceReceiver(receiver)) {
        return;
      }
      const GnssTimeResult result = tracker.accept(2427U, tow_ms);
      ASSERT_NE(result, GnssTimeResult::Invalid);
      ASSERT_NE(result, GnssTimeResult::Duplicate);
      if (result == GnssTimeResult::ResetAccepted) {
        estimator.reset();
      }
      ASSERT_TRUE(tracker.latest().has_value());
      estimator.observe(
        {reception_time_s, tracker.latest()->absolute_seconds, 0.02, false});
    };

  constexpr std::uint32_t start_tow_ms = 297091000U;
  double last_master_reception_s = 1000.0;
  for (std::uint32_t second = 0U; second < 20U; ++second) {
    for (std::uint32_t tenth = 0U; tenth < 10U; ++tenth) {
      const std::uint32_t master_tow_ms = start_tow_ms + second * 1000U + tenth * 100U;
      last_master_reception_s = 1000.0 + static_cast<double>(second) +
        static_cast<double>(tenth) * 0.1;
      consume(kGnssClockReferenceReceiver, master_tow_ms, last_master_reception_s);
      consume(secondary_receiver, master_tow_ms, last_master_reception_s + 0.001);
      if (tenth == 2U) {
        consume(
          base_receiver, start_tow_ms + second * 1000U,
          last_master_reception_s + 0.002);
      }
    }
  }

  EXPECT_EQ(tracker.diagnostics().resets, 0U);
  EXPECT_EQ(tracker.diagnostics().accepted, 200U);
  const auto mapping = estimator.map(last_master_reception_s, last_master_reception_s);
  ASSERT_TRUE(mapping.has_value());
  EXPECT_EQ(mapping->state, TimeSyncState::Coarse);
}

}  // namespace
}  // namespace fgo_gil_localizer

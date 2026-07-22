#include "fgo_gil_localizer/reference_station_tracker.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

using fgo_gil_localizer::ReferenceStationSample;
using fgo_gil_localizer::ReferenceStationTracker;
using fgo_gil_localizer::ReferenceStationTrackerConfig;
using fgo_gil_localizer::ReferenceStationUpdate;

TEST(ReferenceStationTracker, RejectsInvalidMessageAndEcefCoordinates)
{
  ReferenceStationTracker tracker;

  EXPECT_EQ(
    tracker.accept({1U, 1077U, 0U, "caster/mount", {-2700000.0, 4300000.0, 3800000.0}}),
    ReferenceStationUpdate::Rejected);
  EXPECT_EQ(
    tracker.accept(
      {1U, 1005U, 0U, "caster/mount",
        {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}),
    ReferenceStationUpdate::Rejected);
  EXPECT_EQ(
    tracker.accept({1U, 1005U, 0U, "caster/mount", {1.0, 2.0, 3.0}}),
    ReferenceStationUpdate::Rejected);
  EXPECT_FALSE(tracker.current().has_value());
}

TEST(ReferenceStationTracker, DistinguishesRepeatAndCoordinateChange)
{
  ReferenceStationTracker tracker;
  const ReferenceStationSample initial{
    42U, 1005U, 20U, "caster/mount", {-2700000.0, 4300000.0, 3800000.0}};

  EXPECT_EQ(tracker.accept(initial), ReferenceStationUpdate::Initial);
  auto repeat = initial;
  repeat.message_type = 1006U;
  repeat.position_ecef_m.x += 0.005;
  EXPECT_EQ(tracker.accept(repeat), ReferenceStationUpdate::Unchanged);
  ASSERT_TRUE(tracker.current().has_value());
  EXPECT_DOUBLE_EQ(tracker.current()->position_ecef_m.x, initial.position_ecef_m.x);

  auto moved = initial;
  moved.position_ecef_m.x += 0.02;
  EXPECT_EQ(tracker.accept(moved), ReferenceStationUpdate::Changed);
  EXPECT_DOUBLE_EQ(tracker.current()->position_ecef_m.x, moved.position_ecef_m.x);
}

TEST(ReferenceStationTracker, StationIdChangeAlwaysRequiresReset)
{
  ReferenceStationTracker tracker;
  const ReferenceStationSample initial{
    42U, 1005U, 20U, "caster/mount", {-2700000.0, 4300000.0, 3800000.0}};
  ASSERT_EQ(tracker.accept(initial), ReferenceStationUpdate::Initial);
  auto changed = initial;
  changed.station_id = 43U;
  EXPECT_EQ(tracker.accept(changed), ReferenceStationUpdate::Changed);
}

TEST(ReferenceStationTracker, FrameOrCasterChangeRequiresReset)
{
  ReferenceStationTracker tracker;
  const ReferenceStationSample initial{
    42U, 1005U, 20U, "caster/mount", {-2700000.0, 4300000.0, 3800000.0}};
  ASSERT_EQ(tracker.accept(initial), ReferenceStationUpdate::Initial);
  auto changed = initial;
  changed.itrf_realization = 21U;
  EXPECT_EQ(tracker.accept(changed), ReferenceStationUpdate::Changed);
  changed.source = "caster/other";
  EXPECT_EQ(tracker.accept(changed), ReferenceStationUpdate::Changed);
}

TEST(ReferenceStationTracker, RejectsInvalidConfiguration)
{
  EXPECT_THROW(
    ReferenceStationTracker(ReferenceStationTrackerConfig{0.0, 5.0e6, 7.0e6}),
    std::invalid_argument);
}

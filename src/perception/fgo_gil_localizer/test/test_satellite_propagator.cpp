#include <gtest/gtest.h>

#include <cmath>

#include "fgo_gil_localizer/satellite_propagator.hpp"

namespace fgo_gil_localizer
{
namespace
{

BroadcastEphemeris circularGpsEphemeris()
{
  BroadcastEphemeris ephemeris;
  ephemeris.satellite = {GnssConstellation::Gps, 7};
  ephemeris.model = EphemerisModel::Keplerian;
  ephemeris.week = 2400;
  ephemeris.toe_s = 100000.0;
  ephemeris.toc_s = ephemeris.toe_s;
  ephemeris.semi_major_axis_m = 26560000.0;
  ephemeris.eccentricity = 0.0;
  ephemeris.ascending_node_rad = 7.2921151467e-5 * ephemeris.toe_s;
  ephemeris.clock_bias_s = 2.0e-5;
  ephemeris.clock_drift_s_s = 1.0e-11;
  return ephemeris;
}

TEST(SatelliteSignal, UsesOfficialUm982SignalSemantics)
{
  const auto gps_l1 = carrierFrequencyHz({GnssConstellation::Gps, 0, false, 0});
  const auto gps_l2c_bit = carrierFrequencyHz({GnssConstellation::Gps, 9, true, 0});
  const auto bds_b1i = carrierFrequencyHz({GnssConstellation::Bds, 0, false, 0});
  const auto glo_l1_minus_seven = carrierFrequencyHz(
    {GnssConstellation::Glonass, 0, false, -7});
  ASSERT_TRUE(gps_l1.has_value());
  ASSERT_TRUE(gps_l2c_bit.has_value());
  ASSERT_TRUE(bds_b1i.has_value());
  ASSERT_TRUE(glo_l1_minus_seven.has_value());
  EXPECT_DOUBLE_EQ(*gps_l1, 1575.42e6);
  EXPECT_DOUBLE_EQ(*gps_l2c_bit, 1227.60e6);
  EXPECT_DOUBLE_EQ(*bds_b1i, 1561.098e6);
  EXPECT_DOUBLE_EQ(*glo_l1_minus_seven, 1602.0e6 - 7.0 * 0.5625e6);
  EXPECT_FALSE(carrierFrequencyHz({GnssConstellation::Gps, 31, false, 0}).has_value());
  EXPECT_FALSE(
    carrierFrequencyHz(
      {GnssConstellation::Glonass, 0, false, -8}).has_value());
}

TEST(SatellitePropagation, CircularOrbitIsFiniteAndClocked)
{
  const BroadcastEphemeris ephemeris = circularGpsEphemeris();
  SatellitePropagator propagator;
  const auto at_toe = propagator.propagate(ephemeris, {ephemeris.week, ephemeris.toe_s});
  ASSERT_TRUE(at_toe.ok());
  EXPECT_NEAR(at_toe.state->position_ecef_m.x, ephemeris.semi_major_axis_m, 1.0e-5);
  EXPECT_NEAR(at_toe.state->position_ecef_m.y, 0.0, 1.0e-5);
  EXPECT_NEAR(at_toe.state->position_ecef_m.z, 0.0, 1.0e-5);
  EXPECT_NEAR(norm(at_toe.state->position_ecef_m), ephemeris.semi_major_axis_m, 1.0e-4);
  EXPECT_GT(norm(at_toe.state->velocity_ecef_m_s), 1000.0);
  EXPECT_NEAR(at_toe.state->clock_offset_s, ephemeris.clock_bias_s, 1.0e-15);
}

TEST(SatellitePropagation, ReceiveFrameAppliesTransmitTimeAndSagnac)
{
  const BroadcastEphemeris ephemeris = circularGpsEphemeris();
  SatellitePropagator propagator;
  const GnssTime receive_time{ephemeris.week, ephemeris.toe_s};
  const Vec3 receiver{6378137.0, 0.0, 0.0};
  const auto direct = propagator.propagate(ephemeris, receive_time);
  const auto corrected = propagator.propagateToReceiveFrame(
    ephemeris, receive_time, receiver);
  ASSERT_TRUE(direct.ok());
  ASSERT_TRUE(corrected.ok());
  EXPECT_LT(corrected.state->transmit_time.tow_s, receive_time.tow_s);
  EXPECT_GT(norm(corrected.state->position_ecef_m - direct.state->position_ecef_m), 100.0);
  const double geometric_transit_s =
    norm(corrected.state->position_ecef_m - receiver) / kSpeedOfLightMps;
  EXPECT_NEAR(
    receive_time.tow_s - corrected.state->transmit_time.tow_s,
    geometric_transit_s, 1.0e-9);
}

TEST(SatellitePropagation, IterativeGeometricTransmitTimeIsDeterministic)
{
  const BroadcastEphemeris ephemeris = circularGpsEphemeris();
  SatellitePropagator propagator;
  const GnssTime receive_time{ephemeris.week, ephemeris.toe_s + 10.0};
  const Vec3 receiver{6378137.0, 100.0, -50.0};
  const auto first = propagator.propagateToReceiveFrame(ephemeris, receive_time, receiver);
  const auto repeated = propagator.propagateToReceiveFrame(ephemeris, receive_time, receiver);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(repeated.ok());
  EXPECT_NEAR(
    norm(first.state->position_ecef_m - repeated.state->position_ecef_m), 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(first.state->transmit_time.tow_s, repeated.state->transmit_time.tow_s);
}

TEST(SatellitePropagation, BdsGeoBranchRemainsFinite)
{
  BroadcastEphemeris ephemeris = circularGpsEphemeris();
  ephemeris.satellite = {GnssConstellation::Bds, 3};
  ephemeris.inclination_rad = 0.05;
  SatellitePropagator propagator;
  const auto result = propagator.propagate(
    ephemeris, {ephemeris.week, ephemeris.toe_s + 30.0});
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(finite(result.state->position_ecef_m));
  EXPECT_NEAR(norm(result.state->position_ecef_m), ephemeris.semi_major_axis_m, 1.0);
}

TEST(SatellitePropagation, GlonassZeroDeltaPreservesBroadcastState)
{
  BroadcastEphemeris ephemeris;
  ephemeris.satellite = {GnssConstellation::Glonass, 45};
  ephemeris.model = EphemerisModel::Glonass;
  ephemeris.week = 2400;
  ephemeris.toe_s = 200000.0;
  ephemeris.toc_s = ephemeris.toe_s;
  ephemeris.position_ecef_m = {19100000.0, 12000000.0, 13000000.0};
  ephemeris.velocity_ecef_m_s = {-1200.0, 2100.0, 900.0};
  ephemeris.acceleration_ecef_m_s2 = {1.0e-6, -2.0e-6, 0.5e-6};
  ephemeris.glonass_clock_bias_s = 3.0e-5;
  ephemeris.glonass_relative_frequency_bias = 2.0e-11;
  SatellitePropagator propagator;
  const auto result = propagator.propagate(ephemeris, {ephemeris.week, ephemeris.toe_s});
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(norm(result.state->position_ecef_m - ephemeris.position_ecef_m), 0.0, 1.0e-9);
  EXPECT_NEAR(norm(result.state->velocity_ecef_m_s - ephemeris.velocity_ecef_m_s), 0.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.state->clock_offset_s, -ephemeris.glonass_clock_bias_s);
}

TEST(SatellitePropagation, RejectsBadHealthAgeAndNonFiniteFields)
{
  SatellitePropagator propagator;
  BroadcastEphemeris unhealthy = circularGpsEphemeris();
  unhealthy.health = 1U;
  EXPECT_EQ(
    propagator.propagate(unhealthy, {unhealthy.week, unhealthy.toe_s}).error,
    SatellitePropagationError::UnhealthySatellite);

  BroadcastEphemeris old = circularGpsEphemeris();
  EXPECT_EQ(
    propagator.propagate(old, {old.week, old.toe_s + 20000.0}).error,
    SatellitePropagationError::EphemerisTooOld);

  BroadcastEphemeris nonfinite = circularGpsEphemeris();
  nonfinite.eccentricity = std::nan("");
  EXPECT_EQ(
    propagator.propagate(nonfinite, {nonfinite.week, nonfinite.toe_s}).error,
    SatellitePropagationError::InvalidEphemeris);
}

}  // namespace
}  // namespace fgo_gil_localizer

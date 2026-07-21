#include <gtest/gtest.h>

#include <cmath>
#include <map>

#include "fgo_gil_localizer/gnss_double_difference.hpp"
#include "fgo_gil_localizer/satellite_propagator.hpp"

namespace fgo_gil_localizer
{
namespace
{

GnssObservation observation(
  const std::uint16_t prn,
  const double pseudorange,
  const double phase_cycles,
  const double doppler_hz = 0.0,
  const double lock_time_s = 10.0)
{
  GnssObservation value;
  value.satellite = {GnssConstellation::Gps, prn};
  value.signal = {GnssConstellation::Gps, 0, false, 0};
  value.channel_number = static_cast<std::uint8_t>(prn);
  value.pseudorange_m = pseudorange;
  value.carrier_phase_cycles = phase_cycles;
  value.doppler_hz = doppler_hz;
  value.pseudorange_std_m = 0.2;
  value.carrier_phase_std_cycles = 0.01;
  value.cn0_db_hz = 45.0;
  value.lock_time_s = lock_time_s;
  value.pseudorange_valid = true;
  value.carrier_phase_valid = true;
  return value;
}

TEST(GnssEpochAligner, MatchesOnceAndBoundsUnmatchedInput)
{
  GnssEpochAligner aligner({0.020, 2, 8});
  GnssObservationEpoch rover{GnssReceiver::Master, {2400, 100.000}, {}};
  GnssObservationEpoch base{GnssReceiver::Base, {2400, 100.012}, {}};
  EXPECT_FALSE(aligner.push(rover).has_value());
  const auto match = aligner.push(base);
  ASSERT_TRUE(match.has_value());
  EXPECT_NEAR(match->time_offset_s, -0.012, 1.0e-9);
  EXPECT_FALSE(aligner.push(rover).has_value());
  EXPECT_EQ(aligner.diagnostics().duplicates, 1U);

  for (int index = 0; index < 4; ++index) {
    rover.time.tow_s = 200.0 + static_cast<double>(index);
    EXPECT_FALSE(aligner.push(rover).has_value());
  }
  EXPECT_EQ(aligner.roverBufferSize(), 2U);
  EXPECT_EQ(aligner.diagnostics().dropped_capacity, 2U);
}

TEST(DoubleDifferenceDiagnostics, RejectReasonsHaveStableNames)
{
  EXPECT_STREQ(toString(ArcResetReason::FirstObservation), "FIRST_OBSERVATION");
  EXPECT_STREQ(toString(ArcResetReason::Reacquired), "REACQUIRED");
  EXPECT_STREQ(toString(ArcResetReason::LockTimeReset), "LOCK_TIME_RESET");
  EXPECT_STREQ(toString(ArcResetReason::TrackingChannelChanged), "TRACKING_CHANNEL_CHANGED");
  EXPECT_STREQ(toString(ArcResetReason::ObservationGap), "OBSERVATION_GAP");
  EXPECT_STREQ(
    toString(ArcResetReason::DopplerPhaseInconsistent), "DOPPLER_PHASE_INCONSISTENT");
  EXPECT_STREQ(toString(ArcResetReason::TimeReversal), "TIME_REVERSAL");
  EXPECT_STREQ(toString(DdRejectReason::InvalidEpoch), "INVALID_EPOCH");
  EXPECT_STREQ(toString(DdRejectReason::BaselineTooLong), "BASELINE_TOO_LONG");
  EXPECT_STREQ(toString(DdRejectReason::MissingBaseObservation), "MISSING_BASE_OBSERVATION");
  EXPECT_STREQ(toString(DdRejectReason::UnsupportedSignal), "UNSUPPORTED_SIGNAL");
  EXPECT_STREQ(toString(DdRejectReason::MissingSatelliteState), "MISSING_SATELLITE_STATE");
  EXPECT_STREQ(toString(DdRejectReason::LowQuality), "LOW_QUALITY");
  EXPECT_STREQ(toString(DdRejectReason::BelowElevationMask), "BELOW_ELEVATION_MASK");
  EXPECT_STREQ(toString(DdRejectReason::MissingReference), "MISSING_REFERENCE");
  EXPECT_STREQ(toString(DdRejectReason::CodeInnovation), "CODE_INNOVATION");
}

TEST(GnssReferenceSelector, UsesElevationHysteresisAndReportsSwitch)
{
  GnssReferenceSelector selector({0.0, 20.0, 5.0 * 3.14159265358979323846 / 180.0});
  const SignalGroup group{GnssConstellation::Gps, 0, false};
  auto selected = selector.select(
      {
        {{GnssConstellation::Gps, 3}, group, 0.80, 45.0},
        {{GnssConstellation::Gps, 7}, group, 0.75, 45.0}});
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.at(group).satellite.prn, 3U);

  selected = selector.select(
      {
        {{GnssConstellation::Gps, 3}, group, 0.80, 45.0},
        {{GnssConstellation::Gps, 7}, group, 0.85, 45.0}});
  EXPECT_EQ(selected.at(group).satellite.prn, 3U);
  EXPECT_FALSE(selected.at(group).switched);

  selected = selector.select(
      {
        {{GnssConstellation::Gps, 3}, group, 0.70, 45.0},
        {{GnssConstellation::Gps, 7}, group, 0.90, 45.0}});
  EXPECT_EQ(selected.at(group).satellite.prn, 7U);
  EXPECT_TRUE(selected.at(group).switched);
}

TEST(AmbiguityArcManager, ResetsOnlyAffectedSignalOnSlip)
{
  AmbiguityArcManager manager;
  GnssObservation first = observation(3, 2.1e7, 1000.0, 5.0, 10.0);
  GnssObservation other = observation(7, 2.2e7, 2000.0, 4.0, 10.0);
  const auto first_arc = manager.update(GnssReceiver::Master, {2400, 100.0}, first);
  const auto other_arc = manager.update(GnssReceiver::Master, {2400, 100.0}, other);
  ASSERT_TRUE(first_arc.new_arc);
  ASSERT_TRUE(other_arc.new_arc);

  first.carrier_phase_cycles = 1005.0;
  first.lock_time_s = 11.0;
  other.carrier_phase_cycles = 2004.0;
  other.lock_time_s = 11.0;
  const auto first_continued = manager.update(GnssReceiver::Master, {2400, 101.0}, first);
  const auto other_continued = manager.update(GnssReceiver::Master, {2400, 101.0}, other);
  EXPECT_FALSE(first_continued.new_arc);
  EXPECT_FALSE(other_continued.new_arc);

  first.carrier_phase_cycles += 8.0;
  first.lock_time_s = 0.1;
  other.carrier_phase_cycles += 4.0;
  other.lock_time_s = 12.0;
  const auto slipped = manager.update(GnssReceiver::Master, {2400, 102.0}, first);
  const auto unaffected = manager.update(GnssReceiver::Master, {2400, 102.0}, other);
  EXPECT_TRUE(slipped.new_arc);
  EXPECT_EQ(slipped.reason, ArcResetReason::LockTimeReset);
  EXPECT_NE(slipped.arc_id, first_arc.arc_id);
  EXPECT_FALSE(unaffected.new_arc);
  EXPECT_EQ(unaffected.arc_id, other_arc.arc_id);
}

TEST(AmbiguityArcManager, UsesUm982DopplerSignForPhasePrediction)
{
  AmbiguityArcManager manager;
  GnssObservation sample = observation(3, 2.1e7, 1000.0, -5.0, 10.0);
  const auto first = manager.update(GnssReceiver::Master, {2400, 100.0}, sample);
  ASSERT_TRUE(first.new_arc);

  sample.carrier_phase_cycles = 995.0;
  sample.lock_time_s = 11.0;
  const auto continued = manager.update(GnssReceiver::Master, {2400, 101.0}, sample);
  EXPECT_FALSE(continued.new_arc);

  sample.carrier_phase_cycles = 1000.0;
  sample.lock_time_s = 12.0;
  const auto inconsistent = manager.update(GnssReceiver::Master, {2400, 102.0}, sample);
  EXPECT_TRUE(inconsistent.new_arc);
  EXPECT_EQ(inconsistent.reason, ArcResetReason::DopplerPhaseInconsistent);
}

TEST(AmbiguityArcManager, SkipsDopplerCheckWhenBaseDopplerIsUnavailable)
{
  AmbiguityArcManager manager;
  GnssObservation sample = observation(3, 2.1e7, 1000.0, 0.0, 10.0);
  sample.doppler_valid = false;
  const auto first = manager.update(GnssReceiver::Base, {2400, 100.0}, sample);
  ASSERT_TRUE(first.new_arc);

  sample.carrier_phase_cycles = 1012.5;
  sample.lock_time_s = 11.0;
  const auto continued = manager.update(GnssReceiver::Base, {2400, 101.0}, sample);
  EXPECT_FALSE(continued.new_arc);
  EXPECT_EQ(continued.arc_id, first.arc_id);
}

TEST(AmbiguityArcManager, UsesReceiverSpecificObservationGap)
{
  AmbiguityArcManager manager;
  GnssObservation sample = observation(3, 2.1e7, 1000.0, 0.0, 10.0);
  sample.doppler_valid = false;
  const auto rover_first = manager.update(GnssReceiver::Master, {2400, 100.0}, sample);
  const auto base_first = manager.update(GnssReceiver::Base, {2400, 100.0}, sample);
  ASSERT_TRUE(rover_first.new_arc);
  ASSERT_TRUE(base_first.new_arc);

  sample.lock_time_s = 13.0;
  const auto rover_gap = manager.update(GnssReceiver::Master, {2400, 103.0}, sample);
  const auto base_gap = manager.update(GnssReceiver::Base, {2400, 103.0}, sample);
  EXPECT_TRUE(rover_gap.new_arc);
  EXPECT_EQ(rover_gap.reason, ArcResetReason::ObservationGap);
  EXPECT_FALSE(base_gap.new_arc);
  EXPECT_EQ(base_gap.arc_id, base_first.arc_id);
}

TEST(DoubleDifferenceBuilder, RecoversGeometryLeverArmAndFloatAmbiguity)
{
  EcefState rover_state;
  rover_state.stamp_s = 100.0;
  rover_state.position_ecef_m = {6378137.0, 0.0, 0.0};
  rover_state.orientation_ecef_body = {};
  const Vec3 lever_arm{0.0, 1.0, 0.0};
  const Vec3 rover_antenna = rover_state.position_ecef_m + lever_arm;
  const Vec3 base{6378137.0, -10.0, 0.0};
  const SatelliteId reference_id{GnssConstellation::Gps, 3};
  const SatelliteId target_id{GnssConstellation::Gps, 7};
  const Vec3 reference_position{26500000.0, 0.0, 4000000.0};
  const Vec3 target_position{21000000.0, 11000000.0, 9000000.0};
  SatelliteStateMap states;
  const Vec3 reference_base_position = reference_position + Vec3{12.0, -4.0, 2.0};
  const Vec3 target_base_position = target_position + Vec3{-7.0, 9.0, -3.0};
  states[reference_id] = {
    {reference_id, {2400, 100.0}, reference_position, {}, 0.0, 0.0},
    {reference_id, {2400, 100.0}, reference_base_position, {}, 0.0, 0.0}};
  states[target_id] = {
    {target_id, {2400, 100.0}, target_position, {}, 0.0, 0.0},
    {target_id, {2400, 100.0}, target_base_position, {}, 0.0, 0.0}};
  const double wavelength = *carrierWavelengthM({GnssConstellation::Gps, 0, false, 0});

  const auto make_receiver_observations = [&](const Vec3 & receiver,
      const Vec3 & receiver_reference_position, const Vec3 & receiver_target_position,
      const double clock_m, const double reference_ambiguity_m,
      const double target_ambiguity_m) {
      const double reference_range = norm(receiver_reference_position - receiver);
      const double target_range = norm(receiver_target_position - receiver);
      return std::vector<GnssObservation>{
      observation(
        3, reference_range + clock_m,
        (reference_range + clock_m + reference_ambiguity_m) / wavelength),
      observation(
        7, target_range + clock_m,
        (target_range + clock_m + target_ambiguity_m) / wavelength)};
    };
  AlignedGnssEpochs epochs;
  epochs.rover = {
    GnssReceiver::Master, {2400, 100.0},
    make_receiver_observations(
      rover_antenna, reference_position, target_position, 100.0, 5.0, 17.0)};
  epochs.base = {
    GnssReceiver::Base, {2400, 100.0},
    make_receiver_observations(
      base, reference_base_position, target_base_position, 30.0, 2.0, 4.0)};
  for (auto & base_observation : epochs.base.observations) {
    base_observation.pseudorange_std_m = 0.0;
    base_observation.carrier_phase_std_cycles = 0.0;
  }

  DoubleDifferenceBuilder builder;
  const auto measurements = builder.build(epochs, states, rover_state, base, lever_arm);
  ASSERT_EQ(measurements.size(), 1U);
  EXPECT_EQ(builder.diagnostics().new_arcs, 4U);
  EXPECT_EQ(
    builder.diagnostics().arc_resets[
      static_cast<std::size_t>(ArcResetReason::FirstObservation)],
    4U);
  const auto & measurement = measurements.front();
  ASSERT_TRUE(measurement.code_valid);
  ASSERT_TRUE(measurement.carrier_valid);
  EXPECT_EQ(measurement.reference.prn, 3U);
  EXPECT_GT(measurement.carrier_sigma_m, 0.014);
  EXPECT_GT(measurement.code_sigma_m, 0.47);
  const auto code = evaluateDdPseudorange(measurement, rover_state);
  ASSERT_TRUE(code.has_value());
  EXPECT_NEAR(code->residual_m, 0.0, 1.0e-6);
  const double ambiguity_dd_m = (17.0 - 4.0) - (5.0 - 2.0);
  const auto carrier = evaluateDdCarrier(measurement, rover_state, ambiguity_dd_m);
  ASSERT_TRUE(carrier.has_value());
  EXPECT_NEAR(carrier->residual_m, 0.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(carrier->ambiguity_jacobian, 1.0);
}

TEST(DoubleDifferenceFactor, AnalyticPoseJacobianMatchesFiniteDifference)
{
  DoubleDifferenceMeasurement measurement;
  measurement.code_valid = true;
  measurement.code_dd_m = 0.0;
  measurement.target_position_ecef_m = {21000000.0, 11000000.0, 9000000.0};
  measurement.reference_position_ecef_m = {26500000.0, 0.0, 4000000.0};
  measurement.target_base_position_ecef_m = measurement.target_position_ecef_m;
  measurement.reference_base_position_ecef_m = measurement.reference_position_ecef_m;
  measurement.base_position_ecef_m = {6378137.0, -10.0, 0.0};
  measurement.lever_arm_body_m = {0.2, -0.1, 0.3};
  EcefState state;
  state.position_ecef_m = {6378137.0, 0.0, 0.0};
  state.orientation_ecef_body = quaternionFromRotationVector({0.1, -0.05, 0.02});
  const auto analytic = evaluateDdPseudorange(measurement, state);
  ASSERT_TRUE(analytic.has_value());
  for (std::size_t column = 0; column < 6U; ++column) {
    const double epsilon = 1.0e-3;
    EcefState plus = state;
    EcefState minus = state;
    if (column < 3U) {
      plus.position_ecef_m[column] += epsilon;
      minus.position_ecef_m[column] -= epsilon;
    } else {
      Vec3 rotation;
      rotation[column - 3U] = epsilon;
      plus.orientation_ecef_body =
        (quaternionFromRotationVector(rotation) * state.orientation_ecef_body).normalized();
      minus.orientation_ecef_body =
        (quaternionFromRotationVector(-rotation) * state.orientation_ecef_body).normalized();
    }
    const auto plus_value = evaluateDdPseudorange(measurement, plus);
    const auto minus_value = evaluateDdPseudorange(measurement, minus);
    ASSERT_TRUE(plus_value.has_value());
    ASSERT_TRUE(minus_value.has_value());
    const double numerical = (plus_value->residual_m - minus_value->residual_m) /
      (2.0 * epsilon);
    EXPECT_NEAR(analytic->pose_jacobian[column], numerical, 2.0e-4);
  }
}

}  // namespace
}  // namespace fgo_gil_localizer

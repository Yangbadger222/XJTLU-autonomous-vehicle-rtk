#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "fgo_gil_localizer/ecef_imu_preintegrator.hpp"
#include "fgo_gil_localizer/gnss_types.hpp"
#include "fgo_gil_localizer/lidar_factors.hpp"

namespace fgo_gil_localizer
{

struct SignalGroup
{
  GnssConstellation constellation = GnssConstellation::Gps;
  std::uint8_t signal_type = 0;
  bool l2c_signal = false;

  bool operator==(const SignalGroup & other) const noexcept
  {
    return constellation == other.constellation && signal_type == other.signal_type &&
           l2c_signal == other.l2c_signal;
  }

  bool operator<(const SignalGroup & other) const noexcept
  {
    if (constellation != other.constellation) {
      return constellation < other.constellation;
    }
    if (signal_type != other.signal_type) {
      return signal_type < other.signal_type;
    }
    return l2c_signal < other.l2c_signal;
  }
};

SignalGroup signalGroup(const SignalKey & signal) noexcept;
double um982AdrToCarrierPhaseCycles(double adr_cycles) noexcept;

struct AlignedGnssEpochs
{
  GnssObservationEpoch rover;
  GnssObservationEpoch base;
  double time_offset_s = 0.0;
};

struct EpochAlignerConfig
{
  double maximum_time_offset_s = 0.020;
  std::size_t maximum_buffered_epochs = 64;
  std::size_t maximum_seen_epochs = 512;
};

struct EpochAlignerDiagnostics
{
  std::uint64_t matched = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t invalid = 0;
  std::uint64_t unsupported_receiver = 0;
  std::uint64_t dropped_capacity = 0;
};

class GnssEpochAligner
{
public:
  explicit GnssEpochAligner(EpochAlignerConfig config = {});

  std::optional<AlignedGnssEpochs> push(const GnssObservationEpoch & epoch);
  const EpochAlignerDiagnostics & diagnostics() const noexcept {return diagnostics_;}
  std::size_t roverBufferSize() const noexcept {return rover_epochs_.size();}
  std::size_t baseBufferSize() const noexcept {return base_epochs_.size();}

private:
  struct EpochKey
  {
    GnssReceiver receiver = GnssReceiver::Master;
    std::uint32_t week = 0;
    std::int64_t milliseconds = 0;

    bool operator<(const EpochKey & other) const noexcept;
  };

  std::optional<AlignedGnssEpochs> matchNewest(bool newest_is_rover);
  bool remember(const GnssObservationEpoch & epoch);
  void enforceCapacity(std::deque<GnssObservationEpoch> & epochs);

  EpochAlignerConfig config_;
  std::deque<GnssObservationEpoch> rover_epochs_;
  std::deque<GnssObservationEpoch> base_epochs_;
  std::set<EpochKey> seen_;
  std::deque<EpochKey> seen_order_;
  EpochAlignerDiagnostics diagnostics_;
};

struct ReferenceCandidate
{
  SatelliteId satellite;
  SignalGroup group;
  double elevation_rad = 0.0;
  double cn0_db_hz = 0.0;
};

struct ReferenceSelection
{
  SatelliteId satellite;
  double elevation_rad = 0.0;
  bool switched = false;
};

struct ReferenceSelectorConfig
{
  double minimum_elevation_rad = 10.0 * 3.14159265358979323846 / 180.0;
  double minimum_cn0_db_hz = 25.0;
  double switch_margin_rad = 5.0 * 3.14159265358979323846 / 180.0;
};

class GnssReferenceSelector
{
public:
  explicit GnssReferenceSelector(ReferenceSelectorConfig config = {});

  std::map<SignalGroup, ReferenceSelection> select(
    const std::vector<ReferenceCandidate> & candidates);

private:
  ReferenceSelectorConfig config_;
  std::map<SignalGroup, SatelliteId> references_;
};

enum class ArcResetReason : std::uint8_t
{
  None,
  FirstObservation,
  Reacquired,
  LockTimeReset,
  TrackingChannelChanged,
  ObservationGap,
  DopplerPhaseInconsistent,
  TimeReversal,
  Count,
};

const char * toString(ArcResetReason reason) noexcept;

struct ArcUpdate
{
  bool usable = false;
  bool new_arc = false;
  std::uint64_t arc_id = 0;
  ArcResetReason reason = ArcResetReason::None;
};

struct AmbiguityArcConfig
{
  double lock_time_tolerance_s = 0.050;
  double maximum_observation_gap_s = 2.0;
  double base_maximum_observation_gap_s = 5.0;
  double doppler_phase_threshold_cycles = 0.75;
};

class AmbiguityArcManager
{
public:
  explicit AmbiguityArcManager(AmbiguityArcConfig config = {});

  ArcUpdate update(
    GnssReceiver receiver,
    const GnssTime & time,
    const GnssObservation & observation);
  std::size_t trackedSignalCount() const noexcept {return states_.size();}

private:
  struct ArcKey
  {
    GnssReceiver receiver = GnssReceiver::Master;
    SatelliteId satellite;
    SignalKey signal;

    bool operator<(const ArcKey & other) const noexcept;
  };

  struct ArcState
  {
    bool active = false;
    GnssTime time;
    double carrier_phase_cycles = 0.0;
    double doppler_hz = 0.0;
    bool doppler_valid = false;
    double lock_time_s = 0.0;
    std::uint8_t channel_number = 0;
    std::uint64_t arc_id = 0;
  };

  ArcUpdate startArc(ArcState & state, ArcResetReason reason);

  AmbiguityArcConfig config_;
  std::map<ArcKey, ArcState> states_;
  std::uint64_t next_arc_id_ = 1;
};

struct DdAmbiguityKey
{
  SignalGroup group;
  SatelliteId reference;
  SatelliteId target;
  std::array<std::uint64_t, 4> receiver_arc_ids{};

  bool operator==(const DdAmbiguityKey & other) const noexcept;
  bool operator<(const DdAmbiguityKey & other) const noexcept;
};

struct DoubleDifferenceMeasurement
{
  GnssTime time;
  SignalGroup group;
  SatelliteId reference;
  SatelliteId target;
  DdAmbiguityKey ambiguity_key;
  Vec3 target_position_ecef_m;
  Vec3 reference_position_ecef_m;
  Vec3 target_base_position_ecef_m;
  Vec3 reference_base_position_ecef_m;
  Vec3 base_position_ecef_m;
  Vec3 lever_arm_body_m;
  double code_dd_m = 0.0;
  double carrier_dd_m = 0.0;
  double code_sigma_m = 0.0;
  double carrier_sigma_m = 0.0;
  double code_target_variance_m2 = 0.0;
  double code_reference_variance_m2 = 0.0;
  double carrier_target_variance_m2 = 0.0;
  double carrier_reference_variance_m2 = 0.0;
  double target_elevation_rad = 0.0;
  double reference_elevation_rad = 0.0;
  bool code_valid = false;
  bool carrier_valid = false;
  bool reference_switched = false;
};

enum class DdRejectReason : std::uint8_t
{
  InvalidEpoch,
  BaselineTooLong,
  MissingBaseObservation,
  UnsupportedSignal,
  MissingSatelliteState,
  LowQuality,
  BelowElevationMask,
  MissingReference,
  CodeInnovation,
  Count,
};

const char * toString(DdRejectReason reason) noexcept;

struct DdBuilderDiagnostics
{
  std::uint64_t aligned_epochs = 0;
  std::uint64_t code_factors = 0;
  std::uint64_t carrier_factors = 0;
  std::uint64_t new_arcs = 0;
  std::array<std::uint64_t, static_cast<std::size_t>(ArcResetReason::Count)> arc_resets{};
  std::array<std::uint64_t, static_cast<std::size_t>(DdRejectReason::Count)> rejected{};
};

struct DoubleDifferenceBuilderConfig
{
  double minimum_elevation_rad = 10.0 * 3.14159265358979323846 / 180.0;
  double minimum_cn0_db_hz = 25.0;
  double maximum_baseline_m = 20000.0;
  double maximum_code_innovation_m = 30.0;
  double minimum_code_sigma_m = 0.05;
  double minimum_carrier_sigma_m = 0.001;
  double unavailable_base_code_sigma_m = 0.30;
  double unavailable_base_carrier_sigma_m = 0.01;
};

struct SatelliteLinkStates
{
  SatelliteState rover;
  SatelliteState base;
};

using SatelliteStateMap = std::map<SatelliteId, SatelliteLinkStates>;

class DoubleDifferenceBuilder
{
public:
  DoubleDifferenceBuilder(
    DoubleDifferenceBuilderConfig config = {},
    ReferenceSelectorConfig reference_config = {},
    AmbiguityArcConfig arc_config = {});

  std::vector<DoubleDifferenceMeasurement> build(
    const AlignedGnssEpochs & epochs,
    const SatelliteStateMap & satellite_states,
    const EcefState & rover_state,
    const Vec3 & base_position_ecef_m,
    const Vec3 & lever_arm_body_m);

  const DdBuilderDiagnostics & diagnostics() const noexcept {return diagnostics_;}

private:
  void reject(DdRejectReason reason);

  DoubleDifferenceBuilderConfig config_;
  GnssReferenceSelector reference_selector_;
  AmbiguityArcManager arc_manager_;
  DdBuilderDiagnostics diagnostics_;
};

struct DdFactorEvaluation
{
  double residual_m = 0.0;
  PoseJacobianRow pose_jacobian{};
  double ambiguity_jacobian = 0.0;
};

std::optional<DdFactorEvaluation> evaluateDdPseudorange(
  const DoubleDifferenceMeasurement & measurement,
  const EcefState & rover_state);
std::optional<DdFactorEvaluation> evaluateDdCarrier(
  const DoubleDifferenceMeasurement & measurement,
  const EcefState & rover_state,
  double ambiguity_m);

}  // namespace fgo_gil_localizer

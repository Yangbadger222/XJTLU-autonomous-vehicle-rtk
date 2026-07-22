#include "fgo_gil_localizer/gnss_double_difference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

#include "fgo_gil_localizer/satellite_propagator.hpp"

namespace fgo_gil_localizer
{

const char * toString(const ArcResetReason reason) noexcept
{
  switch (reason) {
    case ArcResetReason::None:
      return "NONE";
    case ArcResetReason::FirstObservation:
      return "FIRST_OBSERVATION";
    case ArcResetReason::Reacquired:
      return "REACQUIRED";
    case ArcResetReason::LockTimeReset:
      return "LOCK_TIME_RESET";
    case ArcResetReason::TrackingChannelChanged:
      return "TRACKING_CHANNEL_CHANGED";
    case ArcResetReason::ObservationGap:
      return "OBSERVATION_GAP";
    case ArcResetReason::DopplerPhaseInconsistent:
      return "DOPPLER_PHASE_INCONSISTENT";
    case ArcResetReason::TimeReversal:
      return "TIME_REVERSAL";
    case ArcResetReason::Count:
      return "COUNT";
  }
  return "UNKNOWN";
}

const char * toString(const DdRejectReason reason) noexcept
{
  switch (reason) {
    case DdRejectReason::InvalidEpoch:
      return "INVALID_EPOCH";
    case DdRejectReason::BaselineTooLong:
      return "BASELINE_TOO_LONG";
    case DdRejectReason::MissingBaseObservation:
      return "MISSING_BASE_OBSERVATION";
    case DdRejectReason::UnsupportedSignal:
      return "UNSUPPORTED_SIGNAL";
    case DdRejectReason::MissingSatelliteState:
      return "MISSING_SATELLITE_STATE";
    case DdRejectReason::LowQuality:
      return "LOW_QUALITY";
    case DdRejectReason::BelowElevationMask:
      return "BELOW_ELEVATION_MASK";
    case DdRejectReason::MissingReference:
      return "MISSING_REFERENCE";
    case DdRejectReason::CodeInnovation:
      return "CODE_INNOVATION";
    case DdRejectReason::Count:
      return "COUNT";
  }
  return "UNKNOWN";
}

namespace
{

double epochDelta(const GnssTime & left, const GnssTime & right)
{
  return (static_cast<double>(left.week) - static_cast<double>(right.week)) *
         kGnssWeekSeconds + left.tow_s - right.tow_s;
}

bool validEpoch(const GnssObservationEpoch & epoch)
{
  return std::isfinite(epoch.time.tow_s) && epoch.time.tow_s >= 0.0 &&
         epoch.time.tow_s < kGnssWeekSeconds;
}

struct ObservationKey
{
  SatelliteId satellite;
  SignalKey signal;

  bool operator<(const ObservationKey & other) const noexcept
  {
    if (satellite < other.satellite) {
      return true;
    }
    if (other.satellite < satellite) {
      return false;
    }
    return signal < other.signal;
  }
};

struct CommonObservation
{
  const GnssObservation * rover = nullptr;
  const GnssObservation * base = nullptr;
  const SatelliteLinkStates * satellite = nullptr;
  double wavelength_m = 0.0;
  double elevation_rad = 0.0;
  ArcUpdate rover_arc;
  ArcUpdate base_arc;
};

double geometricRange(const Vec3 & satellite, const Vec3 & receiver)
{
  return norm(satellite - receiver);
}

double predictedDoubleDifference(
  const Vec3 & target_rover_satellite,
  const Vec3 & target_base_satellite,
  const Vec3 & reference_rover_satellite,
  const Vec3 & reference_base_satellite,
  const Vec3 & rover_antenna,
  const Vec3 & base)
{
  return geometricRange(target_rover_satellite, rover_antenna) -
         geometricRange(target_base_satellite, base) -
         geometricRange(reference_rover_satellite, rover_antenna) +
         geometricRange(reference_base_satellite, base);
}

std::optional<Vec3> rangeGradient(const Vec3 & satellite, const Vec3 & receiver)
{
  const Vec3 delta = receiver - satellite;
  const double distance = norm(delta);
  if (!std::isfinite(distance) || distance < 1.0) {
    return std::nullopt;
  }
  return delta / distance;
}

bool finiteObservation(const GnssObservation & observation)
{
  return std::isfinite(observation.pseudorange_m) &&
         std::isfinite(observation.carrier_phase_cycles) &&
         (!observation.doppler_valid || std::isfinite(observation.doppler_hz)) &&
         std::isfinite(observation.pseudorange_std_m) &&
         std::isfinite(observation.carrier_phase_std_cycles) &&
         std::isfinite(observation.cn0_db_hz) && std::isfinite(observation.lock_time_s);
}

double usableSigma(const double reported, const double minimum, const double unavailable)
{
  return std::isfinite(reported) && reported > 0.0 ?
         std::max(reported, minimum) : std::max(unavailable, minimum);
}

}  // namespace

SignalGroup signalGroup(const SignalKey & signal) noexcept
{
  return {signal.constellation, signal.signal_type, signal.l2c_signal};
}

double um982AdrToCarrierPhaseCycles(const double adr_cycles) noexcept
{
  return -adr_cycles;
}

bool GnssEpochAligner::EpochKey::operator<(const EpochKey & other) const noexcept
{
  return std::tie(receiver, week, milliseconds) <
         std::tie(other.receiver, other.week, other.milliseconds);
}

GnssEpochAligner::GnssEpochAligner(EpochAlignerConfig config)
: config_(config)
{
  if (!std::isfinite(config_.maximum_time_offset_s) ||
    config_.maximum_time_offset_s < 0.0 || config_.maximum_buffered_epochs == 0U ||
    config_.maximum_seen_epochs == 0U)
  {
    throw std::invalid_argument("GNSS epoch aligner configuration is outside valid bounds");
  }
}

bool GnssEpochAligner::remember(const GnssObservationEpoch & epoch)
{
  const EpochKey key{
    epoch.receiver, epoch.time.week,
    static_cast<std::int64_t>(std::llround(epoch.time.tow_s * 1000.0))};
  if (seen_.find(key) != seen_.end()) {
    return false;
  }
  seen_.insert(key);
  seen_order_.push_back(key);
  while (seen_order_.size() > config_.maximum_seen_epochs) {
    seen_.erase(seen_order_.front());
    seen_order_.pop_front();
  }
  return true;
}

void GnssEpochAligner::enforceCapacity(std::deque<GnssObservationEpoch> & epochs)
{
  while (epochs.size() > config_.maximum_buffered_epochs) {
    epochs.pop_front();
    ++diagnostics_.dropped_capacity;
  }
}

std::optional<AlignedGnssEpochs> GnssEpochAligner::push(const GnssObservationEpoch & epoch)
{
  if (!validEpoch(epoch)) {
    ++diagnostics_.invalid;
    return std::nullopt;
  }
  if (epoch.receiver != GnssReceiver::Master && epoch.receiver != GnssReceiver::Base) {
    ++diagnostics_.unsupported_receiver;
    return std::nullopt;
  }
  if (!remember(epoch)) {
    ++diagnostics_.duplicates;
    return std::nullopt;
  }
  const bool rover = epoch.receiver == GnssReceiver::Master;
  auto & destination = rover ? rover_epochs_ : base_epochs_;
  destination.push_back(epoch);
  enforceCapacity(destination);
  return matchNewest(rover);
}

std::optional<AlignedGnssEpochs> GnssEpochAligner::matchNewest(const bool newest_is_rover)
{
  auto & newest_queue = newest_is_rover ? rover_epochs_ : base_epochs_;
  auto & other_queue = newest_is_rover ? base_epochs_ : rover_epochs_;
  if (newest_queue.empty() || other_queue.empty()) {
    return std::nullopt;
  }
  const GnssObservationEpoch & newest = newest_queue.back();
  auto closest = other_queue.end();
  double closest_abs_delta = std::numeric_limits<double>::infinity();
  for (auto iterator = other_queue.begin(); iterator != other_queue.end(); ++iterator) {
    const double delta = std::abs(epochDelta(newest.time, iterator->time));
    if (delta < closest_abs_delta) {
      closest_abs_delta = delta;
      closest = iterator;
    }
  }
  if (closest == other_queue.end() || closest_abs_delta > config_.maximum_time_offset_s) {
    return std::nullopt;
  }

  AlignedGnssEpochs result;
  if (newest_is_rover) {
    result.rover = newest;
    result.base = *closest;
  } else {
    result.rover = *closest;
    result.base = newest;
  }
  result.time_offset_s = epochDelta(result.rover.time, result.base.time);
  newest_queue.pop_back();
  other_queue.erase(closest);
  ++diagnostics_.matched;
  return result;
}

GnssReferenceSelector::GnssReferenceSelector(ReferenceSelectorConfig config)
: config_(config)
{
  if (!std::isfinite(config_.minimum_elevation_rad) ||
    !std::isfinite(config_.minimum_cn0_db_hz) ||
    !std::isfinite(config_.switch_margin_rad) || config_.minimum_elevation_rad < -1.57 ||
    config_.minimum_elevation_rad > 1.57 || config_.switch_margin_rad < 0.0)
  {
    throw std::invalid_argument("GNSS reference selector configuration is outside valid bounds");
  }
}

std::map<SignalGroup, ReferenceSelection> GnssReferenceSelector::select(
  const std::vector<ReferenceCandidate> & candidates)
{
  std::map<SignalGroup, std::vector<ReferenceCandidate>> grouped;
  for (const auto & candidate : candidates) {
    if (std::isfinite(candidate.elevation_rad) && std::isfinite(candidate.cn0_db_hz) &&
      candidate.elevation_rad >= config_.minimum_elevation_rad &&
      candidate.cn0_db_hz >= config_.minimum_cn0_db_hz)
    {
      grouped[candidate.group].push_back(candidate);
    }
  }

  std::map<SignalGroup, ReferenceSelection> selections;
  for (auto & entry : grouped) {
    auto & group_candidates = entry.second;
    const auto best = std::max_element(
      group_candidates.begin(), group_candidates.end(),
      [](const ReferenceCandidate & left, const ReferenceCandidate & right) {
        return left.elevation_rad < right.elevation_rad;
      });
    if (best == group_candidates.end()) {
      continue;
    }
    auto previous = references_.find(entry.first);
    if (previous != references_.end()) {
      const auto current = std::find_if(
        group_candidates.begin(), group_candidates.end(),
        [&previous](const ReferenceCandidate & candidate) {
          return candidate.satellite == previous->second;
        });
      if (current != group_candidates.end() &&
        current->elevation_rad + config_.switch_margin_rad >= best->elevation_rad)
      {
        selections[entry.first] = {current->satellite, current->elevation_rad, false};
        continue;
      }
    }
    const bool switched = previous != references_.end() && !(previous->second == best->satellite);
    references_[entry.first] = best->satellite;
    selections[entry.first] = {best->satellite, best->elevation_rad, switched};
  }
  return selections;
}

bool AmbiguityArcManager::ArcKey::operator<(const ArcKey & other) const noexcept
{
  if (receiver != other.receiver) {
    return receiver < other.receiver;
  }
  if (satellite < other.satellite) {
    return true;
  }
  if (other.satellite < satellite) {
    return false;
  }
  return signal < other.signal;
}

AmbiguityArcManager::AmbiguityArcManager(AmbiguityArcConfig config)
: config_(config)
{
  if (!std::isfinite(config_.lock_time_tolerance_s) ||
    !std::isfinite(config_.maximum_observation_gap_s) ||
    !std::isfinite(config_.base_maximum_observation_gap_s) ||
    !std::isfinite(config_.doppler_phase_threshold_cycles) ||
    config_.lock_time_tolerance_s < 0.0 || config_.maximum_observation_gap_s <= 0.0 ||
    config_.base_maximum_observation_gap_s <= 0.0 ||
    config_.doppler_phase_threshold_cycles <= 0.0)
  {
    throw std::invalid_argument("ambiguity arc configuration is outside valid bounds");
  }
}

ArcUpdate AmbiguityArcManager::startArc(ArcState & state, const ArcResetReason reason)
{
  state.active = true;
  state.arc_id = next_arc_id_++;
  return {true, true, state.arc_id, reason};
}

ArcUpdate AmbiguityArcManager::update(
  const GnssReceiver receiver,
  const GnssTime & time,
  const GnssObservation & observation)
{
  const ArcKey key{receiver, observation.satellite, observation.signal};
  ArcState & state = states_[key];
  if (!validEpoch({receiver, time, {}}) || !finiteObservation(observation) ||
    !observation.carrier_phase_valid)
  {
    state.active = false;
    return {};
  }

  ArcResetReason reset = ArcResetReason::None;
  if (state.arc_id == 0U) {
    reset = ArcResetReason::FirstObservation;
  } else if (!state.active) {
    reset = ArcResetReason::Reacquired;
  } else {
    const double delta_s = epochDelta(time, state.time);
    if (delta_s <= 0.0) {
      reset = ArcResetReason::TimeReversal;
    } else if (delta_s > (receiver == GnssReceiver::Base ?
      config_.base_maximum_observation_gap_s : config_.maximum_observation_gap_s))
    {
      reset = ArcResetReason::ObservationGap;
    } else if (observation.lock_time_s + config_.lock_time_tolerance_s < state.lock_time_s) {
      reset = ArcResetReason::LockTimeReset;
    } else if (observation.channel_number != state.channel_number) {
      reset = ArcResetReason::TrackingChannelChanged;
    } else if (observation.doppler_valid && state.doppler_valid) {
      const double phase_prediction_error =
        observation.carrier_phase_cycles - state.carrier_phase_cycles +
        0.5 * (observation.doppler_hz + state.doppler_hz) * delta_s;
      if (std::abs(phase_prediction_error) > config_.doppler_phase_threshold_cycles) {
        reset = ArcResetReason::DopplerPhaseInconsistent;
      }
    }
  }

  ArcUpdate result;
  if (reset != ArcResetReason::None) {
    result = startArc(state, reset);
  } else {
    result = {true, false, state.arc_id, ArcResetReason::None};
  }
  state.time = time;
  state.carrier_phase_cycles = observation.carrier_phase_cycles;
  state.doppler_hz = observation.doppler_hz;
  state.doppler_valid = observation.doppler_valid;
  state.lock_time_s = observation.lock_time_s;
  state.channel_number = observation.channel_number;
  return result;
}

bool DdAmbiguityKey::operator==(const DdAmbiguityKey & other) const noexcept
{
  return group == other.group && reference == other.reference && target == other.target &&
         receiver_arc_ids == other.receiver_arc_ids;
}

bool DdAmbiguityKey::operator<(const DdAmbiguityKey & other) const noexcept
{
  if (group < other.group) {
    return true;
  }
  if (other.group < group) {
    return false;
  }
  if (reference < other.reference) {
    return true;
  }
  if (other.reference < reference) {
    return false;
  }
  if (target < other.target) {
    return true;
  }
  if (other.target < target) {
    return false;
  }
  return receiver_arc_ids < other.receiver_arc_ids;
}

DoubleDifferenceBuilder::DoubleDifferenceBuilder(
  DoubleDifferenceBuilderConfig config,
  ReferenceSelectorConfig reference_config,
  AmbiguityArcConfig arc_config)
: config_(config), reference_selector_(reference_config), arc_manager_(arc_config)
{
  if (!std::isfinite(config_.minimum_elevation_rad) ||
    !std::isfinite(config_.minimum_cn0_db_hz) ||
    !std::isfinite(config_.maximum_baseline_m) ||
    !std::isfinite(config_.maximum_code_innovation_m) ||
    !std::isfinite(config_.minimum_code_sigma_m) ||
    !std::isfinite(config_.minimum_carrier_sigma_m) ||
    !std::isfinite(config_.unavailable_base_code_sigma_m) ||
    !std::isfinite(config_.unavailable_base_carrier_sigma_m) ||
    config_.maximum_baseline_m <= 0.0 || config_.maximum_code_innovation_m <= 0.0 ||
    config_.minimum_code_sigma_m <= 0.0 || config_.minimum_carrier_sigma_m <= 0.0 ||
    config_.unavailable_base_code_sigma_m <= 0.0 ||
    config_.unavailable_base_carrier_sigma_m <= 0.0)
  {
    throw std::invalid_argument("double-difference builder configuration is outside valid bounds");
  }
}

void DoubleDifferenceBuilder::reject(const DdRejectReason reason)
{
  ++diagnostics_.rejected[static_cast<std::size_t>(reason)];
}

std::vector<DoubleDifferenceMeasurement> DoubleDifferenceBuilder::build(
  const AlignedGnssEpochs & epochs,
  const SatelliteStateMap & satellite_states,
  const EcefState & rover_state,
  const Vec3 & base_position_ecef_m,
  const Vec3 & lever_arm_body_m)
{
  std::vector<DoubleDifferenceMeasurement> result;
  if (!validEpoch(epochs.rover) || !validEpoch(epochs.base) ||
    epochs.rover.receiver != GnssReceiver::Master ||
    epochs.base.receiver != GnssReceiver::Base || !finite(rover_state.position_ecef_m) ||
    !finite(rover_state.orientation_ecef_body) || !finite(base_position_ecef_m) ||
    !finite(lever_arm_body_m))
  {
    reject(DdRejectReason::InvalidEpoch);
    return result;
  }
  const Vec3 rover_antenna = rover_state.position_ecef_m +
    rover_state.orientation_ecef_body.rotate(lever_arm_body_m);
  if (norm(rover_antenna - base_position_ecef_m) > config_.maximum_baseline_m) {
    reject(DdRejectReason::BaselineTooLong);
    return result;
  }
  ++diagnostics_.aligned_epochs;

  std::map<ObservationKey, const GnssObservation *> base_observations;
  for (const auto & observation : epochs.base.observations) {
    base_observations[{observation.satellite, observation.signal}] = &observation;
  }
  std::map<ObservationKey, CommonObservation> common;
  std::vector<ReferenceCandidate> candidates;
  for (const auto & rover_observation : epochs.rover.observations) {
    const ObservationKey key{rover_observation.satellite, rover_observation.signal};
    const auto base_iterator = base_observations.find(key);
    if (base_iterator == base_observations.end()) {
      reject(DdRejectReason::MissingBaseObservation);
      continue;
    }
    const GnssObservation & base_observation = *base_iterator->second;
    const auto wavelength = carrierWavelengthM(rover_observation.signal);
    if (!wavelength.has_value()) {
      reject(DdRejectReason::UnsupportedSignal);
      continue;
    }
    const auto satellite = satellite_states.find(rover_observation.satellite);
    if (satellite == satellite_states.end() ||
      !finite(satellite->second.rover.position_ecef_m) ||
      !finite(satellite->second.base.position_ecef_m))
    {
      reject(DdRejectReason::MissingSatelliteState);
      continue;
    }
    if (!finiteObservation(rover_observation) || !finiteObservation(base_observation) ||
      rover_observation.cn0_db_hz < config_.minimum_cn0_db_hz ||
      base_observation.cn0_db_hz < config_.minimum_cn0_db_hz)
    {
      reject(DdRejectReason::LowQuality);
      continue;
    }
    const Vec3 line_of_sight = satellite->second.rover.position_ecef_m - rover_antenna;
    const double line_norm = norm(line_of_sight);
    const double up_norm = norm(rover_antenna);
    if (line_norm < 1.0 || up_norm < 1.0) {
      reject(DdRejectReason::MissingSatelliteState);
      continue;
    }
    const double elevation = std::asin(
      std::clamp(
        dot(line_of_sight / line_norm, rover_antenna / up_norm), -1.0, 1.0));
    if (elevation < config_.minimum_elevation_rad) {
      reject(DdRejectReason::BelowElevationMask);
      continue;
    }

    CommonObservation value;
    value.rover = &rover_observation;
    value.base = &base_observation;
    value.satellite = &satellite->second;
    value.wavelength_m = *wavelength;
    value.elevation_rad = elevation;
    value.rover_arc = arc_manager_.update(
      GnssReceiver::Master, epochs.rover.time, rover_observation);
    value.base_arc = arc_manager_.update(
      GnssReceiver::Base, epochs.base.time, base_observation);
    diagnostics_.new_arcs += static_cast<std::uint64_t>(value.rover_arc.new_arc) +
      static_cast<std::uint64_t>(value.base_arc.new_arc);
    for (const ArcUpdate & update : {value.rover_arc, value.base_arc}) {
      if (update.new_arc && update.reason != ArcResetReason::None) {
        ++diagnostics_.arc_resets[static_cast<std::size_t>(update.reason)];
      }
    }
    common[key] = value;
    if (value.rover_arc.usable && value.base_arc.usable) {
      candidates.push_back(
        {
          rover_observation.satellite, signalGroup(rover_observation.signal), elevation,
          std::min(rover_observation.cn0_db_hz, base_observation.cn0_db_hz)});
    }
  }

  const auto references = reference_selector_.select(candidates);
  for (const auto & target_entry : common) {
    const ObservationKey & target_key = target_entry.first;
    const CommonObservation & target = target_entry.second;
    const SignalGroup group = signalGroup(target_key.signal);
    const auto reference_selection = references.find(group);
    if (reference_selection == references.end()) {
      reject(DdRejectReason::MissingReference);
      continue;
    }
    if (target_key.satellite == reference_selection->second.satellite) {
      continue;
    }
    const auto reference_iterator = std::find_if(
      common.begin(), common.end(),
      [&reference_selection, &group](const auto & entry) {
        return entry.first.satellite == reference_selection->second.satellite &&
        signalGroup(entry.first.signal) == group;
      });
    if (reference_iterator == common.end()) {
      reject(DdRejectReason::MissingReference);
      continue;
    }
    const ObservationKey & reference_key = reference_iterator->first;
    const CommonObservation & reference = reference_iterator->second;
    DoubleDifferenceMeasurement measurement;
    measurement.time = epochs.rover.time;
    measurement.group = group;
    measurement.reference = reference_key.satellite;
    measurement.target = target_key.satellite;
    measurement.target_position_ecef_m = target.satellite->rover.position_ecef_m;
    measurement.reference_position_ecef_m = reference.satellite->rover.position_ecef_m;
    measurement.target_base_position_ecef_m = target.satellite->base.position_ecef_m;
    measurement.reference_base_position_ecef_m = reference.satellite->base.position_ecef_m;
    measurement.base_position_ecef_m = base_position_ecef_m;
    measurement.lever_arm_body_m = lever_arm_body_m;
    measurement.target_elevation_rad = target.elevation_rad;
    measurement.reference_elevation_rad = reference.elevation_rad;
    measurement.reference_switched = reference_selection->second.switched;

    const double predicted = predictedDoubleDifference(
      measurement.target_position_ecef_m, measurement.target_base_position_ecef_m,
      measurement.reference_position_ecef_m, measurement.reference_base_position_ecef_m,
      rover_antenna, base_position_ecef_m);
    const bool code_inputs_valid = target.rover->pseudorange_valid &&
      target.base->pseudorange_valid && reference.rover->pseudorange_valid &&
      reference.base->pseudorange_valid;
    if (code_inputs_valid) {
      measurement.code_dd_m =
        target.rover->pseudorange_m - target.base->pseudorange_m -
        reference.rover->pseudorange_m + reference.base->pseudorange_m;
      const double target_rover_sigma = usableSigma(
        target.rover->pseudorange_std_m, config_.minimum_code_sigma_m,
        config_.minimum_code_sigma_m);
      const double target_base_sigma = usableSigma(
        target.base->pseudorange_std_m, config_.minimum_code_sigma_m,
        config_.unavailable_base_code_sigma_m);
      const double reference_rover_sigma = usableSigma(
        reference.rover->pseudorange_std_m, config_.minimum_code_sigma_m,
        config_.minimum_code_sigma_m);
      const double reference_base_sigma = usableSigma(
        reference.base->pseudorange_std_m, config_.minimum_code_sigma_m,
        config_.unavailable_base_code_sigma_m);
      measurement.code_target_variance_m2 = target_rover_sigma * target_rover_sigma +
        target_base_sigma * target_base_sigma;
      measurement.code_reference_variance_m2 =
        reference_rover_sigma * reference_rover_sigma +
        reference_base_sigma * reference_base_sigma;
      measurement.code_sigma_m = std::sqrt(
        measurement.code_target_variance_m2 + measurement.code_reference_variance_m2);
      measurement.code_valid = std::isfinite(measurement.code_dd_m) &&
        std::abs(measurement.code_dd_m - predicted) <= config_.maximum_code_innovation_m;
      if (!measurement.code_valid) {
        reject(DdRejectReason::CodeInnovation);
      } else {
        ++diagnostics_.code_factors;
      }
    }

    measurement.carrier_valid = target.rover_arc.usable && target.base_arc.usable &&
      reference.rover_arc.usable && reference.base_arc.usable;
    if (measurement.carrier_valid) {
      measurement.carrier_dd_m =
        target.wavelength_m *
        (target.rover->carrier_phase_cycles - target.base->carrier_phase_cycles) -
        reference.wavelength_m *
        (reference.rover->carrier_phase_cycles - reference.base->carrier_phase_cycles);
      const double target_rover_sigma = usableSigma(
        target.wavelength_m * target.rover->carrier_phase_std_cycles,
        config_.minimum_carrier_sigma_m, config_.minimum_carrier_sigma_m);
      const double target_base_sigma = usableSigma(
        target.wavelength_m * target.base->carrier_phase_std_cycles,
        config_.minimum_carrier_sigma_m, config_.unavailable_base_carrier_sigma_m);
      const double reference_rover_sigma = usableSigma(
        reference.wavelength_m * reference.rover->carrier_phase_std_cycles,
        config_.minimum_carrier_sigma_m, config_.minimum_carrier_sigma_m);
      const double reference_base_sigma = usableSigma(
        reference.wavelength_m * reference.base->carrier_phase_std_cycles,
        config_.minimum_carrier_sigma_m, config_.unavailable_base_carrier_sigma_m);
      measurement.carrier_target_variance_m2 = target_rover_sigma * target_rover_sigma +
        target_base_sigma * target_base_sigma;
      measurement.carrier_reference_variance_m2 =
        reference_rover_sigma * reference_rover_sigma +
        reference_base_sigma * reference_base_sigma;
      measurement.carrier_sigma_m = std::sqrt(
        measurement.carrier_target_variance_m2 +
        measurement.carrier_reference_variance_m2);
      measurement.ambiguity_key = {
        group, measurement.reference, measurement.target,
        {target.rover_arc.arc_id, target.base_arc.arc_id,
          reference.rover_arc.arc_id, reference.base_arc.arc_id}};
      measurement.carrier_valid = std::isfinite(measurement.carrier_dd_m) &&
        std::isfinite(measurement.carrier_sigma_m);
      if (measurement.carrier_valid) {
        ++diagnostics_.carrier_factors;
      }
    }
    if (measurement.code_valid || measurement.carrier_valid) {
      result.push_back(measurement);
    }
  }
  return result;
}

std::optional<DdFactorEvaluation> evaluateDdPseudorange(
  const DoubleDifferenceMeasurement & measurement,
  const EcefState & rover_state)
{
  if (!measurement.code_valid || !finite(rover_state.position_ecef_m) ||
    !finite(rover_state.orientation_ecef_body))
  {
    return std::nullopt;
  }
  const Vec3 lever_ecef = rover_state.orientation_ecef_body.rotate(measurement.lever_arm_body_m);
  const Vec3 rover_antenna = rover_state.position_ecef_m + lever_ecef;
  const auto target_gradient = rangeGradient(measurement.target_position_ecef_m, rover_antenna);
  const auto reference_gradient = rangeGradient(
    measurement.reference_position_ecef_m, rover_antenna);
  if (!target_gradient.has_value() || !reference_gradient.has_value()) {
    return std::nullopt;
  }
  const Vec3 position_gradient = *target_gradient - *reference_gradient;
  const Vec3 rotation_gradient = cross(lever_ecef, position_gradient);
  const double predicted = predictedDoubleDifference(
    measurement.target_position_ecef_m, measurement.target_base_position_ecef_m,
    measurement.reference_position_ecef_m, measurement.reference_base_position_ecef_m,
    rover_antenna, measurement.base_position_ecef_m);
  DdFactorEvaluation evaluation;
  evaluation.residual_m = predicted - measurement.code_dd_m;
  evaluation.pose_jacobian = {
    position_gradient.x, position_gradient.y, position_gradient.z,
    rotation_gradient.x, rotation_gradient.y, rotation_gradient.z};
  return evaluation;
}

std::optional<DdFactorEvaluation> evaluateDdCarrier(
  const DoubleDifferenceMeasurement & measurement,
  const EcefState & rover_state,
  const double ambiguity_m)
{
  if (!measurement.carrier_valid || !std::isfinite(ambiguity_m)) {
    return std::nullopt;
  }
  DoubleDifferenceMeasurement code_measurement = measurement;
  code_measurement.code_valid = true;
  code_measurement.code_dd_m = measurement.carrier_dd_m;
  auto evaluation = evaluateDdPseudorange(code_measurement, rover_state);
  if (!evaluation.has_value()) {
    return std::nullopt;
  }
  evaluation->residual_m += ambiguity_m;
  evaluation->ambiguity_jacobian = 1.0;
  return evaluation;
}

}  // namespace fgo_gil_localizer

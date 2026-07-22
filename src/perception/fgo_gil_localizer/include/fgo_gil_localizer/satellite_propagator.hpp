#pragma once

#include <cstdint>
#include <optional>

#include "fgo_gil_localizer/gnss_types.hpp"

namespace fgo_gil_localizer
{

enum class SatellitePropagationError : std::uint8_t
{
  None,
  InvalidTime,
  InvalidEphemeris,
  UnhealthySatellite,
  EphemerisTooOld,
  UnsupportedConstellation,
  UnsupportedSignal,
  KeplerDidNotConverge,
  NumericalFailure,
};

struct SatellitePropagationConfig
{
  double maximum_kepler_age_s = 14400.0;
  double maximum_glonass_age_s = 1800.0;
  double glonass_integration_step_s = 30.0;
  bool reject_unhealthy = true;
};

struct SatellitePropagationResult
{
  SatellitePropagationError error = SatellitePropagationError::None;
  std::optional<SatelliteState> state;

  bool ok() const noexcept {return state.has_value();}
};

std::optional<double> carrierFrequencyHz(const SignalKey & signal) noexcept;
std::optional<double> carrierWavelengthM(const SignalKey & signal) noexcept;

class SatellitePropagator
{
public:
  explicit SatellitePropagator(SatellitePropagationConfig config = {});

  SatellitePropagationResult propagate(
    const BroadcastEphemeris & ephemeris,
    const GnssTime & query_time) const;

  SatellitePropagationResult propagateToReceiveFrame(
    const BroadcastEphemeris & ephemeris,
    const GnssTime & receive_time,
    double pseudorange_m) const;

private:
  SatellitePropagationResult propagateKeplerian(
    const BroadcastEphemeris & ephemeris,
    const GnssTime & query_time) const;
  SatellitePropagationResult propagateGlonass(
    const BroadcastEphemeris & ephemeris,
    const GnssTime & query_time) const;

  SatellitePropagationConfig config_;
};

double wrappedWeekDelta(
  const GnssTime & time, std::uint32_t reference_week,
  double reference_tow_s);
GnssTime addGnssSeconds(const GnssTime & time, double delta_s);

}  // namespace fgo_gil_localizer

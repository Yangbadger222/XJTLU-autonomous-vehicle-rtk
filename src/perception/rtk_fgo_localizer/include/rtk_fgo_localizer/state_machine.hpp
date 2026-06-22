#pragma once

#include "rtk_fgo_localizer/rtk_quality.hpp"

#include <cstddef>
#include <string>

namespace rtk_fgo_localizer
{

enum class LocalizationState
{
  LocalOnly,
  RtkCandidate,
  RtkLocked,
  RtkDegraded,
  RtkRecovery,
  FaultHold,
};

std::string toString(LocalizationState state);

class LocalizationStateMachine
{
public:
  explicit LocalizationStateMachine(
    std::size_t recovery_min_samples = 8,
    std::size_t degraded_to_local_samples = 3);

  LocalizationState state() const;
  std::size_t strong_sample_count() const;
  std::size_t invalid_sample_count() const;

  LocalizationState update(RtkGateMode gate_mode, bool graph_residual_ok);
  void markRecoveryCommitted();
  void markRecoveryRejected();
  void resetFault();

private:
  void acceptStrongCandidate();
  void acceptWeakOrDiagnostic();
  void acceptRejected();

  LocalizationState state_ = LocalizationState::LocalOnly;
  std::size_t recovery_min_samples_ = 8;
  std::size_t degraded_to_local_samples_ = 3;
  std::size_t strong_sample_count_ = 0;
  std::size_t invalid_sample_count_ = 0;
};

}  // namespace rtk_fgo_localizer

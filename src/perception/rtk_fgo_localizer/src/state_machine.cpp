#include "rtk_fgo_localizer/state_machine.hpp"

namespace rtk_fgo_localizer
{

std::string toString(LocalizationState state)
{
  switch (state) {
    case LocalizationState::LocalOnly:
      return "LOCAL_ONLY";
    case LocalizationState::RtkCandidate:
      return "RTK_CANDIDATE";
    case LocalizationState::RtkLocked:
      return "RTK_LOCKED";
    case LocalizationState::RtkDegraded:
      return "RTK_DEGRADED";
    case LocalizationState::RtkRecovery:
      return "RTK_RECOVERY";
    case LocalizationState::FaultHold:
      return "FAULT_HOLD";
  }
  return "UNKNOWN";
}

LocalizationStateMachine::LocalizationStateMachine(
  std::size_t recovery_min_samples,
  std::size_t degraded_to_local_samples)
: recovery_min_samples_(recovery_min_samples),
  degraded_to_local_samples_(degraded_to_local_samples)
{
}

LocalizationState LocalizationStateMachine::state() const
{
  return state_;
}

std::size_t LocalizationStateMachine::strong_sample_count() const
{
  return strong_sample_count_;
}

std::size_t LocalizationStateMachine::invalid_sample_count() const
{
  return invalid_sample_count_;
}

LocalizationState LocalizationStateMachine::update(RtkGateMode gate_mode, bool graph_residual_ok)
{
  if (state_ == LocalizationState::FaultHold) {
    return state_;
  }
  if (!graph_residual_ok && gate_mode == RtkGateMode::StrongCandidate) {
    state_ = LocalizationState::FaultHold;
    strong_sample_count_ = 0;
    return state_;
  }

  switch (gate_mode) {
    case RtkGateMode::StrongCandidate:
      acceptStrongCandidate();
      break;
    case RtkGateMode::WeakCandidate:
    case RtkGateMode::DiagnosticOnly:
      acceptWeakOrDiagnostic();
      break;
    case RtkGateMode::Rejected:
      acceptRejected();
      break;
  }
  return state_;
}

void LocalizationStateMachine::markRecoveryCommitted()
{
  if (state_ == LocalizationState::RtkRecovery) {
    state_ = LocalizationState::RtkLocked;
    strong_sample_count_ = recovery_min_samples_;
    invalid_sample_count_ = 0;
  }
}

void LocalizationStateMachine::markRecoveryRejected()
{
  if (state_ == LocalizationState::RtkLocked) {
    state_ = LocalizationState::RtkDegraded;
  } else if (state_ == LocalizationState::RtkCandidate ||
    state_ == LocalizationState::RtkRecovery)
  {
    state_ = LocalizationState::LocalOnly;
  }
  strong_sample_count_ = 0;
  invalid_sample_count_ = 0;
}

void LocalizationStateMachine::resetFault()
{
  if (state_ == LocalizationState::FaultHold) {
    state_ = LocalizationState::LocalOnly;
    invalid_sample_count_ = 0;
    strong_sample_count_ = 0;
  }
}

void LocalizationStateMachine::acceptStrongCandidate()
{
  ++strong_sample_count_;
  invalid_sample_count_ = 0;

  if (state_ == LocalizationState::LocalOnly || state_ == LocalizationState::RtkDegraded) {
    state_ = LocalizationState::RtkCandidate;
  }
  if (state_ == LocalizationState::RtkCandidate &&
    strong_sample_count_ >= recovery_min_samples_)
  {
    state_ = LocalizationState::RtkRecovery;
  }
}

void LocalizationStateMachine::acceptWeakOrDiagnostic()
{
  strong_sample_count_ = 0;
  if (state_ == LocalizationState::RtkLocked) {
    state_ = LocalizationState::RtkDegraded;
  } else if (state_ == LocalizationState::RtkCandidate ||
    state_ == LocalizationState::RtkRecovery)
  {
    state_ = LocalizationState::LocalOnly;
  }
}

void LocalizationStateMachine::acceptRejected()
{
  strong_sample_count_ = 0;
  ++invalid_sample_count_;

  if (state_ == LocalizationState::RtkLocked) {
    state_ = LocalizationState::RtkDegraded;
  } else if (state_ == LocalizationState::RtkCandidate ||
    state_ == LocalizationState::RtkRecovery)
  {
    state_ = LocalizationState::LocalOnly;
  } else if (state_ == LocalizationState::RtkDegraded &&
    invalid_sample_count_ >= degraded_to_local_samples_)
  {
    state_ = LocalizationState::LocalOnly;
  }
}

}  // namespace rtk_fgo_localizer

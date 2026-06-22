#include <gtest/gtest.h>

#include "rtk_fgo_localizer/state_machine.hpp"

namespace rtk_fgo_localizer
{
namespace
{

TEST(StateMachine, StartsLocalOnly)
{
  LocalizationStateMachine sm;
  EXPECT_EQ(sm.state(), LocalizationState::LocalOnly);
}

TEST(StateMachine, RequiresPersistentFixedBeforeRecovery)
{
  LocalizationStateMachine sm;
  for (int i = 0; i < 7; ++i) {
    sm.update(RtkGateMode::StrongCandidate, true);
  }
  EXPECT_EQ(sm.state(), LocalizationState::RtkCandidate);

  sm.update(RtkGateMode::StrongCandidate, true);
  EXPECT_EQ(sm.state(), LocalizationState::RtkRecovery);
}

TEST(StateMachine, DegradesWhenLockedRtkDrops)
{
  LocalizationStateMachine sm;
  for (int i = 0; i < 9; ++i) {
    sm.update(RtkGateMode::StrongCandidate, true);
  }
  sm.markRecoveryCommitted();
  EXPECT_EQ(sm.state(), LocalizationState::RtkLocked);

  sm.update(RtkGateMode::Rejected, false);
  EXPECT_EQ(sm.state(), LocalizationState::RtkDegraded);
}

TEST(StateMachine, FaultsWhenStrongCandidateFailsGraphResidual)
{
  LocalizationStateMachine sm;
  sm.update(RtkGateMode::StrongCandidate, false);
  EXPECT_EQ(sm.state(), LocalizationState::FaultHold);
}

}  // namespace
}  // namespace rtk_fgo_localizer

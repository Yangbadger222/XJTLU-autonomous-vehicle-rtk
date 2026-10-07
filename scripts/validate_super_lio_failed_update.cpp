#include "lio/ESKF.h"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
  LI2Sup::ESKF filter;
  filter.SetObservationSamples(100);
  filter.SetObsTime(1.);
  const auto prior = filter.GetCov();
  const bool invalid = filter.UpdateObserve([](const LI2Sup::ESKF::KFState&, BASIC::M6& h, BASIC::V6& b) {
    h = BASIC::M6::Identity()*1000.;
    b.setZero();
    b[0] = std::numeric_limits<BASIC::scalar>::quiet_NaN();
  });
  assert(!invalid);
  assert(!filter.GetObservationQuality().valid && !filter.GetObservationQuality().eligible);
  assert(filter.GetObservationQuality().reason == "nonfinite_observation_residual");
  assert((filter.GetCov()-prior).norm()==0.);
  const bool valid = filter.UpdateObserve([](const LI2Sup::ESKF::KFState&, BASIC::M6& h, BASIC::V6& b) {
    h = BASIC::M6::Identity()*1000.; b.setZero();
  });
  assert(valid && filter.GetObservationQuality().eligible);
  std::cout << "PASS: actual ESKF invalid-residual rollback clears eligibility,preserves covariance and recovers on a valid measured update\n";
}

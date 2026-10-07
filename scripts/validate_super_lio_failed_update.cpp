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
  LI2Sup::ESKF propagated;
  propagated.init_=true;
  propagated.SetInitialConditions(LI2Sup::ESKF::Options(),BASIC::V3::Zero(),BASIC::V3::Zero());
  LI2Sup::IMUData imu;imu.secs=1.;imu.gyr=BASIC::V3(.2,0,0);imu.acc=BASIC::V3(0,0,9.8);
  propagated.SetObsTime(1.1);
  assert(!propagated.Predict(imu));
  LI2Sup::DynamicState forward,robot;
  assert(!propagated.Predict(imu,forward,robot));
  imu.secs=1.05;
  assert(propagated.Predict(imu));
  assert(propagated.Predict(imu,forward,robot));
  auto cross_prior=LI2Sup::ESKF::COV::Identity().eval();
  cross_prior(0,9)=cross_prior(9,0)=.2;
  propagated.SetCov(cross_prior);
  propagated.SetObservationSamples(100);
  LI2Sup::ESKF control=propagated;
  const auto before=propagated.GetDynamicState();
  int iteration=0;
  assert(!propagated.UpdateObserve([&](const LI2Sup::ESKF::KFState&,BASIC::M6& h,BASIC::V6& b) {
    h=BASIC::M6::Identity()*1000.;b.setZero();b[0]=iteration++ ?
      std::numeric_limits<BASIC::scalar>::quiet_NaN() : .5;
  }));
  const auto after=propagated.GetDynamicState();
  assert((before.R-after.R).norm()<1e-7 && (before.p-after.p).norm()<1e-7 &&
         (before.v-after.v).norm()<1e-7 && (before.w-after.w).norm()<1e-7 && (before.a-after.a).norm()<1e-7);
  imu.secs=1.08;
  LI2Sup::DynamicState control_forward,control_robot;
  assert(propagated.Predict(imu,forward,robot));
  assert(control.Predict(imu,control_forward,control_robot));
  assert((forward.R-control_forward.R).norm()<1e-7 && (forward.p-control_forward.p).norm()<1e-7 &&
         (forward.v-control_forward.v).norm()<1e-7 && (forward.w-control_forward.w).norm()<1e-7);
  assert(propagated.UpdateObserve([](const LI2Sup::ESKF::KFState&,BASIC::M6& h,BASIC::V6& b) {
    h=BASIC::M6::Identity()*1000.;b.setZero();b[0]=.5;
  }));
  const auto posterior=propagated.GetSysState();
  assert(std::abs(posterior.bg[0])>1e-6);
  assert(std::abs(propagated.GetDynamicState().w[0]-(.2-posterior.bg[0]))<1e-6);
  std::cout << "PASS: actual ESKF invalid-residual eligibility/covariance rollback,forward-cache rollback and posterior-bias angular-rate consistency\n";
}

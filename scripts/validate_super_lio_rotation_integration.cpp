// Actual native ESKF prediction with ideal synthetic IMU, not physical calibration.
#include "lio/ESKF.h"
#include <cassert>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cmath>
int main() {
  LI2Sup::ESKF filter;
  filter.SetInitialConditions(LI2Sup::ESKF::Options(),BASIC::V3::Zero(),BASIC::V3::Zero());
  filter.init_=true;
  LI2Sup::IMUData imu;imu.secs=1.;imu.gyr=BASIC::V3(.023,.045,.087);imu.acc=BASIC::V3(0,0,9.8);
  filter.SetObsTime(imu.secs+.01);assert(!filter.Predict(imu));
  LI2Sup::DynamicState forward,robot;assert(!filter.Predict(imu,forward,robot));
  const double tolerance=std::sqrt(std::numeric_limits<float>::epsilon());
  double max_gram=0.,max_det=0.;
  for(int i=1;i<=60000;++i) {
    imu.secs=1.+i*.005;filter.SetObsTime(imu.secs+.01);
    assert(filter.Predict(imu));assert(filter.Predict(imu,forward,robot));
    const auto R=filter.GetNavState().R.R_.cast<double>().eval();
    const double gram=(R.transpose()*R-Eigen::Matrix3d::Identity()).norm(),det=std::abs(R.determinant()-1.);
    max_gram=std::max(max_gram,gram);max_det=std::max(max_det,det);
    if(i%10000==0||gram>tolerance||det>tolerance) {
      std::cout<<std::setprecision(17)<<"step="<<i<<" gram="<<gram<<" determinant_error="<<det<<" tolerance="<<tolerance<<std::endl;
    }
    assert(gram<=tolerance && det<=tolerance);
    const auto F=forward.R.cast<double>().eval();
    assert((F.transpose()*F-Eigen::Matrix3d::Identity()).norm()<=tolerance);
  }
  const auto final=filter.GetNavState().R.R_.cast<double>().eval();
  const Eigen::Vector3d omega=imu.gyr.cast<double>();
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(omega.norm()*300.,omega.normalized()));
  assert(Eigen::Quaterniond(final).angularDistance(expected)<1e-3);
  for(int mode=0;mode<2;++mode) {
    BASIC::SO3 rotation;
    const BASIC::V3 increment=imu.gyr*.005f;
    for(int i=0;i<10000;++i) {
      if(mode==0)rotation.update(increment);else rotation.updateRhs(increment);
    }
    const auto R=rotation.R_.cast<double>().eval();
    assert((R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<=tolerance);
    const Eigen::Quaterniond target(Eigen::AngleAxisd(omega.norm()*50.,omega.normalized()));
    assert(Eigen::Quaterniond(R).angularDistance(target)<1e-3);
  }
  for(int fault=0;fault<4;++fault) {
    BASIC::SO3 bad;
    if(fault==0)bad.R_.setZero();
    if(fault==1)bad.R_(0,0)=std::numeric_limits<float>::quiet_NaN();
    if(fault==2)bad.R_(0,0)=-1.f;
    if(fault==3)bad.R_(0,1)=.001f;
    assert(!(bad*BASIC::SO3()).R_.allFinite());
    assert(!(bad*bad).R_.allFinite()); // reflections must not cancel into valid identity.
  }
  std::cout<<"PASS: actual ESKF 60000 predictions and forward cache stay rigid; max gram="<<max_gram<<" max determinant="<<max_det<<"\n";
}

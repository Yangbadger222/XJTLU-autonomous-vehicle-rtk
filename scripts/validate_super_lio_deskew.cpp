// Run the actual native deskew entry on boundary/corrupt measurement groups.
// Synthetic points and IMU values test software math, not physical calibration.
#include "lio/super_lio.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

class Probe : public LI2Sup::SuperLIO {
public:
  Probe() {
    kf_ = std::make_shared<LI2Sup::ESKF>();
    kf_->SetInitialConditions(LI2Sup::ESKF::Options(),BASIC::V3::Zero(),BASIC::V3::Zero());
    kf_->init_ = true;
    LI2Sup::IMUData imu;imu.secs=1.;imu.acc=BASIC::V3(0,0,9.8);
    kf_->SetObsTime(1.01);kf_->Predict(imu);imu.secs=1.01;kf_->Predict(imu);
    kf_->SetLastObsTime(1.01);
    scan_undistort_full_.reset(new BASIC::PointCloudType());
  }
  bool run(const LI2Sup::MeasureGroup& input) { measures_=input;return Propagation_Undistort(); }
  double time() const { return kf_->GetTime(); }
  auto covariance() const { return kf_->GetCov(); }
  auto quality() const { return kf_->GetObservationQuality(); }
  auto output() const { return scan_undistort_full_; }
};

LI2Sup::MeasureGroup fixture() {
  LI2Sup::MeasureGroup input;
  input.lidar.start_time=1.01;input.lidar.end_time=1.04;
  input.lidar.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  for(double offset : {0.,.01,.03})input.lidar.pc->emplace_back(1.,.5,.2,10.,offset);
  for(double time : {1.015,1.025,1.035}) {
    LI2Sup::IMUData imu;imu.secs=time;imu.acc=BASIC::V3(0,0,9.8);input.imu.push_back(imu);
  }
  auto after=input.imu.back();after.secs=1.045;after.gyr=BASIC::V3(0,0,.2);input.imu_after_scan=after;
  return input;
}

int main() {
  for(int size=0;size<3;++size) {
    Probe probe;auto input=fixture();input.imu.resize(size);const auto cov=probe.covariance();
    assert(!probe.run(input));assert(probe.time()==1.01 && (probe.covariance()-cov).norm()==0);
    assert(probe.quality().reason=="insufficient_synchronized_imu_samples");
  }
  for(int fault=0;fault<6;++fault) {
    Probe probe;auto input=fixture();const auto cov=probe.covariance();
    if(fault==0)input.lidar.end_time=1.;
    if(fault==1)input.lidar.start_time=.9;
    if(fault==2)input.imu[1].secs=input.imu[0].secs;
    if(fault==3)input.imu[1].gyr[0]=std::numeric_limits<BASIC::scalar>::quiet_NaN();
    if(fault==4)input.imu_after_scan.reset();
    if(fault==5)input.imu_after_scan->secs=1.039;
    assert(!probe.run(input));assert(probe.time()==1.01 && (probe.covariance()-cov).norm()==0);
    assert(!probe.quality().valid && !probe.quality().eligible);
  }
  Probe valid;assert(valid.run(fixture()));assert(valid.time()==1.04);
  assert(valid.output()->size()==3);
  for(const auto& point: valid.output()->points) assert(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z));
  auto second=fixture();second.lidar.start_time=1.04;second.lidar.end_time=1.07;
  // The real lookahead is retained for the following scan, not consumed twice
  // into the first scan or used as a future measurement timestamp.
  for(auto& imu:second.imu)imu.secs+=.03;
  second.imu.front()=*fixture().imu_after_scan;
  second.imu_after_scan->secs+=.03;
  assert(valid.run(second));assert(valid.time()==1.07);
  std::cout << "PASS: actual native deskew rejects 0/1/2 IMUs,backward/overlap/duplicate/nonfinite/unbracketed times before mutation; exact endpoints and reused real lookahead produce finite scan-end output\n";
}

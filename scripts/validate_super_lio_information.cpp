// Independent equation, degeneracy and covariance probes linked to the actual patch.
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <cassert>
#include <iostream>
#include <limits>
#include "lio/observation_quality.h"

int main() {
  using Matrix6 = Eigen::Matrix<double,6,6>;
  using Matrix12 = Eigen::Matrix<double,12,12>;
  // Three orthogonal planes with spread-out points make six pose directions observable.
  Matrix6 information = Matrix6::Zero();
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for (int axis=0;axis<3;++axis) for(int a=-3;a<=3;++a) for(int b=-3;b<=3;++b) {
    Eigen::Vector3d point(a*.4,b*.3,1.2), normal=Eigen::Vector3d::Unit(axis);
    Eigen::Matrix<double,1,6> legacy;
    Eigen::Matrix3d skew;
    skew << 0,-point.z(),point.y(),point.z(),0,-point.x(),-point.y(),point.x(),0;
    legacy.head<3>()=-normal.transpose()*rotation*skew;
    legacy.tail<3>()=normal.transpose();
    Eigen::Matrix<double,6,1> native;
    native.head<3>()=point.cross(rotation.transpose()*normal);
    native.tail<3>()=normal;
    assert((legacy.transpose()-native).norm()<1e-10);
    information += native*1000.*native.transpose();
  }
  const auto good=LI2Sup::assessObservation(information,147);
  assert(good.valid && good.eligible);
  // For unrelated PSD priors, the old 12-block always dominates the conservative bound.
  for(int seed=0;seed<100;++seed) {
    Matrix6 factor=Matrix6::Zero();
    for(int r=0;r<6;++r) for(int c=0;c<6;++c) factor(r,c)=std::sin(seed+r*13+c*7);
    Matrix12 old=Matrix12::Zero();
    old.block<6,6>(0,0)=information+factor*factor.transpose();
    old.block<6,6>(6,6)=Matrix6::Identity()*100000.;
    assert(Eigen::SelfAdjointEigenSolver<Matrix12>(old).eigenvalues().minCoeff()+1e-8>=good.legacy_lower_bound);
  }
  // A single horizontal plane leaves translation/yaw directions unobservable even with many points.
  Matrix6 plane=Matrix6::Zero();
  for(int a=-5;a<=5;++a)for(int b=-5;b<=5;++b) {
    Eigen::Matrix<double,6,1> j;j << b*.3,-a*.3,0,0,0,1;
    plane += 1000.*j*j.transpose();
  }
  assert(LI2Sup::assessObservation(plane,121).valid);
  assert(!LI2Sup::assessObservation(plane,121).eligible);
  assert(!LI2Sup::assessObservation(information,49).valid);
  Matrix6 bad=information; bad(0,0)=std::numeric_limits<double>::quiet_NaN();
  assert(!LI2Sup::assessObservation(bad,147).valid);
  auto worst=good; LI2Sup::accumulateObservationQuality(worst,LI2Sup::assessObservation(plane,121));
  assert(!worst.eligible && worst.iterations==2);
  Eigen::Matrix<double,18,18> p=Eigen::Matrix<double,18,18>::Identity();
  p(0,3)=p(3,0)=.2; p(6,9)=p(9,6)=.3;
  const auto quarter_turn=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto pose=LI2Sup::poseCovariance(p,quarter_turn);
  const auto twist=LI2Sup::twistCovariance(p,Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),.01);
  assert(std::abs(pose(0,4)-.2)<1e-10 && std::abs(pose(0,3))<1e-10);
  assert(std::abs(twist(0,3)+.3)<1e-10 && std::abs(twist(3,3)-1.01)<1e-10);
  assert(Eigen::SelfAdjointEigenSolver<Matrix6>(pose).eigenvalues().minCoeff()>0);
  assert(Eigen::SelfAdjointEigenSolver<Matrix6>(twist).eigenvalues().minCoeff()>0);
  // Review counterexample: body velocity depends on estimated attitude even
  // with identity extrinsics. The earlier mixed 6x6 rotation omitted .85^2.
  Eigen::Matrix<double,18,18> attitude=Eigen::Matrix<double,18,18>::Identity();
  attitude.block<3,3>(6,6)=1e-6*Eigen::Matrix3d::Identity();
  auto body=LI2Sup::twistCovariance(attitude,Eigen::Matrix3d::Identity(),Eigen::Vector3d(.85,0,0),.01);
  assert(std::abs(body(1,1)-.722501)<1e-10);
  // Independent central differences through exp(right-error), v and gyro
  // bias exercise all theta/v/bg columns and their signed cross blocks.
  Eigen::Matrix<double,6,18> numerical=Eigen::Matrix<double,6,18>::Zero();
  Eigen::Vector3d velocity(.85,.13,-.07);
  const double epsilon=1e-6;
  auto value=[&](const Eigen::Matrix<double,18,1>& error) {
    Eigen::Vector3d angle=error.head<3>();
    const Eigen::Matrix3d perturbed=rotation*(angle.norm()>0 ?
        Eigen::AngleAxisd(angle.norm(),angle.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity());
    Eigen::Matrix<double,6,1> result;
    result.head<3>()=perturbed.transpose()*(velocity+error.segment<3>(6));
    result.tail<3>()=-error.segment<3>(9);
    return result;
  };
  for(int column=0;column<18;++column) {
    Eigen::Matrix<double,18,1> error=Eigen::Matrix<double,18,1>::Zero();error[column]=epsilon;
    numerical.col(column)=(value(error)-value(-error))/(2.*epsilon);
  }
  Eigen::Matrix<double,18,18> factor;
  for(int r=0;r<18;++r)for(int c=0;c<18;++c)factor(r,c)=std::sin(r*13+c*7+.2);
  auto joint=(factor*factor.transpose()+.01*Eigen::Matrix<double,18,18>::Identity()).eval();
  auto expected=(numerical*joint*numerical.transpose()).eval();
  expected.block<3,3>(3,3)+=.01*Eigen::Matrix3d::Identity();
  assert((LI2Sup::twistCovariance(joint,rotation,velocity,.01)-expected).norm()<1e-7);
  std::cout << "PASS: actual Jacobian,100 PSD-prior bounds,plane degeneration,insufficient/nonfinite features,worst iteration,full pose/body-twist covariance and independent attitude/cross-block finite differences\n";
}

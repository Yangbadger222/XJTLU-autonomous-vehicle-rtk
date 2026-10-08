#include <Eigen/Geometry>
#include <iostream>
int main() {
 Eigen::Vector3f position=Eigen::Vector3f::Zero();
 Eigen::Quaterniond q(Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ()));
 const auto expected=q.toRotationMatrix()*Eigen::Vector3d(2.,.5,.2)+position.cast<double>();
 std::cout<<expected.norm()<<"\n";
}

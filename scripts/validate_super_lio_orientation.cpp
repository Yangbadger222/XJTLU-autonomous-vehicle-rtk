// Actual installed ROSWrapper::pub_odom boundary; no estimator/driver/actuator.
// Small SO(3) storage drift must not become a non-unit ROS quaternion.
#include "lio/super_lio.h"
#include <tf2_msgs/msg/tf_message.hpp>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <limits>
#include <thread>
#include <vector>
// Protected fixture seam; calls the actual Output(), no production test hook.
class OutputProbe : public LI2Sup::SuperLIO {
public:
 OutputProbe(const LI2Sup::ROSWrapper::Ptr& wrapper,const LI2Sup::ESKF::Ptr& filter) {
   data_wrapper_=wrapper;kf_=filter;
   scan_undistort_full_=std::make_shared<BASIC::PointCloudType>();
   BASIC::PointType point;point.x=2.f;point.y=.5f;point.z=.2f;
   scan_undistort_full_->push_back(point);ds_undistort_=scan_undistort_full_;
 }
 void run(const LI2Sup::NavState& state) {
   kf_->SetX(LI2Sup::SysState(state.timestamp,state.R,state.p,state.v));Output();
 }
};
int main(int argc,char**argv) {
 const char*d=std::getenv("ROS_DOMAIN_ID"),*l=std::getenv("ROS_LOCALHOST_ONLY");
 if(!d||!l||std::strcmp(d,"105")||std::strcmp(l,"1"))return 2;
 rclcpp::init(argc,argv);
 rclcpp::NodeOptions options;
 options.parameter_overrides({rclcpp::Parameter("lio.sensor.lidar_type",1),
   rclcpp::Parameter("lio.map.save_map",false)});
 auto wrapper=std::make_shared<LI2Sup::ROSWrapper>(options);
 auto filter=std::make_shared<LI2Sup::ESKF>();wrapper->setESKF(filter);
 // Anisotropic PSD prior with theta/p/v/bg cross blocks; no physical estimate.
 LI2Sup::ESKF::COV factor=LI2Sup::ESKF::COV::Zero();
 for(int r=0;r<18;++r)for(int c=0;c<18;++c)factor(r,c)=std::sin(r*13+c*7+.2f);
 filter->SetCov((factor*factor.transpose()+.01f*LI2Sup::ESKF::COV::Identity()).eval());
 const auto covariance=filter->GetCov().cast<double>().eval();
 auto sink=std::make_shared<rclcpp::Node>("source_orientation_probe");
 std::vector<nav_msgs::msg::Odometry> odoms;
 std::vector<geometry_msgs::msg::TransformStamped> transforms;
 std::vector<std_msgs::msg::String> health;
 std::vector<sensor_msgs::msg::PointCloud2> clouds;
 auto csub=sink->create_subscription<sensor_msgs::msg::PointCloud2>("/lio/cloud_world",10,
   [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){clouds.push_back(*m);});
 auto osub=sink->create_subscription<nav_msgs::msg::Odometry>("/lio/odom",10,
   [&](nav_msgs::msg::Odometry::ConstSharedPtr m){odoms.push_back(*m);});
 auto tsub=sink->create_subscription<tf2_msgs::msg::TFMessage>("/tf",rclcpp::SensorDataQoS(),
   [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m){for(const auto&t:m->transforms)
     if(t.header.frame_id=="world"&&t.child_frame_id=="imu")transforms.push_back(t);});
 auto hsub=sink->create_subscription<std_msgs::msg::String>("/lio/health",10,
   [&](std_msgs::msg::String::ConstSharedPtr m){health.push_back(*m);});
 rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(wrapper);exec.add_node(sink);
 auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
   while(std::chrono::steady_clock::now()<end){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
 spin(400);
 LI2Sup::NavState state;
 state.v<<.3f,-.2f,.1f;
 const BASIC::M3 nominal=Eigen::AngleAxisf(.3f,BASIC::V3::UnitZ()).toRotationMatrix();
 for(float scale:{1.f,1.000024f}) {
   state.R.R_=nominal*scale;state.timestamp=sink->now().seconds();
   const auto before=odoms.size(),tb=transforms.size();
   wrapper->pub_odom(state);spin(100);
   assert(odoms.size()==before+1 && transforms.size()==tb+1);
   const auto&q=odoms.back().pose.pose.orientation;
   const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
   std::cout<<"source scale="<<scale<<" published norm2="<<std::setprecision(17)<<norm<<std::endl;
   assert(std::isfinite(norm)&&std::abs(norm-1.)<1e-5);
   const auto&t=transforms.back().transform.rotation;
   assert(q.x==t.x&&q.y==t.y&&q.z==t.z&&q.w==t.w);
   const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
   const Eigen::Vector3d recovered=rotation.toRotationMatrix()*Eigen::Vector3d(
     odoms.back().twist.twist.linear.x,odoms.back().twist.twist.linear.y,odoms.back().twist.twist.linear.z);
   assert((recovered-state.v.cast<double>()).norm()<1e-6);
   // Independent full-state central differences, not calls to covariance helpers.
   Eigen::Matrix<double,6,18> jp=Eigen::Matrix<double,6,18>::Zero(),jt=jp;
   const double epsilon=1e-6;
   const auto R=rotation.toRotationMatrix();
   auto value=[&](const Eigen::Matrix<double,18,1>& error) {
     const auto angle=error.head<3>().eval();
     const Eigen::Matrix3d delta=angle.norm()>0 ?
       Eigen::AngleAxisd(angle.norm(),angle.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity();
     Eigen::Matrix<double,12,1> result;
     result.segment<3>(0)=error.segment<3>(3);
     result.segment<3>(3)=R*angle;
     result.segment<3>(6)=(R*delta).transpose()*(state.v.cast<double>()+error.segment<3>(6));
     result.segment<3>(9)=-error.segment<3>(9);return result;
   };
   for(int c=0;c<18;++c) {
     Eigen::Matrix<double,18,1> error=Eigen::Matrix<double,18,1>::Zero();error[c]=epsilon;
     const auto derivative=((value(error)-value(-error))/(2.*epsilon)).eval();
     jp.col(c)=derivative.head<6>();jt.col(c)=derivative.tail<6>();
   }
   const auto expected_pose=(jp*covariance*jp.transpose()).eval();
   auto expected_twist=(jt*covariance*jt.transpose()).eval();
   expected_twist.block<3,3>(3,3)+=filter->GetGyroSampleVariance()*Eigen::Matrix3d::Identity();
   for(int r=0;r<6;++r)for(int c=0;c<6;++c) {
     assert(std::abs(odoms.back().pose.covariance[r*6+c]-expected_pose(r,c))<1e-7);
     assert(std::abs(odoms.back().twist.covariance[r*6+c]-expected_twist(r,c))<1e-7);
   }
 }
 for(int fault=0;fault<5;++fault) {
   state.R.R_=BASIC::M3::Identity();
   if(fault==0)state.R.R_.setZero();
   if(fault==1)state.R.R_(0,0)=std::numeric_limits<float>::quiet_NaN();
   if(fault==2)state.R.R_(0,0)=-1.f;
   if(fault==3)state.R.R_(0,1)=.001f; // det=1, Gram error out of bounds.
   if(fault==4)state.R.R_*=1.0002f; // float drift beyond representation bound.
   state.timestamp=sink->now().seconds();
   const auto before=odoms.size(),tb=transforms.size(),hb=health.size();
   wrapper->pub_odom(state);spin(100);
   assert(odoms.size()==before&&transforms.size()==tb);
   assert(health.size()==hb+1);
   assert(health.back().data.find("source_rotation_not_valid")!=std::string::npos);
 }
 // Real Output world-cloud boundary: valid -> invalid only 40ms later.
 OutputProbe output(wrapper,filter);
 LI2Sup::g_visual_map=true;LI2Sup::g_visual_dense=true;LI2Sup::g_pub_step=1;
 state.R.R_=nominal;state.timestamp=sink->now().seconds();
 auto cb=clouds.size();output.run(state);spin(100);assert(clouds.size()==cb+1);
 state.R.R_.setZero();state.timestamp+=.04;
 cb=clouds.size();const auto ob=odoms.size(),tb=transforms.size();
 output.run(state);spin(100);
 assert(clouds.size()==cb && odoms.size()==ob && transforms.size()==tb);
 state.R.R_=nominal*1.000024f;state.timestamp=sink->now().seconds();
 cb=clouds.size();output.run(state);spin(100);assert(clouds.size()==cb+1);
 BASIC::PointCloudType cloud;pcl::fromROSMsg(clouds.back(),cloud);assert(cloud.size()==1);
 const auto& q=odoms.back().pose.pose.orientation;
 const auto expected=Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix()*Eigen::Vector3d(2.,.5,.2)+state.p.cast<double>();
 assert((Eigen::Vector3d(cloud[0].x,cloud[0].y,cloud[0].z)-expected).norm()<1e-6);
 exec.remove_node(wrapper);exec.remove_node(sink);wrapper.reset();sink.reset();rclcpp::shutdown();
 std::cout<<"PASS: actual source pose/TF/body-twist/full covariance/world-cloud share one rotation; invalid SO3 withheld with explicit failure"<<std::endl;
}

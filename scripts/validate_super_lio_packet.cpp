// Actual ROSWrapper parser/synchronizer, deliberately unordered MID offsets.
// No estimator, sensor driver, physical serial or motion publisher starts.
#include "ros/ROSWrapper.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

int main(int argc,char** argv) {
  const char* domain=std::getenv("ROS_DOMAIN_ID");
  const char* local=std::getenv("ROS_LOCALHOST_ONLY");
  if(!domain || !local || std::strcmp(domain,"105") || std::strcmp(local,"1")) {
    std::cerr << "packet probe requires isolated domain105 and localhost-only before ROS init\n";
    return 2;
  }
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("lio.sensor.lidar_type",1),
    rclcpp::Parameter("lio.ros.lidar_topic","/livox/lidar"),rclcpp::Parameter("lio.ros.imu_topic","/livox/imu"),
    rclcpp::Parameter("lio.sensor.filter_rate",4),
    rclcpp::Parameter("lio.sensor.blind",.5),rclcpp::Parameter("lio.sensor.maxrange",25.),
    rclcpp::Parameter("lio.map.save_map",false)});
  auto wrapper=std::make_shared<LI2Sup::ROSWrapper>(options);
  auto inactive_filter=std::make_shared<LI2Sup::ESKF>();
  wrapper->setESKF(inactive_filter); // constructor contract; init_ stays false
  auto fixture=std::make_shared<rclcpp::Node>("unordered_packet_probe");
  auto lidar=fixture->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar",10);
  auto imu=fixture->create_publisher<sensor_msgs::msg::Imu>("/livox/imu",100);
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(wrapper);executor.add_node(fixture);
  const auto discovered=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(std::chrono::steady_clock::now()<discovered){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  auto send_imus=[&]() {
    for(int i=0;i<5;++i) {sensor_msgs::msg::Imu sample;sample.header.stamp.sec=10;sample.header.stamp.nanosec=i*10000000;
      sample.linear_acceleration.z=1.;imu->publish(sample);}
  };
  livox_ros_driver2::msg::CustomMsg packet;packet.header.stamp.sec=10;packet.point_num=12;packet.points.resize(12);
  for(auto& point:packet.points){point.x=2.;point.reflectivity=10;point.tag=0;point.offset_time=20000000;}
  packet.points[0].offset_time=5000000;packet.points[4].offset_time=35000000;packet.points[8].offset_time=20000000;
  lidar->publish(packet);send_imus();
  LI2Sup::MeasureGroup group;bool synchronized=false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(std::chrono::steady_clock::now()<deadline && !synchronized) {
    executor.spin_some();synchronized=wrapper->sync_measure(group);std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(synchronized);
  assert(std::abs(group.lidar.end_time-10.035)<1e-9); // fails on last-selected-offset upstream behavior
  assert(group.lidar.pc->size()==3 && group.imu.size()==4);
  assert(std::abs(group.lidar.pc->points[0].offset_time-.005)<1e-8);
  assert(std::abs(group.lidar.pc->points[1].offset_time-.035)<1e-8);
  assert(std::abs(group.lidar.pc->points[2].offset_time-.020)<1e-8);
  assert(group.imu_after_scan && std::abs(group.imu_after_scan->secs-10.04)<1e-9);
  wrapper->clear();packet.point_num=100;lidar->publish(packet);send_imus();
  const auto invalid_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(200);
  while(std::chrono::steady_clock::now()<invalid_deadline){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  assert(!wrapper->sync_measure(group)); // malformed array/count must never be indexed
  executor.remove_node(wrapper);executor.remove_node(fixture);wrapper.reset();fixture.reset();rclcpp::shutdown();
  std::cout << "PASS: actual unordered MID parser uses maximum accepted offset, preserves each point time and real lookahead, rejects mismatched array/count\n";
}

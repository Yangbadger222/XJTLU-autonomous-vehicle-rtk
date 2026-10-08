#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <cmath>
using json=nlohmann::json;
class Probe: public rclcpp::Node {
 std::ofstream out_;
 rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_,static_;
 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
 static std::string gid(const rclcpp::MessageInfo& i){
  std::ostringstream s; s<<std::hex<<std::setfill('0');
  for(auto x:i.get_rmw_message_info().publisher_gid.data)s<<std::setw(2)<<unsigned(x);
  return s.str();
 }
 void write(json j){
  j["receive_ros_ns"]=now().nanoseconds();
  j["receive_steady_ns"]=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  out_<<j.dump()<<"\n";
 }
public:
 Probe(const std::string& path):Node("research_tf_timing_probe"),out_(path){
  if(!out_)throw std::runtime_error("output cannot be opened");
  auto collect=[this](tf2_msgs::msg::TFMessage::ConstSharedPtr m,const rclcpp::MessageInfo& info,bool is_static){
   for(const auto& t:m->transforms){
    if(t.child_frame_id!="odom" && t.child_frame_id!="base_footprint")continue;
    const auto&q=t.transform.rotation;const auto&p=t.transform.translation;
    const int64_t stamp=int64_t(t.header.stamp.sec)*1000000000LL+t.header.stamp.nanosec;
    write({{"event","tf"},{"static",is_static},{"gid",gid(info)},
       {"parent",t.header.frame_id},{"child",t.child_frame_id},{"stamp_ns",stamp},
       {"quaternion",{q.x,q.y,q.z,q.w}},{"norm2",q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w},
       {"translation",{p.x,p.y,p.z}}});
   }
  };
  tf_=create_subscription<tf2_msgs::msg::TFMessage>("/tf",rclcpp::SensorDataQoS(),
   [collect](tf2_msgs::msg::TFMessage::ConstSharedPtr m,const rclcpp::MessageInfo&i){collect(m,i,false);});
  static_=create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",rclcpp::QoS(100).reliable().transient_local(),
   [collect](tf2_msgs::msg::TFMessage::ConstSharedPtr m,const rclcpp::MessageInfo&i){collect(m,i,true);});
  flag_=create_subscription<std_msgs::msg::Bool>("/research/tf_integrity",10,
   [this](std_msgs::msg::Bool::ConstSharedPtr m){write({{"event","guard"},{"valid",m->data}});});
  odom_=create_subscription<nav_msgs::msg::Odometry>("/research/odom_control",10,
   [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
    const auto&q=m->pose.pose.orientation;
    write({{"event","control_odom"},{"stamp_ns",int64_t(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec},
      {"frame",m->header.frame_id},{"child",m->child_frame_id},
      {"quaternion",{q.x,q.y,q.z,q.w}},{"norm2",q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w}});
   });
  write({{"event","scope"},{"label","DEBUG-tf09c"},{"scope","read-only native per-message GID/TF/guard timing; no motion or authority publisher"}});
 }
 ~Probe(){out_.flush();}
};
int main(int argc,char**argv){
 if(argc!=2)return 2;
 const std::string path=argv[1];
 rclcpp::init(argc,argv);
 rclcpp::spin(std::make_shared<Probe>(path));
 rclcpp::shutdown();
}

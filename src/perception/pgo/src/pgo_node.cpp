
// 文件最新改动时间：2025/11/13 -- 20:15
// 文件最新改动人：Claude Sonnet 4.5
// 操作者：You-guesssssss

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>

namespace {
std::string getRuntimeRoot() {
    const char* runtime_root = std::getenv("FYP_RUNTIME_ROOT");
    if (runtime_root != nullptr && runtime_root[0] != '\0') {
        return std::string(runtime_root);
    }
    const char* home = std::getenv("HOME");
    return std::string(home != nullptr ? home : "/home/jetson") + "/XJTLU-autonomous-vehicle/runtime-data";
}

std::string getRuntimePath(const std::string& relative_path) {
    return getRuntimeRoot() + "/" + relative_path;
}

bool ensureDirectory(const std::string& directory) {
    if (directory.empty()) {
        return false;
    }

    std::string current = directory[0] == '/' ? "/" : "";
    std::stringstream path_stream(directory);
    std::string segment;
    while (std::getline(path_stream, segment, '/')) {
        if (segment.empty()) {
            continue;
        }
        if (!current.empty() && current.back() != '/') {
            current += "/";
        }
        current += segment;
        if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
            return false;
        }
    }
    return true;
}
}

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    bool publish_tf = true;
    double global_map_pub_rate = 1.0;  // 全局地图发布频率 (Hz)
    double global_map_resolution = 0.1;  // 全局地图降采样分辨率 (m)
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time;
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        
        bool enable_log = shouldEnableLogging("pgo_node");
        
        if (enable_log) {
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm* now_tm = std::localtime(&now_time);

            std::string log_dir = getRuntimePath("logs/pgo");
            ensureDirectory(log_dir);

            std::ostringstream filename_stream;
            filename_stream << log_dir << "/log_"
                            << (now_tm->tm_year + 1900)  // 年份
                            << std::setw(2) << std::setfill('0') << (now_tm->tm_mon + 1) // 月份
                            << std::setw(2) << std::setfill('0') << now_tm->tm_mday // 日期
                            << "_"
                            << std::setw(2) << std::setfill('0') << now_tm->tm_hour // 小时
                            << std::setw(2) << std::setfill('0') << now_tm->tm_min // 分钟
                            << std::setw(2) << std::setfill('0') << now_tm->tm_sec // 秒
                            << ".txt"; // 文件扩展名

            m_log_file.open(filename_stream.str(), std::ios::app);

            if (!m_log_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to open log file");
                throw std::runtime_error("Unable to open log file");
            } else {
                RCLCPP_INFO(this->get_logger(), "Logging enabled: %s", filename_stream.str().c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging disabled by config");
        }

        loadParameters();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        
        rclcpp::QoS qos = rclcpp::QoS(50);  // 队列大小从 10 增加到 50
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10000);
        m_global_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pgo/global_map", 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(50), m_cloud_sub, m_odom_sub);  // 同步器队列从 10 增加到 50
        m_sync->setAgePenalty(1.0);  // 时间容忍度从 0.1 增加到 1.0
        m_sync->setMaxIntervalDuration(rclcpp::Duration(0, 100000000));  // 设置最大时间间隔为 100ms
        
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        
        // 声明：此处为上次改动的旧代码
        // 上次修改目的：将定时器周期设置为 20ms（50Hz）以解决 TF 时间戳不同步问题并与 FASTLIO2 同步
        // ============================================================================
        // m_timer = this->create_wall_timer(20ms, std::bind(&PGONode::timerCB, this));
        // ============================================================================
        
        // 此处代码改动的时间：2025/11/13 -- 20:15
        // 此处代码改动人：Claude Sonnet 4.5
        // 操作者：You-guesssssss
        // 改动原因：用户需求将 PGO 节点的消息发布频率从 50Hz 降低至 20Hz，以减轻系统负载和 CPU 压力，并与 FASTLIO2 节点的 20Hz 频率保持一致
        // 改动内容：将 create_wall_timer() 的第一个参数从 20ms 改为 50ms，将定时器执行频率从 50Hz 降低至 20Hz
        // 改动影响：
        //   1. PGO 处理频率降低至 20Hz，队列中的点云数据处理速度变慢
        //   2. TF 广播频率降低至 20Hz（map → odom 变换每 50ms 更新一次）
        //   3. 降低 CPU 占用率，减轻 Jetson 平台负载
        //   4. 与 FASTLIO2 节点的 20Hz 频率保持一致，提高系统整体协调性
        //   5. 可能增加 TF 延迟，但配合 transform_tolerance 设置仍可保证 Nav2 正常工作
        //   6. 队列积压风险增加，需要监控 cloud_buffer 大小（已有监控代码）
        //   7. 回环检测和图优化执行频率降低，但由于这些操作耗时较长，降频有助于系统稳定性
        m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
        
        // 创建全局地图发布定时器，默认 1Hz
        int global_map_period_ms = static_cast<int>(1000.0 / m_node_config.global_map_pub_rate);
        m_global_map_timer = this->create_wall_timer(
            std::chrono::milliseconds(global_map_period_ms), 
            std::bind(&PGONode::publishGlobalMap, this));
        
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("/pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~PGONode()
    {
        if (m_log_file.is_open()) {
            m_log_file.close();
            RCLCPP_INFO(this->get_logger(), "Log file closed.");
        }
    }

    bool shouldEnableLogging(const std::string& node_key)
    {
        try {
            std::string config_path = getRuntimePath("config/log_switch.yaml");
            YAML::Node config = YAML::LoadFile(config_path);
            
            if (config[node_key]) {
                bool enable = config[node_key]["enable_logging"].as<bool>();
                return enable;
            }
            
            RCLCPP_WARN(this->get_logger(), "No logging config found for '%s', enabling by default", node_key.c_str());
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read log config: %s", e.what());
            return true; // 配置文件读取失败，默认启用日志
        }
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();
        if (config["publish_tf"]) {
            m_node_config.publish_tf = config["publish_tf"].as<bool>();
        }

        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
        
        // 读取全局地图发布参数（可选，有默认值）
        if (config["global_map_pub_rate"]) {
            m_node_config.global_map_pub_rate = config["global_map_pub_rate"].as<double>();
        }
        if (config["global_map_resolution"]) {
            m_node_config.global_map_resolution = config["global_map_resolution"].as<double>();
        }
        RCLCPP_INFO(this->get_logger(), "Global map publish rate: %.2f Hz, resolution: %.3f m, publish_tf=%s",
                    m_node_config.global_map_pub_rate,
                    m_node_config.global_map_resolution,
                    m_node_config.publish_tf ? "true" : "false");
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);
        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }
        m_state.last_message_time = cp.pose.second;

        RCLCPP_INFO(this->get_logger(), "Received synced cloud and odom: cloud_time=%.6f, points=%u", 
                    cp.pose.second, cloud_msg->width * cloud_msg->height);
        
        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", Received synced cloud and odom: cloud_time=" << cp.pose.second 
                      << ", points=" << (cloud_msg->width * cloud_msg->height) << std::endl;
            m_log_file.flush();
        }

        size_t queue_size = m_state.cloud_buffer.size();
        if (queue_size > 30) {
            RCLCPP_WARN(this->get_logger(), 
                        "PGO queue size high: %zu (may cause message drop)", 
                        queue_size);
            if (m_log_file.is_open()) {
                auto ros_time = this->now();
                int64_t ros_timestamp = ros_time.nanoseconds();
                m_log_file << "ROS_timestamp: " << ros_timestamp 
                          << ", WARNING: Queue size high: " << queue_size << std::endl;
                m_log_file.flush();
            }
        }
        
        static int sync_count = 0;
        static auto last_log_time = this->now();
        sync_count++;
        auto current_time = this->now();
        if ((current_time - last_log_time).seconds() > 10.0) {
            RCLCPP_INFO(this->get_logger(), 
                        "Sync stats: rate=%.2f Hz, queue_size=%zu", 
                        sync_count / 10.0, queue_size);
            if (m_log_file.is_open()) {
                auto ros_time = this->now();
                int64_t ros_timestamp = ros_time.nanoseconds();
                m_log_file << "ROS_timestamp: " << ros_timestamp 
                          << ", Sync stats: rate=" << (sync_count / 10.0) 
                          << " Hz, queue_size=" << queue_size << std::endl;
                m_log_file.flush();
            }
            sync_count = 0;
            last_log_time = current_time;
        }

        cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                       odom_msg->pose.pose.orientation.x,
                                       odom_msg->pose.pose.orientation.y,
                                       odom_msg->pose.pose.orientation.z)
                        .toRotationMatrix();
        cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        cp.cloud = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *cp.cloud);
        m_state.cloud_buffer.push(cp);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        if (!m_node_config.publish_tf)
            return;

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    // 发布基于关键帧构建的全局点云地图
    void publishGlobalMap()
    {
        // 如果没有订阅者，不发布
        if (m_global_map_pub->get_subscription_count() == 0)
            return;
        
        // 如果没有关键帧，不发布
        if (m_pgo->keyPoses().size() == 0)
            return;
        
        CloudType::Ptr global_map(new CloudType);
        
        // 遍历所有关键帧，将点云转换到全局坐标系并合并
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {
            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
            CloudType::Ptr world_cloud(new CloudType);
            
            // 使用优化后的全局位姿将点云转换到全局坐标系
            pcl::transformPointCloud(*body_cloud, *world_cloud, 
                                     m_pgo->keyPoses()[i].t_global, 
                                     Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            *global_map += *world_cloud;
        }
        
        // 对全局地图进行降采样以减少数据量
        if (m_node_config.global_map_resolution > 0 && global_map->size() > 0)
        {
            pcl::VoxelGrid<PointType> voxel_grid;
            voxel_grid.setLeafSize(m_node_config.global_map_resolution, 
                                   m_node_config.global_map_resolution, 
                                   m_node_config.global_map_resolution);
            voxel_grid.setInputCloud(global_map);
            CloudType::Ptr filtered_map(new CloudType);
            voxel_grid.filter(*filtered_map);
            global_map = filtered_map;
        }
        
        // 转换为 ROS 消息并发布
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*global_map, cloud_msg);
        cloud_msg.header.frame_id = m_node_config.map_frame;
        cloud_msg.header.stamp = this->now();
        
        m_global_map_pub->publish(cloud_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published global map with %zu points from %zu key poses", 
                     global_map->size(), m_pgo->keyPoses().size());
    }

    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->historyPairs().size() == 0)
            return;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();
        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    void timerCB()
    {
        CloudWithPose cp;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (m_state.cloud_buffer.empty())
                return;
            cp = m_state.cloud_buffer.front();
            m_state.cloud_buffer.pop();  // 只弹出一个元素
        }
        
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        
        bool is_key_pose = m_pgo->addKeyPose(cp);
        if (!is_key_pose)
        {
            sendBroadCastTF(cur_time);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Added key pose: total_poses=%zu, time=%.6f", 
                    m_pgo->keyPoses().size(), cp.pose.second);
        
        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", Added key pose: total_poses=" << m_pgo->keyPoses().size() 
                      << ", time=" << cp.pose.second << std::endl;
            m_log_file.flush();
        }

        size_t loop_pairs_before = m_pgo->historyPairs().size();
        m_pgo->searchForLoopPairs();
        size_t loop_pairs_after = m_pgo->historyPairs().size();
        
        if (loop_pairs_after > loop_pairs_before)
        {
            RCLCPP_INFO(this->get_logger(), "Loop detected! Total loop pairs: %zu", loop_pairs_after);
            
            if (m_log_file.is_open()) {
                auto ros_time = this->now();
                int64_t ros_timestamp = ros_time.nanoseconds();
                m_log_file << "ROS_timestamp: " << ros_timestamp 
                          << ", Loop detected! Total loop pairs: " << loop_pairs_after << std::endl;
                m_log_file.flush();
            }
        }

        m_pgo->smoothAndUpdate();

        sendBroadCastTF(cur_time);

        publishLoopMarkers(cur_time);
    }

    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", Saving map to: " << map_path.string() 
                      << ", total_poses=" << m_pgo->keyPoses().size() << std::endl;
            m_log_file.flush();
        }

        std::ofstream txt_file(poses_txt_path);

        CloudType::Ptr ret(new CloudType);
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {

            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            }
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global, Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            *ret += *world_cloud;
        }
        txt_file.close();
        pcl::io::savePCDFileBinary(map_path.string(), *ret);
        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_global_map_timer;  // 全局地图发布定时器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_map_pub;  // 全局地图发布器
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::ofstream m_log_file;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}

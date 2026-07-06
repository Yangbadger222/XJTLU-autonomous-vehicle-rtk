
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
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <cmath>
// ✅ GPS 融合修改开始 - 2025/12/01 - 添加 GPS 相关头文件
#include <sensor_msgs/msg/nav_sat_fix.hpp>  // GPS 数据消息类型
#include <std_msgs/msg/float64_multi_array.hpp>
#include <GeographicLib/LocalCartesian.hpp> // GPS 坐标转换库
// ✅ GPS 融合修改结束 - 头文件添加完成
// P2（FRC）：keyframe 导出与回环修正状态
#include <std_msgs/msg/float32_multi_array.hpp>
#include <frc_msgs/msg/keyframe_array.hpp>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include <pcl/io/io.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

namespace {
constexpr double kRadToDeg = 57.29577951308232;

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

std::string getSessionLogPath(const std::string& filename, const std::string& fallback_subdir) {
    const char* session_dir = std::getenv("FYP_LOG_SESSION_DIR");
    if (session_dir != nullptr && session_dir[0] != '\0') {
        std::string dir(session_dir);
        ensureDirectory(dir);
        return dir + "/" + filename;
    }

    std::string dir = getRuntimePath(fallback_subdir);
    ensureDirectory(dir);

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time);

    std::ostringstream ss;
    ss << dir << "/log_"
       << (now_tm->tm_year + 1900)
       << std::setw(2) << std::setfill('0') << (now_tm->tm_mon + 1)
       << std::setw(2) << std::setfill('0') << now_tm->tm_mday
       << "_"
       << std::setw(2) << std::setfill('0') << now_tm->tm_hour
       << std::setw(2) << std::setfill('0') << now_tm->tm_min
       << std::setw(2) << std::setfill('0') << now_tm->tm_sec
       << ".txt";
    return ss.str();
}

// P2（FRC）：session_id = launch_with_logs.sh 会话时间戳。
// FYP_LOG_SESSION_DIR 形如 .../runtime-data/logs/<timestamp>/data，取上一级目录名。
std::string getSessionId() {
    const char* session_dir = std::getenv("FYP_LOG_SESSION_DIR");
    if (session_dir == nullptr || session_dir[0] == '\0') {
        return "unknown";
    }
    std::filesystem::path p(session_dir);
    if (p.filename() == "data") {
        p = p.parent_path();
    }
    std::string name = p.filename().string();
    return name.empty() ? "unknown" : name;
}

bool pgoVerboseDiagEnabled() {
    static const bool enabled = []() {
        const char* value = std::getenv("PGO_VERBOSE_DIAG");
        if (value == nullptr || value[0] == '\0') {
            return false;
        }
        const std::string flag(value);
        return flag == "1" || flag == "true" || flag == "TRUE" ||
               flag == "yes" || flag == "YES" || flag == "on" || flag == "ON";
    }();
    return enabled;
}
}

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "odom";
    bool publish_tf = true;
    double global_map_pub_rate = 1.0;
    double global_map_resolution = 0.1;
    
    // ✅ GPS 融合修改开始 - 2025/12/01 - 添加 GPS 配置参数
    std::string gps_topic = "/fix";               // GPS 话题名称
    bool enable_gps = true;                        // GPS 功能启用开关
    double gps_noise_xy = 2.5;                     // GPS 水平噪声（米）
    double gps_noise_z = 5.0;                      // GPS 垂直噪声（米）
    int gps_factor_interval = 5;                   // 每 N 个关键帧添加 GPS 因子
    double gps_quality_hdop_max = 3.0;             // 最大 HDOP 阈值
    int gps_quality_sat_min = 6;                   // 最小卫星数量
    double gps_drift_threshold = 2.0;              // 漂移检测阈值（米）
    int gps_alert_interval = 3;                    // 漂移预警间隔
    int gps_emergency_interval = 1;                // 严重漂移间隔
    std::string gps_origin_mode = "auto";         // GPS ENU 原点模式: auto/fixed
    double gps_origin_lat = 0.0;                   // fixed 模式下的原点纬度
    double gps_origin_lon = 0.0;                   // fixed 模式下的原点经度
    double gps_origin_alt = 0.0;                   // fixed 模式下的原点海拔
    int gps_alignment_min_points = 5;              // 旋转估计最少配对数
    double gps_alignment_min_spread_m = 5.0;       // 旋转估计最小空间展幅
    int gps_alignment_warmup_factors = 5;          // 热身 GPS 因子个数
    double gps_alignment_warmup_sigma = 10.0;      // 热身 GPS 水平 sigma
    std::string gps_alignment_topic = "/gps_corridor/enu_to_map";
    // ✅ GPS 融合修改结束 - NodeConfig 扩展完成

    // P2（FRC）：keyframe 导出与修正状态参数
    double frc_keyframes_pub_rate = 1.0;           // /pgo/keyframes 发布频率（Hz）
    double correction_jump_thresh = 0.05;          // offset 平移跳变阈值（米），超过视为进入修正窗口
    double correction_window_s = 3.0;              // 修正窗口保持时长（秒）
};

struct AlignmentPair
{
    size_t key_idx = 0;
    Eigen::Vector2d enu_xy = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_xy = Eigen::Vector2d::Zero();
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time = 0.0;  // Initialize monotonic sync state so the first matched pair is accepted.
    
    // ✅ GPS 融合修改开始 - 2025/12/01 - 添加 GPS 状态变量
    std::queue<sensor_msgs::msg::NavSatFix::ConstSharedPtr> gps_buffer;  // GPS 数据缓存队列
    bool gps_origin_set = false;                   // GPS 原点是否已设置
    double origin_lat = 0.0;                       // 原点纬度
    double origin_lon = 0.0;                       // 原点经度
    double origin_alt = 0.0;                       // 原点海拔
    std::vector<AlignmentPair> alignment_pairs;    // ENU/map 配对缓存
    bool alignment_valid = false;                  // ENU->map 变换是否有效
    double alignment_theta = 0.0;                  // ENU->map 2D 旋转角
    double alignment_tx = 0.0;                     // ENU->map 2D 平移 x
    double alignment_ty = 0.0;                     // ENU->map 2D 平移 y
    int gps_factors_added = 0;                     // 已加入的 GPS 因子数
    // ✅ GPS 融合修改结束 - NodeState 扩展完成

    // P2（FRC）：修正窗口状态
    bool correction_offset_init = false;           // 是否已记录初始 offset
    Eigen::Vector3d last_offset_t = Eigen::Vector3d::Zero();  // 上一次 smoothAndUpdate 后的 offsetT
    double last_jump_m = 0.0;                      // 最近一次超阈值跳变量（米）
    double correction_until = 0.0;                 // 修正窗口截止时刻（节点时钟秒）

    // Keep the latest map->odom transform available at timer frequency even when LIO drops frames.
    bool last_tf_valid = false;
    Eigen::Quaterniond last_tf_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d last_tf_translation = Eigen::Vector3d::Zero();
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        
        bool enable_log = shouldEnableLogging("pgo_node");
        
        if (enable_log) {
            std::string log_path = getSessionLogPath("pgo.log", "logs/pgo");
            m_log_file.open(log_path, std::ios::app);

            if (!m_log_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to open log file");
                throw std::runtime_error("Unable to open log file");
            } else {
                RCLCPP_INFO(this->get_logger(), "Logging enabled: %s", log_path.c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging disabled by config");
        }

        loadParameters();
        if (m_node_config.enable_gps && m_node_config.gps_origin_mode == "fixed") {
            setGpsOrigin(m_node_config.gps_origin_lat,
                         m_node_config.gps_origin_lon,
                         m_node_config.gps_origin_alt,
                         "fixed parameter");
        }
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        
        rclcpp::QoS qos = rclcpp::QoS(50);  // 队列大小从 10 增加到 50
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10000);
        m_global_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pgo/global_map", 10);
        m_optimized_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/pgo/optimized_odom", 100);
        m_alignment_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            m_node_config.gps_alignment_topic, 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(50), m_cloud_sub, m_odom_sub);  // 同步器队列从 10 增加到 50
        m_sync->setAgePenalty(1.0);  // 时间容忍度从 0.1 增加到 1.0
        m_sync->setMaxIntervalDuration(rclcpp::Duration(0, 100000000));  // 设置最大时间间隔为 100ms
        
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        
        // ✅ GPS 融合修改开始 - 2025/12/01 - 初始化 GPS 订阅器
        if (m_node_config.enable_gps) {
            m_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                m_node_config.gps_topic, 
                rclcpp::QoS(10),
                std::bind(&PGONode::gpsCB, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "GPS subscriber enabled on topic: %s", 
                        m_node_config.gps_topic.c_str());
            
            // 仅在未设置时初始化为null（fixed模式下已在上方setGpsOrigin中创建）
            if (!m_geo_converter) { m_geo_converter = nullptr; }
        }
        // ✅ GPS 融合修改结束 - GPS 订阅器初始化完成
        
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
        m_timer = this->create_wall_timer(20ms, std::bind(&PGONode::timerCB, this));
        int global_map_period_ms = static_cast<int>(1000.0 / m_node_config.global_map_pub_rate);
        if (global_map_period_ms <= 0) {
            global_map_period_ms = 1000;
        }
        m_global_map_timer = this->create_wall_timer(
            std::chrono::milliseconds(global_map_period_ms),
            std::bind(&PGONode::publishGlobalMap, this));
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("/pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));

        // P2（FRC）：keyframe 数组（1 Hz 定时器，复用 global_map_timer 模式）与修正状态发布
        m_session_id = getSessionId();
        m_keyframes_pub = this->create_publisher<frc_msgs::msg::KeyframeArray>("/pgo/keyframes", 10);
        m_correction_status_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pgo/correction_status", 10);
        int keyframes_period_ms = static_cast<int>(1000.0 / m_node_config.frc_keyframes_pub_rate);
        if (keyframes_period_ms <= 0) {
            keyframes_period_ms = 1000;
        }
        m_keyframes_timer = this->create_wall_timer(
            std::chrono::milliseconds(keyframes_period_ms),
            std::bind(&PGONode::publishKeyframes, this));
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
            
            RCLCPP_WARN(this->get_logger(), "No logging config found for '%s', disabling by default", node_key.c_str());
            return false;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read log config: %s", e.what());
            return false; // 配置文件读取失败，默认关闭逐帧日志，避免现场运行刷爆 stdout
        }
    }

    void loadParameters()
    {
        const std::string config_path = this->declare_parameter<std::string>("config_path", "");

        m_node_config.cloud_topic = this->declare_parameter<std::string>("cloud_topic", "/fastlio2/body_cloud");
        m_node_config.odom_topic = this->declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
        m_node_config.map_frame = this->declare_parameter<std::string>("map_frame", "map");
        m_node_config.local_frame = this->declare_parameter<std::string>("local_frame", "odom");
        m_node_config.publish_tf = this->declare_parameter<bool>("publish_tf", m_node_config.publish_tf);
        m_node_config.global_map_pub_rate = this->declare_parameter<double>("global_map_pub_rate", 1.0);
        m_node_config.global_map_resolution = this->declare_parameter<double>("global_map_resolution", 0.1);

        m_pgo_config.key_pose_delta_deg = this->declare_parameter<double>("key_pose_delta_deg", 5.0);
        m_pgo_config.key_pose_delta_trans = this->declare_parameter<double>("key_pose_delta_trans", 0.1);
        m_pgo_config.loop_search_radius = this->declare_parameter<double>("loop_search_radius", 1.0);
        m_pgo_config.loop_time_tresh = this->declare_parameter<double>("loop_time_tresh", 60.0);
        m_pgo_config.loop_score_tresh = this->declare_parameter<double>("loop_score_tresh", 0.15);
        m_pgo_config.loop_submap_half_range = this->declare_parameter<int>("loop_submap_half_range", 5);
        m_pgo_config.submap_resolution = this->declare_parameter<double>("submap_resolution", 0.1);
        m_pgo_config.min_loop_detect_duration = this->declare_parameter<double>("min_loop_detect_duration", 5.0);

        m_node_config.enable_gps = this->declare_parameter<bool>("gps.enable", m_node_config.enable_gps);
        m_node_config.gps_topic = this->declare_parameter<std::string>("gps.topic", m_node_config.gps_topic);
        m_node_config.gps_noise_xy = this->declare_parameter<double>("gps.noise_xy", m_node_config.gps_noise_xy);
        m_node_config.gps_noise_z = this->declare_parameter<double>("gps.noise_z", m_node_config.gps_noise_z);
        m_node_config.gps_factor_interval = this->declare_parameter<int>("gps.factor_interval", m_node_config.gps_factor_interval);
        m_node_config.gps_alignment_min_points = this->declare_parameter<int>(
            "gps.alignment_min_points", m_node_config.gps_alignment_min_points);
        m_node_config.gps_alignment_min_spread_m = this->declare_parameter<double>(
            "gps.alignment_min_spread_m", m_node_config.gps_alignment_min_spread_m);
        m_node_config.gps_alignment_warmup_factors = this->declare_parameter<int>(
            "gps.alignment_warmup_factors", m_node_config.gps_alignment_warmup_factors);
        m_node_config.gps_alignment_warmup_sigma = this->declare_parameter<double>(
            "gps.alignment_warmup_sigma", m_node_config.gps_alignment_warmup_sigma);
        m_node_config.gps_alignment_topic = this->declare_parameter<std::string>(
            "gps.alignment_topic", m_node_config.gps_alignment_topic);
        // P2（FRC）参数
        m_node_config.frc_keyframes_pub_rate = this->declare_parameter<double>(
            "frc.keyframes_pub_rate", m_node_config.frc_keyframes_pub_rate);
        m_node_config.correction_jump_thresh = this->declare_parameter<double>(
            "frc.correction_jump_thresh", m_node_config.correction_jump_thresh);
        m_node_config.correction_window_s = this->declare_parameter<double>(
            "frc.correction_window_s", m_node_config.correction_window_s);
        m_node_config.gps_quality_hdop_max = this->declare_parameter<double>("gps.quality_hdop_max", m_node_config.gps_quality_hdop_max);
        m_node_config.gps_quality_sat_min = this->declare_parameter<int>("gps.quality_sat_min", m_node_config.gps_quality_sat_min);
        m_node_config.gps_drift_threshold = this->declare_parameter<double>("gps.drift_threshold", m_node_config.gps_drift_threshold);
        m_node_config.gps_alert_interval = this->declare_parameter<int>("gps.alert_interval", m_node_config.gps_alert_interval);
        m_node_config.gps_emergency_interval = this->declare_parameter<int>("gps.emergency_interval", m_node_config.gps_emergency_interval);
        m_node_config.gps_origin_mode = this->declare_parameter<std::string>("gps.origin_mode", m_node_config.gps_origin_mode);
        m_node_config.gps_origin_lat = this->declare_parameter<double>("gps.origin_lat", m_node_config.gps_origin_lat);
        m_node_config.gps_origin_lon = this->declare_parameter<double>("gps.origin_lon", m_node_config.gps_origin_lon);
        m_node_config.gps_origin_alt = this->declare_parameter<double>("gps.origin_alt", m_node_config.gps_origin_alt);

        if (!config_path.empty())
        {
            loadLegacyConfig(config_path);
            syncParametersToRos();
        }

        normalizeGpsOriginMode();
        if (m_node_config.gps_factor_interval <= 0) {
            RCLCPP_WARN(this->get_logger(), "gps.factor_interval=%d is invalid, forcing to 1",
                        m_node_config.gps_factor_interval);
            m_node_config.gps_factor_interval = 1;
        }
        if (m_node_config.gps_alignment_min_points < 2) {
            RCLCPP_WARN(this->get_logger(), "gps.alignment_min_points=%d is too small, forcing to 2",
                        m_node_config.gps_alignment_min_points);
            m_node_config.gps_alignment_min_points = 2;
        }
        if (m_node_config.gps_alignment_min_spread_m < 0.0) {
            RCLCPP_WARN(this->get_logger(), "gps.alignment_min_spread_m=%.3f is invalid, forcing to 0.0",
                        m_node_config.gps_alignment_min_spread_m);
            m_node_config.gps_alignment_min_spread_m = 0.0;
        }

        RCLCPP_INFO(this->get_logger(),
                    "GPS config loaded: interval=%d, noise_xy=%.2f, origin_mode=%s, align_topic=%s, align_min_points=%d, align_min_spread=%.2f",
                    m_node_config.gps_factor_interval,
                    m_node_config.gps_noise_xy,
                    m_node_config.gps_origin_mode.c_str(),
                    m_node_config.gps_alignment_topic.c_str(),
                    m_node_config.gps_alignment_min_points,
                    m_node_config.gps_alignment_min_spread_m);
    }

    void loadLegacyConfig(const std::string &config_path)
    {
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE: %s", config_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM LEGACY YAML CONFIG PATH: %s", config_path.c_str());

        if (config["cloud_topic"])
            m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        if (config["odom_topic"])
            m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        if (config["map_frame"])
            m_node_config.map_frame = config["map_frame"].as<std::string>();
        if (config["local_frame"])
            m_node_config.local_frame = config["local_frame"].as<std::string>();
        if (config["publish_tf"])
            m_node_config.publish_tf = config["publish_tf"].as<bool>();
        if (config["global_map_pub_rate"])
            m_node_config.global_map_pub_rate = config["global_map_pub_rate"].as<double>();
        if (config["global_map_resolution"])
            m_node_config.global_map_resolution = config["global_map_resolution"].as<double>();

        if (config["key_pose_delta_deg"])
            m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        if (config["key_pose_delta_trans"])
            m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        if (config["loop_search_radius"])
            m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        if (config["loop_time_tresh"])
            m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        if (config["loop_score_tresh"])
            m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        if (config["loop_submap_half_range"])
            m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        if (config["submap_resolution"])
            m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        if (config["min_loop_detect_duration"])
            m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();

        if (config["gps"])
        {
            if (config["gps"]["enable"])
                m_node_config.enable_gps = config["gps"]["enable"].as<bool>();
            if (config["gps"]["topic"])
                m_node_config.gps_topic = config["gps"]["topic"].as<std::string>();
            if (config["gps"]["noise_xy"])
                m_node_config.gps_noise_xy = config["gps"]["noise_xy"].as<double>();
            if (config["gps"]["noise_z"])
                m_node_config.gps_noise_z = config["gps"]["noise_z"].as<double>();
            if (config["gps"]["factor_interval"])
                m_node_config.gps_factor_interval = config["gps"]["factor_interval"].as<int>();
            if (config["gps"]["quality_hdop_max"])
                m_node_config.gps_quality_hdop_max = config["gps"]["quality_hdop_max"].as<double>();
            if (config["gps"]["quality_sat_min"])
                m_node_config.gps_quality_sat_min = config["gps"]["quality_sat_min"].as<int>();
            if (config["gps"]["drift_threshold"])
                m_node_config.gps_drift_threshold = config["gps"]["drift_threshold"].as<double>();
            if (config["gps"]["alert_interval"])
                m_node_config.gps_alert_interval = config["gps"]["alert_interval"].as<int>();
            if (config["gps"]["emergency_interval"])
                m_node_config.gps_emergency_interval = config["gps"]["emergency_interval"].as<int>();
            if (config["gps"]["origin_mode"])
                m_node_config.gps_origin_mode = config["gps"]["origin_mode"].as<std::string>();
            if (config["gps"]["origin_lat"])
                m_node_config.gps_origin_lat = config["gps"]["origin_lat"].as<double>();
            if (config["gps"]["origin_lon"])
                m_node_config.gps_origin_lon = config["gps"]["origin_lon"].as<double>();
            if (config["gps"]["origin_alt"])
                m_node_config.gps_origin_alt = config["gps"]["origin_alt"].as<double>();
            if (config["gps"]["alignment_min_points"])
                m_node_config.gps_alignment_min_points = config["gps"]["alignment_min_points"].as<int>();
            if (config["gps"]["alignment_min_spread_m"])
                m_node_config.gps_alignment_min_spread_m = config["gps"]["alignment_min_spread_m"].as<double>();
            if (config["gps"]["alignment_warmup_factors"])
                m_node_config.gps_alignment_warmup_factors = config["gps"]["alignment_warmup_factors"].as<int>();
            if (config["gps"]["alignment_warmup_sigma"])
                m_node_config.gps_alignment_warmup_sigma = config["gps"]["alignment_warmup_sigma"].as<double>();
        }
    }

    void syncParametersToRos()
    {
        this->set_parameters({
            rclcpp::Parameter("cloud_topic", m_node_config.cloud_topic),
            rclcpp::Parameter("odom_topic", m_node_config.odom_topic),
            rclcpp::Parameter("map_frame", m_node_config.map_frame),
            rclcpp::Parameter("local_frame", m_node_config.local_frame),
            rclcpp::Parameter("publish_tf", m_node_config.publish_tf),
            rclcpp::Parameter("global_map_pub_rate", m_node_config.global_map_pub_rate),
            rclcpp::Parameter("global_map_resolution", m_node_config.global_map_resolution),
            rclcpp::Parameter("key_pose_delta_deg", m_pgo_config.key_pose_delta_deg),
            rclcpp::Parameter("key_pose_delta_trans", m_pgo_config.key_pose_delta_trans),
            rclcpp::Parameter("loop_search_radius", m_pgo_config.loop_search_radius),
            rclcpp::Parameter("loop_time_tresh", m_pgo_config.loop_time_tresh),
            rclcpp::Parameter("loop_score_tresh", m_pgo_config.loop_score_tresh),
            rclcpp::Parameter("loop_submap_half_range", m_pgo_config.loop_submap_half_range),
            rclcpp::Parameter("submap_resolution", m_pgo_config.submap_resolution),
            rclcpp::Parameter("min_loop_detect_duration", m_pgo_config.min_loop_detect_duration),
            rclcpp::Parameter("gps.enable", m_node_config.enable_gps),
            rclcpp::Parameter("gps.topic", m_node_config.gps_topic),
            rclcpp::Parameter("gps.noise_xy", m_node_config.gps_noise_xy),
            rclcpp::Parameter("gps.noise_z", m_node_config.gps_noise_z),
            rclcpp::Parameter("gps.factor_interval", m_node_config.gps_factor_interval),
            rclcpp::Parameter("gps.alignment_min_points", m_node_config.gps_alignment_min_points),
            rclcpp::Parameter("gps.alignment_min_spread_m", m_node_config.gps_alignment_min_spread_m),
            rclcpp::Parameter("gps.alignment_warmup_factors", m_node_config.gps_alignment_warmup_factors),
            rclcpp::Parameter("gps.alignment_warmup_sigma", m_node_config.gps_alignment_warmup_sigma),
            rclcpp::Parameter("gps.quality_hdop_max", m_node_config.gps_quality_hdop_max),
            rclcpp::Parameter("gps.quality_sat_min", m_node_config.gps_quality_sat_min),
            rclcpp::Parameter("gps.drift_threshold", m_node_config.gps_drift_threshold),
            rclcpp::Parameter("gps.alert_interval", m_node_config.gps_alert_interval),
            rclcpp::Parameter("gps.emergency_interval", m_node_config.gps_emergency_interval),
            rclcpp::Parameter("gps.origin_mode", m_node_config.gps_origin_mode),
            rclcpp::Parameter("gps.origin_lat", m_node_config.gps_origin_lat),
            rclcpp::Parameter("gps.origin_lon", m_node_config.gps_origin_lon),
            rclcpp::Parameter("gps.origin_alt", m_node_config.gps_origin_alt),
        });
    }

    void normalizeGpsOriginMode()
    {
        std::transform(m_node_config.gps_origin_mode.begin(), m_node_config.gps_origin_mode.end(),
                       m_node_config.gps_origin_mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (m_node_config.gps_origin_mode != "auto" && m_node_config.gps_origin_mode != "fixed") {
            RCLCPP_WARN(this->get_logger(),
                        "Unknown gps.origin_mode '%s', fallback to auto",
                        m_node_config.gps_origin_mode.c_str());
            m_node_config.gps_origin_mode = "auto";
        }
    }
    
    // ✅ GPS 融合修改开始 - 2025/12/01 - 新增 GPS 回调函数
    void setGpsOrigin(double lat, double lon, double alt, const std::string &source)
    {
        m_state.origin_lat = lat;
        m_state.origin_lon = lon;
        m_state.origin_alt = alt;
        m_state.gps_origin_set = true;
        m_geo_converter = std::make_shared<GeographicLib::LocalCartesian>(lat, lon, alt);

        RCLCPP_INFO(this->get_logger(),
                    "GPS origin set from %s: lat=%.8f, lon=%.8f, alt=%.2f",
                    source.c_str(), lat, lon, alt);
    }

    void gpsCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);

        if (gps_msg->status.status < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GPS status invalid, skipping");
            return;
        }

        if (!std::isfinite(gps_msg->latitude) || !std::isfinite(gps_msg->longitude) ||
            !std::isfinite(gps_msg->altitude)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GPS sample contains non-finite values, skipping");
            return;
        }

        if (gps_msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GPS covariance type unknown, skipping");
            return;
        }

        const double horizontal_variance = std::max(gps_msg->position_covariance[0], gps_msg->position_covariance[4]);
        const double variance_limit = std::pow(m_node_config.gps_noise_xy * m_node_config.gps_quality_hdop_max, 2.0);
        if (!std::isfinite(horizontal_variance) || horizontal_variance > variance_limit) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GPS covariance too large (%.3f > %.3f), skipping",
                                 horizontal_variance, variance_limit);
            return;
        }

        // 设置原点（仅第一次）
        if (!m_state.gps_origin_set) {
            if (m_node_config.gps_origin_mode == "fixed") {
                setGpsOrigin(m_node_config.gps_origin_lat,
                             m_node_config.gps_origin_lon,
                             m_node_config.gps_origin_alt,
                             "fixed parameter");
            } else {
                setGpsOrigin(gps_msg->latitude,
                             gps_msg->longitude,
                             gps_msg->altitude,
                             "first valid GPS");
            }
        }

        // 缓存 GPS 数据
        m_state.gps_buffer.push(gps_msg);

        // 限制队列大小
        while (m_state.gps_buffer.size() > 50) {
            m_state.gps_buffer.pop();
        }

        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp
                      << ", GPS received: lat=" << gps_msg->latitude
                      << ", lon=" << gps_msg->longitude
                      << ", status=" << (int)gps_msg->status.status << std::endl;
            m_log_file.flush();
        }
    }
    // ✅ GPS 融合修改结束 - GPS 回调函数添加完成
    
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }
        m_state.last_message_time = cp.pose.second;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received synced cloud and odom: cloud_time=%.6f, points=%u",
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

    builtin_interfaces::msg::Time currentTimeMsg()
    {
        const int64_t now_ns = this->get_clock()->now().nanoseconds();
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = static_cast<int32_t>(now_ns / 1000000000LL);
        cur_time.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
        return cur_time;
    }

    void publishMapOdomTf(
        const Eigen::Quaterniond &q,
        const Eigen::Vector3d &t,
        const builtin_interfaces::msg::Time &time)
    {
        if (!m_node_config.publish_tf)
            return;

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        Eigen::Quaterniond q(m_pgo->offsetR());
        Eigen::Vector3d t = m_pgo->offsetT();
        publishMapOdomTf(q, t, time);
        m_state.last_tf_rotation = q;
        m_state.last_tf_translation = t;
        m_state.last_tf_valid = true;
    }

    void publishLastTfWithCurrentTime()
    {
        if (!m_state.last_tf_valid)
            return;
        builtin_interfaces::msg::Time cur_time = currentTimeMsg();
        publishMapOdomTf(m_state.last_tf_rotation, m_state.last_tf_translation, cur_time);
    }

    void publishOptimizedOdom(const CloudWithPose &cp, builtin_interfaces::msg::Time &time)
    {
        if (!m_optimized_odom_pub)
            return;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = m_node_config.map_frame;
        odom.child_frame_id = "base_link";

        Eigen::Matrix3d r_global = m_pgo->offsetR() * cp.pose.r;
        V3D t_global = m_pgo->offsetR() * cp.pose.t + m_pgo->offsetT();
        Eigen::Quaterniond q(r_global);
        q.normalize();

        odom.pose.pose.position.x = t_global.x();
        odom.pose.pose.position.y = t_global.y();
        odom.pose.pose.position.z = t_global.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        m_optimized_odom_pub->publish(odom);
    }

    void publishGlobalMap()
    {
        if (!m_global_map_pub || m_global_map_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->keyPoses().empty())
            return;

        CloudType::Ptr global_map(new CloudType);
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {
            const auto &key_pose = m_pgo->keyPoses()[i];
            if (!key_pose.body_cloud || key_pose.body_cloud->empty())
                continue;

            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(
                *key_pose.body_cloud,
                *world_cloud,
                key_pose.t_global,
                Eigen::Quaterniond(key_pose.r_global));
            *global_map += *world_cloud;
        }

        if (global_map->empty())
            return;

        if (m_node_config.global_map_resolution > 0.0)
        {
            pcl::VoxelGrid<PointType> voxel_grid;
            voxel_grid.setLeafSize(
                m_node_config.global_map_resolution,
                m_node_config.global_map_resolution,
                m_node_config.global_map_resolution);
            voxel_grid.setInputCloud(global_map);
            CloudType::Ptr filtered_map(new CloudType);
            voxel_grid.filter(*filtered_map);
            global_map = filtered_map;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*global_map, cloud_msg);
        cloud_msg.header.frame_id = m_node_config.map_frame;
        cloud_msg.header.stamp = this->now();
        m_global_map_pub->publish(cloud_msg);
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

    void updateAlignmentPairsFromGraph()
    {
        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        for (auto &pair : m_state.alignment_pairs)
        {
            if (pair.key_idx >= poses.size())
                continue;
            pair.map_xy = poses[pair.key_idx].t_global.head<2>();
        }
    }

    double computeAlignmentSpreadM() const
    {
        double max_spread = 0.0;
        for (size_t i = 0; i < m_state.alignment_pairs.size(); ++i)
        {
            for (size_t j = i + 1; j < m_state.alignment_pairs.size(); ++j)
            {
                const double spread =
                    (m_state.alignment_pairs[i].enu_xy - m_state.alignment_pairs[j].enu_xy).norm();
                max_spread = std::max(max_spread, spread);
            }
        }
        return max_spread;
    }

    bool solveAlignmentTransform(double &theta, double &tx, double &ty) const
    {
        if (m_state.alignment_pairs.size() < static_cast<size_t>(m_node_config.gps_alignment_min_points))
            return false;

        if (computeAlignmentSpreadM() < m_node_config.gps_alignment_min_spread_m)
            return false;

        Eigen::Vector2d enu_centroid = Eigen::Vector2d::Zero();
        Eigen::Vector2d map_centroid = Eigen::Vector2d::Zero();
        for (const auto &pair : m_state.alignment_pairs)
        {
            enu_centroid += pair.enu_xy;
            map_centroid += pair.map_xy;
        }
        enu_centroid /= static_cast<double>(m_state.alignment_pairs.size());
        map_centroid /= static_cast<double>(m_state.alignment_pairs.size());

        double sum_dot = 0.0;
        double sum_cross = 0.0;
        for (const auto &pair : m_state.alignment_pairs)
        {
            const Eigen::Vector2d enu_delta = pair.enu_xy - enu_centroid;
            const Eigen::Vector2d map_delta = pair.map_xy - map_centroid;
            sum_dot += enu_delta.dot(map_delta);
            sum_cross += enu_delta.x() * map_delta.y() - enu_delta.y() * map_delta.x();
        }

        if (!std::isfinite(sum_dot) || !std::isfinite(sum_cross))
            return false;
        if (std::abs(sum_dot) < 1e-9 && std::abs(sum_cross) < 1e-9)
            return false;

        theta = std::atan2(sum_cross, sum_dot);
        const Eigen::Rotation2Dd rotation(theta);
        const Eigen::Vector2d translation = map_centroid - rotation * enu_centroid;
        tx = translation.x();
        ty = translation.y();
        return std::isfinite(theta) && std::isfinite(tx) && std::isfinite(ty);
    }

    void recomputeAlignmentTransform()
    {
        updateAlignmentPairsFromGraph();

        const bool was_valid = m_state.alignment_valid;
        const double prev_theta = m_state.alignment_theta;
        const double prev_tx = m_state.alignment_tx;
        const double prev_ty = m_state.alignment_ty;

        double theta = 0.0;
        double tx = 0.0;
        double ty = 0.0;
        const bool solved = solveAlignmentTransform(theta, tx, ty);
        m_state.alignment_valid = solved;
        if (!solved)
            return;

        m_state.alignment_theta = theta;
        m_state.alignment_tx = tx;
        m_state.alignment_ty = ty;

        if (!was_valid) {
            RCLCPP_INFO(this->get_logger(),
                        "ENU->map alignment ready: pairs=%zu spread=%.2fm theta=%.2fdeg tx=%.2f ty=%.2f",
                        m_state.alignment_pairs.size(),
                        computeAlignmentSpreadM(),
                        theta * kRadToDeg,
                        tx,
                        ty);
        } else if (std::abs(theta - prev_theta) > (1.0 / kRadToDeg) ||
                   std::hypot(tx - prev_tx, ty - prev_ty) > 0.5) {
            RCLCPP_INFO(this->get_logger(),
                        "ENU->map alignment updated: theta=%.2fdeg tx=%.2f ty=%.2f",
                        theta * kRadToDeg,
                        tx,
                        ty);
        }
    }

    void publishAlignmentTransform()
    {
        if (!m_alignment_pub)
            return;

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {
            m_state.alignment_theta,
            m_state.alignment_tx,
            m_state.alignment_ty,
            m_state.alignment_valid ? 1.0 : 0.0,
        };
        m_alignment_pub->publish(msg);
    }

    // P2（FRC）：以 1 Hz 导出位姿图全部 keyframe（id + map 系全局位姿）。
    // 回环修正后历史位姿整体刷新，下游（memory_manager）据此让锚跟随位姿图移动。
    void publishKeyframes()
    {
        if (!m_keyframes_pub || m_keyframes_pub->get_subscription_count() == 0)
            return;
        const std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        if (poses.empty())
            return;

        frc_msgs::msg::KeyframeArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = m_node_config.map_frame;
        msg.session_id = m_session_id;
        msg.ids.reserve(poses.size());
        msg.poses.reserve(poses.size());
        for (size_t i = 0; i < poses.size(); ++i)
        {
            msg.ids.push_back(static_cast<int64_t>(i));
            geometry_msgs::msg::Pose pose;
            pose.position.x = poses[i].t_global.x();
            pose.position.y = poses[i].t_global.y();
            pose.position.z = poses[i].t_global.z();
            Eigen::Quaterniond q(poses[i].r_global);
            q.normalize();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            msg.poses.push_back(pose);
        }
        m_keyframes_pub->publish(msg);
    }

    // P2（FRC）：smoothAndUpdate 前后对比 offsetT，平移跳变超阈值则进入修正窗口。
    // 修正窗口内 correcting=1，供 health_aggregator 与离线归因过滤消费。
    void updateCorrectionStatus()
    {
        const Eigen::Vector3d offset_t = m_pgo->offsetT();
        const double now_s = this->get_clock()->now().seconds();

        if (!m_state.correction_offset_init)
        {
            m_state.last_offset_t = offset_t;
            m_state.correction_offset_init = true;
            return;
        }

        const double jump = (offset_t - m_state.last_offset_t).norm();
        m_state.last_offset_t = offset_t;
        if (jump > m_node_config.correction_jump_thresh)
        {
            m_state.last_jump_m = jump;
            m_state.correction_until = now_s + m_node_config.correction_window_s;
            RCLCPP_INFO(this->get_logger(),
                        "PGO correction detected: jump=%.3fm window=%.1fs",
                        jump, m_node_config.correction_window_s);
        }
    }

    void publishCorrectionStatus()
    {
        if (!m_correction_status_pub)
            return;
        const double now_s = this->get_clock()->now().seconds();
        const bool correcting = now_s < m_state.correction_until;
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            correcting ? 1.0f : 0.0f,
            static_cast<float>(m_state.last_jump_m),
            static_cast<float>(m_pgo->historyPairs().size()),
        };
        m_correction_status_pub->publish(msg);
    }

    void timerCB()
    {
        CloudWithPose cp;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (m_state.cloud_buffer.empty()) {
                publishLastTfWithCurrentTime();
                return;
            }
            cp = m_state.cloud_buffer.front();
            m_state.cloud_buffer.pop();  // 只弹出一个元素
        }
        
        // Publish TF/odom with the current ROS time so Nav2 can look up fresh transforms
        // against live controller / costmap requests instead of stale sensor timestamps.
        builtin_interfaces::msg::Time cur_time = currentTimeMsg();

        if (pgoVerboseDiagEnabled()) {
            fprintf(stderr, "[DIAG] timerCB: pts=%zu time=%.6f kp=%zu\n", cp.cloud ? cp.cloud->size() : (size_t)0, cp.pose.second, m_pgo->keyPoses().size());
        }
        bool is_key_pose = m_pgo->addKeyPose(cp);
        if (!is_key_pose)
        {
            sendBroadCastTF(cur_time);
            publishOptimizedOdom(cp, cur_time);
            publishAlignmentTransform();
            publishCorrectionStatus();
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

        // ✅ GPS 融合修改开始 - 2025/12/01 - 在关键帧添加后尝试添加 GPS 因子
        if (m_node_config.enable_gps && m_state.gps_origin_set) {
            if (pgoVerboseDiagEnabled()) {
                fprintf(stderr, "[DIAG] calling tryAddGPSFactor\n");
            }
            tryAddGPSFactor(cp.pose.second);
            if (pgoVerboseDiagEnabled()) {
                fprintf(stderr, "[DIAG] tryAddGPSFactor done\n");
            }
        }
        // ✅ GPS 融合修改结束 - GPS 因子触发逻辑添加完成

        if (pgoVerboseDiagEnabled()) {
            fprintf(stderr, "[DIAG] calling searchForLoopPairs, key_poses=%zu\n", m_pgo->keyPoses().size());
        }
        size_t loop_pairs_before = m_pgo->historyPairs().size();
        m_pgo->searchForLoopPairs();
        size_t loop_pairs_after = m_pgo->historyPairs().size();
        if (pgoVerboseDiagEnabled()) {
            fprintf(stderr, "[DIAG] searchForLoopPairs done\n");
        }
        
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

        if (pgoVerboseDiagEnabled()) {
            fprintf(stderr, "[DIAG] calling smoothAndUpdate\n");
        }
        m_pgo->smoothAndUpdate();
        if (pgoVerboseDiagEnabled()) {
            fprintf(stderr, "[DIAG] smoothAndUpdate done\n");
        }
        updateCorrectionStatus();
        if (m_node_config.enable_gps) {
            recomputeAlignmentTransform();
        }

        if (pgoVerboseDiagEnabled()) {
            fprintf(stderr, "[DIAG] calling sendBroadCastTF + publish\n");
        }
        sendBroadCastTF(cur_time);
        publishOptimizedOdom(cp, cur_time);
        publishLoopMarkers(cur_time);
        publishAlignmentTransform();
        publishCorrectionStatus();
        if (pgoVerboseDiagEnabled()) {
            fprintf(stderr, "[DIAG] timerCB complete for key pose\n");
        }
    }

    // ✅ GPS 融合修改开始 - 2025/12/01 - 新增 tryAddGPSFactor 函数
    void tryAddGPSFactor(double keyframe_time)
    {
        if (!m_geo_converter) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GPS converter not ready, skip GPS factor");
            return;
        }

        // 检查是否该添加 GPS 因子（按间隔）
        size_t current_idx = m_pgo->keyPoses().size() - 1;
        if (current_idx % m_node_config.gps_factor_interval != 0) {
            return;
        }
        
        // 查找时间最接近的 GPS 数据
        sensor_msgs::msg::NavSatFix::ConstSharedPtr closest_gps = nullptr;
        double min_time_diff = 1.0;  // 最大允许 1 秒时差
        
        std::queue<sensor_msgs::msg::NavSatFix::ConstSharedPtr> temp_queue = m_state.gps_buffer;
        while (!temp_queue.empty()) {
            auto gps = temp_queue.front();
            temp_queue.pop();
            
            double gps_time = gps->header.stamp.sec + gps->header.stamp.nanosec * 1e-9;
            double time_diff = std::abs(gps_time - keyframe_time);
            
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_gps = gps;
            }
        }
        
        if (!closest_gps) {
            RCLCPP_WARN(this->get_logger(), "No matching GPS data found for keyframe %zu", current_idx);
            return;
        }
        
        // GPS 坐标转换到 ENU
        double x, y, z;
        m_geo_converter->Forward(closest_gps->latitude, closest_gps->longitude, 
                                 closest_gps->altitude, x, y, z);

        const Eigen::Vector2d enu_xy(x, y);
        const Eigen::Vector2d map_xy = m_pgo->keyPoses()[current_idx].t_global.head<2>();
        auto existing_pair = std::find_if(
            m_state.alignment_pairs.begin(), m_state.alignment_pairs.end(),
            [current_idx](const AlignmentPair &pair) { return pair.key_idx == current_idx; });
        if (existing_pair == m_state.alignment_pairs.end()) {
            m_state.alignment_pairs.push_back({current_idx, enu_xy, map_xy});
        } else {
            existing_pair->enu_xy = enu_xy;
            existing_pair->map_xy = map_xy;
        }

        recomputeAlignmentTransform();
        if (!m_state.alignment_valid) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for ENU->map alignment: pairs=%zu/%d spread=%.2fm/%.2fm",
                                 m_state.alignment_pairs.size(),
                                 m_node_config.gps_alignment_min_points,
                                 computeAlignmentSpreadM(),
                                 m_node_config.gps_alignment_min_spread_m);
            return;
        }

        const Eigen::Rotation2Dd rotation(m_state.alignment_theta);
        const Eigen::Vector2d aligned_map_xy =
            rotation * enu_xy + Eigen::Vector2d(m_state.alignment_tx, m_state.alignment_ty);

        double sigma_override = -1.0;
        if (m_state.gps_factors_added < m_node_config.gps_alignment_warmup_factors) {
            sigma_override = m_node_config.gps_alignment_warmup_sigma;
        }

        // 调用 SimplePGO 的接口添加 GPS 因子
        m_pgo->addGPSFactor(current_idx, V3D(aligned_map_xy.x(), aligned_map_xy.y(), z),
                            closest_gps->position_covariance,
                            m_node_config.gps_noise_xy,
                            m_node_config.gps_noise_z,
                            sigma_override);
        m_state.gps_factors_added += 1;

        RCLCPP_INFO(this->get_logger(),
                    "GPS factor added: idx=%zu, ENU=(%.2f, %.2f, %.2f), map=(%.2f, %.2f), time_diff=%.3f, warmup_sigma=%.2f",
                    current_idx, x, y, z,
                    aligned_map_xy.x(), aligned_map_xy.y(),
                    min_time_diff,
                    sigma_override);
        
        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", GPS factor added: idx=" << current_idx 
                      << ", ENU=(" << x << ", " << y << ", " << z << ")" << std::endl;
            m_log_file.flush();
        }
    }
    // ✅ GPS 融合修改结束 - tryAddGPSFactor 函数添加完成

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
    rclcpp::TimerBase::SharedPtr m_global_map_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_optimized_odom_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_alignment_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    // P2（FRC）成员
    rclcpp::Publisher<frc_msgs::msg::KeyframeArray>::SharedPtr m_keyframes_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_correction_status_pub;
    rclcpp::TimerBase::SharedPtr m_keyframes_timer;
    std::string m_session_id;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::ofstream m_log_file;
    
    // ✅ GPS 融合修改开始 - 2025/12/01 - 添加 GPS 成员变量
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_sub;
    std::shared_ptr<GeographicLib::LocalCartesian> m_geo_converter;
    // ✅ GPS 融合修改结束 - GPS 成员变量添加完成
};


static void crash_handler(int sig) {
    void *bt[50];
    int n = backtrace(bt, 50);
    fprintf(stderr, "\n=== PGO_NODE CRASH sig=%d ===\n", sig);
    backtrace_symbols_fd(bt, n, STDERR_FILENO);
    fprintf(stderr, "=== END BACKTRACE ===\n");
    fflush(stderr);
    _exit(128 + sig);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGSEGV, crash_handler);
    signal(SIGABRT, crash_handler);
    signal(SIGFPE, crash_handler);
    signal(SIGBUS, crash_handler);
    if (pgoVerboseDiagEnabled()) {
        fprintf(stderr, "[DIAG] PGO crash handler installed\n");
    }
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}


// 文件最新改动时间：2025/11/13 -- 20:15
// 文件最新改动人：Claude Sonnet 4.5
// 操作者：You-guesssssss

#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
}

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "base_link";
    std::string world_frame = "odom";
    bool print_time_cost = false;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node 文件运行成功");
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");

        bool enable_log = shouldEnableLogging("fastlio2_lio_node");
        
        if (enable_log) {
            std::string log_path = getSessionLogPath("fastlio2.log", "logs/fastlio2");
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

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 500);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 500);
        m_localization_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud_localization", 50);
        m_nav2_obstacle_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud_nav2_obstacles", 50);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        // P1（FRC）：退化度量 [min_eig, cond, regularized]，与 odom 同节奏发布。
        // 刻意保持 Float32MultiArray 而非 frc_msgs/Health，避免 fastlio2 依赖 frc_msgs；
        // 语义翻译由 frc_health_aggregator 负责。
        m_degeneracy_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("degeneracy", 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);

        // fastlio2 消息发布的频率更改处
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));
    }

    ~LIONode()
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
        const std::vector<double> default_t_il = {-0.011, -0.02329, 0.04412};
        const std::vector<double> default_r_il = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        const std::string config_path = this->declare_parameter<std::string>("config_path", "");

        m_node_config.imu_topic = this->declare_parameter<std::string>("imu_topic", "/livox/imu");
        m_node_config.lidar_topic = this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        m_node_config.body_frame = this->declare_parameter<std::string>("body_frame", "base_link");
        m_node_config.world_frame = this->declare_parameter<std::string>("world_frame", "odom");
        m_node_config.print_time_cost = this->declare_parameter<bool>("print_time_cost", false);

        m_builder_config.lidar_filter_num = this->declare_parameter<int>("lidar_filter_num", 6);
        m_builder_config.lidar_min_range = this->declare_parameter<double>("lidar_min_range", 0.5);
        m_builder_config.lidar_max_range = this->declare_parameter<double>("lidar_max_range", 15.0);
        m_builder_config.scan_resolution = this->declare_parameter<double>("scan_resolution", 0.15);
        m_builder_config.map_resolution = this->declare_parameter<double>("map_resolution", 0.3);
        m_builder_config.cube_len = this->declare_parameter<double>("cube_len", 300.0);
        m_builder_config.det_range = this->declare_parameter<double>("det_range", 60.0);
        m_builder_config.move_thresh = this->declare_parameter<double>("move_thresh", 1.5);
        m_builder_config.na = this->declare_parameter<double>("na", 0.01);
        m_builder_config.ng = this->declare_parameter<double>("ng", 0.01);
        m_builder_config.nba = this->declare_parameter<double>("nba", 0.0001);
        m_builder_config.nbg = this->declare_parameter<double>("nbg", 0.0001);
        m_builder_config.imu_init_num = this->declare_parameter<int>("imu_init_num", 20);
        m_builder_config.min_imu_samples_per_lidar =
            this->declare_parameter<int>("min_imu_samples_per_lidar", 3);
        m_builder_config.near_search_num = this->declare_parameter<int>("near_search_num", 5);
        m_builder_config.ieskf_max_iter = this->declare_parameter<int>("ieskf_max_iter", 5);
        m_builder_config.gravity_align = this->declare_parameter<bool>("gravity_align", true);
        m_builder_config.esti_il = this->declare_parameter<bool>("esti_il", false);
        m_builder_config.lidar_cov_inv = this->declare_parameter<double>("lidar_cov_inv", 1000.0);
        m_builder_config.publish_cloud_height_filter_enabled =
            this->declare_parameter<bool>("publish_cloud_height_filter_enabled", false);
        m_builder_config.publish_cloud_min_z =
            this->declare_parameter<double>("publish_cloud_min_z", -0.33);
        m_builder_config.publish_cloud_max_z =
            this->declare_parameter<double>("publish_cloud_max_z", 0.30);
        m_builder_config.localization_cloud_enabled =
            this->declare_parameter<bool>("localization_cloud_enabled", true);
        m_builder_config.localization_cloud_min_z =
            this->declare_parameter<double>("localization_cloud_min_z", -0.20);
        m_builder_config.localization_cloud_max_z =
            this->declare_parameter<double>("localization_cloud_max_z", 1.80);
        m_builder_config.nav2_obstacle_cloud_enabled =
            this->declare_parameter<bool>("nav2_obstacle_cloud_enabled", true);
        m_builder_config.nav2_obstacle_cloud_min_z =
            this->declare_parameter<double>("nav2_obstacle_cloud_min_z", -0.20);
        m_builder_config.nav2_obstacle_cloud_max_z =
            this->declare_parameter<double>("nav2_obstacle_cloud_max_z", 1.20);
        m_builder_config.nav2_obstacle_self_filter_enabled =
            this->declare_parameter<bool>("nav2_obstacle_self_filter_enabled", false);
        m_builder_config.nav2_obstacle_self_filter_min_x =
            this->declare_parameter<double>("nav2_obstacle_self_filter_min_x", -0.40);
        m_builder_config.nav2_obstacle_self_filter_max_x =
            this->declare_parameter<double>("nav2_obstacle_self_filter_max_x", 0.40);
        m_builder_config.nav2_obstacle_self_filter_min_y =
            this->declare_parameter<double>("nav2_obstacle_self_filter_min_y", -0.30);
        m_builder_config.nav2_obstacle_self_filter_max_y =
            this->declare_parameter<double>("nav2_obstacle_self_filter_max_y", 0.30);

        auto t_il_vec = this->declare_parameter<std::vector<double>>("t_il", default_t_il);
        auto r_il_vec = this->declare_parameter<std::vector<double>>("r_il", default_r_il);
        applyExtrinsics(t_il_vec, r_il_vec);

        if (!config_path.empty())
        {
            loadLegacyConfig(config_path);
            syncParametersToRos();
        }
    }

    void applyExtrinsics(const std::vector<double> &t_il_vec, const std::vector<double> &r_il_vec)
    {
        if (t_il_vec.size() != 3)
        {
            throw std::runtime_error("FAST-LIO2 parameter t_il must contain exactly 3 values");
        }
        if (r_il_vec.size() != 9)
        {
            throw std::runtime_error("FAST-LIO2 parameter r_il must contain exactly 9 values");
        }

        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2],
                                 r_il_vec[3], r_il_vec[4], r_il_vec[5],
                                 r_il_vec[6], r_il_vec[7], r_il_vec[8];
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

        if (config["imu_topic"])
            m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        if (config["lidar_topic"])
            m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        if (config["body_frame"])
            m_node_config.body_frame = config["body_frame"].as<std::string>();
        if (config["world_frame"])
            m_node_config.world_frame = config["world_frame"].as<std::string>();
        if (config["print_time_cost"])
            m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        if (config["lidar_filter_num"])
            m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        if (config["lidar_min_range"])
            m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        if (config["lidar_max_range"])
            m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        if (config["scan_resolution"])
            m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        if (config["map_resolution"])
            m_builder_config.map_resolution = config["map_resolution"].as<double>();
        if (config["cube_len"])
            m_builder_config.cube_len = config["cube_len"].as<double>();
        if (config["det_range"])
            m_builder_config.det_range = config["det_range"].as<double>();
        if (config["move_thresh"])
            m_builder_config.move_thresh = config["move_thresh"].as<double>();
        if (config["na"])
            m_builder_config.na = config["na"].as<double>();
        if (config["ng"])
            m_builder_config.ng = config["ng"].as<double>();
        if (config["nba"])
            m_builder_config.nba = config["nba"].as<double>();
        if (config["nbg"])
            m_builder_config.nbg = config["nbg"].as<double>();
        if (config["imu_init_num"])
            m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        if (config["min_imu_samples_per_lidar"])
            m_builder_config.min_imu_samples_per_lidar =
                config["min_imu_samples_per_lidar"].as<int>();
        if (config["near_search_num"])
            m_builder_config.near_search_num = config["near_search_num"].as<int>();
        if (config["ieskf_max_iter"])
            m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        if (config["gravity_align"])
            m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        if (config["esti_il"])
            m_builder_config.esti_il = config["esti_il"].as<bool>();
        if (config["lidar_cov_inv"])
            m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
        if (config["publish_cloud_height_filter_enabled"])
            m_builder_config.publish_cloud_height_filter_enabled =
                config["publish_cloud_height_filter_enabled"].as<bool>();
        if (config["publish_cloud_min_z"])
            m_builder_config.publish_cloud_min_z =
                config["publish_cloud_min_z"].as<double>();
        if (config["publish_cloud_max_z"])
            m_builder_config.publish_cloud_max_z =
                config["publish_cloud_max_z"].as<double>();
        if (config["localization_cloud_enabled"])
            m_builder_config.localization_cloud_enabled =
                config["localization_cloud_enabled"].as<bool>();
        if (config["localization_cloud_min_z"])
            m_builder_config.localization_cloud_min_z =
                config["localization_cloud_min_z"].as<double>();
        if (config["localization_cloud_max_z"])
            m_builder_config.localization_cloud_max_z =
                config["localization_cloud_max_z"].as<double>();
        if (config["nav2_obstacle_cloud_enabled"])
            m_builder_config.nav2_obstacle_cloud_enabled =
                config["nav2_obstacle_cloud_enabled"].as<bool>();
        if (config["nav2_obstacle_cloud_min_z"])
            m_builder_config.nav2_obstacle_cloud_min_z =
                config["nav2_obstacle_cloud_min_z"].as<double>();
        if (config["nav2_obstacle_cloud_max_z"])
            m_builder_config.nav2_obstacle_cloud_max_z =
                config["nav2_obstacle_cloud_max_z"].as<double>();
        if (config["nav2_obstacle_self_filter_enabled"])
            m_builder_config.nav2_obstacle_self_filter_enabled =
                config["nav2_obstacle_self_filter_enabled"].as<bool>();
        if (config["nav2_obstacle_self_filter_min_x"])
            m_builder_config.nav2_obstacle_self_filter_min_x =
                config["nav2_obstacle_self_filter_min_x"].as<double>();
        if (config["nav2_obstacle_self_filter_max_x"])
            m_builder_config.nav2_obstacle_self_filter_max_x =
                config["nav2_obstacle_self_filter_max_x"].as<double>();
        if (config["nav2_obstacle_self_filter_min_y"])
            m_builder_config.nav2_obstacle_self_filter_min_y =
                config["nav2_obstacle_self_filter_min_y"].as<double>();
        if (config["nav2_obstacle_self_filter_max_y"])
            m_builder_config.nav2_obstacle_self_filter_max_y =
                config["nav2_obstacle_self_filter_max_y"].as<double>();

        const std::vector<double> t_il_vec =
            config["t_il"] ? config["t_il"].as<std::vector<double>>() : std::vector<double>{m_builder_config.t_il.x(), m_builder_config.t_il.y(), m_builder_config.t_il.z()};
        const std::vector<double> r_il_vec =
            config["r_il"] ? config["r_il"].as<std::vector<double>>() : std::vector<double>{
                m_builder_config.r_il(0, 0), m_builder_config.r_il(0, 1), m_builder_config.r_il(0, 2),
                m_builder_config.r_il(1, 0), m_builder_config.r_il(1, 1), m_builder_config.r_il(1, 2),
                m_builder_config.r_il(2, 0), m_builder_config.r_il(2, 1), m_builder_config.r_il(2, 2)};
        applyExtrinsics(t_il_vec, r_il_vec);
    }

    void syncParametersToRos()
    {
        const std::vector<double> t_il = {
            m_builder_config.t_il.x(), m_builder_config.t_il.y(), m_builder_config.t_il.z()};
        const std::vector<double> r_il = {
            m_builder_config.r_il(0, 0), m_builder_config.r_il(0, 1), m_builder_config.r_il(0, 2),
            m_builder_config.r_il(1, 0), m_builder_config.r_il(1, 1), m_builder_config.r_il(1, 2),
            m_builder_config.r_il(2, 0), m_builder_config.r_il(2, 1), m_builder_config.r_il(2, 2)};

        this->set_parameters({
            rclcpp::Parameter("imu_topic", m_node_config.imu_topic),
            rclcpp::Parameter("lidar_topic", m_node_config.lidar_topic),
            rclcpp::Parameter("body_frame", m_node_config.body_frame),
            rclcpp::Parameter("world_frame", m_node_config.world_frame),
            rclcpp::Parameter("print_time_cost", m_node_config.print_time_cost),
            rclcpp::Parameter("lidar_filter_num", m_builder_config.lidar_filter_num),
            rclcpp::Parameter("lidar_min_range", m_builder_config.lidar_min_range),
            rclcpp::Parameter("lidar_max_range", m_builder_config.lidar_max_range),
            rclcpp::Parameter("scan_resolution", m_builder_config.scan_resolution),
            rclcpp::Parameter("map_resolution", m_builder_config.map_resolution),
            rclcpp::Parameter("cube_len", m_builder_config.cube_len),
            rclcpp::Parameter("det_range", m_builder_config.det_range),
            rclcpp::Parameter("move_thresh", m_builder_config.move_thresh),
            rclcpp::Parameter("na", m_builder_config.na),
            rclcpp::Parameter("ng", m_builder_config.ng),
            rclcpp::Parameter("nba", m_builder_config.nba),
            rclcpp::Parameter("nbg", m_builder_config.nbg),
            rclcpp::Parameter("imu_init_num", m_builder_config.imu_init_num),
            rclcpp::Parameter("min_imu_samples_per_lidar", m_builder_config.min_imu_samples_per_lidar),
            rclcpp::Parameter("near_search_num", m_builder_config.near_search_num),
            rclcpp::Parameter("ieskf_max_iter", m_builder_config.ieskf_max_iter),
            rclcpp::Parameter("gravity_align", m_builder_config.gravity_align),
            rclcpp::Parameter("esti_il", m_builder_config.esti_il),
            rclcpp::Parameter("t_il", t_il),
            rclcpp::Parameter("r_il", r_il),
            rclcpp::Parameter("lidar_cov_inv", m_builder_config.lidar_cov_inv),
            rclcpp::Parameter("publish_cloud_height_filter_enabled", m_builder_config.publish_cloud_height_filter_enabled),
            rclcpp::Parameter("publish_cloud_min_z", m_builder_config.publish_cloud_min_z),
            rclcpp::Parameter("publish_cloud_max_z", m_builder_config.publish_cloud_max_z),
            rclcpp::Parameter("localization_cloud_enabled", m_builder_config.localization_cloud_enabled),
            rclcpp::Parameter("localization_cloud_min_z", m_builder_config.localization_cloud_min_z),
            rclcpp::Parameter("localization_cloud_max_z", m_builder_config.localization_cloud_max_z),
            rclcpp::Parameter("nav2_obstacle_cloud_enabled", m_builder_config.nav2_obstacle_cloud_enabled),
            rclcpp::Parameter("nav2_obstacle_cloud_min_z", m_builder_config.nav2_obstacle_cloud_min_z),
            rclcpp::Parameter("nav2_obstacle_cloud_max_z", m_builder_config.nav2_obstacle_cloud_max_z),
            rclcpp::Parameter("nav2_obstacle_self_filter_enabled", m_builder_config.nav2_obstacle_self_filter_enabled),
            rclcpp::Parameter("nav2_obstacle_self_filter_min_x", m_builder_config.nav2_obstacle_self_filter_min_x),
            rclcpp::Parameter("nav2_obstacle_self_filter_max_x", m_builder_config.nav2_obstacle_self_filter_max_x),
            rclcpp::Parameter("nav2_obstacle_self_filter_min_y", m_builder_config.nav2_obstacle_self_filter_min_y),
            rclcpp::Parameter("nav2_obstacle_self_filter_max_y", m_builder_config.nav2_obstacle_self_filter_max_y),
        });
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Received IMU data: accel(%.2f, %.2f, %.2f), gyro(%.2f, %.2f, %.2f), time: %.6f",
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, timestamp);
        
        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", Received IMU data: accel(" 
                      << msg->linear_acceleration.x << ", " << msg->linear_acceleration.y << ", " << msg->linear_acceleration.z
                      << "), gyro(" 
                      << msg->angular_velocity.x << ", " << msg->angular_velocity.y << ", " << msg->angular_velocity.z
                      << "), time: " << timestamp << std::endl;
            m_log_file.flush();
        }
        
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Received LIDAR data: %zu points, time: %.6f",
            cloud->size(),
            Utils::getSec(msg->header));
        
        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", Received LIDAR data: " << cloud->size() 
                      << " points, time: " << Utils::getSec(msg->header) << std::endl;
            m_log_file.flush();
        }
        
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            if (m_package.cloud->points.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Empty point cloud, dropping frame");
                m_state_data.lidar_buffer.pop_front();
                return false;
            }
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    CloudType::Ptr filterBodyCloudByRelativeHeight(
        const CloudType::Ptr &body_cloud,
        const CloudType::Ptr &world_cloud,
        double min_z,
        double max_z,
        const std::string &label)
    {
        if (!body_cloud || !world_cloud)
            return body_cloud;

        if (body_cloud->size() != world_cloud->size())
        {
            RCLCPP_WARN(this->get_logger(),
                        "%s height filter skipped because body/world cloud sizes differ: %zu vs %zu",
                        label.c_str(),
                        body_cloud->size(),
                        world_cloud->size());
            return body_cloud;
        }

        CloudType::Ptr filtered_body_cloud(new CloudType);
        filtered_body_cloud->reserve(body_cloud->size());

        const double body_origin_z_in_world = m_kf->x().t_wi.z();
        std::size_t dropped_points = 0;

        for (std::size_t i = 0; i < body_cloud->size(); ++i)
        {
            const double relative_height = world_cloud->points[i].z - body_origin_z_in_world;
            if (relative_height < min_z || relative_height > max_z)
            {
                dropped_points++;
                continue;
            }

            filtered_body_cloud->points.push_back(body_cloud->points[i]);
        }

        filtered_body_cloud->width = filtered_body_cloud->points.size();
        filtered_body_cloud->height = 1;
        filtered_body_cloud->is_dense = false;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "%s height filter kept %zu/%zu points with relative z window [%.2f, %.2f]",
            label.c_str(),
            filtered_body_cloud->points.size(),
            body_cloud->points.size(),
            min_z,
            max_z);

        if (filtered_body_cloud->points.empty() && dropped_points > 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "%s height filter dropped all %zu points",
                        label.c_str(),
                        dropped_points);
        }

        return filtered_body_cloud;
    }

    CloudType::Ptr filterNav2VehicleReturns(const CloudType::Ptr &body_cloud)
    {
        if (!body_cloud || !m_builder_config.nav2_obstacle_self_filter_enabled)
            return body_cloud;

        CloudType::Ptr filtered_cloud(new CloudType);
        filtered_cloud->reserve(body_cloud->size());
        std::size_t dropped_points = 0;
        for (const auto &point : body_cloud->points)
        {
            const bool inside_vehicle =
                point.x >= m_builder_config.nav2_obstacle_self_filter_min_x &&
                point.x <= m_builder_config.nav2_obstacle_self_filter_max_x &&
                point.y >= m_builder_config.nav2_obstacle_self_filter_min_y &&
                point.y <= m_builder_config.nav2_obstacle_self_filter_max_y;
            if (inside_vehicle)
            {
                ++dropped_points;
                continue;
            }
            filtered_cloud->points.push_back(point);
        }
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = false;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "FAST-LIO2 Nav2 vehicle filter removed %zu/%zu points in x=[%.2f, %.2f], y=[%.2f, %.2f]",
            dropped_points,
            body_cloud->size(),
            m_builder_config.nav2_obstacle_self_filter_min_x,
            m_builder_config.nav2_obstacle_self_filter_max_x,
            m_builder_config.nav2_obstacle_self_filter_min_y,
            m_builder_config.nav2_obstacle_self_filter_max_y);
        return filtered_cloud;
    }

    void publishNav2ObstacleCloud(
        const CloudType::Ptr &body_cloud,
        const CloudType::Ptr &world_cloud,
        const double &time)
    {
        if (!m_builder_config.nav2_obstacle_cloud_enabled)
            return;
        if (m_nav2_obstacle_cloud_pub->get_subscription_count() <= 0)
            return;

        CloudType::Ptr nav2_body_cloud = filterBodyCloudByRelativeHeight(
            body_cloud,
            world_cloud,
            m_builder_config.nav2_obstacle_cloud_min_z,
            m_builder_config.nav2_obstacle_cloud_max_z,
            "FAST-LIO2 Nav2 obstacle cloud");
        nav2_body_cloud = filterNav2VehicleReturns(nav2_body_cloud);
        publishCloud(m_nav2_obstacle_cloud_pub, nav2_body_cloud, m_node_config.body_frame, time);
    }

    void publishLocalizationCloud(
        const CloudType::Ptr &body_cloud,
        const CloudType::Ptr &world_cloud,
        const double &time)
    {
        if (!m_builder_config.localization_cloud_enabled)
            return;
        if (m_localization_cloud_pub->get_subscription_count() <= 0)
            return;

        CloudType::Ptr localization_cloud = filterBodyCloudByRelativeHeight(
            body_cloud,
            world_cloud,
            m_builder_config.localization_cloud_min_z,
            m_builder_config.localization_cloud_max_z,
            "FAST-LIO2 localization cloud");
        publishCloud(m_localization_cloud_pub, localization_cloud, m_node_config.body_frame, time);
    }

    std::pair<CloudType::Ptr, CloudType::Ptr> filterPublishedClouds(
        const CloudType::Ptr &body_cloud,
        const CloudType::Ptr &world_cloud)
    {
        if (!m_builder_config.publish_cloud_height_filter_enabled)
            return {body_cloud, world_cloud};

        if (!body_cloud || !world_cloud)
            return {body_cloud, world_cloud};

        if (body_cloud->size() != world_cloud->size())
        {
            RCLCPP_WARN(this->get_logger(),
                        "Published cloud filter skipped because body/world cloud sizes differ: %zu vs %zu",
                        body_cloud->size(), world_cloud->size());
            return {body_cloud, world_cloud};
        }

        CloudType::Ptr filtered_body_cloud(new CloudType);
        CloudType::Ptr filtered_world_cloud(new CloudType);
        filtered_body_cloud->reserve(body_cloud->size());
        filtered_world_cloud->reserve(world_cloud->size());

        const double body_origin_z_in_world = m_kf->x().t_wi.z();
        std::size_t dropped_points = 0;

        for (std::size_t i = 0; i < body_cloud->size(); ++i)
        {
            const double relative_height = world_cloud->points[i].z - body_origin_z_in_world;
            if (relative_height < m_builder_config.publish_cloud_min_z ||
                relative_height > m_builder_config.publish_cloud_max_z)
            {
                dropped_points++;
                continue;
            }

            filtered_body_cloud->points.push_back(body_cloud->points[i]);
            filtered_world_cloud->points.push_back(world_cloud->points[i]);
        }

        filtered_body_cloud->width = filtered_body_cloud->points.size();
        filtered_body_cloud->height = 1;
        filtered_body_cloud->is_dense = false;
        filtered_world_cloud->width = filtered_world_cloud->points.size();
        filtered_world_cloud->height = 1;
        filtered_world_cloud->is_dense = false;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "FAST-LIO2 publish cloud height filter kept %zu/%zu points with relative z window [%.2f, %.2f]",
            filtered_body_cloud->points.size(),
            body_cloud->points.size(),
            m_builder_config.publish_cloud_min_z,
            m_builder_config.publish_cloud_max_z);

        if (filtered_body_cloud->points.empty() && dropped_points > 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "FAST-LIO2 publish cloud height filter dropped all %zu points",
                        dropped_points);
        }

        return {filtered_body_cloud, filtered_world_cloud};
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    // P1（FRC）：发布 IESKF 量测块退化度量，供 health_aggregator/事件归因消费。
    void publishDegeneracy()
    {
        if (m_degeneracy_pub->get_subscription_count() <= 0)
            return;
        const DegeneracyMetrics &metrics = m_kf->degeneracy();
        if (!metrics.valid)
            return;
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            static_cast<float>(metrics.min_eig),
            static_cast<float>(metrics.cond),
            metrics.regularized ? 1.0f : 0.0f,
        };
        m_degeneracy_pub->publish(msg);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;

        if (m_package.imus.size() < static_cast<size_t>(m_builder_config.min_imu_samples_per_lidar))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Dropping LIDAR package with only %zu IMU samples (minimum %d)",
                m_package.imus.size(),
                m_builder_config.min_imu_samples_per_lidar);
            return;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Processing sync package: %zu IMU samples, %zu LIDAR points",
            m_package.imus.size(),
            m_package.cloud->size());
        
        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", Processing sync package: " << m_package.imus.size() 
                      << " IMU samples, " << m_package.cloud->size() << " LIDAR points" << std::endl;
            m_log_file.flush();
        }
        
        auto t1 = std::chrono::high_resolution_clock::now();
        bool process_accepted = m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (!process_accepted)
        {
            if (m_builder->status() == BuilderStatus::MAPPING)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "FAST-LIO2 rejected package without valid LiDAR correction; not publishing TF/odom");
            }
            return;
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Current position: x=%.2f, y=%.2f, z=%.2f",
            m_kf->x().t_wi.x(), m_kf->x().t_wi.y(), m_kf->x().t_wi.z());
        
        if (m_log_file.is_open()) {
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();
            m_log_file << "ROS_timestamp: " << ros_timestamp 
                      << ", Current position: x=" << m_kf->x().t_wi.x() 
                      << ", y=" << m_kf->x().t_wi.y() 
                      << ", z=" << m_kf->x().t_wi.z() << std::endl;
            m_log_file.flush();
        }

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, this->now().seconds());

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, this->now().seconds());

        publishDegeneracy();

        CloudType::Ptr body_cloud =
            m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
        CloudType::Ptr world_cloud =
            m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        auto [filtered_body_cloud, filtered_world_cloud] =
            filterPublishedClouds(body_cloud, world_cloud);

        const double cloud_publish_time = this->now().seconds();
        publishLocalizationCloud(body_cloud, world_cloud, cloud_publish_time);
        publishNav2ObstacleCloud(body_cloud, world_cloud, cloud_publish_time);
        publishCloud(m_body_cloud_pub, filtered_body_cloud, m_node_config.body_frame, cloud_publish_time);
        publishCloud(m_world_cloud_pub, filtered_world_cloud, m_node_config.world_frame, cloud_publish_time);

        publishPath(m_path_pub, m_node_config.world_frame, this->now().seconds());
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_localization_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_nav2_obstacle_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_degeneracy_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::ofstream m_log_file;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<LIONode>());
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("lio_node"), "Node 文件运行出错 %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include "interface/srv/global_relocalize.hpp"
#include "interface/msg/localization_status.hpp"
#include "localizers/scan_context_index.h"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud_localization";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    std::string pcd_map;
    std::string map_id;
    std::string alignment_file;
    std::string descriptor_index;
    double update_hz = 1.0;
    double max_tf_input_age_s = 2.0;
    double tf_republish_hz = 20.0;
    double status_hz = 5.0;
    bool continuous_icp = true;
    bool auto_global_localization = true;
    int global_max_candidates = 5;
    double global_retry_interval_s = 3.0;
    int global_max_automatic_attempts = 5;
    int global_min_cloud_points = 200;
    double min_candidate_score_gap = 0.03;
    double min_overlap_ratio = 0.35;
    double correction_alpha = 0.15;
    double max_correction_translation_m = 0.30;
    double max_correction_yaw_rad = 0.20;
    int max_consecutive_failures = 5;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    bool has_published_tf = false;
    rclcpp::Time last_send_tf_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_republish_tf_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_status_publish_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    builtin_interfaces::msg::Time last_message_time;
    builtin_interfaces::msg::Time last_tf_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    uint8_t localization_state = interface::msg::LocalizationStatus::UNINITIALIZED;
    std::string state_reason = "startup";
    int consecutive_failures = 0;
    int global_attempts = 0;
    rclcpp::Time last_global_attempt_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    double rough_score = std::numeric_limits<double>::infinity();
    double refine_score = std::numeric_limits<double>::infinity();
    double overlap_ratio = 0.0;
    double correction_translation_m = 0.0;
    double correction_yaw_rad = 0.0;
    double correction_translation_rate_mps = 0.0;
    double correction_yaw_rate_rps = 0.0;
    double candidate_score_gap = 0.0;
    builtin_interfaces::msg::Time last_trusted_stamp;
    rclcpp::Time last_correction_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);
        loadMapAlignment();
        loadDescriptorIndex();
        loadStartupMap();

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_global_reloc_srv = this->create_service<interface::srv::GlobalRelocalize>(
            "global_relocalize",
            std::bind(&LocalizerNode::globalRelocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
        m_status_pub = this->create_publisher<interface::msg::LocalizationStatus>("status", 10);

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        m_config.pcd_map = this->declare_parameter<std::string>("pcd_map", "");
        m_config.map_id = this->declare_parameter<std::string>("map_id", "");
        m_config.alignment_file = this->declare_parameter<std::string>("alignment_file", "");
        m_config.descriptor_index = this->declare_parameter<std::string>("descriptor_index", "");
        m_config.auto_global_localization =
            this->declare_parameter<bool>("auto_global_localization", true);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();
        if (config["max_tf_input_age_s"])
            m_config.max_tf_input_age_s = config["max_tf_input_age_s"].as<double>();
        if (config["tf_republish_hz"])
            m_config.tf_republish_hz = config["tf_republish_hz"].as<double>();
        if (config["status_hz"])
            m_config.status_hz = config["status_hz"].as<double>();
        if (config["continuous_icp"])
            m_config.continuous_icp = config["continuous_icp"].as<bool>();
        if (config["global_max_candidates"])
            m_config.global_max_candidates = config["global_max_candidates"].as<int>();
        if (config["global_retry_interval_s"])
            m_config.global_retry_interval_s = config["global_retry_interval_s"].as<double>();
        if (config["global_max_automatic_attempts"])
            m_config.global_max_automatic_attempts = config["global_max_automatic_attempts"].as<int>();
        if (config["global_min_cloud_points"])
            m_config.global_min_cloud_points = config["global_min_cloud_points"].as<int>();
        if (config["min_candidate_score_gap"])
            m_config.min_candidate_score_gap = config["min_candidate_score_gap"].as<double>();
        if (config["min_overlap_ratio"])
            m_config.min_overlap_ratio = config["min_overlap_ratio"].as<double>();
        if (config["correction_alpha"])
            m_config.correction_alpha = config["correction_alpha"].as<double>();
        if (config["max_correction_translation_m"])
            m_config.max_correction_translation_m = config["max_correction_translation_m"].as<double>();
        if (config["max_correction_yaw_rad"])
            m_config.max_correction_yaw_rad = config["max_correction_yaw_rad"].as<double>();
        if (config["max_consecutive_failures"])
            m_config.max_consecutive_failures = config["max_consecutive_failures"].as<int>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
    }
    void setState(uint8_t state, const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        m_state.localization_state = state;
        m_state.state_reason = reason;
    }
    void loadMapAlignment()
    {
        m_map2d_from_map3d.setIdentity();
        if (m_config.alignment_file.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No 2D/3D alignment file configured; using identity");
            return;
        }
        try
        {
            YAML::Node root = YAML::LoadFile(m_config.alignment_file);
            if (!root["accepted"].as<bool>())
                throw std::runtime_error("alignment is not accepted");
            YAML::Node transform = root["transform"];
            const float yaw = transform["yaw"].as<float>();
            m_map2d_from_map3d.block<3, 3>(0, 0) =
                Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
            m_map2d_from_map3d(0, 3) = transform["x"].as<float>();
            m_map2d_from_map3d(1, 3) = transform["y"].as<float>();
            RCLCPP_INFO(this->get_logger(), "Loaded 2D/3D alignment: %s", m_config.alignment_file.c_str());
        }
        catch (const std::exception &error)
        {
            m_alignment_valid = false;
            setState(interface::msg::LocalizationStatus::LOST, "alignment_load_failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to load map alignment: %s", error.what());
        }
    }
    void loadDescriptorIndex()
    {
        if (m_config.descriptor_index.empty())
            return;
        try
        {
            if (!m_scan_context.load(m_config.descriptor_index))
                throw std::runtime_error("descriptor index is empty or invalid");
            RCLCPP_INFO(this->get_logger(), "Loaded Scan Context index: %s", m_config.descriptor_index.c_str());
        }
        catch (const std::exception &error)
        {
            m_descriptor_valid = false;
            setState(interface::msg::LocalizationStatus::LOST, "descriptor_index_load_failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to load descriptor index: %s", error.what());
        }
    }
    void loadStartupMap()
    {
        if (m_config.pcd_map.empty())
        {
            setState(interface::msg::LocalizationStatus::UNINITIALIZED, "pcd_map_missing");
            RCLCPP_INFO(this->get_logger(), "No startup PCD map configured; waiting for /localizer/relocalize");
            return;
        }
        setState(interface::msg::LocalizationStatus::MAP_LOADING, "loading_pcd_map");
        if (!std::filesystem::exists(m_config.pcd_map))
        {
            RCLCPP_ERROR(this->get_logger(), "Startup PCD map not found: %s", m_config.pcd_map.c_str());
            setState(interface::msg::LocalizationStatus::LOST, "pcd_map_not_found");
            return;
        }
        if (!m_localizer->loadMap(m_config.pcd_map))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load startup PCD map: %s", m_config.pcd_map.c_str());
            setState(interface::msg::LocalizationStatus::LOST, "pcd_map_load_failed");
            return;
        }
        if (!m_alignment_valid)
        {
            setState(interface::msg::LocalizationStatus::LOST, "alignment_invalid");
            return;
        }
        if (!m_descriptor_valid)
        {
            setState(interface::msg::LocalizationStatus::LOST, "descriptor_index_invalid");
            return;
        }
        setState(
            m_config.auto_global_localization && !m_scan_context.empty()
                ? interface::msg::LocalizationStatus::WAITING_FOR_SENSORS
                : interface::msg::LocalizationStatus::WAITING_FOR_INITIAL_POSE,
            "map_loaded");
        RCLCPP_INFO(this->get_logger(), "Loaded startup PCD map: %s", m_config.pcd_map.c_str());
    }
    void timerCB()
    {
        publishStatus();
        if (!m_state.message_received)
            return;

        bool localize_success;
        bool service_received;
        bool should_attempt_global = false;
        std::size_t cloud_points = 0;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            cloud_points = m_state.last_cloud->size();
        }
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            localize_success = m_state.localize_success;
            service_received = m_state.service_received;
            const rclcpp::Time now = this->now();
            const bool attempts_remaining =
                m_config.global_max_automatic_attempts <= 0 ||
                m_state.global_attempts < m_config.global_max_automatic_attempts;
            const bool retry_due =
                m_state.last_global_attempt_time.nanoseconds() == 0 ||
                (now - m_state.last_global_attempt_time).seconds() >=
                    m_config.global_retry_interval_s;
            if (m_config.auto_global_localization && !localize_success && !service_received &&
                attempts_remaining && retry_due && !m_scan_context.empty() &&
                cloud_points >= static_cast<std::size_t>(
                    std::max(1, m_config.global_min_cloud_points)))
            {
                ++m_state.global_attempts;
                m_state.last_global_attempt_time = now;
                should_attempt_global = true;
            }
            else if (m_config.auto_global_localization && !localize_success &&
                     cloud_points < static_cast<std::size_t>(
                         std::max(1, m_config.global_min_cloud_points)))
            {
                m_state.localization_state =
                    interface::msg::LocalizationStatus::WAITING_FOR_SENSORS;
                m_state.state_reason = "waiting_for_global_cloud";
            }
        }

        if (should_attempt_global)
        {
            int tested = 0;
            float best_score = std::numeric_limits<float>::infinity();
            performGlobalRelocalization("", m_config.global_max_candidates, tested, best_score);
        }

        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            localize_success = m_state.localize_success;
            service_received = m_state.service_received;
        }

        if (!localize_success && !service_received)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "PCD map loaded, but map->odom is gated until relocalization succeeds");
            return;
        }

        if (localize_success)
            republishLatestTF();

        rclcpp::Duration diff = this->now() - m_state.last_send_tf_time;
        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz);

        if (!update_tf)
            return;

        if (!m_config.continuous_icp && !service_received && m_state.has_published_tf)
            return;

        m_state.last_send_tf_time = this->now();

        M4F initial_guess_map2d = M4F::Identity();
        if (service_received)
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            initial_guess_map2d = m_state.initial_guess;
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            initial_guess_map2d.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess_map2d.block<3, 1>(0, 3) =
                (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }
        M4F initial_guess_map3d = m_map2d_from_map3d.inverse() * initial_guess_map2d;

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        if (!isTransformStampFresh(current_time))
            return;

        if (!service_received && m_state.has_published_tf && !isNewerStamp(current_time, m_state.last_tf_time))
            return;

        bool result = m_localizer->align(initial_guess_map3d);
        if (result)
        {
            const M4F map2d_body = m_map2d_from_map3d * initial_guess_map3d;
            applyAlignment(map2d_body, current_local_r, current_local_t, current_time, service_received);
            return;
        }
        handleAlignmentFailure(service_received ? "initial_pose_alignment_failed" : "runtime_alignment_failed");
    }
    static double yawFromRotation(const M3D &rotation)
    {
        return std::atan2(rotation(1, 0), rotation(0, 0));
    }
    static M3D planarRotation(double yaw)
    {
        return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    void handleAlignmentFailure(const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        ++m_state.consecutive_failures;
        m_state.state_reason = reason;
        if (m_state.localize_success && m_state.consecutive_failures < m_config.max_consecutive_failures)
        {
            m_state.localization_state = interface::msg::LocalizationStatus::DEGRADED;
            return;
        }
        m_state.localize_success = false;
        m_state.service_received = false;
        m_state.localization_state = interface::msg::LocalizationStatus::LOST;
    }
    bool applyAlignment(
        const M4F &map2d_body,
        const M3D &current_local_r,
        const V3D &current_local_t,
        const builtin_interfaces::msg::Time &current_time,
        bool force_relocalization)
    {
        const double overlap = m_localizer->lastOverlapRatio();
        if (overlap < m_config.min_overlap_ratio)
        {
            handleAlignmentFailure("pointcloud_overlap_too_low");
            return false;
        }

        const M3D map_body_r = map2d_body.block<3, 3>(0, 0).cast<double>();
        const V3D map_body_t = map2d_body.block<3, 1>(0, 3).cast<double>();
        const double candidate_offset_yaw =
            yawFromRotation(map_body_r) - yawFromRotation(current_local_r);
        const M3D candidate_offset_r = planarRotation(candidate_offset_yaw);
        V3D candidate_offset_t =
            map_body_t - candidate_offset_r * current_local_t;
        candidate_offset_t.z() = 0.0;

        double correction_translation = 0.0;
        double correction_yaw = 0.0;
        if (m_state.has_published_tf && !force_relocalization)
        {
            const M3D predicted_r = m_state.last_offset_r * current_local_r;
            const V3D predicted_t = m_state.last_offset_r * current_local_t + m_state.last_offset_t;
            correction_translation =
                (map_body_t.head<2>() - predicted_t.head<2>()).norm();
            correction_yaw = std::abs(yawFromRotation(map_body_r * predicted_r.transpose()));
            if (correction_translation > m_config.max_correction_translation_m ||
                correction_yaw > m_config.max_correction_yaw_rad)
            {
                handleAlignmentFailure("runtime_correction_jump");
                return false;
            }
            const double alpha = std::clamp(m_config.correction_alpha, 0.0, 1.0);
            Eigen::Quaterniond old_q(m_state.last_offset_r);
            Eigen::Quaterniond candidate_q(candidate_offset_r);
            m_state.last_offset_r = old_q.slerp(alpha, candidate_q).normalized().toRotationMatrix();
            m_state.last_offset_t =
                (1.0 - alpha) * m_state.last_offset_t + alpha * candidate_offset_t;
        }
        else
        {
            m_state.last_offset_r = candidate_offset_r;
            m_state.last_offset_t = candidate_offset_t;
        }

        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            const rclcpp::Time correction_time = this->now();
            const double correction_dt =
                m_state.last_correction_time.nanoseconds() > 0
                    ? std::max(1.0e-3, (correction_time - m_state.last_correction_time).seconds())
                    : 0.0;
            m_state.localize_success = true;
            m_state.service_received = false;
            m_state.localization_state = interface::msg::LocalizationStatus::LOCALIZED;
            m_state.state_reason = force_relocalization ? "relocalization_accepted" : "runtime_correction_accepted";
            m_state.consecutive_failures = 0;
            m_state.global_attempts = 0;
            m_state.rough_score = m_localizer->lastRoughScore();
            m_state.refine_score = m_localizer->lastRefineScore();
            m_state.overlap_ratio = overlap;
            m_state.correction_translation_m = correction_translation;
            m_state.correction_yaw_rad = correction_yaw;
            m_state.correction_translation_rate_mps =
                correction_dt > 0.0 ? correction_translation / correction_dt : 0.0;
            m_state.correction_yaw_rate_rps =
                correction_dt > 0.0 ? correction_yaw / correction_dt : 0.0;
            m_state.last_trusted_stamp = current_time;
            m_state.last_correction_time = correction_time;
        }
        const rclcpp::Time publish_time = this->now();
        sendBroadCastTF(publish_time);
        m_state.last_republish_tf_time = publish_time;
        publishMapCloud(current_time);
        m_state.last_tf_time = current_time;
        m_state.has_published_tf = true;
        return true;
    }
    bool performGlobalRelocalization(
        const std::string &region,
        int max_candidates,
        int &tested,
        float &best_score)
    {
        if (m_scan_context.empty())
            return false;
        setState(interface::msg::LocalizationStatus::GLOBAL_SEARCHING, "scan_context_query");
        CloudType::Ptr cloud(new CloudType);
        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            *cloud = *m_state.last_cloud;
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
        }
        if (!isTransformStampFresh(current_time))
        {
            handleAlignmentFailure("global_input_stale");
            return false;
        }
        m_localizer->setInput(cloud);
        float score_gap = 0.0F;
        const auto candidates = m_scan_context.query(
            cloud, region, std::max(1, max_candidates), score_gap);
        if (candidates.empty())
        {
            handleAlignmentFailure("no_global_candidates");
            return false;
        }
        best_score = candidates.front().descriptor_score;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.candidate_score_gap = score_gap;
        }
        if (candidates.size() > 1 && score_gap < m_config.min_candidate_score_gap)
        {
            handleAlignmentFailure("ambiguous_global_candidates");
            return false;
        }
        std::vector<M4F> guesses;
        guesses.reserve(candidates.size());
        for (const auto &candidate : candidates)
            guesses.push_back(candidate.pose);
        M4F best_map3d_body = M4F::Identity();
        if (!m_localizer->alignCandidates(guesses, best_map3d_body, tested))
        {
            handleAlignmentFailure("global_icp_failed");
            return false;
        }
        return applyAlignment(
            m_map2d_from_map3d * best_map3d_body,
            current_local_r,
            current_local_t,
            current_time,
            true);
    }
    static std::string stateLabel(uint8_t state)
    {
        switch (state)
        {
        case interface::msg::LocalizationStatus::UNINITIALIZED: return "UNINITIALIZED";
        case interface::msg::LocalizationStatus::MAP_LOADING: return "MAP_LOADING";
        case interface::msg::LocalizationStatus::WAITING_FOR_SENSORS: return "WAITING_FOR_SENSORS";
        case interface::msg::LocalizationStatus::GLOBAL_SEARCHING: return "GLOBAL_SEARCHING";
        case interface::msg::LocalizationStatus::WAITING_FOR_INITIAL_POSE: return "WAITING_FOR_INITIAL_POSE";
        case interface::msg::LocalizationStatus::LOCALIZED: return "LOCALIZED";
        case interface::msg::LocalizationStatus::DEGRADED: return "DEGRADED";
        case interface::msg::LocalizationStatus::RELOCALIZING: return "RELOCALIZING";
        case interface::msg::LocalizationStatus::LOST: return "LOST";
        default: return "UNKNOWN";
        }
    }
    void publishStatus()
    {
        const rclcpp::Time now = this->now();
        if (m_config.status_hz > 0.0 &&
            (now - m_state.last_status_publish_time).seconds() < (1.0 / m_config.status_hz))
            return;
        m_state.last_status_publish_time = now;
        interface::msg::LocalizationStatus status;
        status.header.stamp = now;
        status.header.frame_id = m_config.map_frame;
        builtin_interfaces::msg::Time input_stamp;
        bool message_received = false;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            input_stamp = m_state.last_message_time;
            message_received = m_state.message_received;
        }
        const double input_stamp_seconds =
            static_cast<double>(input_stamp.sec) +
            static_cast<double>(input_stamp.nanosec) * 1.0e-9;
        const double input_age_s = message_received
            ? std::max(0.0, now.seconds() - input_stamp_seconds)
            : -1.0;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            const bool sensors_ready =
                message_received &&
                (m_config.max_tf_input_age_s <= 0.0 || input_age_s <= m_config.max_tf_input_age_s);
            if (m_state.localize_success && !sensors_ready)
            {
                m_state.localization_state = interface::msg::LocalizationStatus::DEGRADED;
                m_state.state_reason = "sensor_input_stale";
            }
            status.state = m_state.localization_state;
            status.state_label = stateLabel(m_state.localization_state);
            status.reason = m_state.state_reason;
            status.map_id = m_config.map_id;
            status.sensors_ready = sensors_ready;
            status.localized =
                m_state.localize_success && status.sensors_ready &&
                m_state.localization_state == interface::msg::LocalizationStatus::LOCALIZED;
            status.input_age_s = static_cast<float>(input_age_s);
            status.rough_score = static_cast<float>(m_state.rough_score);
            status.refine_score = static_cast<float>(m_state.refine_score);
            status.overlap_ratio = static_cast<float>(m_state.overlap_ratio);
            status.correction_translation_m = static_cast<float>(m_state.correction_translation_m);
            status.correction_yaw_rad = static_cast<float>(m_state.correction_yaw_rad);
            status.correction_translation_rate_mps =
                static_cast<float>(m_state.correction_translation_rate_mps);
            status.correction_yaw_rate_rps =
                static_cast<float>(m_state.correction_yaw_rate_rps);
            status.candidate_score_gap = static_cast<float>(m_state.candidate_score_gap);
            status.last_trusted_stamp = m_state.last_trusted_stamp;
        }
        m_status_pub->publish(status);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> lock(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void sendBroadCastTF(const rclcpp::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
            m_state.localization_state = interface::msg::LocalizationStatus::RELOCALIZING;
            m_state.state_reason = "manual_initial_pose_received";
            m_state.consecutive_failures = 0;
            m_state.global_attempts = 0;
            m_state.last_global_attempt_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    void globalRelocCB(
        const std::shared_ptr<interface::srv::GlobalRelocalize::Request> request,
        std::shared_ptr<interface::srv::GlobalRelocalize::Response> response)
    {
        if (!request->descriptor_index.empty() && request->descriptor_index != m_config.descriptor_index)
        {
            try
            {
                if (!m_scan_context.load(request->descriptor_index))
                    throw std::runtime_error("descriptor index is empty or invalid");
                m_config.descriptor_index = request->descriptor_index;
                m_descriptor_valid = true;
            }
            catch (const std::exception &error)
            {
                response->success = false;
                response->message = error.what();
                return;
            }
        }
        int tested = 0;
        float best_score = std::numeric_limits<float>::infinity();
        response->success = performGlobalRelocalization(
            request->region,
            request->max_candidates > 0 ? request->max_candidates : m_config.global_max_candidates,
            tested,
            best_score);
        response->candidates_tested = tested;
        response->best_score = best_score;
        response->message = response->success ? "global relocalization accepted" : m_state.state_reason;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    bool isNewerStamp(const builtin_interfaces::msg::Time &candidate, const builtin_interfaces::msg::Time &last) const
    {
        if (candidate.sec != last.sec)
            return candidate.sec > last.sec;
        return candidate.nanosec > last.nanosec;
    }
    bool isTransformStampFresh(const builtin_interfaces::msg::Time &stamp)
    {
        if (m_config.max_tf_input_age_s <= 0.0)
            return true;

        const double stamp_seconds = static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
        const double age_s = this->now().seconds() - stamp_seconds;
        if (age_s <= m_config.max_tf_input_age_s)
            return true;

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Skipping stale map->odom broadcast because synchronized input stamp is %.3f seconds old",
            age_s);
        return false;
    }
    void republishLatestTF()
    {
        if (!m_state.has_published_tf || m_config.tf_republish_hz <= 0.0)
            return;

        const rclcpp::Time now = this->now();
        const rclcpp::Duration diff = now - m_state.last_republish_tf_time;
        if (diff.seconds() < (1.0 / m_config.tf_republish_hz))
            return;

        sendBroadCastTF(now);
        m_state.last_republish_tf_time = now;
    }
    void publishMapCloud(const builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        CloudType::Ptr map2d_cloud(new CloudType);
        pcl::transformPointCloud(*map_cloud, *map2d_cloud, m_map2d_from_map3d);
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map2d_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    ScanContextIndex m_scan_context;
    M4F m_map2d_from_map3d = M4F::Identity();
    bool m_alignment_valid = true;
    bool m_descriptor_valid = true;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Service<interface::srv::GlobalRelocalize>::SharedPtr m_global_reloc_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<interface::msg::LocalizationStatus>::SharedPtr m_status_pub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}

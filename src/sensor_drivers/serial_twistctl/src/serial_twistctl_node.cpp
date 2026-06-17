// 包含内存管理相关的标准库头文件
#include <memory>
// 包含字符串处理相关的标准库头文件
#include <string>
// 包含输入输出流相关的标准库头文件
#include <iostream>
// 包含时间相关的标准库头文件
#include <chrono>
// 包含线程相关的标准库头文件
#include <thread>
// 包含文件流相关的标准库头文件
#include <fstream>
// 包含格式化输出相关的标准库头文件
#include <iomanip>
// 包含时间函数相关的标准库头文件
#include <ctime>
// 包含ROS2核心库头文件
#include "rclcpp/rclcpp.hpp"
// 包含几何消息Twist类型的头文件
#include "geometry_msgs/msg/twist.hpp"
// 包含串口通信库头文件
#include "serial/serial.h"
// 包含yaml-cpp头文件，用于解析YAML配置文件
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <cstdlib>
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>
#include "serial_twistctl/twist_command.hpp"

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

// 检查是否启用日志的辅助函数
bool shouldEnableLogging(const std::string& node_key) {
    try {
        YAML::Node config = YAML::LoadFile(getRuntimePath("config/log_switch.yaml"));
        if (config[node_key] && config[node_key]["enable_logging"]) {
            return config[node_key]["enable_logging"].as<bool>();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading log config: " << e.what() << std::endl;
    }
    return true; // 默认启用日志
}

// 使用chrono命名空间中的字面量操作符
using namespace std::chrono_literals;

// 定义SerialTwistCtlNode类，继承自rclcpp::Node
class SerialTwistCtlNode : public rclcpp::Node
{
public:
    // SerialTwistCtlNode类的构造函数
    SerialTwistCtlNode()
    : Node("serial_twistctl_node")
    {
        // 初始化最后消息时间为当前时间
        last_message_time_ = std::chrono::steady_clock::now();
        // 记录节点启动成功信息
        RCLCPP_INFO(this->get_logger(), "Node 文件运行成功");

        // 检查是否启用日志
        bool enable_log = shouldEnableLogging("serial_twistctl_node");
        
        if (enable_log) {
            std::string log_path = getSessionLogPath("serial_twistctl.log", "logs/twist_log");
            log_file_.open(log_path, std::ios::app);

            if (!log_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to open log file");
                throw std::runtime_error("Unable to open log file");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging disabled by config");
        }

        // 声明端口参数
        this->declare_parameter<std::string>("port", "/dev/serial_twistctl");
        // 声明波特率参数
        this->declare_parameter<int>("baudrate", 115200);
        // 声明发送尝试次数参数
        this->declare_parameter<int>("send_attempts", 1);
        // 声明发送尝试间隔参数
        this->declare_parameter<int>("delay_between_attempts_ms", 0);
        // ROS angular.z 到底盘 wc 的比例/符号映射。
        this->declare_parameter<double>("angular_z_scale", 1.0);

        // 获取端口参数值
        port_ = this->get_parameter("port").as_string();
        // 获取波特率参数值
        baudrate_ = this->get_parameter("baudrate").as_int();
        // 获取发送尝试次数参数值
        send_attempts_ = this->get_parameter("send_attempts").as_int();
        // 获取发送尝试间隔参数值
        delay_between_attempts_ms_ = this->get_parameter("delay_between_attempts_ms").as_int();
        // 获取底盘角速度映射参数值
        angular_z_scale_ = this->get_parameter("angular_z_scale").as_double();

        // 设置串口端口
        try {
            serial_port_.setPort(port_);
            // 设置串口波特率
            serial_port_.setBaudrate(baudrate_);
            // 设置串口超时时间
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            // 打开串口
            serial_port_.open();
        } catch (const serial::IOException& e) {
            // 记录错误日志，表示无法打开串口
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port %s: %s", port_.c_str(), e.what());
            // 抛出运行时错误
            throw std::runtime_error("Serial port initialization failed");
        }

        // 检查串口是否成功打开
        if (!serial_port_.isOpen()) {
            // 记录错误日志，表示串口未成功打开
            RCLCPP_ERROR(this->get_logger(), "Serial port did not open successfully!");
            // 抛出运行时错误
            throw std::runtime_error("Serial port did not open successfully");
        } else {
            // 记录信息日志，表示串口成功打开
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", port_.c_str());
        }

        // 创建订阅者，订阅/cmd_vel话题
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&SerialTwistCtlNode::twist_callback, this, std::placeholders::_1)
        );

        // 创建定时器，每秒输出等待消息
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&SerialTwistCtlNode::timer_callback, this)
        );

        // 记录信息日志，表示节点已启动并订阅话题
        RCLCPP_INFO(this->get_logger(), "serial_twistctl_node node has started and subscribed to topic...");
    }

    // SerialTwistCtlNode类的析构函数
    ~SerialTwistCtlNode()
    {
        // 检查串口是否打开，如果是则关闭
        if (serial_port_.isOpen()) {
            serial_port_.close();
            // 记录信息日志，表示串口已关闭
            RCLCPP_INFO(this->get_logger(), "Serial port closed.");
        }

        // 检查日志文件是否打开，如果是则关闭
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    // 定时器回调函数
    void timer_callback()
    {
        // 检查距离最后消息的时间
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_msg = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_message_time_).count();
        
        // 如果超过5秒没有收到消息，输出等待信息
        if (time_since_last_msg >= 5) {
            RCLCPP_INFO(this->get_logger(), "waiting for data pack incoming ......");
        }
    }

    // Twist消息回调函数
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 更新最后消息时间
        last_message_time_ = std::chrono::steady_clock::now();

        // 提取线速度x分量
        float linear_x = msg->linear.x;
        // 提取角速度z分量；按底盘约定映射后再发给 STM32。
        float angular_z = msg->angular.z;
        float scaled_angular_z = static_cast<float>(angular_z * angular_z_scale_);

        // 将Twist消息转换为串口命令字符串
        std::string command = serial_twistctl::formatTwistCommand(
            linear_x,
            angular_z,
            angular_z_scale_);

        // 记录接收到的Twist消息内容
        RCLCPP_INFO(
            this->get_logger(),
            "[TWIST_RX] Received Twist message - linear.x=%.3f, angular.z=%.3f, scaled_angular.z=%.3f",
            linear_x,
            angular_z,
            scaled_angular_z);

        // 循环发送命令多次以确保接收
        for (int i = 0; i < send_attempts_; ++i) {
            // 检查串口是否打开
            if (!serial_port_.isOpen()) {
                RCLCPP_ERROR(this->get_logger(), "[SERIAL_ERROR] Serial port is not open!");
                if (log_file_.is_open()) {
                    log_file_ << "ERROR: Serial port is not open!" << std::endl;
                }
                continue;
            }

            // 获取ROS系统时间戳（19位纳秒格式）
            auto ros_time = this->now();
            int64_t ros_timestamp = ros_time.nanoseconds();

            // 通过串口发送命令
            size_t bytes_written = serial_port_.write(command);

            // 记录字节计数（包含时间戳）
            RCLCPP_INFO(this->get_logger(), "[SERIAL_TX] Timestamp: %ld, Sending command (%d/%d): %s [%zu bytes written]", 
                        ros_timestamp, i+1, send_attempts_, command.c_str(), bytes_written);

            // 如果日志文件打开，则写入日志（包含ROS时间戳）
            if (log_file_.is_open()) {
                log_file_ << "ROS_timestamp: " << ros_timestamp 
                         << ", [SERIAL_TX] Sending command (" << i+1 << "/" << send_attempts_ << "): " 
                         << command << " [" << bytes_written << " bytes]" << std::endl;
                log_file_.flush();
            }

            // 延迟指定的毫秒数
            if (i < send_attempts_ - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_between_attempts_ms_));
            }
        }
    }

    // 串口对象
    serial::Serial serial_port_;
    // 订阅者对象
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    // 定时器对象
    rclcpp::TimerBase::SharedPtr timer_;
    // 最后接收消息的时间
    std::chrono::steady_clock::time_point last_message_time_;
    // 日志文件输出流
    std::ofstream log_file_;

    // 端口参数
    std::string port_;
    // 波特率参数
    int baudrate_;
    // 发送尝试次数参数
    int send_attempts_;
    // 发送尝试间隔参数
    int delay_between_attempts_ms_;
    // ROS angular.z 到底盘 wc 的比例/符号映射
    double angular_z_scale_;
};

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 尝试创建节点并运行
    try {
        // 创建SerialTwistCtlNode节点对象
        auto node = std::make_shared<SerialTwistCtlNode>();
        // 运行节点，进入事件循环
        rclcpp::spin(node);
    }
    catch (const std::exception &e) {
        // 记录错误信息
        RCLCPP_ERROR(rclcpp::get_logger("serial_twistctl_node"), "Node 文件运行出错 %s", e.what());
    }

    // 关闭ROS2
    rclcpp::shutdown();
    // 返回0表示程序正常结束
    return 0;
}

// 包含ROS2核心头文件，用于节点创建
#include <rclcpp/rclcpp.hpp>
// 包含磁场消息头文件，用于磁力计数据
#include <sensor_msgs/msg/magnetic_field.hpp>
// 包含里程计消息头文件，用于位姿和速度数据
#include <nav_msgs/msg/odometry.hpp>
// 包含IMU消息头文件，用于惯性测量单元数据
#include <sensor_msgs/msg/imu.hpp>
// P3（FRC）：底盘控制模式/手柄按键上行消息
#include <frc_msgs/msg/chassis_status.hpp>
// 包含文件控制头文件，用于文件描述符操作
#include <fcntl.h>
// 包含终端控制头文件，用于串口配置
#include <termios.h>
// 包含unistd头文件，用于系统调用
#include <unistd.h>
// 包含时间库头文件，用于时间测量
#include <chrono>
// 包含字符串流头文件，用于字符串处理
#include <sstream>
// 包含字符串头文件，用于字符串操作
#include <string>
// 包含cstring头文件，用于strerror
#include <cstring>
// 包含向量头文件，用于动态数组
#include <vector>
// 包含数学头文件，用于数学计算
#include <cmath>
// 包含输入输出操纵头文件，用于格式化输出
#include <iomanip>
// 包含线程头文件，用于多线程
#include <thread>
// 包含文件流头文件，用于文件输出
#include <fstream>
// 包含时间头文件，用于时间函数
#include <ctime>
// 包含系统状态头文件，用于创建目录
#include <sys/stat.h>
// 包含系统类型头文件，用于目录操作
#include <sys/types.h>
// 包含yaml-cpp头文件，用于解析YAML配置文件
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <cerrno>

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

// 定义串口读取节点类，继承自ROS2节点
class SerialReaderNode : public rclcpp::Node
{
public:
    // 构造函数，初始化节点并设置发布者和串口
    SerialReaderNode()
        : Node("serial_reader_node")
    {
        // 记录节点启动成功信息
        RCLCPP_INFO(this->get_logger(), "Node 文件运行成功");
        // 创建磁力计发布者
        mag_pub_  = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
        // 创建里程计发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_CBoar", 10);
        // 创建IMU发布者
        imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        // P3（FRC）：创建底盘状态发布者（ctrl_mode + ps2_key）
        chassis_status_pub_ = this->create_publisher<frc_msgs::msg::ChassisStatus>("/chassis/status", 10);

        // 声明串口设备参数
        this->declare_parameter<std::string>("port", "/dev/serial_twistctl");
        // 声明波特率参数
        this->declare_parameter<int>("baud", 115200);
        // 获取串口设备参数
        port_ = this->get_parameter("port").as_string();
        // 获取波特率参数
        baud_ = this->get_parameter("baud").as_int();
        // 记录串口参数信息
        RCLCPP_INFO(this->get_logger(), "Serial port param: %s, baud=%d", port_.c_str(), baud_);

        // 尝试打开并配置串口
        if (!openSerialPort(port_, baud_)) {
            // 记录错误信息，串口打开失败
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
            // 抛出运行时错误
            throw std::runtime_error("Failed to open serial port");
        }
        // 记录串口打开成功信息
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully!");

        // 检查是否启用日志
        bool enable_log = shouldEnableLogging("serial_reader_node");
        
        if (enable_log) {
            std::string log_path = getSessionLogPath("serial_reader.log", "logs/reader_log");
            log_file_.open(log_path, std::ios::out | std::ios::app);
            if (!log_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_path.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Log file opened: %s", log_path.c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging disabled by config");
        }

        // 设置读取线程运行标志
        run_read_thread_ = true;
        // 启动读取串口数据的线程
        read_thread_ = std::thread(&SerialReaderNode::readSerialLoop, this);
    }

    // 析构函数，清理资源
    ~SerialReaderNode()
    {
        // 停止读取线程
        run_read_thread_ = false;
        // 等待线程结束
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        // 关闭串口文件描述符
        if (fd_ >= 0) {
            close(fd_);
        }
        // 关闭日志文件
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    // 日志文件流
    std::ofstream log_file_;

    // 打开并配置串口函数
    bool openSerialPort(const std::string &port, int baud)
    {
        // 打开串口设备文件
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        // 检查打开是否成功
        if (fd_ < 0) {
            // 记录错误信息
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", port.c_str(), strerror(errno));
            // 返回失败
            return false;
        }

        // 获取当前终端属性
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            // 记录错误信息
            RCLCPP_ERROR(this->get_logger(), "Failed to get terminal attributes: %s", strerror(errno));
            // 返回失败
            return false;
        }

        // 根据波特率设置速度
        speed_t baud_rate;
        // 选择波特率
        switch (baud) {
            case 115200: baud_rate = B115200; break;
            case 921600: baud_rate = B921600; break;
            case 460800: baud_rate = B460800; break;
            case 57600:  baud_rate = B57600;  break;
            default:
            // 默认使用115200
            baud_rate = B115200;
            // 记录警告信息，不支持的波特率
            RCLCPP_WARN(this->get_logger(), "Unsupported baud rate. Using 115200 by default.");
        }
        // 设置输出波特率
        cfsetospeed(&tty, baud_rate);
        // 设置输入波特率
        cfsetispeed(&tty, baud_rate);

        // 设置8位数据位，无校验，1位停止位
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= (CLOCAL | CREAD);

        // 设置原始模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // 设置读取配置，最少0字节，超时1秒
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 10;

        // 应用终端属性
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            // 记录错误信息
            RCLCPP_ERROR(this->get_logger(), "Failed to set terminal attributes: %s", strerror(errno));
            // 返回失败
            return false;
        }
        // 返回成功
        return true;
    }

    // 读取串口数据循环函数
    void readSerialLoop()
    {
        // 记录开始读取信息
        RCLCPP_INFO(this->get_logger(), "Start reading serial data loop.");
        // 初始化缓冲区
        std::string buffer;
        buffer.reserve(256);

        // 循环读取数据
        while (rclcpp::ok() && run_read_thread_) {
            // 读取单个字符
            char c;
            int n = ::read(fd_, &c, 1);
            // 如果读取到数据
            if (n > 0) {
                // 如果是换行符或回车符
                if (c == '\n' || c == '\r') {
                    // 如果缓冲区不为空
                    if (!buffer.empty()) {
                        // 解析并发布数据
                        parseAndPublish(buffer);
                        // 清空缓冲区
                        buffer.clear();
                    }
                } else {
                    // 将字符添加到缓冲区
                    buffer.push_back(c);
                }
            } else {
                // 休眠10毫秒
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    // 解析并发布数据函数
    void parseAndPublish(const std::string &line)
    {
        // P3（FRC）：CSV 解析改为 "前 16 字段必须齐全，第 17/18 字段可选"。
        // 旧固件 16 字段照常解析（回归不变）；新固件追加 ctrl_mode + ps2_key。
        std::vector<double> vals;
        vals.reserve(18);
        // 创建字符串流
        std::stringstream ss(line);
        // 解析逗号分隔的值
        for (int i = 0; i < 18; i++) {
            // 获取令牌
            std::string token;
            if (!std::getline(ss, token, ',')) {
                break;  // 流耗尽：由下方字段数检查决定是否丢弃
            }
            // 尝试转换为double
            try {
                vals.push_back(std::stod(token));
            } catch (...) {
                // 解析失败则丢弃
                return;  // 解析失败则丢弃
            }
        }

        // 检查值数量是否足够（前 16 字段必须齐全）
        if (vals.size() < 16) {
            // 返回
            return;
        }

        // 第 17/18 字段缺省补 0（旧固件兼容）
        const double ctrl_mode_raw = vals.size() >= 17 ? vals[16] : 0.0;
        const double ps2_key_raw = vals.size() >= 18 ? vals[17] : 0.0;

        // 获取当前时间
        auto now = this->now();
        int64_t ros_timestamp = now.nanoseconds();  // 转换为19位纳秒格式

        // 添加控制台输出日志
        RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Publishing sensor data: position(%.6f, %.6f, %.6f), orientation(%.6f, %.6f, %.6f, %.6f), linear_vel(%.6f, %.6f, %.6f), angular_vel(%.6f, %.6f, %.6f), magnetic(%.6f, %.6f, %.6f), timestamp: %ld",
                vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6],
                vals[7], vals[8], vals[9], vals[10], vals[11], vals[12],
                vals[13], vals[14], vals[15],
                ros_timestamp);  // 使用纳秒格式

        // 写入日志文件
        if (log_file_.is_open()) {
            log_file_ << "Publishing sensor data: position(" << std::fixed << std::setprecision(6)
                    << vals[0] << ", " << vals[1] << ", " << vals[2] << "), orientation("
                    << vals[3] << ", " << vals[4] << ", " << vals[5] << ", " << vals[6] << "), linear_vel("
                    << vals[7] << ", " << vals[8] << ", " << vals[9] << "), angular_vel("
                    << vals[10] << ", " << vals[11] << ", " << vals[12] << "), magnetic("
                    << vals[13] << ", " << vals[14] << ", " << vals[15] << "), timestamp: "
                    << ros_timestamp << "\n";  // 使用纳秒格式
            log_file_.flush();
        }

        // 创建磁力计消息
        sensor_msgs::msg::MagneticField mag_msg;
        // 设置时间戳
        mag_msg.header.stamp    = now;
        // 设置坐标系
        mag_msg.header.frame_id = "base_link";
        // 设置磁场值
        mag_msg.magnetic_field.x = vals[13];
        mag_msg.magnetic_field.y = vals[14];
        mag_msg.magnetic_field.z = vals[15];
        // 发布磁力计消息
        mag_pub_->publish(mag_msg);

        // 创建里程计消息
        nav_msgs::msg::Odometry odom;
        // 设置时间戳
        odom.header.stamp    = now;
        // 设置坐标系
        odom.header.frame_id = "odom";
        // 设置子坐标系
        odom.child_frame_id  = "base_link";
        // 设置位置
        odom.pose.pose.position.x    = vals[0];
        odom.pose.pose.position.y    = vals[1];
        odom.pose.pose.position.z    = vals[2];
        // 设置方向
        odom.pose.pose.orientation.x = vals[3];
        odom.pose.pose.orientation.y = vals[4];
        odom.pose.pose.orientation.z = vals[5];
        odom.pose.pose.orientation.w = vals[6];
        // 设置线速度 (应用低通滤波)
        double raw_linear_x = vals[7];
        double raw_linear_y = vals[8];
        double raw_angular_z = -vals[12]; // 保持之前的方向修正

        if (first_reading_) {
            last_linear_x_ = raw_linear_x;
            last_linear_y_ = raw_linear_y;
            last_angular_z_ = raw_angular_z;
            first_reading_ = false;
        } else {
            // 低通滤波公式: out = alpha * new + (1 - alpha) * old
            last_linear_x_ = filter_alpha_ * raw_linear_x + (1.0 - filter_alpha_) * last_linear_x_;
            last_linear_y_ = filter_alpha_ * raw_linear_y + (1.0 - filter_alpha_) * last_linear_y_;
            last_angular_z_ = filter_alpha_ * raw_angular_z + (1.0 - filter_alpha_) * last_angular_z_;
        }

        odom.twist.twist.linear.x  = last_linear_x_;
        odom.twist.twist.linear.y  = last_linear_y_;
        odom.twist.twist.linear.z   = vals[9];
        // 设置角速度
        odom.twist.twist.angular.x = vals[10];
        odom.twist.twist.angular.y = vals[11];
        odom.twist.twist.angular.z = last_angular_z_;
        // 发布里程计消息
        odom_pub_->publish(odom);

        // 创建IMU消息
        sensor_msgs::msg::Imu imu_msg;
        // 设置时间戳
        imu_msg.header.stamp            = now;
        // 设置坐标系
        imu_msg.header.frame_id         = "base_link";
        // 设置方向，与里程计相同
        imu_msg.orientation             = odom.pose.pose.orientation;
        // 设置角速度
        imu_msg.angular_velocity.x      = vals[10];
        imu_msg.angular_velocity.y      = vals[11];
        imu_msg.angular_velocity.z      = vals[12];
        // 设置线加速度为0（下位机未输出）
        imu_msg.linear_acceleration.x   = 0.0;
        imu_msg.linear_acceleration.y   = 0.0;
        imu_msg.linear_acceleration.z   = 0.0;
        // 发布IMU消息
        imu_pub_->publish(imu_msg);

        // P3（FRC）：发布底盘控制模式与手柄按键
        frc_msgs::msg::ChassisStatus chassis_msg;
        chassis_msg.header.stamp = now;
        chassis_msg.header.frame_id = "base_link";
        chassis_msg.ctrl_mode = static_cast<uint8_t>(std::lround(ctrl_mode_raw));
        chassis_msg.ps2_key = static_cast<uint8_t>(std::lround(ps2_key_raw));
        chassis_status_pub_->publish(chassis_msg);
    }

    // 串口设备路径
    std::string port_;
    // 波特率
    int baud_;
    // 文件描述符
    int fd_{-1};
    // 读取线程运行标志
    bool run_read_thread_{false};
    // 读取线程
    std::thread read_thread_;

    // 滤波相关变量
    double last_linear_x_{0.0};
    double last_linear_y_{0.0};
    double last_angular_z_{0.0};
    bool first_reading_{true};
    const double filter_alpha_{0.3}; // 滤波系数 0.0-1.0，越小越平滑但延迟越高，0.3 是个折中值

    // 磁力计发布者
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    // 里程计发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub_;
    // IMU发布者
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    // P3（FRC）：底盘状态发布者
    rclcpp::Publisher<frc_msgs::msg::ChassisStatus>::SharedPtr  chassis_status_pub_;
};

// 主函数，程序入口
int main(int argc, char **argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 尝试创建节点并运行
    try {
        // 创建节点实例
        auto node = std::make_shared<SerialReaderNode>();
        // 运行节点
        rclcpp::spin(node);
    }
    catch (const std::exception &e) {
        // 记录错误信息
        RCLCPP_ERROR(rclcpp::get_logger("serial_reader_node"), "Node 文件运行出错 %s", e.what());
    }
    
    // 关闭ROS2
    rclcpp::shutdown();
    // 返回0
    return 0;
}

// FRC 残差风险 costmap 插件（Nav2 Layer）。
// 设计不变量（实施文档 §4/P4）：
// - 默认 enabled=false：装载但旁路，disabled 时 costmap 行为与未装插件逐字节一致；
// - 只增不减 / 致命不触碰 / 置信度门，全部收口在 risk_gate.hpp 纯函数；
// - watchdog：RiskGrid 超时 0.5s 自动旁路（risk_pipeline 停发 -> 等价于没装）；
// - 本层不做任何 TF：RiskGrid 已由 risk_pipeline 以 odom 系（=local costmap
//   global_frame）发布，直接按世界坐标采样。
// - /frc/enable、/frc/disable（std_srvs/Trigger）+ 动态参数 enabled/tau/max_cost。

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "frc_costmap_layer/risk_gate.hpp"
#include "frc_msgs/msg/risk_grid.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace frc_costmap_layer
{

class FrcLayer : public nav2_costmap_2d::Layer
{
public:
    FrcLayer() = default;

    void onInitialize() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double *min_x, double *min_y,
                      double *max_x, double *max_y) override;
    void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                     int min_i, int min_j, int max_i, int max_j) override;
    void reset() override;
    bool isClearable() override { return false; }

private:
    void riskCB(const frc_msgs::msg::RiskGrid::SharedPtr msg);
    void enableCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void disableCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters);
    bool riskFresh() const;

    rclcpp::Subscription<frc_msgs::msg::RiskGrid>::SharedPtr risk_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        dyn_params_handler_;

    mutable std::mutex mutex_;
    frc_msgs::msg::RiskGrid::SharedPtr latest_;
    rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};

    double tau_ = 0.6;
    int max_cost_ = 200;
    double watchdog_timeout_s_ = 0.5;
    std::string risk_topic_ = "/frc/risk_grid";
};

}  // namespace frc_costmap_layer

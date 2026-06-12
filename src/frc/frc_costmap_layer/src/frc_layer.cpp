#include "frc_costmap_layer/frc_layer.hpp"

#include <algorithm>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace frc_costmap_layer
{

void FrcLayer::onInitialize()
{
    auto node = node_.lock();
    if (!node)
    {
        throw std::runtime_error("FrcLayer: failed to lock node");
    }

    declareParameter("enabled", rclcpp::ParameterValue(false));
    declareParameter("tau", rclcpp::ParameterValue(0.6));
    declareParameter("max_cost", rclcpp::ParameterValue(200));
    declareParameter("watchdog_timeout_s", rclcpp::ParameterValue(0.5));
    declareParameter("risk_topic", rclcpp::ParameterValue(std::string("/frc/risk_grid")));

    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "tau", tau_);
    node->get_parameter(name_ + "." + "max_cost", max_cost_);
    node->get_parameter(name_ + "." + "watchdog_timeout_s", watchdog_timeout_s_);
    node->get_parameter(name_ + "." + "risk_topic", risk_topic_);

    risk_sub_ = node->create_subscription<frc_msgs::msg::RiskGrid>(
        risk_topic_, rclcpp::QoS(2),
        std::bind(&FrcLayer::riskCB, this, std::placeholders::_1));

    enable_srv_ = node->create_service<std_srvs::srv::Trigger>(
        "/frc/enable",
        std::bind(&FrcLayer::enableCB, this, std::placeholders::_1,
                  std::placeholders::_2));
    disable_srv_ = node->create_service<std_srvs::srv::Trigger>(
        "/frc/disable",
        std::bind(&FrcLayer::disableCB, this, std::placeholders::_1,
                  std::placeholders::_2));

    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(&FrcLayer::dynamicParametersCallback, this,
                  std::placeholders::_1));

    current_ = true;
    RCLCPP_INFO(logger_,
                "FrcLayer initialized: enabled=%s tau=%.2f max_cost=%d "
                "watchdog=%.2fs topic=%s",
                enabled_ ? "true" : "false", tau_, max_cost_,
                watchdog_timeout_s_, risk_topic_.c_str());
}

void FrcLayer::riskCB(const frc_msgs::msg::RiskGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = msg;
    last_msg_time_ = clock_->now();
}

bool FrcLayer::riskFresh() const
{
    if (!latest_)
        return false;
    const double age = (clock_->now() - last_msg_time_).seconds();
    return age <= watchdog_timeout_s_;
}

void FrcLayer::updateBounds(double /*robot_x*/, double /*robot_y*/,
                            double /*robot_yaw*/, double *min_x, double *min_y,
                            double *max_x, double *max_y)
{
    if (!enabled_)
        return;   // 旁路：不扩 bounds，不触发任何重绘

    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_)
        return;
    const auto &info = latest_->risk.info;
    const double x0 = info.origin.position.x;
    const double y0 = info.origin.position.y;
    const double x1 = x0 + info.width * info.resolution;
    const double y1 = y0 + info.height * info.resolution;
    *min_x = std::min(*min_x, x0);
    *min_y = std::min(*min_y, y0);
    *max_x = std::max(*max_x, x1);
    *max_y = std::max(*max_y, y1);
}

void FrcLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                           int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;                       // 旁路：master 逐字节不变

    frc_msgs::msg::RiskGrid::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!riskFresh())
            return;                   // watchdog：超时自动旁路
        msg = latest_;
    }

    RiskGridView view;
    view.origin_x = msg->risk.info.origin.position.x;
    view.origin_y = msg->risk.info.origin.position.y;
    view.resolution = msg->risk.info.resolution;
    view.width = static_cast<int>(msg->risk.info.width);
    view.height = static_cast<int>(msg->risk.info.height);
    view.risk_data = msg->risk.data.data();
    view.conf_data = msg->confidence.data.data();
    if (msg->confidence.data.size() != msg->risk.data.size())
        return;                       // 长度不一致按损坏帧丢弃

    const float tau = static_cast<float>(tau_);
    const unsigned char max_cost =
        static_cast<unsigned char>(std::clamp(max_cost_, 0, 253));
    unsigned char *master = master_grid.getCharMap();
    const unsigned int size_x = master_grid.getSizeInCellsX();

    for (int j = min_j; j < max_j; ++j)
    {
        for (int i = min_i; i < max_i; ++i)
        {
            double wx, wy;
            master_grid.mapToWorld(i, j, wx, wy);
            float risk01, conf01;
            if (!view.sample(wx, wy, risk01, conf01))
                continue;
            const unsigned int idx = j * size_x + i;
            master[idx] = gatedCost(master[idx], risk01, conf01, tau, max_cost);
        }
    }
}

void FrcLayer::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
    current_ = true;
}

void FrcLayer::enableCB(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    enabled_ = true;
    res->success = true;
    res->message = "frc_layer enabled";
    RCLCPP_WARN(logger_, "frc_layer ENABLED via /frc/enable");
}

void FrcLayer::disableCB(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    enabled_ = false;
    res->success = true;
    res->message = "frc_layer disabled";
    RCLCPP_WARN(logger_, "frc_layer DISABLED via /frc/disable");
}

rcl_interfaces::msg::SetParametersResult FrcLayer::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto &param : parameters)
    {
        const auto &pname = param.get_name();
        if (pname == name_ + ".enabled")
            enabled_ = param.as_bool();
        else if (pname == name_ + ".tau")
            tau_ = param.as_double();
        else if (pname == name_ + ".max_cost")
            max_cost_ = static_cast<int>(param.as_int());
        else if (pname == name_ + ".watchdog_timeout_s")
            watchdog_timeout_s_ = param.as_double();
    }
    result.successful = true;
    return result;
}

}  // namespace frc_costmap_layer

PLUGINLIB_EXPORT_CLASS(frc_costmap_layer::FrcLayer, nav2_costmap_2d::Layer)

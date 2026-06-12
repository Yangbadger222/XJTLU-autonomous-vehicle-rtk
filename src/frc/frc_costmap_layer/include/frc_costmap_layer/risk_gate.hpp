// FRC 风险注入的安全核心：纯函数、零 ROS 依赖，三条不变量全部收口于此。
// 1) 致命不变：current >= LETHAL(254) 的格子绝不触碰（含 NO_INFORMATION 255）
// 2) 只增不减：输出 = max(current, 注入值)
// 3) 上限截断：注入值 <= max_cost（建议 <= 200，防止把可通行区堵死）
// 不变量单测见 test/test_frc_layer.cpp。

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace frc_costmap_layer
{

constexpr unsigned char kLethalObstacle = 254;   // = nav2 LETHAL_OBSTACLE

inline unsigned char gatedCost(unsigned char current, float risk01, float conf01,
                               float tau, unsigned char max_cost)
{
    if (conf01 < tau)
        return current;                            // 置信度门
    if (current >= kLethalObstacle)
        return current;                            // 致命/未知格不触碰
    if (risk01 < 0.0f)
        risk01 = 0.0f;
    if (risk01 > 1.0f)
        risk01 = 1.0f;
    const unsigned char add =
        static_cast<unsigned char>(risk01 * static_cast<float>(max_cost));
    return std::max(current, add);                 // 只增不减
}

// odom 轴对齐风险栅格的只读视图（OccupancyGrid data: 0-100，<0 视为无数据）
struct RiskGridView
{
    double origin_x = 0.0;
    double origin_y = 0.0;
    double resolution = 0.1;
    int width = 0;
    int height = 0;
    const int8_t *risk_data = nullptr;
    const int8_t *conf_data = nullptr;

    bool sample(double wx, double wy, float &risk01, float &conf01) const
    {
        if (risk_data == nullptr || conf_data == nullptr || resolution <= 0.0)
            return false;
        // floor 而非 int 截断：负数侧截断向零会把 -0.1 误判为格 0
        const int ix = static_cast<int>(std::floor((wx - origin_x) / resolution));
        const int iy = static_cast<int>(std::floor((wy - origin_y) / resolution));
        if (ix < 0 || ix >= width || iy < 0 || iy >= height)
            return false;
        const int idx = iy * width + ix;
        const int8_t r = risk_data[idx];
        const int8_t c = conf_data[idx];
        if (r < 0 || c < 0)
            return false;
        risk01 = static_cast<float>(r) / 100.0f;
        conf01 = static_cast<float>(c) / 100.0f;
        return true;
    }
};

}  // namespace frc_costmap_layer

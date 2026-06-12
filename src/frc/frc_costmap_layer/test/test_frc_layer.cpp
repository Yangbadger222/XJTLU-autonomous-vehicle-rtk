// 三不变量 + watchdog 采样的单元测试（纯函数，无 ROS 运行时依赖）。

#include <gtest/gtest.h>

#include <vector>

#include "frc_costmap_layer/risk_gate.hpp"

using frc_costmap_layer::RiskGridView;
using frc_costmap_layer::gatedCost;
using frc_costmap_layer::kLethalObstacle;

// 不变量 1：致命/未知格绝不触碰
TEST(GatedCost, NeverTouchesLethal)
{
    EXPECT_EQ(gatedCost(254, 1.0f, 1.0f, 0.6f, 200), 254);
    EXPECT_EQ(gatedCost(255, 1.0f, 1.0f, 0.6f, 200), 255);  // NO_INFORMATION
    EXPECT_EQ(gatedCost(kLethalObstacle, 0.5f, 0.9f, 0.1f, 253),
              kLethalObstacle);
}

// 不变量 2：只增不减
TEST(GatedCost, NeverDecreases)
{
    for (unsigned int cur = 0; cur < 254; ++cur)
    {
        for (float risk : {0.0f, 0.3f, 0.7f, 1.0f})
        {
            const unsigned char out = gatedCost(
                static_cast<unsigned char>(cur), risk, 1.0f, 0.6f, 200);
            EXPECT_GE(out, cur);
        }
    }
}

// 不变量 3：注入上限截断
TEST(GatedCost, CappedAtMaxCost)
{
    EXPECT_EQ(gatedCost(0, 1.0f, 1.0f, 0.6f, 200), 200);
    EXPECT_EQ(gatedCost(0, 2.0f, 1.0f, 0.6f, 200), 200);   // risk 越界也截断
    EXPECT_LE(gatedCost(100, 1.0f, 1.0f, 0.6f, 150), 150);
    // current 比注入高时保持 current（与只增不减一致）
    EXPECT_EQ(gatedCost(220, 1.0f, 1.0f, 0.6f, 200), 220);
}

// 置信度门
TEST(GatedCost, ConfidenceGate)
{
    EXPECT_EQ(gatedCost(10, 1.0f, 0.59f, 0.6f, 200), 10);   // 低于 tau 不注入
    EXPECT_EQ(gatedCost(10, 1.0f, 0.61f, 0.6f, 200), 200);
}

TEST(RiskGridView, SamplesAndBounds)
{
    // 2x2 栅格 @1m，原点 (10, 20)
    std::vector<int8_t> risk = {0, 50, 100, 0};
    std::vector<int8_t> conf = {100, 80, 60, 0};
    RiskGridView v;
    v.origin_x = 10.0;
    v.origin_y = 20.0;
    v.resolution = 1.0;
    v.width = 2;
    v.height = 2;
    v.risk_data = risk.data();
    v.conf_data = conf.data();

    float r, c;
    ASSERT_TRUE(v.sample(10.5, 20.5, r, c));   // cell (0,0)
    EXPECT_FLOAT_EQ(r, 0.0f);
    ASSERT_TRUE(v.sample(11.5, 20.5, r, c));   // cell (1,0)
    EXPECT_FLOAT_EQ(r, 0.5f);
    EXPECT_FLOAT_EQ(c, 0.8f);
    ASSERT_TRUE(v.sample(10.5, 21.5, r, c));   // cell (0,1)
    EXPECT_FLOAT_EQ(r, 1.0f);

    EXPECT_FALSE(v.sample(9.9, 20.5, r, c));   // 越界
    EXPECT_FALSE(v.sample(12.1, 20.5, r, c));
    EXPECT_FALSE(v.sample(10.5, 22.1, r, c));
}

TEST(RiskGridView, RejectsNegativeData)
{
    std::vector<int8_t> risk = {-1};
    std::vector<int8_t> conf = {100};
    RiskGridView v;
    v.origin_x = 0.0;
    v.origin_y = 0.0;
    v.resolution = 1.0;
    v.width = 1;
    v.height = 1;
    v.risk_data = risk.data();
    v.conf_data = conf.data();
    float r, c;
    EXPECT_FALSE(v.sample(0.5, 0.5, r, c));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

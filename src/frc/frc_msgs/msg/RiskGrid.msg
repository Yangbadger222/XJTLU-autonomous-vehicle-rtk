# FRC 残差风险场。risk/confidence 与 local costmap 同为 odom 系，
# 由 risk_pipeline_node 以 10 Hz 发布；frc_costmap_layer 只消费不做 TF。

std_msgs/Header header

# 风险栅格：data 取值 0-100（OccupancyGrid 习惯），-1 表示未知/不注入
nav_msgs/OccupancyGrid risk
# 置信度栅格：data 取值 0-100，对应 0.0-1.0
nav_msgs/OccupancyGrid confidence

# 胜出来源掩码常量（按位）
uint8 SOURCE_MAP_ANCHOR=1
uint8 SOURCE_FEATURE_RETRIEVAL=2
uint8 SOURCE_MODEL=4

# 本帧参与融合的来源按位或
uint8 source_mask

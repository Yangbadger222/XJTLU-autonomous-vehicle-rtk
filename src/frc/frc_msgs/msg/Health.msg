# 系统健康状态聚合（degradation-aware 条件输入）。
# health_aggregator_node 以 10 Hz 发布。

std_msgs/Header header

# FAST-LIO2 IESKF 量测块正则化前最小特征值（/fastlio2/degeneracy[0]）
float32 lio_min_eig
# 量测块条件数（/fastlio2/degeneracy[1]）
float32 lio_cond
# 本帧是否触发了退化正则化（/fastlio2/degeneracy[2] > 0.5）
bool lio_degenerate
# PGO 是否处于回环/GPS 修正窗口内
bool pgo_correcting
# 最近一次 map->odom 修正量（米）
float32 pgo_last_jump
# RTK/GNSS 状态：-1 无 fix，0 单点，2 DGPS/RTK（NavSatStatus.status 透传）
int8 rtk_status
# 当前车体线速度 / 角速度（来自 /odom_CBoar）
float32 v
float32 w

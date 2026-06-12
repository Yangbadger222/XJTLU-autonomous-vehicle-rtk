# PGO 关键帧位姿数组（map 系，回环修正后整体刷新）。
# pgo_node 以 1 Hz 发布；id 即 keyPoses() 数组下标，单调追加不重排。

std_msgs/Header header
string session_id
int64[] ids
geometry_msgs/Pose[] poses

# FRC 失败/风险事件标记。
# 由 frc_event_marker_node（自动挖掘）、scripts/frc/frc_mark.py（人工 CLI）
# 或手柄 SELECT 边沿（经 /chassis/status）产生。

std_msgs/Header header

# 事件类型常量
string TYPE_TAKEOVER=takeover
string TYPE_STUCK=stuck
string TYPE_RECOVERY=recovery
string TYPE_NEAR_COLLISION=near_collision
string TYPE_JERK=jerk
string TYPE_PLAN_INSTABILITY=plan_instability
string TYPE_MANUAL=manual

# 严重级别常量（对应标签层级）
string SEVERITY_GOLD=gold
string SEVERITY_SILVER=silver
string SEVERITY_BRONZE=bronze

string type
string severity
# 事件发生时机器人在 map 系下的位姿
geometry_msgs/Pose pose
string route_id
string layout_id
string note
# 复用 launch_with_logs.sh 的日志会话时间戳
string session_id

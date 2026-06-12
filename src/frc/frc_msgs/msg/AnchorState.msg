# 单个风险锚的状态快照（map 系）。

string KIND_MAP=map
string KIND_FEATURE=feature

string STATE_CANDIDATE=candidate
string STATE_CONFIRMED=confirmed
string STATE_STALE=stale
string STATE_RETIRED=retired

int64 anchor_id
string kind
string state
float64 p_usable
float64 severity
int32 hit_count
# 锚挂载的 PGO keyframe id（session 内局部量）与相对偏移
int64 keyframe_id
float64 dx
float64 dy
float64 dyaw
# map 系位姿快照（跨 session 重挂用）
float64 map_x
float64 map_y
float64 map_yaw
string scene_id
string session_id
builtin_interfaces/Time last_verified

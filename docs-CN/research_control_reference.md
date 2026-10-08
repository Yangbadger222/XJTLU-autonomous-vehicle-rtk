# 现有 MID360 记录导出的车体控制参考

MID360 自带 IMU。本适配使用 e54c6af 的 hardware_spec、工厂 LiDAR/IMU 位移和原 URDF 姿态，没有引入额外 IMU 或猜测标定。

记录的 LiDAR 安装位置为 (-.07,.12,.447)m，参考是车体 XY 原点的名义地面高度。原零安装 RPY 与 t_IL=(-.011,-.02329,.04412)m 得到 IMU 坐标中的车体原点 r_IB=(.059,-.14329,-.40288)m；它不是名义 z=.229m 的 URDF base_link。

p_WB=p_WI+R_WI r_IB，v_B=v_I+omega_I cross r_IB。适配完整六维 pose/twist 协方差和交叉项，避免车体固定原地转动时把偏置 IMU 的运动误认为车体平移。

原 /lio/odom_vehicle 与 odom->base_footprint 保留 IMU 原点合同供原 RTK 使用。独立 /research/odom_control 的 child 为 chassis_control_origin，合同 corridor_e54c6af_mid360_ground_control_origin_v1；EGO、跟踪器、观察器和控制台使用此明确参考，TimedTrajectory2D 携带合同。map->odom 仍由原 RTK 唯一拥有。

朝向恢复使用原 .05m/s、2°/s、1秒连续速率容差确认，检查完整三维速度和采集时间连续性。这不是物理制动证明。恢复保留实测角速度/采集间隔导出的角加速度，连续曲线检查原限值且命令平移为零。authority、health、TF、stop、speed、map、trajectory 或 consent 拒绝均立即撤销授权；RTK 失权时 LIO 正常也必须停车。

原串口程序与分配的 PTY 测试见 AUDIT_REPORT.md。这里使用现有名义配置模型，未冒充新的现场测量。车端实际覆盖、地面支持、KEY/手柄/物理制动与急停仍需现场验收。

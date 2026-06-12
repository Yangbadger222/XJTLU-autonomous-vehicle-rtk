"""健康状态聚合节点：把 LIO 退化 / PGO 修正 / GNSS / 底盘运动汇成 frc_msgs/Health。

输入（全部低频小消息）：
- /fastlio2/degeneracy  Float32MultiArray [min_eig, cond, regularized]（P1）
- /pgo/correction_status Float32MultiArray [correcting, last_jump_m, loop_pairs]（P2）
- /fix                  sensor_msgs/NavSatFix（status 透传为 rtk_status）
- /odom_CBoar           nav_msgs/Odometry（底盘回读 v/w；topic 实名如此拼写）

输出：/frc/health frc_msgs/Health @ 10 Hz。
输入超时（stale_timeout_s）后对应字段回退保守默认值。
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray

from frc_msgs.msg import Health


class HealthAggregatorNode(Node):
    def __init__(self):
        super().__init__("frc_health_aggregator")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("stale_timeout_s", 1.0)
        # 与 IESKF 正则化阈值一致；min_eig 低于它即视为退化
        self.declare_parameter("lio_min_eig_degenerate", 75.0)

        self._stale_s = float(self.get_parameter("stale_timeout_s").value)
        self._eig_thresh = float(
            self.get_parameter("lio_min_eig_degenerate").value)

        self._degeneracy = None     # (stamp, [min_eig, cond, regularized])
        self._correction = None     # (stamp, [correcting, jump, loops])
        self._fix_status = None     # (stamp, int8)
        self._vw = None             # (stamp, v, w)

        self.create_subscription(Float32MultiArray, "/fastlio2/degeneracy",
                                 self._on_degeneracy, 10)
        self.create_subscription(Float32MultiArray, "/pgo/correction_status",
                                 self._on_correction, 10)
        self.create_subscription(NavSatFix, "/fix", self._on_fix, 10)
        self.create_subscription(Odometry, "/odom_CBoar", self._on_odom, 10)

        self._pub = self.create_publisher(Health, "/frc/health", 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info("frc_health_aggregator started")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_degeneracy(self, msg):
        if len(msg.data) >= 3:
            self._degeneracy = (self._now_s(), list(msg.data[:3]))

    def _on_correction(self, msg):
        if len(msg.data) >= 2:
            self._correction = (self._now_s(), list(msg.data))

    def _on_fix(self, msg):
        self._fix_status = (self._now_s(), int(msg.status.status))

    def _on_odom(self, msg):
        self._vw = (self._now_s(),
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z))

    def _fresh(self, item):
        return item is not None and (self._now_s() - item[0]) < self._stale_s

    def _tick(self):
        msg = Health()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self._fresh(self._degeneracy):
            min_eig, cond, reg = self._degeneracy[1]
            msg.lio_min_eig = float(min_eig)
            msg.lio_cond = float(cond)
            msg.lio_degenerate = bool(reg > 0.5 or min_eig < self._eig_thresh)
        else:
            # 无数据按"未知不告警"处理；min_eig=0 标记不可用
            msg.lio_min_eig = 0.0
            msg.lio_cond = 0.0
            msg.lio_degenerate = False

        if self._fresh(self._correction):
            data = self._correction[1]
            msg.pgo_correcting = bool(data[0] > 0.5)
            msg.pgo_last_jump = float(data[1])
        if self._fresh(self._fix_status):
            msg.rtk_status = self._fix_status[1]
        else:
            msg.rtk_status = -1
        if self._fresh(self._vw):
            msg.v = self._vw[1]
            msg.w = self._vw[2]

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HealthAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class Nav2CloudRetime(Node):
    def __init__(self):
        super().__init__("nav2_cloud_retime")

        self.declare_parameter("input_topic", "cloud_in")
        self.declare_parameter("output_topic", "cloud_out")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Retiming {input_topic} to {output_topic} for Nav2 costmaps")

    def on_cloud(self, msg):
        out = PointCloud2()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CloudRetime()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

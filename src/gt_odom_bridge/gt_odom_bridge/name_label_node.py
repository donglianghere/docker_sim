#!/usr/bin/env python3
"""
RViz里给每架飞机头顶标一个跟着走的文字标签(自己的namespace，比如"NX01")。

订阅`dlio/odom_node/odom`（不是`state`）是故意的：这个话题名不管
LOCALIZATION_SOURCE是`dlio`（真DLIO）还是`gt`（gt_odom_bridge_node转发
Gazebo真值）都存在、且同名——见gt_odom_bridge_node.py自己的注释，两者
二选一发布到同一个最终话题名。这个标签节点因此不用关心当前用的是哪种
定位源，直接订阅这个名字就行。

frame_id用`{ns}/odom`：跟mighty的`map_frame_id`（use_hardware=true时是
`{ns}/map`）通过`static_tf_map_to_odom`那条恒等TF是同一个点，RViz Fixed
Frame挂哪个都能正确解算出这个marker的位置。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class NameLabelNode(Node):

    def __init__(self):
        super().__init__('name_label_node')

        self.declare_parameter('label_text', '')
        self.declare_parameter('z_offset', 0.4)

        ns = self.get_namespace().strip('/')
        label_text = self.get_parameter('label_text').value
        self.label_text = label_text if label_text else ns
        self.z_offset = self.get_parameter('z_offset').value
        self.odom_frame = f'{ns}/odom' if ns else 'odom'

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Marker, 'name_label', qos)
        self.sub = self.create_subscription(
            Odometry, 'dlio/odom_node/odom', self._odom_cb, qos)

        self.get_logger().info(
            f"name_label_node up: text='{self.label_text}', frame={self.odom_frame}")

    def _odom_cb(self, msg: Odometry):
        m = Marker()
        m.header.frame_id = self.odom_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'name_label'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = msg.pose.pose.position.x
        m.pose.position.y = msg.pose.pose.position.y
        m.pose.position.z = msg.pose.pose.position.z + self.z_offset
        m.pose.orientation.w = 1.0
        m.scale.z = 0.35
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = self.label_text
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = NameLabelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

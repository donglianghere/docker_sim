#!/usr/bin/env python3
"""nlink_parser2(真实UWB驱动) -> uwb_origin_bridge标准接口的桥接节点。

背景：`uwb_origin_bridge`的`origin_setter_node`只认一个标准话题格式
`uwb/pose_abs`(geometry_msgs/PoseStamped，相对话题，命名空间下解析成
`/{ns}/uwb/pose_abs`)——仿真里这个话题由`uwb_sim`的`uwb_ground_truth_node.py`
拿Gazebo真值伪造；真机上换成nooploop LinkTrack标签的真实解算结果。

`nlink_parser2`（打过`nlink_parser2_namespace.patch`补上namespace参数之后）
在标签配置成对应协议帧类型时，会在相对话题`nlink_linktrack_nodeframe2`上
发布`LinktrackNodeframe2`消息，其中`pos_3d`是标签在UWB基站阵列标定坐标系下
解出的绝对位置，`quaternion`是标签自带IMU融合出的姿态。这个节点把它转成
`uwb_origin_bridge`已经在用的标准接口，下游代码不用改一行。

⚠️ 未实测确认事项（等真实硬件到位后需要核对，当前只做到语法/接口层面的
桥接，没有连过真实LinkTrack设备）：
  1. 协议帧类型——`nlink_parser2`一个标签具体发哪个话题(nodeframe0~7/
     tagframe0)由上位机NAssistant配置决定，不是ROS侧能选的，这里假设
     配置成了nodeframe2；如果实际配置的是别的帧类型（比如tagframe0，
     字段结构类似但没有`nodes`数组），需要改这个节点订阅的消息类型。
  2. `quaternion`字段的分量顺序——`nlink_parser2`的.msg文件只声明了
     `float32[4] quaternion`，没有在字段注释里说是[x,y,z,w]还是
     [w,x,y,z]，这里按ROS常见约定假设是[x,y,z,w]，需要用真实设备核对。
  3. `pos_3d`单位——假设是米（配合`origin_setter_node`等下游按米处理的
     假设），需要用真实设备核对量级是否对得上。
  4. 当前`origin_setter_node.py`只读PoseStamped的position字段，没有用
     orientation——这里依然把quaternion填进去，是为了给以后可能用到姿态
     的消费者留好接口，哪怕分量顺序猜错也只影响orientation这个目前没人
     用的字段，不影响position、不影响起飞点锁定功能本身。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nlink_parser2.msg import LinktrackNodeframe2


class NlinkPoseBridgeNode(Node):

    def __init__(self):
        super().__init__('nlink_pose_bridge')

        self.declare_parameter('nodeframe_topic', 'nlink_linktrack_nodeframe2')
        self.declare_parameter('pose_topic', 'uwb/pose_abs')
        # 跟uwb_sim的uwb_ground_truth_node.py保持一致（那边frame_id写死'world'，
        # 见该文件"/{ns}/uwb/pose_abs (geometry_msgs/msg/PoseStamped, frame_id="world")"
        # 的说明），下游origin_setter_node的world_frame参数默认值也是'world'。
        self.declare_parameter('frame_id', 'world')

        self.frame_id = self.get_parameter('frame_id').value

        self._pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.create_subscription(
            LinktrackNodeframe2, self.get_parameter('nodeframe_topic').value,
            self._nodeframe_cb, 10)

        self.get_logger().info(
            f"nlink_pose_bridge就绪：{self.get_parameter('nodeframe_topic').value} "
            f"-> {self.get_parameter('pose_topic').value}")

    def _nodeframe_cb(self, msg: LinktrackNodeframe2):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(msg.pos_3d[0])
        pose.pose.position.y = float(msg.pos_3d[1])
        pose.pose.position.z = float(msg.pos_3d[2])
        # 分量顺序假设[x,y,z,w]，见文件头未实测确认事项——当前下游只用position。
        pose.pose.orientation.x = float(msg.quaternion[0])
        pose.pose.orientation.y = float(msg.quaternion[1])
        pose.pose.orientation.z = float(msg.quaternion[2])
        pose.pose.orientation.w = float(msg.quaternion[3])
        self._pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = NlinkPoseBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

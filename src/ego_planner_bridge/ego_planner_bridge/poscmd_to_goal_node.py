#!/usr/bin/env python3
"""ego_planner(quadrotor_msgs/msg/PositionCommand) -> ros2_px4_stack(dynus_interfaces/msg/Goal)桥接节点。

背景：ego_planner的traj_server原生发布position_cmd
（quadrotor_msgs/msg/PositionCommand，字段position/velocity/acceleration/
jerk/yaw/yaw_dot），px4ctrl/so3ctrl两条控制器链路直接remap('cmd',
'position_cmd')就能吃这个话题，但CONTROLLER=ros2_px4_stack这条链路的
dynus_offboard_node.py（track_dynus_traj）只认dynus_interfaces/msg/Goal
（字段p/v/a/j/yaw/dyaw），订阅的是绝对话题名/{VEH_NAME}/goal，没有任何
节点把PositionCommand转成这个格式——这正是之前"PLANNER=ego_planner+
CONTROLLER=ros2_px4_stack"组合只打警告、飞机不会动的原因。

两个消息的字段语义几乎一一对应（跟px4ctrl_bridge/goal_to_poscmd.py反过来
的关系一致），这里同样只做直通的字段搬运，不做插值/平滑/限幅——
dynus_offboard_node.py自己的_pack_into_traj()/get_orientation()/
get_angular()已经用微分平坦度公式重新算了姿态/角速度前馈，不依赖这个桥接
节点做额外处理。

已知未消费的PositionCommand字段（按设计明确忽略，不是遗漏）：
  - kx/kv：ego_planner自带的一套增益覆盖机制，dynus_offboard_node.py走的
    是完全不同的PX4位置控制器/微分平坦度路径，没有对应的增益输入点。
  - trajectory_id/trajectory_flag：Goal消息里没有对应字段，dynus生态里
    轨迹状态是靠dynus_offboard_node.py自己的FSM(TAKEOFF/TRAJECTORY/RETURN)
    管理的，不依赖上游告知。
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from dynus_interfaces.msg import Goal
from quadrotor_msgs.msg import PositionCommand


class PosCmdToGoal(Node):
    def __init__(self):
        super().__init__('poscmd_to_goal')

        # 跟traj_server.cpp里create_publisher<PositionCommand>("position_cmd", 50)
        # 用的默认QoS（RELIABLE+VOLATILE）对齐。
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        # 跟dynus_offboard_node.py里goal_qos（RELIABLE+VOLATILE, depth10）对齐，
        # 这是它自己订阅Goal话题时要求的QoS。
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # dynus_offboard_node.py订阅的是绝对话题名f'/{veh}/goal'（VEH_NAME
        # 环境变量，跟这套集成里的NAMESPACE同值），不是相对话题名，这里必须
        # 显式拼出同一个绝对话题名，不能指望namespace remap帮忙解析。
        veh = os.environ.get('VEH_NAME')
        if not veh:
            self.get_logger().error(
                '必须设置VEH_NAME环境变量（跟dynus_offboard_node.py读取的是同一个），'
                '否则无法拼出目标Goal话题名')
        goal_topic = f'/{veh}/goal'

        self._pub = self.create_publisher(Goal, goal_topic, pub_qos)
        self._sub = self.create_subscription(
            PositionCommand, 'position_cmd', self._on_poscmd, sub_qos)
        self.get_logger().info(f'poscmd_to_goal up: position_cmd -> {goal_topic}')

    def _on_poscmd(self, msg: PositionCommand) -> None:
        out = Goal()
        out.header = msg.header
        out.p.x = msg.position.x
        out.p.y = msg.position.y
        out.p.z = msg.position.z
        out.v.x = msg.velocity.x
        out.v.y = msg.velocity.y
        out.v.z = msg.velocity.z
        out.a.x = msg.acceleration.x
        out.a.y = msg.acceleration.y
        out.a.z = msg.acceleration.z
        out.j.x = msg.jerk.x
        out.j.y = msg.jerk.y
        out.j.z = msg.jerk.z
        out.yaw = msg.yaw
        out.dyaw = msg.yaw_dot
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PosCmdToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

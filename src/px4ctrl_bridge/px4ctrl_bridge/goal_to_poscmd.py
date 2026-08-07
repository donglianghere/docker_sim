#!/usr/bin/env python3
"""mighty(dynus_interfaces/msg/Goal) -> px4ctrl(quadrotor_msgs/msg/PositionCommand)桥接节点。

mighty发布的Goal和px4ctrl吃的PositionCommand字段语义几乎一一对应
（p/v/a/j/yaw/dyaw <-> position/velocity/acceleration/jerk/yaw/yaw_dot），
这里只做直通的字段搬运，不做任何插值/平滑/限幅——px4ctrl自己的控制律
（LinearControl::calculateControl）已经是基于位置/速度误差的PD控制，不
依赖这个桥接节点做额外处理。

已知未消费的Goal字段（按设计明确忽略，不是遗漏）：
  - power：px4ctrl自己的状态机(AUTO_HOVER/CMD_CTRL/AUTO_TAKEOFF/AUTO_LAND)
    独立管理解锁/上锁，不依赖mighty这个字段来决定电机是否可以转。
  - mode_xy/mode_z：px4ctrl只支持"完整位置+速度+加速度误差"这一种跟踪
    模式，没有mighty那种按轴切换position/velocity/acceleration-only的
    子模式，所以这两个字段没有对应的下游消费点。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from dynus_interfaces.msg import Goal
from quadrotor_msgs.msg import PositionCommand


class GoalToPosCmd(Node):
    def __init__(self):
        super().__init__('goal_to_poscmd')

        # 跟mighty_node.cpp发布"goal"话题用的critical_qos（RELIABLE+VOLATILE，
        # depth 10）保持一致，避免QoS不兼容导致订阅不到（这个项目里DLIO点云
        # 那次QoS不匹配的教训，见README——这里特意对齐，不是漏掉的风险点）。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(PositionCommand, 'cmd', 10)
        self._sub = self.create_subscription(Goal, 'goal', self._on_goal, qos)
        self._traj_id = 0

    def _on_goal(self, msg: Goal) -> None:
        out = PositionCommand()
        out.header = msg.header
        out.position.x = msg.p.x
        out.position.y = msg.p.y
        out.position.z = msg.p.z
        out.velocity.x = msg.v.x
        out.velocity.y = msg.v.y
        out.velocity.z = msg.v.z
        out.acceleration.x = msg.a.x
        out.acceleration.y = msg.a.y
        out.acceleration.z = msg.a.z
        out.jerk.x = msg.j.x
        out.jerk.y = msg.j.y
        out.jerk.z = msg.j.z
        out.yaw = msg.yaw
        out.yaw_dot = msg.dyaw
        # trajectory_id/trajectory_flag只是px4ctrl input.cpp里存进
        # Command_Data_t::msg的原始字段，控制律/状态机全程都不读取这两个值，
        # 单调自增只是方便调试时用`ros2 topic echo`肉眼确认桥接节点还活着、
        # 消息没有卡死重复。
        self._traj_id += 1
        out.trajectory_id = self._traj_id
        out.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GoalToPosCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

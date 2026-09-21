#!/usr/bin/env python3
"""position_cmd中继/仲裁节点。

对应《2026大赛任务系统开发执行方案.md》阶段3。这是精降(阶段6)和环绕
飞行(阶段4)共用的基础设施——两者都需要在"不改traj_server/px4ctrl/
so3ctrl任何一行代码"的前提下，能临时接管/改写发给控制器的position_cmd。

拓扑（只改了px4ctrl_docker_sim.launch.py/so3ctrl_docker_sim.launch.py/
pt4ctrl_docker_sim.launch.py里`cmd`这一个remap目标，traj_server/
px4ctrl_node/so3ctrl_node/pt4ctrl_node本身一行没动）：

    traj_server --[position_cmd]--> 本节点 --[position_cmd_relayed]--> px4ctrl/so3ctrl/pt4ctrl
                                       ^
                    precision_servo_node(阶段6.2,还没实现) --[precision_land_cmd]--+

三种relay_mode（用ROS2标准参数机制切换，不是自定义service/msg——切换方式
就是调用节点自带的标准`~/set_parameters`服务，或者最简单直接
`ros2 param set /<ns>/position_cmd_relay relay_mode <mode>`，阶段7任务
状态机要切模式时照这个调用即可，不需要为此单独生成一个rosidl接口包）：
  - normal：默认值，`position_cmd`收到什么就原样转发到`position_cmd_
    relayed`，字段一个不改。这个模式下的行为必须跟"根本没有这个中继
    节点、px4ctrl直接订阅position_cmd"完全一致（验证标准见执行方案），
    所以normal分支刻意写得最简单——收到消息立即原样republish，不做
    任何拷贝/改写之外的处理，不引入额外延迟。
  - precision_land：改成完全订阅`precision_land_cmd`（阶段6.2的
    precision_servo_node发布），`position_cmd`收到的消息在这个模式下
    被直接丢弃——"完全接管转发"，不是"两路都转发"。
  - orbit_yaw_override：仍然转发`position_cmd`的位置/速度/加速度等
    字段，但用最近一次收到的里程计位置+`orbit_center_x`/
    `orbit_center_y`两个参数实时算`yaw=atan2(cy-y, cx-x)`，覆写`yaw`/
    `yaw_dot`两个字段（yaw_dot用相邻两次算出的yaw做有限差分，不是照抄
    原始消息里的yaw_dot——原始yaw_dot是"沿轨迹切线方向"那套语义下算的，
    覆写了yaw之后继续用旧的yaw_dot在语义上是错的）。
  - pillar_aim（2026-09-09新增）：跟precision_land同一个"完全接管"
    模式，只是数据源换成`fire_pillar_aim_cmd`（`fire_pillar_aim_node.py`
    发布，检测到高层着火点AprilTag之后算出的"正对着火面"悬停目标）。
    没有单独复用precision_land这个名字，是因为两者语义不同（一个是
    垂直下降，一个是水平接近+垂直于目标面悬停），任务状态机切换模式
    时如果搞混了很危险，用不同名字强制调用方明确写清楚"现在要做的是
    哪种接管"。
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node

VALID_MODES = ('normal', 'precision_land', 'orbit_yaw_override', 'pillar_aim', 'formation')


class PositionCmdRelayNode(Node):
    def __init__(self):
        super().__init__('position_cmd_relay')

        self.declare_parameter(
            'relay_mode', 'normal',
            ParameterDescriptor(description=f'取值范围: {VALID_MODES}'),
        )
        self.declare_parameter('orbit_center_x', 0.0)
        self.declare_parameter('orbit_center_y', 0.0)

        self._latest_odom_xy = None  # (x, y)，来自dlio/odom_node/odom
        self._last_orbit_yaw = None
        self._last_orbit_stamp = None

        self.pub = self.create_publisher(PositionCommand, 'position_cmd_relayed', 10)

        self.create_subscription(PositionCommand, 'position_cmd', self._on_position_cmd, 10)
        self.create_subscription(PositionCommand, 'precision_land_cmd', self._on_precision_land_cmd, 10)
        self.create_subscription(PositionCommand, 'fire_pillar_aim_cmd', self._on_pillar_aim_cmd, 10)
        # 2026-09-20新增：编队轨迹跟随。跟precision_land/pillar_aim同一个
        # "完全接管"语义，数据源换成formation_follower_node发的
        # formation_cmd——跟随者的设定点逐点复现长机走过的轨迹，必须绕开
        # ego_planner（规划器会自己重新规划、抄近道切内弯，实测轨迹很乱）。
        self.create_subscription(PositionCommand, 'formation_cmd', self._on_formation_cmd, 10)
        # 跟px4ctrl_docker_sim.launch.py里px4ctrl_node_ego_planner的odom
        # remap用同一个话题名——两边应该看到同一份"控制器实际在用的里程计"，
        # 不是另开一路可能跟控制器不一致的定位源。
        self.create_subscription(Odometry, 'dlio/odom_node/odom', self._on_odom, 10)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info('position_cmd_relay就绪，relay_mode=normal（默认透传，行为等价于没有这个节点）')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'relay_mode' and p.value not in VALID_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=f"relay_mode必须是{VALID_MODES}之一，收到'{p.value}'",
                )
        for p in params:
            if p.name == 'relay_mode':
                self.get_logger().info(f'relay_mode -> {p.value}')
        return SetParametersResult(successful=True)

    @property
    def relay_mode(self) -> str:
        return self.get_parameter('relay_mode').value

    def _on_odom(self, msg: Odometry):
        self._latest_odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_position_cmd(self, msg: PositionCommand):
        mode = self.relay_mode
        if mode in ('precision_land', 'pillar_aim'):
            # 完全接管：traj_server这一路在这两个模式下都被丢弃，不转发。
            return
        if mode == 'orbit_yaw_override':
            self._publish_with_orbit_yaw(msg)
            return
        # normal：原样转发，不做任何改写。
        self.pub.publish(msg)

    def _on_precision_land_cmd(self, msg: PositionCommand):
        if self.relay_mode != 'precision_land':
            return
        self.pub.publish(msg)

    def _on_pillar_aim_cmd(self, msg: PositionCommand):
        if self.relay_mode != 'pillar_aim':
            return
        self.pub.publish(msg)

    def _on_formation_cmd(self, msg: PositionCommand):
        """编队轨迹跟随指令（2026-09-20新增）。跟上面两个"完全接管"模式
        同一个形状：只在对应relay_mode下转发，其余模式原样丢弃。"""
        if self.relay_mode != 'formation':
            return
        self.pub.publish(msg)

    def _publish_with_orbit_yaw(self, msg: PositionCommand):
        if self._latest_odom_xy is None:
            # 还没收到过里程计，没法算朝向目标点的yaw——原样转发比"拿一个
            # 猜测值覆写yaw"更安全（宁可保留原有切线朝向，不要用错误朝向
            # 覆盖），等odom来了自然会切到覆写逻辑。
            self.get_logger().warn(
                'orbit_yaw_override模式但还没收到过odom，本帧不覆写yaw，原样转发',
                throttle_duration_sec=5.0,
            )
            self.pub.publish(msg)
            return

        cx = self.get_parameter('orbit_center_x').value
        cy = self.get_parameter('orbit_center_y').value
        x, y = self._latest_odom_xy
        yaw = math.atan2(cy - y, cx - x)

        stamp = msg.header.stamp
        now_sec = stamp.sec + stamp.nanosec * 1e-9
        yaw_dot = 0.0
        if self._last_orbit_yaw is not None and self._last_orbit_stamp is not None:
            dt = now_sec - self._last_orbit_stamp
            if dt > 1e-6:
                dyaw = math.atan2(math.sin(yaw - self._last_orbit_yaw), math.cos(yaw - self._last_orbit_yaw))
                yaw_dot = dyaw / dt
        self._last_orbit_yaw = yaw
        self._last_orbit_stamp = now_sec

        msg.yaw = yaw
        msg.yaw_dot = yaw_dot
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionCmdRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""自动裁判节点——读Gazebo真值，不读选手自己上报的任务状态。

对应《2026大赛任务系统开发执行方案.md》阶段7.2。归属`sim-world`容器
（见设计方案5.3容器归属表："场景重置服务、自动裁判节点 | sim-world |
需要读写Gazebo模型状态，这类权限只有sim-world容器有"），不是
`flight-stack`——这是这个包里第一个不跑在flight-stack-nx01/nx02里的
节点。

**裁判判据故意不读`mission_state`话题**（阶段7.1定义的那个，选手/
参考任务逻辑自己上报"我现在在searching/loading/..."）——裁判只信
Gazebo的真实模型位姿（订阅`gazebo_ros_state`插件发布的
`gazebo_msgs/ModelStates`，world文件里配的话题名是
`/plug/model_states_plug`，见worlds/fire_drill_room.world的
`gazebo_ros_state`插件配置），不然选手上报假状态就能骗过裁判——这是
"裁判独立于选手代码"这条设计原则的直接体现。

**目标点坐标不是从`fire_drill_room_layout.yaml`静态读的，是每帧从
ModelStates里按模型名实时查的**——这是为了跟阶段7.3
`scenario_reset_node`的随机化重置配合：如果物资点/着火点被
scenario_reset_node随机挪了新位置，裁判用ModelStates查到的就是挪动后
的真实位置，不会拿旧的静态坐标误判。`gazebo_ros_state`发布的
ModelStates本来就包含所有模型（含静态模型），不需要额外订阅别的话题。

判据（方案原文的例子："水平距离≤1米+朝向误差≤阈值，连续满足3秒"）：
每个checkpoint配置(目标模型名, 半径, 持续时间, 可选的朝向阈值,
可选的限定哪架机)，检查对应飞机(NX01/NX02的模型名本身就是spawn时的
entity名，同样能在ModelStates里查到位姿)到目标的水平距离，连续满足
半径要求达到设定时长才判定完成，用`diagnostic_msgs/DiagnosticArray`
发布每个checkpoint的实时状态（level=OK已完成/WARN进行中/ERROR从未
达标过，不新增自定义消息类型）。
"""
import math
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from std_srvs.srv import Trigger


@dataclass
class Checkpoint:
    name: str
    target_model: str  # 要查询位姿的Gazebo模型名，比如'supply_point_marker'
    radius_m: float = 1.0
    hold_duration_s: float = 3.0
    drone_ns: Optional[str] = None  # None=检查所有已配置的飞机；指定则只检查这一架
    target_yaw_rad: Optional[float] = None  # None=不检查朝向
    yaw_tolerance_rad: float = 0.3
    # 运行时状态，不是配置——每架飞机独立维护自己对这个checkpoint的
    # "连续满足计时"，一架机中途飞出半径不会影响另一架机已经攒的时长。
    _entered_at: dict = field(default_factory=dict)  # {drone_ns: monotonic_time_entered}
    _completed: dict = field(default_factory=dict)  # {drone_ns: True}


def _default_checkpoints() -> list:
    """默认checkpoint列表——覆盖物资点/地面火情/高层着火点立柱/起降点
    这几个方案里明确要判定的目标，不假设具体是哪架机去做哪个（阶段
    7.4/阶段10的任务分工逻辑还没写），对所有已配置的飞机都检查。起降点
    是per-drone的（2026-09-09场景重新设计：起飞=降落合一，模型名从
    `landing_pad_nx0N`改成`helipad_nx0N`），指定了drone_ns。"""
    return [
        Checkpoint(name='reach_supply_point', target_model='supply_point_marker', radius_m=1.0, hold_duration_s=3.0),
        Checkpoint(name='reach_ground_fire_point', target_model='fire_point_marker', radius_m=1.0, hold_duration_s=3.0),
        # 2026-09-09"高层着火点标识不固定贴哪根立柱"改动：target_model
        # 从写死的'pillar_2_fire'改成'fire_apriltag_marker'——这个独立
        # 模型每次reset会被scenario_reset_node瞬移到"随机选中的那根
        # 立柱的随机选中的那个面"，裁判只需要认准这个模型本身的实时
        # ModelStates位姿，不用关心它这一局到底贴在哪根立柱上。
        Checkpoint(name='reach_fire_pillar', target_model='fire_apriltag_marker', radius_m=1.5, hold_duration_s=3.0),
        Checkpoint(name='nx01_return_pad', target_model='helipad_nx01', radius_m=1.0, hold_duration_s=3.0, drone_ns='NX01'),
        Checkpoint(name='nx02_return_pad', target_model='helipad_nx02', radius_m=1.0, hold_duration_s=3.0, drone_ns='NX02'),
    ]


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionJudgeNode(Node):
    def __init__(self):
        super().__init__('mission_judge_node')

        self.declare_parameter('drone_namespaces', ['NX01', 'NX02'])
        self.declare_parameter('model_states_topic', '/plug/model_states_plug')
        self.declare_parameter('publish_rate_hz', 2.0)

        self.drone_namespaces = list(self.get_parameter('drone_namespaces').value)
        self.checkpoints = _default_checkpoints()
        self._latest_poses = {}  # {model_name: (x, y, yaw)}

        topic = self.get_parameter('model_states_topic').value
        self.create_subscription(ModelStates, topic, self._on_model_states, 10)

        self.pub = self.create_publisher(DiagnosticArray, 'mission_judge/checkpoints', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        # 阶段7.3 scenario_reset_node场景重置之后会调这个service，清空
        # 所有checkpoint的完成状态——不然道具被随机挪了新位置，裁判还
        # 留着"旧位置已经达标"的记录，场景重置就变得不干净。
        self.create_service(Trigger, '~/reset_checkpoints', self._on_reset_checkpoints)

        self.get_logger().info(
            f'订阅"{topic}"，监控{len(self.checkpoints)}个checkpoint，'
            f'飞机命名空间={self.drone_namespaces}'
        )

    def _on_model_states(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            yaw = _yaw_from_quaternion(pose.orientation)
            self._latest_poses[name] = (pose.position.x, pose.position.y, yaw)

    def _on_timer(self):
        now = time.monotonic()
        out = DiagnosticArray()
        out.header.stamp = self.get_clock().now().to_msg()

        for cp in self.checkpoints:
            if cp.target_model not in self._latest_poses:
                continue  # 还没收到过这个模型的位姿(比如场景还没加载完)，跳过

            tx, ty, tyaw = self._latest_poses[cp.target_model]
            drones_to_check = [cp.drone_ns] if cp.drone_ns else self.drone_namespaces

            for ns in drones_to_check:
                if ns not in self._latest_poses:
                    continue
                dx, dy, dyaw = self._latest_poses[ns]
                dist = math.hypot(dx - tx, dy - ty)
                yaw_ok = True
                if cp.target_yaw_rad is not None:
                    yaw_err = math.atan2(math.sin(dyaw - cp.target_yaw_rad), math.cos(dyaw - cp.target_yaw_rad))
                    yaw_ok = abs(yaw_err) <= cp.yaw_tolerance_rad

                in_range = (dist <= cp.radius_m) and yaw_ok

                if cp._completed.get(ns):
                    level, held_s = DiagnosticStatus.OK, cp.hold_duration_s
                elif in_range:
                    entered = cp._entered_at.get(ns)
                    if entered is None:
                        cp._entered_at[ns] = now
                        entered = now
                    held_s = now - entered
                    if held_s >= cp.hold_duration_s:
                        cp._completed[ns] = True
                        level = DiagnosticStatus.OK
                        self.get_logger().info(f'checkpoint完成: {ns} {cp.name} (持续{held_s:.1f}s)')
                    else:
                        level = DiagnosticStatus.WARN
                else:
                    cp._entered_at.pop(ns, None)
                    level, held_s = DiagnosticStatus.ERROR, 0.0

                status = DiagnosticStatus()
                status.name = f'{ns}/{cp.name}'
                status.level = level
                status.message = (
                    'completed' if level == DiagnosticStatus.OK
                    else f'in_range, holding {held_s:.1f}/{cp.hold_duration_s:.1f}s' if level == DiagnosticStatus.WARN
                    else f'out_of_range, dist={dist:.2f}m (need<={cp.radius_m}m)'
                )
                status.values = [
                    KeyValue(key='distance_m', value=f'{dist:.3f}'),
                    KeyValue(key='radius_m', value=f'{cp.radius_m}'),
                    KeyValue(key='target_model', value=cp.target_model),
                ]
                out.status.append(status)

        if out.status:
            self.pub.publish(out)

    def _on_reset_checkpoints(self, request, response):
        for cp in self.checkpoints:
            cp._entered_at.clear()
            cp._completed.clear()
        self.get_logger().info(f'checkpoint状态已清空（共{len(self.checkpoints)}个checkpoint）')
        response.success = True
        response.message = f'reset {len(self.checkpoints)} checkpoints'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionJudgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

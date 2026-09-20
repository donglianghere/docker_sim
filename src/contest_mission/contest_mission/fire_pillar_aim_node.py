#!/usr/bin/env python3
"""高层着火点瞄准节点：检测到贴在某根立柱上的AprilTag ID1之后，把侦查机
飞到"正对贴tag那个竖直面、机头对准着火点中心"的悬停点，同时算出任务机
（负责后续"投射"动作的第二架飞机）该去的等待点。

跟阶段6.2`precision_servo_node.py`是同一个技术路线（不做单目6DOF位姿
解算，理由见该文件头说明——YOLO给的2D bbox本身也解不出深度/朝向，
这条路在真机上根本走不通），但解决的是不同维度的问题：精降只需要
"水平往哪修、修多少"这一个标量级信息；这里要满足"垂直于目标所在的
竖直面"这个朝向约束，纯像素偏移解不出来，得换一条路——**不用视觉解
姿态，改用激光雷达已经在测的立柱朝向**（`pillar_detector_node.py`用
`cv2.minAreaRect`拟合出来的yaw）+**检测命中那一刻无人机自己的方位角**，
两者一起喂给`fire_pillar_approach_helper.snap_to_nearest_face()`吸附
到最近的候选竖直面，换算出目标悬停点——摄像头/AprilTag检测在这里只
起"确认目标在视野里"这一个二值触发作用，不提供任何位姿信息，因此
换成真机YOLO识别时这条链路完全不用改一行代码（YOLO输出同样是
`vision_msgs/Detection2DArray`，只要class_id对得上就能触发）。

跟阶段3`position_cmd_relay_node.py`的接口方式：新增`fire_pillar_aim_cmd`
话题（`quadrotor_msgs/PositionCommand`），照抄`precision_land_cmd`的
"计算好目标就按控制频率持续发布"模式；中继节点新增`pillar_aim`
relay_mode消费这个话题，用法完全对称，不重新发明一套接口。

任务机等待点**这个节点只负责算出来、发布出去**（`fire_pillar_staging_
pose`话题），不负责决定"哪架机是任务机、什么时候真的让任务机飞过去"
——那是阶段7.4任务状态机的编排逻辑，还没实现（见DEBUG_JOURNAL.md
2026-09-09相关讨论），这次范围明确限定在"瞄准操作+算等待点"两件事。

**检测命中只锁定一次**（`~/reset_aim` service清空锁定状态，供下一轮
任务/测试重新触发）：一旦命中过一次就固定住目标点，不随后续帧的
方位角变化持续重算——命中时刻之后无人机自己会开始朝目标点飞，方位角
会持续变化，如果每帧都重新吸附，飞行过程中方位角跨越45度边界时
吸附结果会跳变，目标点跟着跳变，正好是要避免的"随机性/不稳定"。

2026-09-10新增"锁定后自动接管"：之前锁定目标只是把计算结果发到
`fire_pillar_aim_cmd`话题，`position_cmd_relay`的`relay_mode`要不要
真的切到`pillar_aim`完全没人管——每次都得手动`ros2 param set`才会
真的接管飞控，用户反馈这一步不该手动做。现在锁定成功那一刻，这个
节点自己去调`position_cmd_relay`的标准`~/set_parameters`service把
`relay_mode`切成`pillar_aim`；`~/reset_aim`清空锁定状态时对称地切
回`normal`（谁接管的谁负责交还，不留在`pillar_aim`模式里让飞机悬停
在半空没人管）。可以用`auto_switch_relay_mode`参数关掉这个自动切换
（默认开），留给以后阶段7.4任务状态机自己接管"什么时候真的要接管
飞控"这个决策时用——不想删这段代码，禁用掉就行。

⚠️ 调`position_cmd_relay`的service是在这个节点自己的subscription
回调（`_on_detections`）内部发起的——照抄`scenario_reset_node.py`
2026-09-08实测踩过的死锁教训（`~/reset_scenario`那个service回调
内部调其它service，默认callback group在单线程executor下会自己等
自己死锁，见该文件头部说明的完整故障描述）：这个service client
单独放进`MutuallyExclusiveCallbackGroup`，配合`main()`里的
`MultiThreadedExecutor`，避免重蹈覆辙。
"""
import math
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseArray
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from contest_mission.fire_pillar_approach_helper import (
    compute_aim_pose,
    compute_staging_pose,
    snap_to_nearest_face,
)
from contest_mission.layout_loader import load_layout


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FirePillarAimNode(Node):
    def __init__(self):
        super().__init__('fire_pillar_aim_node')

        layout = load_layout()
        fire_tag_id = int(layout['fire_apriltag_id'])

        self.declare_parameter('target_class_id', f'apriltag:{fire_tag_id}')
        # 1.5米跟mission_judge_node.py的reach_fire_pillar checkpoint
        # radius_m用同一个值——悬停在裁判判定半径以内，不是随便取的数。
        self.declare_parameter('standoff_m', 1.5)
        self.declare_parameter('staging_lateral_offset_deg', 30.0)
        self.declare_parameter('control_rate_hz', 10.0)
        # 候选立柱离检测命中位置太远就不认——避免误把远处别的立柱当成
        # "命中时刻附近那根"，1.5倍立柱间距最小值(2米)留了余量。
        self.declare_parameter('max_pillar_match_dist_m', 3.0)
        # 锁定成功后自动把position_cmd_relay切到pillar_aim模式（见文件头
        # 说明）。关掉的话锁定/发布fire_pillar_aim_cmd照常发生，只是不会
        # 自动接管飞控，留给以后任务状态机自己决定这个时机。
        self.declare_parameter('auto_switch_relay_mode', True)
        self.declare_parameter('relay_set_parameters_service', 'position_cmd_relay/set_parameters')

        self._odom_x_y_z_yaw: Optional[Tuple[float, float, float, float]] = None
        self._pillar_candidates: list = []  # [(x, y, yaw), ...]
        self._locked_aim: Optional[Tuple[float, float, float]] = None
        self._locked_staging: Optional[Tuple[float, float, float]] = None
        self._locked_hover_z: float = 3.0  # 命中前不会被用到，默认值不影响行为

        self.aim_pub = self.create_publisher(PositionCommand, 'fire_pillar_aim_cmd', 10)
        self.staging_pub = self.create_publisher(PositionCommand, 'fire_pillar_staging_pose', 10)
        # 对外广播锁定的瞄准点坐标本身（跟fire_pillar_aim_cmd内容一致，但
        # 语义不同：aim_cmd是控制指令，喂给position_cmd_relay的pillar_aim
        # 模式去真正驱动飞控；这个话题是只读状态广播，给第四部分全流程
        # 任务程序读，用来告诉SUPPLY角色"投射完之后飞到RECON锁定的这个点
        # 接着投"（见方案文档1.2节）。故意开两个话题而不是复用一个，
        # 避免以后"改控制指令格式"和"改对外广播格式"这两件不相关的事
        # 被迫绑在一起改。
        self.aim_pose_pub = self.create_publisher(PositionCommand, 'fire_pillar_aim_pose', 10)

        self.create_subscription(Odometry, 'dlio/odom_node/odom', self._on_odom, 10)
        self.create_subscription(PoseArray, 'pillar_candidates', self._on_pillar_candidates, 10)
        self.create_subscription(Detection2DArray, 'vision/detections', self._on_detections, 10)

        self.create_service(Trigger, '~/reset_aim', self._on_reset_aim)

        # 单独的callback group，见文件头"死锁教训"说明——不能跟这个节点
        # 默认的callback group共用。
        client_cb_group = MutuallyExclusiveCallbackGroup()
        relay_svc_name = self.get_parameter('relay_set_parameters_service').value
        self.relay_set_params_cli = self.create_client(
            SetParameters, relay_svc_name, callback_group=client_cb_group,
        )

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_control_tick)

        self.get_logger().info(
            f"fire_pillar_aim_node就绪，target_class_id={self.get_parameter('target_class_id').value}，"
            f"命中后悬停点发到fire_pillar_aim_cmd（配合position_cmd_relay的pillar_aim模式），"
            f"任务机等待点发到fire_pillar_staging_pose，"
            f"同时把瞄准点坐标本身广播到fire_pillar_aim_pose（给第四部分全流程"
            f"任务程序读，告诉SUPPLY角色投射完之后飞到这个位置接着投）"
        )

    def _on_odom(self, msg: Odometry):
        self._odom_x_y_z_yaw = (
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
            _quaternion_to_yaw(msg.pose.pose.orientation),
        )

    def _on_pillar_candidates(self, msg: PoseArray):
        self._pillar_candidates = [
            (p.position.x, p.position.y, _quaternion_to_yaw(p.orientation))
            for p in msg.poses
        ]

    def _on_reset_aim(self, request, response):
        self._locked_aim = None
        self._locked_staging = None
        self.get_logger().info('瞄准状态已清空，等待下一次检测命中')
        if bool(self.get_parameter('auto_switch_relay_mode').value):
            self._set_relay_mode('normal')
        response.success = True
        response.message = 'aim reset'
        return response

    def _set_relay_mode(self, mode: str) -> bool:
        """调position_cmd_relay的标准~/set_parameters service切relay_mode。
        阻塞等待结果（超时3秒），失败只记日志不抛异常——接管/交还失败
        不应该让这个节点本身崩掉，飞机大不了继续停在原来的relay_mode下，
        比节点崩溃更安全。"""
        if not self.relay_set_params_cli.service_is_ready():
            self.get_logger().error(
                f'{self.relay_set_params_cli.srv_name}服务不可用，无法把relay_mode切到{mode}——'
                f'检查relay_set_parameters_service参数是否对应真实的position_cmd_relay节点名'
            )
            return False

        req = SetParameters.Request()
        param = Parameter()
        param.name = 'relay_mode'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode)
        req.parameters = [param]

        future = self.relay_set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            self.get_logger().error(f'切换relay_mode到{mode}超时/失败')
            return False
        result = future.result().results[0]
        if not result.successful:
            self.get_logger().error(f'切换relay_mode到{mode}被拒绝: {result.reason}')
        return bool(result.successful)

    def _on_detections(self, msg: Detection2DArray):
        if self._locked_aim is not None:
            return  # 已经锁定过目标，忽略后续检测帧——见文件头"只锁定一次"说明
        if self._odom_x_y_z_yaw is None or not self._pillar_candidates:
            return

        target_id = self.get_parameter('target_class_id').value
        hit = any(
            det.results and det.results[0].hypothesis.class_id == target_id
            for det in msg.detections
        )
        if not hit:
            return

        drone_x, drone_y, drone_z, _ = self._odom_x_y_z_yaw
        max_dist = float(self.get_parameter('max_pillar_match_dist_m').value)
        nearest = min(
            self._pillar_candidates,
            key=lambda c: math.hypot(c[0] - drone_x, c[1] - drone_y),
        )
        pillar_x, pillar_y, pillar_yaw = nearest
        if math.hypot(pillar_x - drone_x, pillar_y - drone_y) > max_dist:
            self.get_logger().warn(
                f'检测到{target_id}但最近候选立柱距离{math.hypot(pillar_x-drone_x, pillar_y-drone_y):.1f}米'
                f'超过max_pillar_match_dist_m={max_dist}，先不锁定，可能候选立柱列表还没更新到位',
                throttle_duration_sec=2.0,
            )
            return

        bearing = math.atan2(drone_y - pillar_y, drone_x - pillar_x)
        face_normal = snap_to_nearest_face(pillar_yaw, bearing)
        standoff = float(self.get_parameter('standoff_m').value)
        lateral = math.radians(float(self.get_parameter('staging_lateral_offset_deg').value))

        self._locked_aim = compute_aim_pose(pillar_x, pillar_y, face_normal, standoff)
        self._locked_staging = compute_staging_pose(pillar_x, pillar_y, face_normal, standoff, lateral)
        # 悬停高度锁定在命中瞬间的当前高度，不随之后飞行过程中的实时高度
        # 漂移——这个节点只管水平位置+朝向，垂直方向刻意保持"命中那一刻
        # 是什么高度就飞去那个高度"，不做额外爬升/下降。
        self._locked_hover_z = drone_z
        self.get_logger().info(
            f'锁定瞄准目标：立柱({pillar_x:.2f},{pillar_y:.2f}) yaw={math.degrees(pillar_yaw):.0f}° '
            f'吸附面法线={math.degrees(face_normal):.0f}° -> '
            f'瞄准点{self._locked_aim} 任务机等待点{self._locked_staging}'
        )
        if bool(self.get_parameter('auto_switch_relay_mode').value):
            if self._set_relay_mode('pillar_aim'):
                self.get_logger().info('position_cmd_relay已自动切到pillar_aim模式，开始接管飞控')

    def _on_control_tick(self):
        if self._locked_aim is None:
            return
        now = self.get_clock().now().to_msg()

        aim_x, aim_y, aim_yaw = self._locked_aim
        aim_cmd = PositionCommand()
        aim_cmd.header.stamp = now
        aim_cmd.header.frame_id = 'map'
        aim_cmd.position.x = aim_x
        aim_cmd.position.y = aim_y
        aim_cmd.position.z = self._locked_hover_z
        aim_cmd.yaw = aim_yaw
        aim_cmd.yaw_dot = 0.0
        self.aim_pub.publish(aim_cmd)
        # 同一份内容再广播一次到fire_pillar_aim_pose——只读状态广播，
        # 不是把fire_pillar_aim_cmd改名，两个话题各自独立发布，见__init__
        # 里aim_pose_pub的注释。
        self.aim_pose_pub.publish(aim_cmd)

        stage_x, stage_y, stage_yaw = self._locked_staging
        stage_cmd = PositionCommand()
        stage_cmd.header.stamp = now
        stage_cmd.header.frame_id = 'map'
        stage_cmd.position.x = stage_x
        stage_cmd.position.y = stage_y
        stage_cmd.position.z = self._locked_hover_z
        stage_cmd.yaw = stage_yaw
        stage_cmd.yaw_dot = 0.0
        self.staging_pub.publish(stage_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FirePillarAimNode()
    # MultiThreadedExecutor配合__init__里client_cb_group的用意，见文件头
    # "死锁教训"说明——单线程executor在这个"回调里再调其它service"的场景
    # 下会死锁，不是这里可以随便省掉的细节。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

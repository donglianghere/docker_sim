#!/usr/bin/env python3
"""场景重置节点——归属`sim-world`容器（原因跟阶段7.2一样：需要读写
Gazebo模型状态，只有sim-world容器有这个权限，见设计方案5.3）。

对应《2026大赛任务系统开发执行方案.md》阶段7.3："提供一个service，
把飞机瞬移回起飞点、清空任务状态机、重新生成阶段1.3里目标物的坐标"。
三件事对应：
1. 飞机瞬移回起飞点：调Gazebo的`SetEntityState`服务，挪回容器启动时
   spawn的原始位置——这个原始位置**不是**写死在这个节点里的，是
   `sim-world-entrypoint.sh`的spawn循环算出来传进来的launch参数
   （`agent_ns`/`agent_reset_x/y/z/yaw`四个数组），保证跟真正spawn时
   用的坐标是同一份数据，不会两处各写一套、慢慢drift出岔子。
2. 清空任务状态机：阶段7.4（真正会维护任务状态的节点）推迟到阶段8
   SDK之后才写，这次没有东西可清——但阶段7.2的`mission_judge_node`
   有内部维护的checkpoint完成状态，场景重置之后这些状态必须一起清空
   （不然道具挪了新位置，裁判还留着"旧位置已达标"的记录），所以这里
   会调用`mission_judge_node`暴露的`~/reset_checkpoints` service。
3. 重新摆放道具坐标：3根候选立柱+1根独立障碍物圆柱+物资点+地面火情
   标识，**位置固定用`fire_drill_room_layout.yaml`里用户指定的坐标**
   （不做随机撒点——2026-09-09用户明确要求"位置布置按我之前给出的
   位置来，不要随机"，此前设计过的min_separation随机撒点方案已废弃）。
   **仍然保留的随机化**是朝向：每次重置给**全部3根立柱**随机挑一个
   连续yaw（0~2π弧度，不限90度倍数），其余模型（障碍物圆柱/物资点/
   地面火情标识）固定yaw=0。
   **2026-09-09第二版**："高层着火点标识"(AprilTag ID1)不再固定贴在
   某根立柱上——`fire_apriltag_marker`是独立模型（不在
   `fixed_positions`里），每次重置额外随机选1根立柱+随机选1个竖直面，
   现算它该在的世界坐标（立柱世界位置+沿该面法线方向偏移
   `mount_standoff_m`，法线方向=立柱当前随机yaw+所选面本地朝向）
   瞬移过去，见`FIRE_TAG_FACE_LOCAL_OFFSETS`。

⚠️ Gazebo服务名`/plug/set_entity_state`是根据world文件里
`gazebo_ros_state`插件的`<namespace>plug</namespace>`配置推算的，
没有在真实跑起来的容器里验证过（写这个节点时容器没有在跑）——如果
build之后调用报"service not available"，先用`ros2 service list`确认
真实服务名，用`set_entity_state_service`参数覆盖，不需要改代码。
消息类型选的是`gazebo_msgs/srv/SetEntityState`+`gazebo_msgs/msg/
EntityState`（字段是`name`，不是ROS1年代`SetModelState`/`ModelState`
那个`model_name`）——这是ROS2版`gazebo_ros_state`插件的现代接口，
但同样没有实机验证过，如果报字段不存在之类的类型错误，这是第二嫌疑。
"""
import math
import random

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from contest_mission.layout_loader import load_layout

# 2026-09-09"高层着火点标识不固定贴哪根立柱"改动：每次重置随机选1根
# 立柱+这4个候选面里随机选1个，把`fire_apriltag_marker`独立模型瞬移到
# 该面对应的世界坐标。(unit_dx, unit_dy)是立柱**本地坐标系**下的单位
# 法线方向，跟立柱当前随机yaw做一次2D旋转就是世界坐标偏移方向；
# extra_yaw是贴片自身相对立柱yaw的额外旋转——贴片box`<size>0.5 0.01
# 0.5</size>`厚度沿本地Y轴，跟±Y面法线天然对齐(extra_yaw=0)，贴到±X面
# 则要把厚度方向转90度对齐X轴(extra_yaw=π/2)。
#
# 2026-09-15订正：贴图实际只在**一面**可见（不是前后对称）。中途改过
# `+Y`的extra_yaw从0改成π，当时是在"pillar_2额外整体转180度"这个hack
# 还生效的情况下测的，pyaw=π+extra_yaw=π会跟pyaw=0+extra_yaw=0算出
# 同一个朝向——两次改动互相抵消，之前判断"extra_yaw=π是对的"这个结论
# 其实是在被这个hack污染的条件下得出的，不能信。hack已撤掉（3根立柱
# 现在统一yaw=0），用户直接在GUI里实测确认：位置挪到(0,8.31)（立柱
# 北侧，远离原点）是对的，但extra_yaw=π会让贴图正面转回朝南（对着
# 原点），等于白挪位置——改回extra_yaw=0，此时tag_yaw=pyaw(0)+0=0，
# 贴图正面朝北，背对原点/NX01起降点，位置和朝向才是一致的。
FIRE_TAG_FACE_LOCAL_OFFSETS = {
    '+X': (1.0, 0.0, math.pi / 2.0),
    '-X': (-1.0, 0.0, math.pi / 2.0),
    '+Y': (0.0, 1.0, 0.0),
    '-Y': (0.0, -1.0, 0.0),
}


class ScenarioResetNode(Node):
    def __init__(self):
        super().__init__('scenario_reset_node')

        # 2026-09-09场景重新设计：默认值改成NX01/NX02新的起降点固定坐标
        # （yaw=90度=机头指向Y轴正方向），实际运行时被sim-world-
        # entrypoint.sh传的launch参数覆盖（跟真正spawn用的是同一份
        # 数字），这里的默认值只在直接单独起这个节点调试时才会用到。
        self.declare_parameter('agent_ns', ['NX01', 'NX02'])
        self.declare_parameter('agent_reset_x', [1.5, -1.5])
        self.declare_parameter('agent_reset_y', [-10.0, -10.0])
        self.declare_parameter('agent_reset_z', [0.1, 0.1])
        self.declare_parameter('agent_reset_yaw', [math.pi / 2.0, math.pi / 2.0])
        self.declare_parameter('set_entity_state_service', '/plug/set_entity_state')

        layout = load_layout()
        # 2026-09-09用户明确要求道具位置不随机、固定用这份yaml里指定的
        # 坐标——每次reset都把下面6个模型摆回各自yaml里写的(x,y)，不
        # 再做随机撒点（min_separation/wall_margin那套随机布局逻辑已
        # 整体移除，不是这里可以顺手保留的死代码）。
        self.fixed_positions = {p['id']: (p['x'], p['y']) for p in layout['pillars']}
        self.fixed_positions['obstacle_cylinder'] = (
            layout['obstacle_cylinder']['x'], layout['obstacle_cylinder']['y'],
        )
        self.fixed_positions['supply_point_marker'] = (
            layout['supply_point']['x'], layout['supply_point']['y'],
        )
        self.fixed_positions['fire_point_marker'] = (
            layout['ground_fire_point']['x'], layout['ground_fire_point']['y'],
        )
        self.pillar_names = [p['id'] for p in layout['pillars']]
        self.fixed_z = {name: 3.0 for name in self.pillar_names}  # 立柱高6米，中心z=3
        self.fixed_z.update({
            'obstacle_cylinder': 3.0,
            'supply_point_marker': 0.0,
            'fire_point_marker': 0.0,
        })
        # 2026-09-09用户纠正：①火情立柱的AprilTag贴片本身姿态是world
        # 文件里的静态bug（贴成水平的、嵌进柱身），跟这里的yaw随机化是
        # 两码事，已经在world文件修掉；②"贴哪个面"不需要单独的离散
        # 选项集合——3根立柱（不分是否贴tag）现在统一做**连续**随机
        # yaw（0~2π弧度，不限90度倍数）。
        #
        # 2026-09-09第二版："贴哪根立柱"也随机了，不再固定贴某一根。
        # `fire_apriltag_marker`（独立模型，见world文件）不在
        # `self.fixed_positions`里——它的世界坐标不是yaml固定值，是
        # 每次reset时现算的（随机选中的那根立柱的当前随机位姿+随机选中
        # 的那个面），单独在`_on_reset_scenario`里处理，不走"遍历
        # fixed_positions"这条通用路径。
        self.fire_apriltag_height_m = float(layout['fire_apriltag_height_m'])
        self.fire_apriltag_mount_standoff_m = float(layout['fire_apriltag_marker']['mount_standoff_m'])

        # ⚠️ 2026-09-08 build后实测踩到的真实死锁：`~/reset_scenario`这个
        # service回调内部要顺序调好几个别的service（SetEntityState x N +
        # mission_judge_node的reset_checkpoints），如果这些client用默认
        # callback group（等价于`~/reset_scenario`自己所在的那个），
        # `rclpy.spin_until_future_complete(self, future)`会在单线程
        # executor下死锁——外层回调（`~/reset_scenario`）自己占着executor
        # 的唯一线程等future，而处理future响应的callback又要靠这个
        # executor线程才能跑，谁都等不到谁，实测8次SetEntityState调用
        # 全部精确卡满3秒超时失败（见DEBUG_JOURNAL.md 2026-09-08"阶段7
        # build验证"条目）。修复：这几个client单独放一个callback group，
        # 配合下面main()里的MultiThreadedExecutor，让"等待响应"和"处理
        # 响应"能分到不同线程，不会自己等自己。
        client_cb_group = MutuallyExclusiveCallbackGroup()
        svc_name = self.get_parameter('set_entity_state_service').value
        self.set_state_cli = self.create_client(SetEntityState, svc_name, callback_group=client_cb_group)

        self.judge_reset_cli = self.create_client(
            Trigger, 'mission_judge_node/reset_checkpoints', callback_group=client_cb_group
        )

        self.create_service(Trigger, '~/reset_scenario', self._on_reset_scenario)

        self.get_logger().info(
            f'scenario_reset_node就绪，set_entity_state服务={svc_name}，'
            f'调用 ros2 service call <ns>/scenario_reset_node/reset_scenario std_srvs/srv/Trigger 触发重置'
        )

    def _set_model_pose(self, model_name: str, x: float, y: float, z: float, yaw: float) -> bool:
        if not self.set_state_cli.service_is_ready():
            self.get_logger().error(
                f'{self.set_state_cli.srv_name}服务不可用，无法重置{model_name}的位置——'
                f'检查set_entity_state_service参数是否对应真实的Gazebo服务名'
            )
            return False

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = model_name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'

        future = self.set_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            self.get_logger().error(f'重置{model_name}位置超时/失败')
            return False
        return bool(future.result().success)

    def _on_reset_scenario(self, request, response):
        summary = []
        all_ok = True

        agent_ns = list(self.get_parameter('agent_ns').value)
        agent_x = list(self.get_parameter('agent_reset_x').value)
        agent_y = list(self.get_parameter('agent_reset_y').value)
        agent_z = list(self.get_parameter('agent_reset_z').value)
        agent_yaw = list(self.get_parameter('agent_reset_yaw').value)
        for ns, x, y, z, yaw in zip(agent_ns, agent_x, agent_y, agent_z, agent_yaw):
            ok = self._set_model_pose(ns, x, y, z, yaw)
            all_ok = all_ok and ok
            summary.append(f'{ns}->({x:.1f},{y:.1f},{z:.1f})' + ('' if ok else 'FAIL'))

        pillar_poses = {}  # {pillar_name: (x, y, yaw)}——留给下面算fire_apriltag_marker位姿用
        for model_name, (x, y) in self.fixed_positions.items():
            # 位置固定用yaml坐标，不随机。2026-09-15用户明确要求"不用再
            # 随机化了"（配合下面固定贴到pillar_2背面这个改动，需要立柱
            # 朝向也是确定值，不然"背面"这个相对概念每次重置都会指向
            # 不同的世界方向，没法测试）——3根立柱的yaw改成固定0度（局部
            # 坐标轴跟世界坐标轴对齐），不再random.uniform。
            # 2026-09-15曾经加过"pillar_2单独转180度"这个hack，配合
            # 下面`chosen_face`凑"贴图挪到另一侧"的效果，但这个hack跟
            # 直接改`chosen_face`选面是同一件事的两种做法，叠加在一起
            # 反而会互相抵消（等价于绕了一圈没变）——用户后来直接明确
            # 要`+Y`面，不需要这个额外转180度的hack，撤掉，3根立柱统一
            # yaw=0（局部坐标轴跟世界坐标轴对齐）。
            yaw = 0.0
            ok = self._set_model_pose(model_name, x, y, self.fixed_z[model_name], yaw)
            all_ok = all_ok and ok
            yaw_note = f' yaw={math.degrees(yaw):.0f}°' if model_name in self.pillar_names else ''
            summary.append(f'{model_name}->({x:.2f},{y:.2f}){yaw_note}' + ('' if ok else 'FAIL'))
            if model_name in self.pillar_names:
                pillar_poses[model_name] = (x, y, yaw)

        # 高层着火点标识：本来是随机选1根立柱+随机选1个竖直面，2026-09-15
        # 用户明确要求"高层火情要设置到2#的背面，不能让NX01起飞就能看到，
        # 不用再随机化了"——固定贴到pillar_2。中途走过弯路：①按理论推算
        # 选过`+Y`+额外180度转向，又加过"pillar_2单独转180度"的hack，
        # 两者叠加互相抵消，等于没变；②怀疑过GUI不刷新，用`pillar_3`
        # 做过诊断，确认了SetEntityState本身生效，gzclient只是运行中
        # 热改模型位姿不会自动重绘，必须完整重建容器（`docker compose
        # down+up`，不是restart/单独重启gzclient）才能看到最新画面。
        # 排除掉这些干扰之后，用户直接明确要`+Y`面，不用再猜。瞬移算法
        # 本身没变，见`FIRE_TAG_FACE_LOCAL_OFFSETS`注释，本质是"立柱
        # 世界位置 + 沿(立柱当前yaw+所选面本地朝向)方向偏移
        # mount_standoff_m"；现在3根立柱yaw都固定0，`+Y`面直接对应
        # 世界+Y方向偏移，不再跟pillar自身旋转hack混在一起算。
        chosen_pillar = 'pillar_2'
        chosen_face = '+Y'
        px, py, pyaw = pillar_poses[chosen_pillar]
        unit_dx, unit_dy, extra_yaw = FIRE_TAG_FACE_LOCAL_OFFSETS[chosen_face]
        standoff = self.fire_apriltag_mount_standoff_m
        world_dx = unit_dx * standoff * math.cos(pyaw) - unit_dy * standoff * math.sin(pyaw)
        world_dy = unit_dx * standoff * math.sin(pyaw) + unit_dy * standoff * math.cos(pyaw)
        tag_x = px + world_dx
        tag_y = py + world_dy
        tag_yaw = pyaw + extra_yaw
        ok = self._set_model_pose('fire_apriltag_marker', tag_x, tag_y, self.fire_apriltag_height_m, tag_yaw)
        all_ok = all_ok and ok
        summary.append(
            f'fire_apriltag_marker->{chosen_pillar}({chosen_face}面) '
            f'({tag_x:.2f},{tag_y:.2f},{self.fire_apriltag_height_m:.1f}) '
            f'yaw={math.degrees(tag_yaw):.0f}°' + ('' if ok else 'FAIL')
        )

        if self.judge_reset_cli.service_is_ready():
            future = self.judge_reset_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() is None or not future.result().success:
                summary.append('mission_judge_node checkpoint重置FAIL')
                all_ok = False
            else:
                summary.append('mission_judge_node checkpoint已重置')
        else:
            summary.append('mission_judge_node/reset_checkpoints服务不可用，跳过（裁判状态可能残留旧值）')

        response.success = all_ok
        response.message = '; '.join(summary)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioResetNode()
    # MultiThreadedExecutor配合__init__里client_cb_group的用意，见那里的
    # 注释——单线程executor在这个"service回调里再调其它service"的场景下
    # 会死锁，不是这里可以随便省掉的细节。
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

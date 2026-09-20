#!/usr/bin/env python3
"""环绕飞行航点生成——纯几何函数，不是ROS节点。

对应《2026大赛任务系统开发执行方案.md》阶段4.2。给阶段7任务状态机调用：
拿到阶段4.1输出的候选立柱坐标之后，生成一圈离散航点喂给现有的
`waypoint_queue`接口（`ego_planner_bridge`已有的多航点顺序飞行能力，
见该包`waypoint_queue_node.py`/相关launch文件），配合
`position_cmd_relay`切到`orbit_yaw_override`模式实时算机头朝向——这个
模块只管"给一个圆心+半径，吐一圈航点坐标"，不管yaw（yaw由中继节点在
飞行时实时算，不是在这里预先算好写进航点里，因为yaw需要飞机当前
位置这个运行时信息，跟航点生成是两件独立的事）。
"""
import math
from typing import List, Tuple


def generate_orbit_waypoints(
    center_x: float,
    center_y: float,
    radius: float,
    z: float,
    num_points: int = 16,
    start_angle_rad: float = 0.0,
    clockwise: bool = False,
) -> List[Tuple[float, float, float]]:
    """生成一圈环绕航点（按角度等分，不是按弧长等分——半径固定的圆环
    两者等价）。

    Args:
        center_x/center_y: 环绕中心（一般是阶段4.1输出的候选立柱坐标）。
        radius: 环绕半径（米），需要选手/任务逻辑保证比立柱半对角线
            +飞机机体半径的安全间距大，本函数不做碰撞检查。
        z: 环绕飞行高度（米），全程固定高度绕圈，不爬升/下降。
        num_points: 一圈离散成几个航点，越多越接近圆、路径越平滑，但
            waypoint_queue要挨个跑完，点数太多会拖慢整体任务节奏，
            默认16个（每22.5度一个点）是精度和效率的折中，不是理论
            最优值。
        start_angle_rad: 起始角度（弧度，0=+x方向，跟math.atan2同一套
            约定），默认从飞机当前朝向立柱的那一侧开始更自然，但那个
            "当前朝向"是运行时信息，这个函数本身是纯几何、不读运行时
            状态，调用方（阶段7任务逻辑）如果想要"从飞机当前位置最近的
            那个点开始绕"，需要自己算好start_angle_rad传进来。
        clockwise: 环绕方向，默认False=逆时针（数学正方向）。

    Returns:
        长度为num_points的(x, y, z)列表，按环绕方向排列，**首尾不重复**
        （最后一个点转回起点之前那个点，不会再生成一次跟起点重合的点，
        调用方如果需要"飞完整整一圈回到起点"，需要自己在末尾再追加
        一次第一个航点）。
    """
    if num_points < 3:
        raise ValueError(f'num_points必须>=3才能围出一个环，收到{num_points}')
    if radius <= 0:
        raise ValueError(f'radius必须>0，收到{radius}')

    sign = -1.0 if clockwise else 1.0
    waypoints = []
    for i in range(num_points):
        angle = start_angle_rad + sign * (2.0 * math.pi * i / num_points)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        waypoints.append((x, y, z))
    return waypoints


def angle_from_center_to_point(center_x: float, center_y: float, x: float, y: float) -> float:
    """给"从飞机当前位置最近的环绕点开始绕"这类调用方算
    start_angle_rad用的小工具：飞机当前位置相对环绕中心的角度。"""
    return math.atan2(y - center_y, x - center_x)

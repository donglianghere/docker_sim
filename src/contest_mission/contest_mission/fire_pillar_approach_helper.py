#!/usr/bin/env python3
"""高层着火点"正对瞄准"相关的纯几何计算——不是ROS节点，供
`fire_pillar_aim_node.py`调用，跟`orbit_flight_helper.py`/
`ground_scan_helper.py`同一个模式（纯函数、无ROS依赖，方便单独测试）。

背景：立柱是0.6x0.6米正方形截面，AprilTag ID1（"高层着火点标识"）
2026-09-09起随机贴在其中一根立柱的某一个竖直面上（见
`scenario_reset_node.py`的`FIRE_TAG_FACE_LOCAL_OFFSETS`）。侦查机检测到
这个tag之后，要飞到"贴tag那个面正前方、机头对准中心"的悬停点——这个
点不能直接用"检测命中那一刻无人机自己的方位角"，因为命中时刻不一定
精确站在正对那个面的位置（可能是从斜侧方先看到的），所以要先靠立柱
自身朝向（激光雷达点云拟合出来的，不是视觉解算）把命中方位角"吸附"到
最近的一个候选面法线方向上，再算目标点——这样目标点精度取决于立柱
朝向的雷达解算精度，不受"具体是哪一帧先触发检测"这个随机性影响。

任务机（负责"投射"动作的第二架飞机）不能跟侦查机挤在同一条正对立柱
中心的直线上（会互相遮挡/有碰撞风险），所以额外提供一个"侧向偏移"的
等待点计算函数——同一个目标面，但方位角上下错开一个角度，先在旁边
等，等侦查机让开之后再切到正前方那条线上。
"""
import math
from typing import Tuple

# 立柱是正方形截面，4个候选竖直面的法线方向——相对立柱自身yaw的偏移量，
# 覆盖0/90/180/270度这4个面，不多不少，跟`scenario_reset_node.py`里
# `FIRE_TAG_FACE_LOCAL_OFFSETS`描述的4个候选面完全对应（贴哪个面是柱身
# 随机转向的自然结果，这里反过来"猜"tag贴在哪个面，同样只需要考虑这4个
# 候选方向）。
_FACE_NORMAL_LOCAL_OFFSETS = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)


def _normalize_angle(angle_rad: float) -> float:
    """归一化到(-pi, pi]，避免累加/比较角度时因为跨越±pi边界算错。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def snap_to_nearest_face(pillar_yaw_rad: float, bearing_rad: float) -> float:
    """把一个连续方位角"吸附"到立柱4个候选竖直面法线里最接近的那个。

    Args:
        pillar_yaw_rad: 立柱自身当前朝向（世界系，弧度）——来自
            `pillar_detector_node`用点云拟合出的最小外接矩形角度，不是
            视觉解算。
        bearing_rad: 待吸附的方位角（世界系，弧度）——通常是"立柱中心
            指向无人机检测命中时刻所在位置"的方向
            `atan2(drone_y-pillar_y, drone_x-pillar_x)`。

    Returns:
        4个候选面法线（pillar_yaw_rad + 0/90/180/270度）里跟
        bearing_rad夹角最小的那个，归一化到(-pi, pi]。立柱4面互相垂直，
        任意方位角跟最近那个候选面的夹角不会超过45度，吸附结果总是
        明确的（不会有两个候选面同样近的退化情况，除非恰好卡在
        45度边界上，这种概率为0的边界情况不特殊处理）。
    """
    best_face = None
    best_diff = None
    for offset in _FACE_NORMAL_LOCAL_OFFSETS:
        candidate = _normalize_angle(pillar_yaw_rad + offset)
        diff = abs(_normalize_angle(bearing_rad - candidate))
        if best_diff is None or diff < best_diff:
            best_diff = diff
            best_face = candidate
    return best_face


def compute_aim_pose(
    pillar_x: float, pillar_y: float, face_normal_rad: float, standoff_m: float,
) -> Tuple[float, float, float]:
    """算"贴tag那个面正前方"的悬停点位姿。

    位置 = 立柱中心 沿face_normal_rad方向后退standoff_m；朝向 = 转回头
    正对立柱中心——站在法线延长线上、机头对准圆心，这个朝向天然垂直于
    该面（法线方向的反方向就是"正对着看"的朝向），不需要额外的姿态
    换算。

    Returns:
        (x, y, yaw)，世界系。
    """
    x = pillar_x + standoff_m * math.cos(face_normal_rad)
    y = pillar_y + standoff_m * math.sin(face_normal_rad)
    yaw = math.atan2(pillar_y - y, pillar_x - x)
    return x, y, yaw


def compute_staging_pose(
    pillar_x: float, pillar_y: float, face_normal_rad: float, standoff_m: float,
    lateral_offset_rad: float,
) -> Tuple[float, float, float]:
    """算任务机的等待点位姿——同一个目标面，方位角上再叠加一个侧向
    偏移`lateral_offset_rad`（正负决定偏左/偏右），避开侦查机"正前方"
    那条进场直线，同时朝向仍然对准立柱中心（任务机让开之后只需要小
    角度转向就能切到正前方，不用大幅度重新定位）。

    Args:
        lateral_offset_rad: 相对`compute_aim_pose`那条正前方直线的
            角度偏移，正数=逆时针偏、负数=顺时针偏，调用方自己决定
            偏多少（典型取值±30度量级，够避开侦查机机身+相机视野，
            又不会绕远路）。

    Returns:
        (x, y, yaw)，世界系。
    """
    angle = face_normal_rad + lateral_offset_rad
    x = pillar_x + standoff_m * math.cos(angle)
    y = pillar_y + standoff_m * math.sin(angle)
    yaw = math.atan2(pillar_y - y, pillar_x - x)
    return x, y, yaw

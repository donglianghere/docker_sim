#!/usr/bin/env python3
"""地面着火点搜索航点生成——纯几何函数，不是ROS节点。

对应《2026大赛任务系统开发执行方案.md》阶段5.1。跟`orbit_flight_
helper.py`同样的定位：只管"给房间边界+相机参数+飞行高度，吐一条弓字形
（boustrophedon/lawnmower）固定航点序列"，喂给`waypoint_queue`接口用；
边扫边订阅阶段2检测结果、命中即中断剩余航点，这部分任务逻辑留给阶段7
的任务状态机，这里不做。
"""
import math
from typing import List, Tuple


def compute_ground_footprint(
    altitude_agl: float,
    hfov_rad: float,
    image_aspect_ratio: float = 480.0 / 640.0,
) -> Tuple[float, float]:
    """下视相机在给定飞行高度下，地面覆盖矩形的(宽, 高)，单位米。

    Gazebo的camera sensor只配了horizontal_fov（见gen_iris_mid360_sdf.py
    的CAMERA_MOUNTS），vertical_fov是按图像宽高比例算出来的
    （标准针孔相机模型，Gazebo/常见相机驱动都是这套换算），这里照抄
    同一套公式，不是凭空定义。
    """
    if not (0 < hfov_rad < math.pi):
        raise ValueError(f'hfov_rad必须在(0, pi)范围内，收到{hfov_rad}')
    if altitude_agl <= 0:
        raise ValueError(f'altitude_agl必须>0，收到{altitude_agl}')

    vfov_rad = 2.0 * math.atan(math.tan(hfov_rad / 2.0) * image_aspect_ratio)
    width = 2.0 * altitude_agl * math.tan(hfov_rad / 2.0)
    height = 2.0 * altitude_agl * math.tan(vfov_rad / 2.0)
    return width, height


def generate_ground_scan_waypoints(
    room_min_x: float,
    room_max_x: float,
    room_min_y: float,
    room_max_y: float,
    altitude_agl: float,
    hfov_rad: float = 1.3963,  # 跟gen_iris_mid360_sdf.py的CAMERA_MOUNTS['down']一致(80度)
    image_aspect_ratio: float = 480.0 / 640.0,
    overlap_ratio: float = 0.3,
    wall_margin: float = 1.0,
) -> List[Tuple[float, float, float]]:
    """生成弓字形（沿x方向来回扫，行与行之间沿y方向递进）固定航点序列。

    Args:
        room_min_x/max_x/min_y/max_y: 房间边界矩形（世界坐标系，跟
            fire_drill_room_layout.yaml的room.size_x/size_y配合，
            调用方自己算好min/max传进来，比如size_x=20时
            min_x=-10/max_x=10——这个函数不假设房间中心一定在原点，
            调用方传实际边界即可）。
        altitude_agl: 飞行高度（离地，米），全程固定高度扫描。
        hfov_rad: 下视相机水平FOV（弧度），默认跟仿真下视相机实际参数
            一致，真的换了相机参数记得同步这个默认值。
        image_aspect_ratio: 图像高/宽比例，用于算垂直FOV，默认跟仿真
            相机640x480一致。
        overlap_ratio: 相邻扫描行的重叠比例（0~1之间，比如0.3=30%
            重叠）——**取地面覆盖矩形"较窄的那条边"当扫描行间距的基准**
            （不管相机实际是横着装还是竖着装，都按更保守的那个方向留
            间距，保证真的有重叠，不会因为搞错哪个轴对应飞行方向就漏扫）。
        wall_margin: 离四面墙的安全距离（米），航点不会贴到墙上。

    Returns:
        (x, y, z)航点列表，按扫描顺序排列（第一行从x_min飞到x_max，
        第二行从x_max飞回x_min，如此往复，"弓字形"由此得名）。
    """
    if room_max_x - 2 * wall_margin <= room_min_x or room_max_y - 2 * wall_margin <= room_min_y:
        raise ValueError('wall_margin太大，房间可用范围被挤没了')
    if not (0.0 <= overlap_ratio < 1.0):
        raise ValueError(f'overlap_ratio必须在[0,1)范围内，收到{overlap_ratio}')

    footprint_w, footprint_h = compute_ground_footprint(altitude_agl, hfov_rad, image_aspect_ratio)
    # 保守起见取较窄的一边当扫描行间距基准——不假设相机哪个轴对准了
    # 飞行方向，宁可扫描行数偏多（保守），也不要因为猜错方向导致漏扫。
    sweep_span = min(footprint_w, footprint_h)
    row_spacing = sweep_span * (1.0 - overlap_ratio)
    if row_spacing <= 0:
        raise ValueError(f'overlap_ratio={overlap_ratio}太大，算出的row_spacing<=0，扫描行会重叠成同一条线')

    x0 = room_min_x + wall_margin
    x1 = room_max_x - wall_margin
    y0 = room_min_y + wall_margin
    y1 = room_max_y - wall_margin

    waypoints: List[Tuple[float, float, float]] = []
    y = y0
    left_to_right = True
    while True:
        y_clamped = min(y, y1)
        if left_to_right:
            waypoints.append((x0, y_clamped, altitude_agl))
            waypoints.append((x1, y_clamped, altitude_agl))
        else:
            waypoints.append((x1, y_clamped, altitude_agl))
            waypoints.append((x0, y_clamped, altitude_agl))
        left_to_right = not left_to_right
        if y_clamped >= y1 - 1e-9:
            break
        y += row_spacing

    return waypoints

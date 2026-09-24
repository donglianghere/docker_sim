#!/usr/bin/env python3
"""通用几何helper——环绕航点/地面扫描航点生成，纯函数，跟ROS完全无关。

**这是一份vendor过来的拷贝，不是`import contest_mission`**：方案2.1节第8条
明确指出`orbit_flight_helper.generate_orbit_waypoints()`/`ground_scan_helper.
generate_ground_scan_waypoints()`这两个纯函数原本活在`contest_mission`包
（跑在flight-stack容器里）——但全流程任务程序实际跑在选手的`contestant-sdk`
容器/Python环境里，跟flight-stack是两个不同的容器，选手容器不应该也不需要
去装`contest_mission`这个跟ROS2节点强耦合的大包（3.1节"选手镜像本来就不
包含飞行栈源码"这条隔离要求）。这两个函数本身只做纯几何计算（`math`模块，
不`import rclpy`/不碰任何ROS消息类型），复制一份到这里不会产生"以后两边
改法不一致"的维护负担太大的问题（方案原话）——如果以后`contest_mission`
那一侧的实现变了，这里需要人工同步一份，但风险很低、频率也低。

逐个函数对应关系（源文件路径均在`src/contest_mission/contest_mission/`下）：
- `generate_orbit_waypoints()`/`angle_from_center_to_point()`——照抄
  `orbit_flight_helper.py`，逻辑一字不改。
- `compute_ground_footprint()`/`generate_ground_scan_waypoints()`——照抄
  `ground_scan_helper.py`，逻辑一字不改。

对外暴露方式：`capabilities.py`里`DroneSDK.generate_orbit_waypoints()`/
`DroneSDK.generate_ground_scan_waypoints()`两个实例方法直接转调这里的
模块级函数（方案2.1节第8条"作为sdk.generate_orbit_waypoints(...)这两个
静态方法/独立函数对外暴露"，两种形式都提到了——这里两种都留：既能
`sdk.generate_orbit_waypoints(...)`调，也能`from contest_sdk.geometry_
helpers import generate_orbit_waypoints`直接当纯函数用，不依赖任何
`DroneSDK`实例，方便写不需要真实ROS2环境的单元测试）。
"""
import math
from typing import List, Optional, Tuple


# ============================================================
# 第1部分：环绕飞行航点生成（vendor自`orbit_flight_helper.py`）
# ============================================================

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
        center_x/center_y: 环绕中心（一般是候选立柱坐标）。
        radius: 环绕半径（米），需要调用方保证比立柱半对角线+飞机机体
            半径的安全间距大，本函数不做碰撞检查。
        z: 环绕飞行高度（米），全程固定高度绕圈，不爬升/下降。
        num_points: 一圈离散成几个航点，越多越接近圆、路径越平滑，但
            要挨个跑完，点数太多会拖慢整体任务节奏，默认16个（每22.5度
            一个点）是精度和效率的折中，不是理论最优值。
        start_angle_rad: 起始角度（弧度，0=+x方向，跟math.atan2同一套
            约定），默认从飞机当前朝向立柱的那一侧开始更自然，但那个
            "当前朝向"是运行时信息，这个函数本身是纯几何、不读运行时
            状态，调用方如果想要"从飞机当前位置最近的那个点开始绕"，
            需要自己算好start_angle_rad传进来（可以配合
            `angle_from_center_to_point()`）。
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


# ============================================================
# 第2部分：地面扫描航点生成（vendor自`ground_scan_helper.py`）
# ============================================================

def compute_ground_footprint(
    altitude_agl: float,
    hfov_rad: float,
    image_aspect_ratio: float = 480.0 / 640.0,
) -> Tuple[float, float]:
    """下视相机在给定飞行高度下，地面覆盖矩形的(宽, 高)，单位米。

    标准针孔相机模型换算（跟仿真相机驱动/Gazebo同一套公式，不是凭空
    定义），只由`hfov_rad`+`altitude_agl`+图像宽高比决定，跟具体某一次
    检测/某一架飞机无关。
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
    hfov_rad: float = 1.3963,  # 跟仿真下视相机默认水平FOV一致(约80度)
    image_aspect_ratio: float = 480.0 / 640.0,
    overlap_ratio: float = 0.3,
    wall_margin: float = 1.0,
    target_size_m: float = 0.0,
    start_from_x_max: bool = False,
) -> List[Tuple[float, float, float]]:
    """生成弓字形（沿x方向来回扫，行与行之间沿y方向递进）固定航点序列。

    Args:
        room_min_x/max_x/min_y/max_y: 房间边界矩形（世界坐标系），调用方
            自己算好min/max传进来——这个函数不假设房间中心一定在原点。
        altitude_agl: 飞行高度（离地，米），全程固定高度扫描。
        hfov_rad: 下视相机水平FOV（弧度），默认跟仿真相机实际参数一致，
            真的换了相机参数记得同步这个默认值。
        image_aspect_ratio: 图像高/宽比例，用于算垂直FOV，默认跟仿真
            相机640x480一致。
        overlap_ratio: 相邻扫描行的重叠比例（0~1之间，比如0.3=30%
            重叠）——取地面覆盖矩形"较窄的那条边"当扫描行间距的基准
            （不管相机实际是横着装还是竖着装，都按更保守的那个方向留
            间距，保证真的有重叠，不会因为搞错哪个轴对应飞行方向就漏扫）。
        wall_margin: 离四面墙的安全距离（米），航点不会贴到墙上。
        target_size_m: 要找的目标自身尺寸（米），默认0（跟原来行为一致）。
            2026-09-22新增：检测器必须看到**完整**的目标才认得出来，只露出
            一部分时画面对比度会升高、但检测为0——实测1.5米搜索时有一行离火点
            约0.5米经过，标签被画面边缘截断，没检出。所以有效覆盖宽度要扣掉
            目标尺寸：行间距 = (覆盖较窄边 - target_size_m) * (1 - overlap_ratio)，
            保证任何位置的目标都至少在某一行里完整入画。
        start_from_x_max: 第一行从x_max端起扫（默认False=从x_min端起）。
            2026-09-24新增，用户提的：飞机停在场地哪一侧，就该从那一侧
            开始扫——侦察机起降点在x正方向那一半，原来固定从x_min起扫，
            开扫之前要先空飞整个场地宽度（实测8米）到对面去，而那条"去
            程"贴着两机起降点的连线飞，那条线上按规则不会摆火情目标，
            等于纯浪费时间。哪一端更近由调用方判断（它才知道飞机在哪），
            这里只负责按要求生成，覆盖范围两种起法完全一样。

    Returns:
        (x, y, z)航点列表，按扫描顺序排列（第一行从起扫那一端飞到对面，
        第二行折回来，如此往复，"弓字形"由此得名）。
    """
    if room_max_x - 2 * wall_margin <= room_min_x or room_max_y - 2 * wall_margin <= room_min_y:
        raise ValueError('wall_margin太大，房间可用范围被挤没了')
    if not (0.0 <= overlap_ratio < 1.0):
        raise ValueError(f'overlap_ratio必须在[0,1)范围内，收到{overlap_ratio}')

    footprint_w, footprint_h = compute_ground_footprint(altitude_agl, hfov_rad, image_aspect_ratio)
    # 保守起见取较窄的一边当扫描行间距基准——不假设相机哪个轴对准了
    # 飞行方向，宁可扫描行数偏多（保守），也不要因为猜错方向导致漏扫。
    sweep_span = min(footprint_w, footprint_h) - target_size_m
    if sweep_span <= 0:
        raise ValueError(
            f'target_size_m={target_size_m}米不小于这个高度下的覆盖宽度'
            f'{min(footprint_w, footprint_h):.2f}米，目标不可能完整入画，要飞高一点')
    row_spacing = sweep_span * (1.0 - overlap_ratio)
    if row_spacing <= 0:
        raise ValueError(f'overlap_ratio={overlap_ratio}太大，算出的row_spacing<=0，扫描行会重叠成同一条线')

    x0 = room_min_x + wall_margin
    x1 = room_max_x - wall_margin
    y0 = room_min_y + wall_margin
    y1 = room_max_y - wall_margin

    waypoints: List[Tuple[float, float, float]] = []
    y = y0
    left_to_right = not start_from_x_max
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


def goto_stalled(
    history: List[Tuple[float, float, float, float]],
    target_xyz: Tuple[float, float, float],
    window_s: float,
    min_move_m: float,
    min_dist_m: float,
) -> bool:
    """判断飞往目标点的过程中飞机是否已经卡住（2026-09-21）。

    `history`是按时间顺序的`(t, x, y, z)`位置采样。卡住的定义：最近
    `window_s`秒内（三维）位移小于`min_move_m`，**并且**当前离目标还有
    `min_dist_m`以上。

    为什么用"一段时间内的位移"而不是"瞬时速度"：实测里程计速度大小的噪声
    均值约0.04m/s、峰值超过0.1m/s（2026-09-21 静止时测的，见
    scripts/measure_static_noise.py），"瞬时速度连续5秒都低于0.1"即使飞机
    完全静止也很难满足；而位置噪声的最大偏移实测只有0.2米左右，"5秒内
    位移小于0.5米"对噪声很稳。

    为什么用三维而不是水平：纯升降的goto（x、y不变只改z）水平位移恒为0，
    按水平算会被误判成卡住。

    为什么`min_dist_m`取得比较小（调用方默认0.5米）：目标点落在障碍物正中
    心时，飞机停下的位置离目标 = 障碍物半径 + 膨胀半径。仿真里障碍圆柱
    半径0.25、膨胀0.6，停在约0.85~0.95米处——门限要是取1米，恰好会把这个
    最典型的情况漏掉。

    采样跨度不足`window_s`时一律返回False：刚起步时规划器还没出轨迹、
    飞机还在加速，那段不能算卡住。
    """
    if len(history) < 2:
        return False
    t_now = history[-1][0]
    if t_now - history[0][0] < window_s:
        return False
    # 取窗口起点：最后一个不晚于 t_now - window_s 的样本
    start = history[0]
    for sample in history:
        if sample[0] <= t_now - window_s:
            start = sample
        else:
            break
    cur = history[-1]
    moved = math.sqrt(sum((cur[i] - start[i]) ** 2 for i in (1, 2, 3)))
    dist = math.sqrt(sum((cur[i + 1] - target_xyz[i]) ** 2 for i in range(3)))
    return moved < min_move_m and dist > min_dist_m


def pull_waypoints_out_of_circles(
    waypoints: List[Tuple[float, float, float]],
    circles: List[Tuple[float, float, float]],
    clearance_m: float,
) -> List[Tuple[float, float, float]]:
    """把落进已知圆形障碍物（含余量）里的航点沿来路往回挪到外面（2026-09-21）。

    用于弓字形搜索航线：已知坐标的障碍物（比如题目给的3根立柱）在规划航线
    时就能处理掉，不用等飞过去卡住再说。未知坐标的障碍物（障碍圆柱、仿地
    模块——按方案不能把坐标写进代码）处理不了，那部分靠`goto()`的卡住检测
    兜底，见`GotoUnreachableError`。

    Args:
        waypoints: `(x, y, z)`航点序列。
        circles: `(cx, cy, r)`已知障碍物的外接圆。方形立柱传半对角线长度
            （比如边长0.6米的立柱传0.42），不是半边长——按半边长算，从对角
            方向过来的航点会漏判。
        clearance_m: 在`r`之外再留的余量。**必须不小于规划器的障碍物膨胀
            半径**（仿真0.6、真机0.8，见`EGO_OBSTACLES_INFLATION`），否则挪出
            来的点仍在膨胀区里，规划器照样到不了。

    怎么挪：沿"上一个航点 -> 这个航点"这条线段往回退，退到刚好出圆的位置。
    这样挪完的点仍然在原来那条扫描线上，弓字结构不被打乱；径向往外推会把
    点推离扫描线。第一个航点没有"上一个"，用它和下一个航点那条线段。
    整条线段都在圆里（退不出来）的航点直接丢掉——那一段地面本来就被障碍物
    占着，不可能有地面火情。

    Returns:
        处理后的新航点列表（不修改传入的列表）。
    """
    expanded = [(cx, cy, r + clearance_m) for cx, cy, r in circles]

    def inside(x: float, y: float) -> bool:
        return any((x - cx) ** 2 + (y - cy) ** 2 < rr * rr for cx, cy, rr in expanded)

    def entry_t(qx, qy, px, py, cx, cy, rr) -> Optional[float]:
        """线段 Q->P 进入圆的参数 t（P 在圆内时，返回最后一个在圆外的 t）。"""
        dx, dy = px - qx, py - qy
        fx, fy = qx - cx, qy - cy
        a = dx * dx + dy * dy
        if a < 1e-12:
            return None
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - rr * rr
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        t1 = (-b - math.sqrt(disc)) / (2 * a)
        return t1 if t1 >= 0.0 else None

    out: List[Tuple[float, float, float]] = []
    for i, (px, py, pz) in enumerate(waypoints):
        if not inside(px, py):
            out.append((px, py, pz))
            continue
        if out:
            qx, qy, _ = out[-1]
        elif i + 1 < len(waypoints):
            qx, qy, _ = waypoints[i + 1]
        else:
            continue          # 唯一一个航点还在圆里，没有参照线段，丢掉
        if inside(qx, qy):
            continue          # 参照点自己也在圆里，整段退不出来，丢掉
        t = 1.0
        for _ in range(len(expanded) + 1):   # 多个圆时逐个退，直到都在外面
            x, y = qx + t * (px - qx), qy + t * (py - qy)
            hit = [(cx, cy, rr) for cx, cy, rr in expanded
                   if (x - cx) ** 2 + (y - cy) ** 2 < rr * rr]
            if not hit:
                break
            ts = [entry_t(qx, qy, px, py, *c) for c in hit]
            if any(v is None for v in ts):
                t = None
                break
            t = min(ts) - 1e-6
        if t is None or t <= 0.0:
            continue
        out.append((qx + t * (px - qx), qy + t * (py - qy), pz))
    return out

"""这次contest task题目允许提前给的已知坐标常量（对应方案0.4节/清单D1
节"已知坐标常量"）。

**这些坐标是选手直接写成程序里的字面量常量，不是读`fire_drill_room_
layout.yaml`**——第四部分程序跑在`contest_sdk`容器里，根本没有
`contest_mission`包/这份yaml文件（3.1节隔离设计，选手容器里不包含
仿真栈源码），真机部署时也是主办方直接给数值、选手抄进代码，从来不
涉及读取任何配置文件。仿真环境里这几个数值直接抄自`fire_drill_room_
layout.yaml`当前的值（抄一次、写死，不是运行时读取），真机部署时
换成主办方现场给的实际数值即可，改的是这几个常量的值，不是读取方式。

**明确不在这份常量表里的东西**（方案0.4节"障碍物/仿地模块坐标视为
未知"+"地面火情/高层火情坐标需要现场发现"）：障碍物圆柱、仿地模块、
地面火情标识(AprilTag2)、高层火情标识(AprilTag1)的坐标——这几个要靠
传感器实时感知/现场探测，不允许写进代码，写了就是"上帝视角"作弊。
"""

# ---- 第一部分任务：航线编队飞行的4个途经点（方案4.2节，题目直接给的
#      具体数值，不是从yaml读——这几个点跟下面takeoff_landing_pads里
#      各自的起降点是两回事，不要混）----

# 航点可由选手替换/增删（2026-09-20起支持命令行 --route 覆盖）：至少2个点，
# 多则不限，按列表顺序依次飞过。坐标是世界系 (x, y)，高度统一用下面的
# CRUISE_AGL_M。
ROUTE_WAYPOINTS_XY = [
    (7.0, -10.0),
    (7.0, 10.0),
    (-8.0, 10.0),
    (-8.0, -10.0),
]

# 巡航AGL高度（方案4.2节"1.5是AGL目标高度"）——全流程程序里所有普通
# 航线goto()调用的z统一用这个值，不需要从yaml读z（yaml本来也没有z字段）。
CRUISE_AGL_M = 1.5

# 长距离分段插值的最大步长（方案4.2节，应对ego_planner长距离重规划
# 死循环的坑）。
MAX_GOTO_STEP_M = 3.0

# ---- 物资点（方案4.3.1节，题目允许提前给的已知坐标）----
SUPPLY_POINT_XY = (-4.0, -6.0)
SUPPLY_POINT_APRILTAG_ID = 'apriltag:0'

# ---- 3根候选立柱坐标（方案4.3.2节，题目允许提前给的已知坐标，"编号"
#      是仿真配置文件自己起的名字，不是题目给的——程序自己按D1的确定性
#      排序规则重新编号，不使用下面这个顺序本身当编号）----
# 2026-09-14：3#立柱从(-6.0,2.0)改成(-5.0,2.0)（跟fire_drill_room_
# layout.yaml同步），原因见该yaml文件pillars小节的注释——D3航线最后
# 一段离x=-10房间墙只有2米净空，(-6,2)离这条航线也只有2米，ego_planner
# 局部避墙的实际航迹被挤进这条窄廊道，实测撞上过3#。
CANDIDATE_PILLARS_XY = [
    (4.0, 4.0),
    (0.0, 8.0),
    (-5.0, 2.0),
]

# 立柱绕飞半径/高度（方案4.3.2节"generate_orbit_waypoints(cx, cy,
# radius=2.5, z_agl=2.0, num_points=12)"给的具体数值）。
PILLAR_ORBIT_RADIUS_M = 2.5
# 2026-09-15用户明确要求绕飞高度要跟高层火情tag挂载高度一致（这里的z
# 是世界坐标Z，不是真的"AGL相对起降点"——见_goto_world_xy()文档，绕飞
# 航点是世界坐标，直接传给world_to_local()，跟world地面z=0近似重合的
# 场景下数值上等价于AGL）：原来2.0跟tag挂载高度3.0差1米，绕飞时前视
# 相机会往上仰角看tag、不是水平对视，抬高到3.0跟fire_drill_room_
# layout.yaml的fire_apriltag_height_m保持一致（同一份"抄一次、写死"
# 的yaml副本，见本文件头说明，yaml改了这里要跟着手动同步）。
PILLAR_ORBIT_Z_AGL_M = 3.0
PILLAR_ORBIT_NUM_POINTS = 12

# ---- 每架飞机自己的起降点（方案4.3节D4/D6提到"飞回自己的起降点"，
#      按物理飞机的namespace查，不是按role查——namespace本身是跟物理
#      飞机绑定的必填参数，不是1.5节禁止的"按namespace分支写业务逻辑"，
#      这里只是查自己是谁，不是判断该做A还是做B）----
TAKEOFF_LANDING_PAD_XY = {
    'NX01': (1.5, -10.0),
    'NX02': (-1.5, -10.0),
}


def own_landing_pad_xy(namespace: str):
    """查自己这架飞机的起降点局部坐标（x, y）。"""
    key = namespace.strip('/').upper()
    if key not in TAKEOFF_LANDING_PAD_XY:
        raise ValueError(
            f"namespace={namespace!r}没有对应的起降点坐标，"
            f"已知的只有{list(TAKEOFF_LANDING_PAD_XY)}")
    return TAKEOFF_LANDING_PAD_XY[key]

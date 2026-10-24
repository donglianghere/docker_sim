#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编队飞行实时监视 + 落地后自动出 PNG 报告图。

飞行中弹一个田字格窗口，四格实时刷新：
  ① 俯视轨迹图——场地边界/三根立柱/两个起降点当底图，两机轨迹折线 + 当前位置
     （带机头方向的三角标）+ 两机连线，连线中点标实时间距；
  ② 双机通信链路面板——拓扑/延迟/频率，以及**实时间距**那个大数字；
  ③ 高度-时间曲线——两机各一条，叠一条巡航高度参考虚线；
  ④ 电压-时间曲线——两机各一条（6S 锂电总电压），叠满电/低压告警参考线。

2026-10-05（用户要求）：③ 原来是"间距 & 高度"、④ 原来是"速度 & 电压"双轴，
现在各只留一个量——间距看 ② 那个实时大数字就够，均速看 ① 左下角的飞行统计；
两者的历史数据照常落 CSV，要复盘再翻 CSV，不占图。

两机都降落（高度低于 LANDED_AGL 且持续 LANDED_HOLD 秒）之后自动存 PNG 并退出；
也可以 Ctrl-C 随时中断，同样会存图。

**数据源用 `uwb/pose_abs` 而不是 `dlio/odom_node/odom`**：两机的 odom 原点各自
锁定，误差会直接叠进"间距"里（formation_follower_node 的注释记着实测同一套代码
两次运行差 1.19 米，2026-09-24 还踩过原点锁偏 0.47/0.67 米的坑）。UWB 绝对系是
两机共用的同一套锚点系，不用换算也没有这个误差。

用法（在 flight-stack 容器里跑，那边的 DDS 环境是验证过的）：
    docker exec -it docker_sim-flight-stack-nx01-1 bash -lc \
      'source /opt/ros/humble/setup.bash; \
       export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 \
              RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
              CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml; \
       python3 /opt/host_scripts/monitor.py --out /logs/formation.png'

没有图形界面（DISPLAY 没给、或 xhost 没放行）时自动退回无窗口模式：不弹窗，
照常记录，结束时仍然出 PNG。
"""
import argparse
import collections
import math
import os
import signal
import sys
import time

import matplotlib
import matplotlib.patches      # 模块级导入：放 main() 里会把 matplotlib 变成局部名

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
import functools
import json
from std_msgs.msg import Float64MultiArray, String
from rclpy.node import Node
# 电池话题要用 sensor data QoS（BEST_EFFORT），见下面订阅处的说明
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import EstimatorStatus, State

# 场地底图。2026-09-29 之前这几个常量是**写死的 fire_drill_room 坐标**，
# 换成样题场景之后俯视图画的还是老房子（房间画在原点居中、立柱在 (4.5,7)
# 那套、起降点在 (±2,-9.5)），轨迹叠在错误的底图上，看着像飞机飞到墙外去了。
# 现在改成从布局 yaml 读，读不到才退回这组老值（并打一行提示，不静默）。
ROOM_X, ROOM_Y = 20.0, 25.0
ROOM_ORIGIN = 'center'               # 'center'=原点在房间中心；'southwest'=在西南角
PILLARS = [(4.5, 7.0), (-4.5, 7.0), (0.0, 0.0)]
PILLAR_HALF = 0.5
#: 孪生柱方向键 `twin_dir` -> 单位方向向量（跟 gen_sample_room_world.py
#: 和 scenario_reset_node.py 的同名表一致）。
_TWIN_UNIT = {'+x': (1.0, 0.0), '-x': (-1.0, 0.0),
              '+y': (0.0, 1.0), '-y': (0.0, -1.0)}
PADS = {'NX01': (2.0, -9.5), 'NX02': (-2.0, -9.5)}
CYLINDERS = []                       # [(x, y, r), ...]
TERRAIN = None                       # (x, y, size_x, size_y)
SUPPLY = None                        # (x, y, size_x, size_y)


def load_scene(path):
    """从布局 yaml 覆盖上面那组底图常量。找不到文件就保持原值。"""
    global ROOM_X, ROOM_Y, ROOM_ORIGIN, PILLARS, PILLAR_HALF, PADS, CYLINDERS, TERRAIN
    global SUPPLY
    import yaml
    with open(path, encoding='utf-8') as f:
        L = yaml.safe_load(f)
    R = L['room']
    ROOM_X, ROOM_Y = float(R['size_x']), float(R['size_y'])
    ROOM_ORIGIN = R.get('origin', 'center')
    PILLAR_HALF = float(L['pillar_size']) / 2.0
    PILLARS = []
    for q in L['pillars']:
        PILLARS.append((float(q['x']), float(q['y'])))
        # 孪生柱底图上要画成第二个方块。方向读 yaml 的 twin_dir（2026-10-19：
        # 1# 楼的孪生柱挪到了正南，原来这里写死"正东"会把底图画错一整栋楼）；
        # 旧键 twin_east: true 等价 '+x'。
        d = _TWIN_UNIT.get(str(q.get('twin_dir') or
                               ('+x' if q.get('twin_east') else '')).lower())
        if d:
            PILLARS.append((float(q['x']) + d[0] * 2 * PILLAR_HALF,
                            float(q['y']) + d[1] * 2 * PILLAR_HALF))
    PADS = {p['id'].upper(): (float(p['x']), float(p['y']))
            for p in L['takeoff_landing_pads']}
    c = L.get('obstacle_cylinder')
    # 2026-10-11 用户去掉了这根柱子（yaml 里 `enabled: false`，键本身故意留着
    # 给 pillar_detector_node 用），底图上就不能再画——监视窗口画一个世界里
    # 不存在的障碍，看轨迹时会以为飞机在"绕障碍"。
    CYLINDERS = ([(float(c['x']), float(c['y']), float(c['diameter']) / 2)]
                 if c and c.get('enabled', True) else [])
    t = L.get('terrain_module')
    TERRAIN = (float(t['x']), float(t['y']), float(t['size_x']), float(t['size_y'])) if t else None
    sp = L.get('supply_point')
    # 物资点是矩形（坐标为中心），没给尺寸就按 0.5 m 见方画。
    SUPPLY = (float(sp['x']), float(sp['y']),
              float(sp.get('size_x', 0.5)), float(sp.get('size_y', 0.5))) if sp else None
COLORS = {'NX01': '#d62728', 'NX02': '#1f77b4'}   # 长机红、僚机蓝

AIRBORNE_Z = 1.0        # 高于这个算"在空中"（判"两机都起飞过"用）
HOVER_DONE_S = 20.0     # 没降落的飞机原地悬停这么久也算任务结束（长机在 A 点选题）
HOVER_DONE_M = 0.8      # 这段时间内水平位移不超过这个算"原地"
LANDED_AGL = 0.3        # 低于这个高度算落地
LANDED_HOLD = 3.0       # 且要持续这么久
# ---- 电池（2026-10-05 用户要求：电压按 6S 电池的**总电压**看）----
# 实测 /{ns}/mavros/battery 的 voltage 本来就是整包总电压（NX01 实读 23.85 V
# ≈ 6×3.97），cell_voltage 数组也只有一项、值等于总电压——所以正常情况下直接
# 画 voltage 就是 6S 总电压，不需要换算。
BATT_CELLS = 6
PACK_FULL_V = 4.20 * BATT_CELLS   # 25.2 V 满电
#: 低压告警线：**用户 2026-10-05 定 20.0 V**（≈3.33 V/cell），不是 3.50/cell
#: 那个保守值。到这儿就该换电池，别再起飞。
PACK_WARN_V = 20.0
# 时间轴一律画全程，不滚动（用户 2026-09-28 要求）——起步那一段恰恰是要看的，
# 滚动窗口会把它推出画面，之前那张报告图就只剩最后 90 秒。


class RoutePath:
    """**规划航线**（起飞点 + 各航点）的折线，用来算两机间距。

    间距的定义（用户 2026-09-28）：两机在同一航段内就量直线距离；不在同一
    航段时，量"经过中间航点的那几条线段长度之和"。这两种情况合起来正好等于
    **两机在航线折线上投影弧长之差**，所以实现上只要把两机都投影到航线上、
    取弧长差就行。

    用规划航线而不是长机走过的轨迹当基准：航线是两机共同的参照，形状固定、
    不受里程计采样率和噪声影响；长机的实飞轨迹每次都不一样，拿它当尺子，
    同一个控制表现量出来的数会随轨迹抖动而变。
    """

    def __init__(self, pts):
        self.xs = [p[0] for p in pts]
        self.ys = [p[1] for p in pts]
        self.arc = [0.0]
        for i in range(1, len(self.xs)):
            self.arc.append(self.arc[-1] + math.hypot(self.xs[i] - self.xs[i-1],
                                                      self.ys[i] - self.ys[i-1]))

    def ok(self):
        return len(self.xs) >= 2

    def is_closed(self):
        """航线首尾是不是同一个点（绕一圈回起降点的闭合回路）。"""
        return (len(self.xs) >= 3
                and math.hypot(self.xs[-1] - self.xs[0], self.ys[-1] - self.ys[0]) < 0.5)

    def gap_between(self, s0, s1):
        """两个弧长坐标之间的间距。

        闭合回路上两点之间有**两条路**，要取短的那条：僚机停在起始站位、
        长机刚离开起点时，正向绕是 61.5 m、反向绕只有 4.46 m，后者才是真实
        的队形间距。不判闭合直接取 |s0-s1| 的话，起步那一段会读出 62 米
        （2026-09-28 实测，一开始误以为是投影歧义，其实是这个）。
        """
        d = abs(s0 - s1)
        if self.is_closed():
            return min(d, self.arc[-1] - d)
        return d

    # "距离差不多近"的判定门限（米）。航线闭合时起降点同时属于第一段和最后
    # 一段，两者到飞机的距离几乎相等，光比距离选出来的弧长会在 0 和全长之间
    # 乱跳（实测间距曲线蹦到 62 m）。落在这个门限内的候选算"同样近"，再按
    # "离上一次最近"来选，相当于解缠绕。
    ALIAS_TOL_M = 0.5

    def project(self, px, py, last_s=None):
        """投影到航线折线上，只要弧长坐标（钳在 [0, 全长] 内）。"""
        return self.project_d(px, py, last_s)[0]

    def project_d(self, px, py, last_s=None):
        """同 project()，但连**到航线的垂距**一起返回：(弧长, 垂距)。

        垂距是用来判"这架飞机到底在不在航线上"的。投影本身永远有解——哪怕
        飞机在十几米外，也会被按到最近的那个端点上，于是两机都被按到同一个
        端点时弧长差读 0。2026-10-15 实测：僚机刚从发射点 (6.4,17.1) 往队列赶、
        人还不在航线上时，和长机都投到航线起点 G 上，间距读 **0.00 m** 而两机
        实际相距 11.5 m——这比读大更危险，因为 0 正是"两机快撞上"的警报值。

        `last_s` 是这架飞机上一次的弧长；传了就用它消歧义（见 ALIAS_TOL_M）。
        """
        cands = []
        for i in range(1, len(self.xs)):
            x0, y0, x1, y1 = self.xs[i-1], self.ys[i-1], self.xs[i], self.ys[i]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg2))
            qx, qy = x0 + t * dx, y0 + t * dy
            d2 = (px - qx) ** 2 + (py - qy) ** 2
            cands.append((d2, self.arc[i-1] + t * math.sqrt(seg2)))
        if not cands:
            return 0.0, float('inf')
        best_d2 = min(c[0] for c in cands)
        if last_s is None:
            pick = min(cands, key=lambda c: c[0])
        else:
            tol = (math.sqrt(best_d2) + self.ALIAS_TOL_M) ** 2
            near = [c for c in cands if c[0] <= tol]
            pick = min(near, key=lambda c: abs(c[1] - last_s))
        # 垂距拿**最近那个候选**的，不是解缠绕后选中的那个：前者才是
        # "离航线多远"这个几何事实，后者只是在几个同样近的候选里挑了一个。
        return pick[1], math.sqrt(best_d2)


class FormationMonitor(Node):
    def __init__(self, names, spacing, cruise_agl, tol, route=None, min_sep=0.0):
        super().__init__('formation_monitor')
        self.names = names
        self.route = RoutePath(route) if route else None
        self.names = list(names)
        self.spacing = spacing
        self.min_sep = float(min_sep)   # ②面板的安全下限线，见 --min-sep
        self.cruise_agl = cruise_agl
        self.tol = tol
        self.t0 = time.monotonic()
        self.wall0 = time.time()              # 跟 t0 同一时刻的墙上时间，换算起飞时刻用
        self.track = {n: {'t': [], 'x': [], 'y': [], 'z': [], 'yaw': []} for n in names}
        # ---- 飞行统计（2026-09-30 用户要求：动画上显示起飞时刻、编队时长、里程、均速）----
        self.takeoff_t = {n: None for n in names}   # 首次高于 AIRBORNE_Z 的时刻（相对 t0）
        self.land_t = {n: None for n in names}      # 判定降落的时刻
        #: 发射记录 {机号: [(x, y, label), ...]}（2026-10-24 加）。
        #: 来源是 SDK 在 shoot() 里广播的 shot_event，每发一条带世界坐标。
        #: 用途：轨迹图上标出在哪儿打的，数据表里累计打了几发——原来
        #: "侦察机 1 发破窗弹、任务机 4 发灭火弹"只有日志文本能证明。
        self.shots = {n: [] for n in names}
        #: 任务阶段时间线 [(相对时刻, 阶段名)]（2026-10-24 加）。
        #: 选手程序调 sdk.mark_phase() 打点，相邻两条之差就是上一段用时。
        #: 只认侦察机那一路——它主导轮次，任务机跟着走。
        self.phases = []
        self.dist_m = {n: 0.0 for n in names}       # 水平飞行里程（累加）
        self._last_xy = {n: None for n in names}    # 上一帧位置，算增量用
        self.gap_t, self.gap_d = [], []      # 沿航线的间距（主指标）
        self.gap_line = []                    # 直线距离，只作参考对照
        self.gap_xy = []                      # 算这个间距时两机的原始 xy，存 CSV 用
        self._all_airborne_seen = False       # 是否出现过"两机同时在空中"
        self.diag_t, self.diag_lag = [], []   # 跟随节点自报的落后量
        self.diag_trim = []                   # 跟随距离的偏置补偿量（data[5]，可能没有）
        self.diag_gap = []                    # 节点自报的实际间距 = 长机轨迹全长 - 僚机投影（data[4]）
        # 2026-10-10：参考点本身的三个量也存下来。排"僚机入列后不跟随"时，光有
        # lag/gap/trim 推不出参考点到底有没有推进——lag 线性涨既可能是参考点被
        # 钳住、也可能是参考点在走而飞机没执行，必须直接看 s_cmd。
        self.diag_scmd = []                   # 参考点弧长 data[1]
        self.diag_sproj = []                  # 僚机投影弧长 data[2]
        self.diag_total = []                   # 长机清洗后轨迹总长 data[3]
        self.diag_sepline = []                # 两机空间直线距离 data[8]
        self.diag_vff = []                    # 速度前馈 data[6]（0 就是没驱动力）
        self.diag_ffused = []                 # 用过长机轨迹前馈的次数 data[7]
        # 两机上一次的航线弧长，投影消歧义用。初值取 0.0 而不是 None：飞行总是
        # 从航线第一个点开始，这个先验是确定的。用 None 的话第一帧没有参考、
        # 只能任选，闭合航线上会选中末端（实测锁在 66 m 上把间距顶到 62）。
        self._last_s = [0.0, 0.0]
        # 沿航线间距只在**编队真在跑**的时段才有意义，门控认僚机的编队
        # diag：它是 formation_follower_node 在跟队时才发的，不跟队就没有。
        #   2026-10-15 换成这个门。原来是"长机离航线起点超过 1 m 就开记"的
        #   **一次性**开关，挡不住两类场景：① 长机起降点本来就不在航线起点上
        #   （真机长机停 (8.0,2.8)、航线起点是 A(2.5,2.5)），门一上来就是开的；
        #   ② 一旦打开就再也不关，而 mission 第 2/3 轮有大段时间根本没在编队
        #   （长机独自飞 A->B->C->G、僚机停在起降点或物资点），那时弧长差
        #   真的就是 36 m，数学上没错、作为"编队间距"却是器讻。
        #   实测证据：2026-10-15 真机 mission 那趟，diag 的时间空洞
        #   190.8->396.5 s 跟大间距那段完全重合。
        self.DIAG_FRESH_S = 1.0
        self._last_diag_t = None
        # 第二道门：两机都得**真在航线上**，弧长差才有意义。见
        # RoutePath.project_d() 的说明：人不在航线上时会被按到端点，两机按到
        # 同一个端点就读 0.00。门限 3 m 的依据：line_goto 的直线横偏实测均值
        # 0.11 m（ego 基线 1.96 m），编队中正常偏差远不到 3 m；而刚起步/入列前
        # 那种"还在十几米外"的情形一定超。
        self.ON_ROUTE_TOL_M = 3.0
        self._low_since = {n: None for n in names}
        self.landed = {n: False for n in names}
        # ---- 链路统计（2026-10-02 新增，田字格右上角那块面板用）----
        # 注意这些量必须在下面 10Hz 降采样**之前**累计，否则测出来的频率
        # 永远是 10Hz，真实收包速率和抖动全被抹平。
        self.link = {n: {
            'recv': collections.deque(maxlen=600),   # 到达时刻(相对 t0)，约 30s@20Hz
            'delay': collections.deque(maxlen=600),  # 端到端延迟 ms（两机已对时到地面站）
            'count': 0,                              # 累计收包数
            'last': None,                            # 最后一次到达时刻
            'max_gap': 0.0,                          # 最长断流时长，演示拔网线时看这个
        } for n in names}
        self.diag_recv = collections.deque(maxlen=600)   # formation_diag 到达时刻：
                                                         # 它在更新=僚机确实在消费长机数据
        for n in names:
            self.create_subscription(PoseStamped, f'/{n}/uwb/pose_abs',
                                     self._make_cb(n), 10)
        # 僚机跟随回路自报的落后量——外面自己拿两机位置算弧长跟节点内部的
        # 基准不一样（折线、采样率、向后延伸段都不同），要判断控制好坏得看它自己的数
        self.create_subscription(Float64MultiArray, f'/{names[1]}/formation_diag',
                                 self._on_diag, 10)
        # ---- 电压（2026-10-05 用户要求：偏航不再监测，改监测电压）----
        # /{ns}/mavros/battery 是 sensor_msgs/BatteryState，实测 2Hz（飞控
        # BATTERY_STATUS 的流速率），所以单独存一份时间序列，不挂在 10Hz 的
        # pose 轨迹里——两者采样率差 5 倍，混在一个 dict 里画图会错位。
        self.batt = {n: {'t': [], 'v': []} for n in names}
        # ⚠️ **必须用 qos_profile_sensor_data（BEST_EFFORT）**。mavros 这一路是
        # 传感器 QoS，用默认的 `10`（RELIABLE）订阅会 QoS 不兼容、**一条都收不到**，
        # 而且不报错——只在节点日志里留一句
        # "New publisher discovered on topic '/NX01/mavros/battery', offering
        #  incompatible QoS. No messages will be received from it.
        #  Last incompatible policy: RELIABILITY"，图上就是一片空白。
        # 2026-10-05 第一趟真机实测踩到。`uwb/pose_abs` 是 RELIABLE，所以那边
        # 照旧用 10，不要一起改。
        # 注：用 `ros2 topic echo` 手验这个话题**验不出来**——CLI 会自动适配
        # 发布端的 QoS，能打印不代表你的节点订得上。
        for n in names:
            self.create_subscription(
                BatteryState, f'/{n}/mavros/battery',
                self._make_batt_cb(n), qos_profile_sensor_data)

        # ---- 飞控模式 / 定位健康（2026-10-19 新增，画在②面板底部的状态带）----
        # 为什么加这个：10-18 两架飞机各掉过一次（NX02 从 2.15 m、NX01 从 1.94 m
        # 被 PX4 盲降放到地上），共同指纹是 `mode: OFFBOARD -> ALTCTL` 同时
        # `pos_horiz_rel_status_flag -> false`（EV 定位源停摆十几秒 ⇒ 水平位置
        # 失效 ⇒ 掉出 OFFBOARD ⇒ 没有遥控器输入就盲降）。而当时**监视器上完全
        # 看不出来**：只看到高度曲线往下掉，分不清是"控制跟不上"还是"定位丢了"，
        # 只能事后去容器里翻 ulog。有了这条带，出事当场就能分清因果。
        # 跟②的间距曲线共用同一条时间轴，所以"定位红了 → 间距随即崩"这种前后
        # 关系一眼可见。
        # ⚠️ QoS 跟电压那一路同理用 BEST_EFFORT：订阅端 BEST_EFFORT 能收
        # RELIABLE 发布者的数据，反过来不行，所以这么写两边都兼容。
        self.fcu = {n: {'mt': [], 'mode': [], 'pt': [], 'pos_ok': []} for n in names}
        for n in names:
            self.create_subscription(String, f'/{n}/shot_event',
                                     functools.partial(self._on_shot, n), 10)
            self.create_subscription(String, f'/{n}/phase_event',
                                     functools.partial(self._on_phase, n), 10)
            self.create_subscription(State, f'/{n}/mavros/state',
                                     self._make_state_cb(n), qos_profile_sensor_data)
            self.create_subscription(EstimatorStatus, f'/{n}/mavros/estimator_status',
                                     self._make_est_cb(n), qos_profile_sensor_data)

    def _make_state_cb(self, name):
        def cb(msg):
            f = self.fcu[name]
            md = str(msg.mode or '')
            # 只在**变化时**追加一个点：mode 是 1~2 Hz 的常量流，全存下来画
            # broken_barh 要几百段；只记跳变，一次飞行通常不到 10 段。
            if not f['mode'] or f['mode'][-1] != md:
                f['mt'].append(time.monotonic() - self.t0)
                f['mode'].append(md)
        return cb

    def _make_est_cb(self, name):
        def cb(msg):
            f = self.fcu[name]
            ok = bool(msg.pos_horiz_rel_status_flag)
            if not f['pos_ok'] or f['pos_ok'][-1] != ok:
                f['pt'].append(time.monotonic() - self.t0)
                f['pos_ok'].append(ok)
        return cb

    def _make_batt_cb(self, name):
        def cb(msg):
            v = float(msg.voltage)
            if not (0.0 < v < 100.0):      # 没读到电压时 mavros 给 nan/0
                return
            if v < 6.0:                    # 万一换成单节电压，折算成 6S 总电压，
                v *= BATT_CELLS            # 否则跟满电/告警参考线不是一个量纲
            d = self.batt[name]
            d['t'].append(time.monotonic() - self.t0)
            d['v'].append(v)
        return cb

    def _make_cb(self, name):
        def cb(msg):
            p, q = msg.pose.position, msg.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            d = self.track[name]
            t = time.monotonic() - self.t0

            # ---- 链路统计：放在降采样 return 之前，必须每条消息都记 ----
            lk = self.link[name]
            if lk['last'] is not None:
                gap = t - lk['last']
                if gap > lk['max_gap']:
                    lk['max_gap'] = gap
            lk['last'] = t
            lk['count'] += 1
            lk['recv'].append(t)
            # 延迟 = 本机接收时刻 - 发布时刻。两机和地面站都已 chrony 对到
            # 同一基准(2026-10-02)，这个差值才有意义；没对时的话是时钟偏差。
            st = msg.header.stamp
            if st.sec or st.nanosec:
                dly = (self.get_clock().now().nanoseconds
                       - (st.sec * 1_000_000_000 + st.nanosec)) / 1e6
                if -1000.0 < dly < 10000.0:        # 明显离谱的丢掉(没对时/stamp异常)
                    lk['delay'].append((t, dly))

            # 降到 10 Hz 存，画图够用，内存也不会涨太快
            if d['t'] and t - d['t'][-1] < 0.1:
                return
            d['t'].append(t); d['x'].append(p.x); d['y'].append(p.y)
            d['z'].append(p.z); d['yaw'].append(yaw)
            self._accumulate(name, t, p.x, p.y, p.z)
            self._update_landed(name, p.z)
            self._note_airborne()
            self._update_gap(t)
        return cb

    def _on_phase(self, name, msg):
        """收一条阶段标记。只认第一架（侦察机）的，避免两机重复打点。"""
        if name != self.names[0]:
            return
        try:
            d = json.loads(msg.data)
            self.phases.append((time.monotonic() - self.t0, str(d.get('phase', ''))))
        except Exception:
            pass

    def phase_durations(self):
        """[(阶段名, 用时秒)]。最后一段没有后继打点就按"到现在"算。"""
        out = []
        for i, (t, nm) in enumerate(self.phases):
            if nm == '完成':
                continue
            end = self.phases[i + 1][0] if i + 1 < len(self.phases) else (time.monotonic() - self.t0)
            out.append((nm, end - t))
        return out

    def _on_shot(self, name, msg):
        """收一条发射事件：SDK 在 shoot() 里每发一发广播一次，带世界坐标。

        解析失败就丢掉——报告上少一个标记，不值得让监视器崩。
        """
        try:
            d = json.loads(msg.data)
            x, y = float(d['x']), float(d['y'])
            if x != x or y != y:          # NaN：拿不到位置时只计数、不标位置
                x = y = None
            self.shots.setdefault(name, []).append((x, y, str(d.get('label', '发射'))))
        except Exception:
            pass

    def _on_diag(self, msg):
        if len(msg.data) >= 1:
            self.diag_recv.append(time.monotonic() - self.t0)
            self.diag_t.append(time.monotonic() - self.t0)
            self._last_diag_t = self.diag_t[-1]   # 沿航线间距的门，见 DIAG_FRESH_S
            self.diag_lag.append(float(msg.data[0]))
            self.diag_trim.append(float(msg.data[5]) if len(msg.data) >= 6 else 0.0)
            # data[4] 是节点自己算的"长机轨迹全长 - 僚机投影弧长"，也就是控制回路
            # 眼里的真实间距。画 3.5+lag 是不对的：lag 是相对**补偿后**的跟随距离
            # 算的（lag = 实际间距 - fd_eff），3.5+lag 会恒比实际间距高出一个 trim。
            self.diag_gap.append(float(msg.data[4]) if len(msg.data) >= 5 else float('nan'))
            _g = lambda i: (float(msg.data[i]) if len(msg.data) > i else float('nan'))
            self.diag_scmd.append(_g(1))
            self.diag_sproj.append(_g(2))
            self.diag_total.append(_g(3))
            self.diag_sepline.append(_g(8))
            self.diag_vff.append(_g(6))
            self.diag_ffused.append(_g(7))

    #: 里程用**锚点累加**而不是逐帧累加：离上一个锚点走够 DIST_ANCHOR_M 才记一笔，
    #: 然后把锚点挪过来。逐帧累加会把 UWB 的厘米级抖动全部攒进里程——第一版用
    #: 0.03 m 死区，1714 帧每帧漏进 5 cm，86 米的航线算出 184 米。锚点法里噪声
    #: 只会让飞机在锚点周围打转、永远够不到门限，天然不累加。
    #: 0.25 m 的弦长在本场地的曲率下跟弧长差可以忽略。
    DIST_ANCHOR_M = 0.25
    DIST_MAX_STEP_M = 3.0      # 超过这个是解算野值，丢掉并重置锚点

    def _accumulate(self, name, t, x, y, z):
        """累计水平里程，并记下起飞时刻（首次高于 AIRBORNE_Z）。"""
        if self.takeoff_t[name] is None and z > AIRBORNE_Z:
            self.takeoff_t[name] = t
        anchor = self._last_xy[name]
        if anchor is None:
            self._last_xy[name] = (x, y)
            return
        step = math.hypot(x - anchor[0], y - anchor[1])
        if step > self.DIST_MAX_STEP_M:
            self._last_xy[name] = (x, y)        # 野值：重置锚点，不计这一段
            return
        if step >= self.DIST_ANCHOR_M:
            self.dist_m[name] += step
            self._last_xy[name] = (x, y)

    def flight_stats(self):
        """返回 (每机统计 list, 编队时长 or None)。

        - 起飞时刻：首次高于 AIRBORNE_Z 的墙上时间；
        - 空中时长：起飞到降落（还没落就算到现在）；
        - 里程：水平位移累加（带死区/跳变门限，见 DIST_MIN_STEP_M）；
        - 平均速度：里程 / 空中时长。**不是巡航速度**——航点停顿、起降段都算在里面。
        - 编队时长：僚机跟随节点发 formation_diag 的首末时刻之差，也就是
          "跟随回路真正在工作"的那段，比"两机都在天上"更贴近编队本身。
        """
        now = time.monotonic() - self.t0
        rows = []
        for n in self.names:
            t_up = self.takeoff_t[n]
            if t_up is None:
                rows.append((n, None, None, self.dist_m[n], None))
                continue
            t_dn = self.land_t[n] if self.land_t[n] is not None else now
            air = max(1e-6, t_dn - t_up)
            rows.append((n, self.wall0 + t_up, air, self.dist_m[n], self.dist_m[n] / air))
        form = (self.diag_t[-1] - self.diag_t[0]) if len(self.diag_t) >= 2 else None
        return rows, form

    def _note_airborne(self):
        """两机同时高于 AIRBORNE_Z 就置位——任务结束判据的前置条件。"""
        if self._all_airborne_seen:
            return
        zs = [self.track[n]['z'][-1] for n in self.names if self.track[n]['z']]
        if len(zs) == len(self.names) and all(z > AIRBORNE_Z for z in zs):
            self._all_airborne_seen = True

    def _update_landed(self, name, z):
        now = time.monotonic()
        if z < LANDED_AGL:
            if self._low_since[name] is None:
                self._low_since[name] = now
            elif now - self._low_since[name] >= LANDED_HOLD:
                # 只有**起飞过**才算降落。少了这个判断，起飞前趴在地上那段就会
                # 记一次"降落时刻"，起飞后又不清，空中时长算出 0（run75 实测
                # NX01 空中 0.0s、均速 1.8e8 m/s）。
                if not self.landed[name] and self.takeoff_t[name] is not None:
                    # 落地时刻算成"开始贴地"那一刻，不是保持够 LANDED_HOLD 的那一刻
                    self.land_t[name] = self._low_since[name] - self.t0
                self.landed[name] = True
        else:
            self._low_since[name] = None
            self.landed[name] = False

    def _update_gap(self, t):
        pts = []
        for n in self.names:
            d = self.track[n]
            if not d['t']:
                return
            pts.append((d['x'][-1], d['y'][-1]))
        if len(pts) != 2:
            return
        # 两机都投影到**规划航线**上，间距 = 弧长差（同段内即直线距离，
        # 跨段即经过航点那几条线段之和）。没给航线就退回直线距离。
        # ⚠️ 投影**每帧都算**，哪怕下面的门关着：project() 靠 last_s 解缠绕
        #    （闭合航线上起降点同时属于首尾两段），断一段不算会让 last_s
        #    停在几百秒前的陈值，编队恢复时第一帧就可能选错分支。
        on_route = True
        if self.route is not None and self.route.ok():
            s0, d0 = self.route.project_d(*pts[0], last_s=self._last_s[0])
            s1, d1 = self.route.project_d(*pts[1], last_s=self._last_s[1])
            self._last_s = [s0, s1]
            gap = self.route.gap_between(s0, s1)
            on_route = max(d0, d1) <= self.ON_ROUTE_TOL_M
        else:
            gap = math.dist(pts[0], pts[1])
        # 门：最近 DIAG_FRESH_S 秒内收到过编队 diag 才算"正在编队"。没在编队
        # 就记 NaN——不是丢样本：matplotlib 遇 NaN 自然**断线**，比跳过不记
        # （那会把空洞两端连成一条直线，看上去像真有数据）诚实。
        # 直线距离（橙线）照记不误：那个量任何时候都是有定义的。
        active = (self._last_diag_t is not None
                  and (t - self._last_diag_t) <= self.DIAG_FRESH_S
                  and on_route)
        self.gap_t.append(t)
        self.gap_d.append(gap if active else float('nan'))
        self.gap_line.append(math.dist(pts[0], pts[1]))
        self.gap_xy.append((pts[0][0], pts[0][1], pts[1][0], pts[1][1]))

    def all_landed(self):
        return all(self.landed[n] for n in self.names) and \
            all(self.track[n]['t'] for n in self.names)

    def mission_done(self):
        """任务是否结束——不能只看"两机都降落了"。

        2026-09-29 样题流程里长机跑完航线是**回 A 点悬停、不降落**的，只判降落
        的话监视脚本永远等不到结束条件，报告就一直不生成（实测 run52 就是这样
        白跑了一整轮，PNG 还停在上一轮）。
        判据放宽成："每架飞机要么已降落，要么已经在原地悬停 HOVER_DONE_S 秒"
        （悬停=这段时间内水平位移不超过 HOVER_DONE_M），且至少有一架真的落了
        ——只有悬停没人降落，多半是还在飞，不算结束。
        """
        if not all(self.track[n]['t'] for n in self.names):
            return False
        # 前置：必须**两机都真的飞起来过**。少了这一条，起飞前就会误判结束——
        # 两机都还在地上算"已降落"，先起飞那架刚离地悬停算"原地悬停"，
        # 条件当场成立（2026-09-29 run54 实测：采样才 137 点，NX02 高度
        # 中位 0.04 米，压根没起飞就出报告了）。
        if not self._all_airborne_seen:
            return False
        if not any(self.landed[n] for n in self.names):
            return False
        now = time.monotonic() - self.t0
        for n in self.names:
            if self.landed[n]:
                continue
            d = self.track[n]
            win = [(t, x, y) for t, x, y in zip(d['t'], d['x'], d['y'])
                   if t >= now - HOVER_DONE_S]
            if len(win) < 5 or (now - win[0][0]) < HOVER_DONE_S * 0.8:
                return False            # 样本不够久，还说不准
            cx = sum(q[1] for q in win) / len(win)
            cy = sum(q[2] for q in win) / len(win)
            if max(math.hypot(q[1] - cx, q[2] - cy) for q in win) > HOVER_DONE_M:
                return False            # 还在动
        return True


def stats_lines(mon):
    """飞行统计的文字行，动画左下角和结束时的文字报告共用同一份。"""
    rows, form = mon.flight_stats()
    out = []
    if form is not None:
        out.append(f'编队飞行 {form:5.1f} s')
    for n, wall, air, dist, v in rows:
        if wall is None:
            out.append(f'{n} 未起飞')
            continue
        out.append(f'{n} 起飞 {time.strftime("%H:%M:%S", time.localtime(wall))}  '
                   f'空中 {air:5.1f}s  里程 {dist:6.1f}m  均速 {v:4.2f}m/s')
    return out


def max_speed(d, win=1.0):
    """全程最大水平速度（米/秒）。"""
    _ts, vs = speed_series(d, win)
    return max(vs) if vs else None


def draw_speed_panel(ax, mon, tlo, tmax):
    """速度曲线：两机的实时速度，各自的平均速度画成虚线参考。"""
    ax.clear()
    ax.set_title('速度曲线')
    rows, _form = mon.flight_stats()
    avg = {n: v for n, _w, _a, _d, v in rows}
    for n in mon.names:
        ts, vs = speed_series(mon.track[n])
        if ts:
            ax.plot(ts, vs, '-', color=COLORS[n], lw=1.3,
                    label=f'{n} 实时 {vs[-1]:.2f} m/s')
        if avg.get(n):
            ax.axhline(avg[n], ls='--', color=COLORS[n], lw=1.0, alpha=0.7,
                       label=f'{n} 平均 {avg[n]:.2f} m/s')
    ax.set_xlim(tlo, max(tmax, tlo + 5))
    ax.set_ylim(bottom=0.0)
    ax.grid(alpha=0.3)
    ax.set_xlabel('t (s)'); ax.set_ylabel('水平速度 (m/s)')
    ax.legend(loc='upper right', fontsize=7, ncol=2)


def draw_state_panel(ax, mon, tlo, tmax):
    """状态时间带：每机一行飞控模式，定位失效用红色叠在同一行上。

    这两项查事故最常用——模式跳 ALTCTL、定位翻 false 都是"飞机被莫名放到
    地上"的指纹，看一眼这张图就知道是哪一秒开始坏的。
    """
    ax.clear()
    ax.set_title('状态时间带')
    hi = max(tmax, tlo + 5)
    for i, n in enumerate(mon.names):
        f = mon.fcu[n]
        y0 = -(i + 1.0)
        buckets = {}
        for k, t_s in enumerate(f['mt']):
            t_e = f['mt'][k + 1] if k + 1 < len(f['mt']) else hi
            if t_e <= t_s:
                continue
            md = f['mode'][k]
            c = ('#2ca02c' if md == 'OFFBOARD' else
                 '#ff7f0e' if md in ('ALTCTL', 'POSCTL') else '#bbbbbb')
            buckets.setdefault(c, []).append((t_s, t_e - t_s))
        for c, spans in buckets.items():
            ax.broken_barh(spans, (y0, 0.7), facecolors=c, alpha=0.9)
        bad = []
        for k, t_s in enumerate(f['pt']):
            if f['pos_ok'][k]:
                continue
            t_e = f['pt'][k + 1] if k + 1 < len(f['pt']) else hi
            if t_e > t_s:
                bad.append((t_s, t_e - t_s))
        if bad:
            ax.broken_barh(bad, (y0, 0.7), facecolors='#d62728', alpha=1.0)
        # 当前模式写在带子右端内侧（不能越出坐标区，否则会糊到隔壁那张图上）
        md_now = f['mode'][-1] if f['mode'] else '—'
        ax.text(0.995, (y0 + 0.35 - (-len(mon.names) - 0.4)) / (len(mon.names) + 0.6),
                md_now, transform=ax.transAxes, va='center', ha='right',
                fontsize=7, color='#222222',
                bbox=dict(fc='white', ec='none', alpha=0.75, pad=1.0))
    ax.set_xlim(tlo, hi)
    ax.set_ylim(-len(mon.names) - 0.4, 0.2)
    ax.set_yticks([-(i + 0.65) for i in range(len(mon.names))])
    ax.set_yticklabels(list(mon.names), fontsize=8)
    ax.grid(axis='x', alpha=0.3)
    ax.set_xlabel('t (s)')
    ax.text(tlo, 0.05, '绿=OFFBOARD　橙=ALTCTL/POSCTL　红=定位失效　灰=其它',
            fontsize=6.5, color='#666666', va='bottom')


def draw_stats_table(ax, mon):
    """底部那条扁表：一机一行，列是各项指标；两机共用的两项写进标题里。"""
    ax.clear()
    ax.axis('off')
    rows, form = mon.flight_stats()
    stat = {n: (wall, air, dist, v) for n, wall, air, dist, v in rows}
    now_rel = time.monotonic() - mon.t0
    # 2026-10-24：原来只有一个"任务用时"总数，分不出三圈。改成按阶段分列，
    # 数据来自选手程序的 sdk.mark_phase() 打点（见 Monitor.phase_durations）。
    cols = ['', '起飞时刻', '空中时间 (s)', '里程 (m)', '位姿频率 (Hz)',
            '最长断流 (s)', '发射 (发)', '编队飞行 (s)',
            '1圈编队 (s)', '2圈灭火 (s)', '3圈灭火 (s)']
    MERGE = (7, 8, 9, 10)                       # 两机共用、上下两格并一格的列
    _ups = [t for t in mon.takeoff_t.values() if t is not None]
    _dns = [mon.land_t[n] for n in mon.names]
    _end = now_rel if any(t is None for t in _dns) else max(_dns)
    # 这几项两机共用：值只写第一行，两行并成一格（见下面的 visible_edges）
    # 三圈用时按 mark_phase 打点的先后取前三段；没打点就是"—"（比如跑的是
    # 单项任务程序，本来就没有三圈）。
    _durs = mon.phase_durations()
    _ring = [f'{d:.0f}' for _nm, d in _durs[:3]] + ['—'] * 3
    shared = ['—' if form is None else f'{form:.1f}'] + _ring[:3]
    body = []
    for who, n in zip(('侦察机', '任务机'), mon.names[:2]):
        wall, air, dist, _v = stat.get(n, (None, None, 0.0, None))
        hz, _jit = _rate(mon.link[n]['recv'], now_rel)
        body.append([
            f'{who} {n}',
            '未起飞' if wall is None else time.strftime('%H:%M:%S', time.localtime(wall)),
            '—' if air is None else f'{air:.0f}',
            f'{dist:.1f}',
            f'{hz:.1f}',
            f"{mon.link[n]['max_gap']:.1f}",
            # 发射弹数：没打过就写 0，打过写"标签 xN"（如 发射破窗弹 x1）
            (lambda _s: '0' if not _s
             else f'{_s[0][2]} x{len(_s)}')(mon.shots.get(n, [])),
            '', '', '', '',                     # 4 个合并列的值单独画在两格正中
        ])
    widths = [0.15] + [0.085] * (len(cols) - 1)   # 2026-10-24 从 9 列加到 11 列，调窄
    tb = ax.table(cellText=body, colLabels=cols, cellLoc='center',
                  colWidths=widths, bbox=[0.0, 0.0, 1.0, 1.0])
    tb.auto_set_font_size(False)
    tb.set_fontsize(9)
    for (r, c), cell in tb.get_celld().items():
        cell.set_edgecolor('#555555')
        cell.set_text_props(color='#111111')
        if r == 0:
            cell.set_facecolor('#dcdcdc')
        elif c == 0:                            # 行首的机号用本机颜色
            cell.set_text_props(fontweight='bold',
                                color=COLORS.get(mon.names[r - 1], 'k'))
        # 两机共用的两列：上下两格并成一格。'LR'+'LRB' 实测不会出对角线伪影，
        # 'BLT' 那一类三边组合会（matplotlib 把路径斜着闭合）。
        if c in MERGE and r == 1:
            cell.visible_edges = 'LR'
        elif c in MERGE and r == 2:
            cell.visible_edges = 'LRB'
    _tot = sum(widths)
    for k, ci in enumerate(MERGE):
        _cx = (sum(widths[:ci]) + widths[ci] / 2.0) / _tot
        ax.text(_cx, 1.0 / 3.0, shared[k], transform=ax.transAxes,
                ha='center', va='center', fontsize=9, color='#111111')


def min_clearance(x, y):
    """机体到最近障碍(四面墙/方柱/圆柱)的水平净距，米。

    纯闭式公式，3 方柱+4 墙实测每帧 1.5 微秒(两机)，相比 matplotlib 一次
    重绘 70~80ms 可以完全忽略，放心每帧算。
    """
    _x0, _y0 = (0.0, 0.0) if ROOM_ORIGIN == 'southwest' else (-ROOM_X / 2, -ROOM_Y / 2)
    best = min(x - _x0, _x0 + ROOM_X - x, y - _y0, _y0 + ROOM_Y - y)   # 四面墙
    for px, py in PILLARS:                                             # 方柱：点到轴对齐矩形
        dx = max(abs(x - px) - PILLAR_HALF, 0.0)
        dy = max(abs(y - py) - PILLAR_HALF, 0.0)
        best = min(best, math.hypot(dx, dy))
    for cx, cy, cr in CYLINDERS:                                       # 圆柱
        best = min(best, max(math.hypot(x - cx, y - cy) - cr, 0.0))
    return best


CLEAR_WARN_M = 0.8      # 净距低于这个值，俯视图上标红报警


def speed_series(d, win=1.0):
    """逐点的水平速度（米/秒）：每个采样点都用 win 秒滑窗按轨迹弧长算。

    返回 (时刻列表, 速度列表)。实时速度、最大速度、速度曲线都从这一份出，
    免得三处各算一遍、口径还不一样。
    """
    n = len(d['t'])
    if n < 2:
        return [], []
    acc = [0.0]
    for k in range(n - 1):
        acc.append(acc[-1] + math.hypot(d['x'][k + 1] - d['x'][k],
                                        d['y'][k + 1] - d['y'][k]))
    ts, vs, i = [], [], 0
    for j in range(1, n):
        while d['t'][j] - d['t'][i] > win and i < j - 1:
            i += 1
        dt = d['t'][j] - d['t'][i]
        if dt > 1e-3:
            ts.append(d['t'][j])
            vs.append((acc[j] - acc[i]) / dt)
    return ts, vs


def current_speed(d, win=1.0):
    """当前水平速度（米/秒），样本不够返回 None。"""
    _ts, vs = speed_series(d, win)
    return vs[-1] if vs else None


def _rate(times, now, win=5.0):
    """最近 win 秒的到达频率(Hz)和到达间隔抖动(标准差, ms)。"""
    recent = [t for t in times if now - t <= win]
    if len(recent) < 2:
        return 0.0, 0.0
    hz = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)
    gaps = [(recent[i] - recent[i - 1]) * 1000.0 for i in range(1, len(recent))]
    mean = sum(gaps) / len(gaps)
    jit = (sum((g - mean) ** 2 for g in gaps) / len(gaps)) ** 0.5
    return hz, jit


def _last_valid(seq, default=float('nan')):
    """最后一个非 NaN 的值。沿航线间距在"没在编队"的时段记的是 NaN
    （见 FormationMonitor.DIAG_FRESH_S），直接取 [-1] 会拿到 nan。"""
    for v in reversed(seq):
        if v == v:
            return v
    return default


def _live_line_gap(mon):
    """两机当前位置的直线距离；任一机还没来过数据就返回 nan。"""
    pts = []
    for n in mon.names:
        d = mon.track[n]
        if not d['t']:
            return float('nan')
        pts.append((d['x'][-1], d['y'][-1]))
    return math.dist(pts[0], pts[1])


def draw_link_panel(ax, mon, now):
    """⚠️ 2026-10-14 起**不再被调用**（用户要求去掉双机通信链路图，
    那一格换成了间距-时间曲线，见 draw() 里的 ②）。函数留着没删：
    要恢复只需把 draw() 里 ② 那段换回 draw_link_panel(ax_link, mon, now)。

    田字格右上：双机通信链路面板。

    一格里分三层画：上层拓扑动画(点在流=在收包)、中层三个大数字、
    下层延迟条带。全部用 axes 归一化坐标手画，不嵌套 inset——inset 在
    每帧 ax.clear() 之后要重建，反而麻烦。
    """
    ax.clear()
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    ax.set_title('双机通信链路')
    lead, foll = mon.names[0], mon.names[1]
    lk_l, lk_f = mon.link[lead], mon.link[foll]
    hz_l, jit_l = _rate(lk_l['recv'], now)
    hz_f, jit_f = _rate(lk_f['recv'], now)
    hz_d, _ = _rate(mon.diag_recv, now)

    def age(lk):
        return 999.0 if lk['last'] is None else now - lk['last']

    def color_of(a):
        return '#2ca02c' if a < 0.5 else ('#ff7f0e' if a < 1.0 else '#d62728')

    # ---------- 上层：拓扑 ----------
    BOXY, BOXH = 0.74, 0.16
    for cx, name, lk in ((0.17, lead, lk_l), (0.63, foll, lk_f)):
        a = age(lk)
        ax.add_patch(matplotlib.patches.FancyBboxPatch(
            (cx, BOXY), 0.20, BOXH, boxstyle='round,pad=0.012',
            fc='#eef5ff', ec=color_of(a), lw=2.0))
        ax.text(cx + 0.10, BOXY + BOXH * 0.62, name, ha='center', va='center',
                fontsize=11, fontweight='bold')
        ax.text(cx + 0.10, BOXY + BOXH * 0.22, '长机' if name == lead else '僚机',
                ha='center', va='center', fontsize=8, color='#555')
    # 机间连线 + 流动的点：点的相位按"实际收包数"推进，没收到包就不动
    y_mid = BOXY + BOXH / 2
    ax.annotate('', xy=(0.63, y_mid), xytext=(0.37, y_mid),
                arrowprops=dict(arrowstyle='-|>', lw=2.0, color=color_of(age(lk_f))))
    phase = (lk_f['count'] % 8) / 8.0
    for k in range(4):
        fx = 0.38 + ((phase + k / 4.0) % 1.0) * 0.24
        ax.plot([fx], [y_mid], 'o', ms=5, color='#1f77b4', alpha=0.85)
    ax.text(0.50, BOXY + BOXH + 0.035, f'/{lead}/uwb/pose_abs', ha='center',
            fontsize=8, color='#444')
    ax.text(0.50, y_mid - 0.072, f'diag 回报 {hz_d:4.1f} Hz', ha='center',
            fontsize=8, color='#2ca02c' if hz_d > 0.5 else '#d62728')

    # ---------- 中层：三个大数字 ----------
    d_l = lk_l['delay'][-1][1] if lk_l['delay'] else float('nan')
    d_f = lk_f['delay'][-1][1] if lk_f['delay'] else float('nan')
    # 2026-10-05：间距的曲线撤了之后，这个大数字就是唯一的实时间距显示，
    # 所以编队还没开始记 gap_d（长机离起点不到 GAP_START_MOVE_M）那一段也得有
    # 数——回退成两机当前位置的直线距离，跟俯视图连线上标的那个数同源。
    # 最新一点是 NaN 就表示"此刻没在编队"（见 DIAG_FRESH_S），这时该退回
    # **实时直线距离**而不是把数字冻在上一个有效值上——这是实时显示，
    # 冻住了比没数还坑人。跟俯视图两机连线上标的那个数同源。
    gap_now = mon.gap_d[-1] if mon.gap_d else float('nan')
    if gap_now != gap_now:
        gap_now = _live_line_gap(mon)
    cells = (('延迟 ms', f'{d_l:.0f}/{d_f:.0f}', '#1f77b4'),
             ('频率 Hz', f'{hz_l:.0f}/{hz_f:.0f}', '#2ca02c'),
             ('间距 m', f'{gap_now:.2f}', '#9467bd'))
    for i, (lab, val, col) in enumerate(cells):
        cx = 0.17 + i * 0.33
        ax.text(cx, 0.60, val, ha='center', va='center', fontsize=19,
                fontweight='bold', color=col)
        ax.text(cx, 0.515, lab, ha='center', va='center', fontsize=8, color='#555')

    # ---------- 下层：延迟条带 ----------
    X0, X1, Y0, Y1, WIN = 0.08, 0.98, 0.07, 0.44, 30.0
    ax.add_patch(matplotlib.patches.Rectangle((X0, Y0), X1 - X0, Y1 - Y0,
                                              fill=False, ec='#ccc', lw=0.8))
    allv = [v for lk in (lk_l, lk_f) for t, v in lk['delay'] if now - t <= WIN]
    vmax = max(max(allv) if allv else 50.0, 20.0) * 1.15
    for lk, name in ((lk_l, lead), (lk_f, foll)):
        pts = [(t, v) for t, v in lk['delay'] if now - t <= WIN]
        if len(pts) < 2:
            continue
        xs = [X0 + (X1 - X0) * (1.0 - (now - t) / WIN) for t, _ in pts]
        ys = [Y0 + (Y1 - Y0) * min(v / vmax, 1.0) for _, v in pts]
        ax.plot(xs, ys, '-', lw=1.2, color=COLORS[name], label=name)
    ax.text(X0 + 0.005, Y1 - 0.03, f'端到端延迟 (上限 {vmax:.0f} ms，最近 {WIN:.0f}s)',
            fontsize=7.5, color='#666')
    ax.text(X0 + 0.005, Y0 + 0.012,
            f'断流最长 {max(lk_l["max_gap"], lk_f["max_gap"]):.1f}s   '
            f'抖动 {jit_l:.0f}/{jit_f:.0f} ms   收包 {lk_l["count"]}/{lk_f["count"]}',
            fontsize=7.5, color='#666')


def draw(fig, axes, mon):
    # 版面：左边六张图（水平轨迹 / 速度曲线 / 间距曲线 ／ 状态时间带 / 高度曲线
    #       / 电压曲线），右边整高一张飞行数据小表。
    ax_xy, ax_link, ax_alt, ax_volt, ax_speed, ax_state = axes
    for ax in (ax_xy, ax_link, ax_alt, ax_volt):
        ax.clear()
    now = time.monotonic() - mon.t0

    # ---- ① 水平轨迹 ----
    # 房间：原点在西南角时左下角就是 (0,0)，在中心时左下角是 (-X/2,-Y/2)
    _x0, _y0 = (0.0, 0.0) if ROOM_ORIGIN == 'southwest' else (-ROOM_X / 2, -ROOM_Y / 2)
    ax_xy.add_patch(matplotlib.patches.Rectangle(
        (_x0, _y0), ROOM_X, ROOM_Y, fill=False, ec='#888', lw=1.2))
    for px, py in PILLARS:
        ax_xy.add_patch(matplotlib.patches.Rectangle(
            (px - PILLAR_HALF, py - PILLAR_HALF), 2 * PILLAR_HALF, 2 * PILLAR_HALF,
            fc='#bbb', ec='#666'))
    for cx, cy, cr in CYLINDERS:
        ax_xy.add_patch(matplotlib.patches.Circle((cx, cy), cr, fc='#9ecae1', ec='#3182bd'))
    if TERRAIN is not None:
        tx, ty, tw, th = TERRAIN
        ax_xy.add_patch(matplotlib.patches.Rectangle(
            (tx - tw / 2, ty - th / 2), tw, th, fc='#f2e6a0', ec='#b8a020'))
    if SUPPLY is not None:
        sx, sy, sw, sh = SUPPLY
        ax_xy.add_patch(matplotlib.patches.Rectangle(
            (sx - sw / 2, sy - sh / 2), sw, sh, fc='#d9c7f5', ec='#7048e8'))
        ax_xy.text(sx, sy + sh / 2 + 0.45, '物资点', color='#7048e8',
                   fontsize=7.5, ha='center', va='bottom')
    for n, (px, py) in PADS.items():
        ax_xy.plot(px, py, 'x', color=COLORS.get(n, 'k'), ms=9, mew=2)
    last = {}
    for n in mon.names:
        d = mon.track[n]
        if not d['t']:
            continue
        ax_xy.plot(d['x'], d['y'], '-', color=COLORS[n], lw=1.6, label=n)
        x, y, yaw = d['x'][-1], d['y'][-1], d['yaw'][-1]
        last[n] = (x, y)
        ax_xy.plot([x, x + math.cos(yaw) * 0.9], [y, y + math.sin(yaw) * 0.9],
                   '-', color=COLORS[n], lw=2.4)
        ax_xy.plot(x, y, 'o', color=COLORS[n], ms=7)
    if len(last) == 2:
        (x1, y1), (x2, y2) = last[mon.names[0]], last[mon.names[1]]
        ax_xy.plot([x1, x2], [y1, y2], '--', color='#444', lw=1.0)
        # 同上：没在编队就标实时直线距离，别标陈值。
        gap_now = mon.gap_d[-1] if mon.gap_d else float('nan')
        if gap_now != gap_now:
            gap_now = math.dist((x1, y1), (x2, y2))
        ax_xy.annotate(f'{gap_now:.2f} m',
                       ((x1 + x2) / 2, (y1 + y2) / 2),
                       fontsize=9, color='#222',
                       bbox=dict(fc='white', ec='none', alpha=0.7, pad=1.5))
    # ---- 发射位置（2026-10-24 加）----
    # 用户要的"体现侦察机打了 1 发破窗弹、任务机打了 4 发灭火弹"：
    # 位置比时刻更有裁判意义，所以标在轨迹图上而不是时间轴上；同一处的连发
    # 自动并成一个标记写 xN，不会像时间轴那样 4 发挤成一团。总数另见数据表。
    for n in mon.names:
        pts = [(x, y) for x, y, _lb in mon.shots.get(n, []) if x is not None]
        if not pts:
            continue
        grouped = []            # [(x, y, 次数)]：0.5 m 内的算同一处
        for x, y in pts:
            for g in grouped:
                if math.dist((x, y), (g[0], g[1])) < 0.5:
                    g[2] += 1
                    break
            else:
                grouped.append([x, y, 1])
        lb = (mon.shots[n][0][2] or '发射')
        # 破窗弹和灭火弹用不同图形，一眼能分（2026-10-24 用户要求）：
        #   破窗弹 -> 朝上的三角，像往楼上冲击；暗紫色
        #   灭火弹 -> 五角星；品红
        # 认标签里的关键字，不认机号——哪架打什么是赛制决定的，可能会变。
        if '破窗' in lb:
            mk, mcol, msz = '^', '#7048e8', 15
        elif '灭火' in lb:
            mk, mcol, msz = '*', '#d6336c', 18
        else:
            mk, mcol, msz = 'P', '#f76707', 14
        for gx, gy, cnt in grouped:
            ax_xy.plot(gx, gy, marker=mk, color=mcol, ms=msz,
                       mec='white', mew=1.2, zorder=6, linestyle='none')
            ax_xy.annotate(f'{lb}x{cnt}' if cnt > 1 else lb, (gx, gy),
                           textcoords='offset points', xytext=(9, -11),
                           fontsize=8, color=mcol, zorder=6,
                           bbox=dict(fc='white', ec='none', alpha=0.75, pad=1.2))

    # ---- 障碍净距：撞过 3# 柱，这个数值要一眼能看见 ----
    # 开销实测 1.5us/帧(两机)，相对一次重绘 70~80ms 完全可忽略，每帧算。
    for n in mon.names:
        d = mon.track[n]
        if not d['t']:
            continue
        if min_clearance(d['x'][-1], d['y'][-1]) < CLEAR_WARN_M:   # 逼近障碍套红圈
            ax_xy.plot([d['x'][-1]], [d['y'][-1]], 'o', ms=16, mfc='none',
                       mec='#d62728', mew=2.2, zorder=6)
    ax_xy.set_title('水平轨迹')

    ax_xy.set_aspect('equal'); ax_xy.grid(alpha=0.3)
    # 等比例会按数据长宽比缩画框，格子用不满。所以反过来：按**格子**的长宽比
    # 把坐标范围对称撑开（比例一点没变，只是多显示一截空场地），这样这一格就
    # 和旁边几张图一样大。
    _pad = 1.0
    _xlo, _xhi = _x0 - _pad, _x0 + ROOM_X + _pad
    _ylo, _yhi = _y0 - _pad, _y0 + ROOM_Y + _pad
    _cell = ax_xy.get_subplotspec().get_position(fig)
    _fw, _fh = fig.get_size_inches()
    _want = (_cell.width * _fw) / max(_cell.height * _fh, 1e-6)
    _xs, _ys = _xhi - _xlo, _yhi - _ylo
    if _xs / _ys < _want:                    # 画面比格子窄 -> 横向撑
        _d = (_want * _ys - _xs) / 2.0
        _xlo -= _d; _xhi += _d
    else:                                    # 比格子宽 -> 纵向撑
        _d = (_xs / _want - _ys) / 2.0
        _ylo -= _d; _yhi += _d
    ax_xy.set_xlim(_xlo, _xhi)
    ax_xy.set_ylim(_ylo, _yhi)
    ax_xy.set_xlabel('x (m)'); ax_xy.set_ylabel('y (m)')
    ax_xy.legend(loc='upper right', fontsize=8)
    # 飞行统计画在轨迹图**旁边**那条独立坐标区里，不压轨迹（见 main 里的 ax_stats）
    if getattr(fig, 'ax_stats', None) is not None:
        draw_stats_table(fig.ax_stats, mon)

    tmax = max([d['t'][-1] for d in mon.track.values() if d['t']] or [0.0])
    tlo = 0.0                                   # 全程，不滚动

    # ---- ② 间距-时间（2026-10-14 用户要求：双机通信链路图不要了）----
    # 原来这格是链路面板（拓扑动画 + 延迟 + 实时间距大数字）。删掉不丢信息：
    # 实时间距在 ① 左上角两机连线中点一直标着；链路延迟/频率本来就落在
    # formation_diag.csv 里，真要查链路去看 CSV 比看动画准。
    # 换成间距-时间曲线——gap_t/gap_d 一直在采集却从来没画出来过
    # （2026-10-05 那次把曲线换成了大数字），而编队调试要看的恰恰是
    # **间距随时间怎么走**：在哪个航点扎下去、在哪段窜上去，一眼就定位。
    # 2026-10-19 用户要求：**沿航线间距（弧长）那条曲线去掉**。它是 10-15 记过
    # 的伪值源——航线不闭合时弧长差会读成整条航线长（实测 67 m、占 38% 样本），
    # 没在编队的时段还得靠僚机 diag 当门记 NaN 才不至于乱画。采集和 CSV 照旧
    # （mon.gap_d 还在写 formation.csv / formation_diag.csv），只是不画了。
    # 这一格现在只看**两机空间直线距离**：它才是"不许靠太近"那条安全要求的
    # 直接量，也是 formation_follower_node 里 min_separation_m 守的那个量。
    ax_link.clear()
    ax_link.set_title('间距曲线')
    if mon.gap_t and len(mon.gap_line) == len(mon.gap_t):
        ax_link.plot(mon.gap_t, mon.gap_line, '-', color='#1f77b4', lw=1.6,
                     label=f'空间直线距离 {mon.gap_line[-1]:.2f} m')
    # 目标间距：编队要守的就是这条线，偏离它多少一目了然。
    ax_link.axhline(mon.spacing, ls='--', color='#2ca02c', lw=1.2,
                    label=f'目标 {mon.spacing:.1f} m')
    # 安全下限：低于它 formation_follower_node 会冻住参考点（现象是僚机不跟队），
    # 2026-10-07 真机两机相撞（最小直线间距 0.03 m）之后加的那道硬保护。
    if mon.min_sep > 0:
        ax_link.axhline(mon.min_sep, ls=':', color='#d62728', lw=1.4,
                        label=f'安全下限 {mon.min_sep:.1f} m')
    ax_link.set_xlim(tlo, max(tmax, tlo + 5)); ax_link.grid(alpha=0.3)
    ax_link.set_xlabel('t (s)'); ax_link.set_ylabel('直线距离 (m)')
    # 状态带 2026-10-21 搬去独立的「状态时间带」那一格，这里只留距离。
    _hi = max([v for v in mon.gap_line if v == v] or [mon.spacing])
    ax_link.set_ylim(0.0, max(mon.spacing, _hi) + 0.5)
    ax_link.legend(loc='upper right', fontsize=7)

    # ---- ③ 高度-时间（2026-10-05 用户要求：这一格只看高度）----
    # 间距的两条线（目标间距虚线 + 节点自报间距）撤掉了：实时值看右上角链路
    # 面板那个大数字，历史值照常落 formation.csv / formation_diag.csv。
    # 曲线用实线、各机一条，起飞段/定高段的台阶一眼能看出来——这一格现在就是
    # 查"起飞到底停在几米""定高有没有把它拉上去"的地方。
    ax_alt.set_title('高度曲线')
    for n in mon.names:
        d = mon.track[n]
        if d['t']:
            ax_alt.plot(d['t'], d['z'], '-', color=COLORS[n], lw=1.4,
                        label=f'{n} 高度'
                              + (f' {d["z"][-1]:.2f} m' if d['z'] else ''))
    # 这条曲线是 uwb/pose_abs 的 z——仿真里它来自 uwb_ground_truth_node，
    # 是**绝对高度**；而定高钉的是测距雷达的**离地高度**。所以飞过仿地模块
    # 上空时曲线会整体抬起来一个模块高度再落回去，那是仿地效果本身，不是超调。
    # 虚线只是"平地上离地 2.0 m 对应的绝对高度"，仅供对照。
    ax_alt.axhline(mon.cruise_agl, ls=':', color='#aaa', lw=1.0,
                   label=f'平地基准 {mon.cruise_agl:.1f} m')
    ax_alt.set_xlim(tlo, max(tmax, tlo + 5)); ax_alt.grid(alpha=0.3)
    ax_alt.set_xlabel('t (s)'); ax_alt.set_ylabel('高度 (m)')
    # 地面到巡航高度之上留一点余量，且一定含 0——否则两机都在 1.5 m 附近时
    # y 轴会被自动缩到 1.4~1.6，看着像在剧烈起伏。
    zs = [z for n in mon.names for z in mon.track[n]['z']]
    if zs:
        ax_alt.set_ylim(min(0.0, min(zs) - 0.1),
                        max(mon.cruise_agl, max(zs)) + 0.3)
    ax_alt.legend(loc='upper right', fontsize=7, ncol=2)

    # ---- ④ 电压-时间（2026-10-05 用户要求：这一格只看电压，纵轴是 6S 总电压）----
    # 速度那条线撤了（均速在 ① 左下角的飞行统计里，瞬时速度也落在 CSV 里），
    # 于是不再需要 twinx——单轴画电压，刻度就是真实伏数，跟满电/告警线对得上。
    # 看什么：电压随负载下坠是正常的，要留意**台阶式骤降**（接触不良/线损）和
    # 整段持续走低压到告警线附近（续航不够，起飞前就该换电池）。
    ax_volt.set_title(f'电压曲线（{BATT_CELLS}S 总电压）')
    for n in mon.names:
        b = mon.batt[n]
        if b['t']:
            ax_volt.plot(b['t'], b['v'], '-', color=COLORS[n], lw=1.4,
                         label=f'{n} {b["v"][-1]:.2f} V')
    ax_volt.axhline(PACK_FULL_V, ls=':', color='#aaa', lw=1.0,
                    label=f'满电 {PACK_FULL_V:.1f} V')
    ax_volt.axhline(PACK_WARN_V, ls='--', color='#d62728', lw=1.2,
                    label=f'告警 {PACK_WARN_V:.1f} V')
    ax_volt.set_xlim(tlo, max(tmax, tlo + 5)); ax_volt.grid(alpha=0.3)
    ax_volt.set_xlabel('t (s)'); ax_volt.set_ylabel('电压 (V)')
    # 纵轴始终含告警线和满电线：只按数据自适应的话，一整趟都在 23~24 V 时
    # 看不出离告警线还有多远，台阶式骤降反而被放大成满屏震荡。
    vs = [v for n in mon.names for v in mon.batt[n]['v']]
    lo = min([PACK_WARN_V] + vs) - 0.3
    hi = max([PACK_FULL_V] + vs) + 0.3
    ax_volt.set_ylim(lo, hi)
    if not vs:      # 一条都没收到：说明 /mavros/battery 没数据，别让人以为电压正常
        ax_volt.text(0.5, 0.5, '没收到 /mavros/battery', transform=ax_volt.transAxes,
                     ha='center', va='center', fontsize=10, color='#d62728')
    ax_volt.legend(loc='lower left', fontsize=7, ncol=2)

    # ---- ⑤ 速度曲线、⑥ 状态时间带 ----
    draw_speed_panel(ax_speed, mon, tlo, tmax)
    draw_state_panel(ax_state, mon, tlo, tmax)


def summarize(mon):
    lines = list(stats_lines(mon))
    if mon.diag_lag:
        import statistics as st
        lines.append(f'节点自报落后量：中位 {st.median(mon.diag_lag):+.2f} m，'
                     f'最小 {min(mon.diag_lag):+.2f}，最大 {max(mon.diag_lag):+.2f}')
    if mon.diag_gap:
        import statistics as st
        g = [v for v in mon.diag_gap if v == v]
        if g:
            lines.append(f'节点自报间距：中位 {st.median(g):.2f} m，'
                         f'最小 {min(g):.2f}，最大 {max(g):.2f}')
    if any(abs(v) > 1e-6 for v in mon.diag_trim):
        lines.append(f'偏置补偿量：末值 {mon.diag_trim[-1]:+.2f} m，'
                     f'最大 {max(mon.diag_trim):+.2f}，最小 {min(mon.diag_trim):+.2f}')
    if mon.gap_d:
        import statistics as st
        # 只统计"正在编队"那些样本，NaN 是没编队的时段（见 DIAG_FRESH_S）。
        gd = [v for v in mon.gap_d if v == v]
        skipped = len(mon.gap_d) - len(gd)
        if gd:
            lines.append(f'沿航线间距：中位 {st.median(gd):.2f} m，'
                         f'最小 {min(gd):.2f}，最大 {max(gd):.2f}'
                         f'（编队中 {len(gd)} 点）')
        else:
            lines.append('沿航线间距：全程没收到编队 diag，这个指标没数')
        if skipped:
            lines.append(f'（其中 {skipped} 点没在编队，曲线上是断开的）')
        if mon.gap_line:
            lines.append(f'（直线距离对照：中位 {st.median(mon.gap_line):.2f} m，'
                         f'最小 {min(mon.gap_line):.2f}，最大 {max(mon.gap_line):.2f}）')
    for n in mon.names:
        d = mon.track[n]
        if d['z']:
            import statistics as st
            med = st.median(d['z'])
            spikes = [z for z in d['z'] if abs(z - med) > 1.5]
            msg = (f'{n}：采样 {len(d["t"])} 点，'
                   f'高度 {min(d["z"]):.2f}~{max(d["z"]):.2f} m（中位 {med:.2f}）')
            if spikes:
                msg += f'，偏离中位 >1.5 m 的野值 {len(spikes)} 个（UWB 抖动？）'
            lines.append(msg)
    return lines


def main():
    ap = argparse.ArgumentParser(description='编队飞行实时监视 + PNG 报告')
    ap.add_argument('--leader', default='NX01')
    ap.add_argument('--follower', default='NX02')
    ap.add_argument('--spacing', type=float, default=4.0)
    ap.add_argument('--cruise-agl', type=float, default=2.0)
    ap.add_argument('--tol', type=float, default=1.2, help='入列容差，画成间距的绿带')
    # 2026-10-19：②面板那条红色安全下限线的值。默认 4.0 跟 .env 的
    # FORMATION_MIN_SEPARATION_M 对齐（entrypoint 以它下发节点参数
    # min_separation_m）；两边不一致时图上那条线就是错的，所以调参改了 .env
    # 要记得也把这个值传进来。0 = 不画这条线。
    ap.add_argument('--min-sep', type=float, default=4.0,
                    help='两机直线距离的安全下限，画成②面板的红色虚线（0=不画）')
    ap.add_argument('--out', default='/logs/formation_report.png')
    ap.add_argument('--timeout', type=float, default=1800.0)
    ap.add_argument('--route', default='',
                    help='规划航线（世界坐标，"x,y x,y ..."），必须跟**长机实际带队跑的'
                         '那条逐点一致**。给了才按"沿航线弧长"算间距，不给退回直线距离。'
                         '⚠️ 闭合回路要把起点**再写一遍当末点**（lead_formation 会把 '
                         'final_xy 接到尾巴上）：RoutePath.is_closed() 靠首尾距离 <0.5 m 判闭合，'
                         '不闭合就不走"环上取短边"保护，两机停在起降点上时间距会读成'
                         '整条航线长（2026-10-15 实测 67 m）。也**不要**把起降点接在开头——'
                         '那正好会把闭合破掉。')
    ap.add_argument('--layout', default='',
                    help='布局 yaml（画俯视底图用：房间/立柱/起降点/障碍/仿地模块）。'
                         '不给就用脚本里写死的 fire_drill_room 那组老坐标。')
    ap.add_argument('--no-gui', action='store_true', help='只出PNG，不弹窗')
    ap.add_argument('--end-on-all-land', '--end-on-leader-land', action='store_true',
                    dest='end_on_all_land',
                    help='改用"**两机都降落**"当收尾判据。多轮流程（综合任务）必须'
                         '用它：轮次之间任务机会降落在起降点、侦察机在 A 点悬停等'
                         '下一轮，正好满足默认判据（一架已降落、另一架原地悬停），'
                         '连续成立就会在半道退出——而轮次间隔本来就可能好几分钟，'
                         '调长确认窗口治不了。而"两机同时在地上"只有最后一轮结束'
                         '才成立，轮次之间侦察机一直在 A 点悬停。'
                         '2026-10-01 改过一次：原来只看侦察机，但最后一轮是侦察机'
                         '先落、任务机还要再飞 85 秒去物资点还器材再回家，监视'
                         '提前关掉了，那段没录上。--end-on-leader-land 是旧名字。')
    ap.add_argument('--end-confirm', type=float, default=40.0,
                    help='结束条件要连续成立多少秒才真的收尾。默认 40 秒——'
                         '任务流程里"一架落地取物资、另一架悬停等待"会让结束'
                         '条件短暂成立，撑不满这个窗口就不会误判结束。')
    args = ap.parse_args()

    if not args.layout:
        print('[监视] ⚠️ 没给 --layout，俯视底图用的是写死的 fire_drill_room 老坐标'
              '（房间原点在中心、立柱 (4.5,7) 那套）。跑样题场景务必带上 --layout，'
              '否则轨迹会叠在错误的底图上。', flush=True)
    if args.layout:
        try:
            load_scene(args.layout)
            print(f'[监视] 底图取自 {args.layout}（房间 {ROOM_X:.0f}x{ROOM_Y:.0f}，'
                  f'原点 {ROOM_ORIGIN}，立柱 {len(PILLARS)} 根）', flush=True)
        except Exception as exc:
            print(f'[监视] 布局 {args.layout} 读不了（{exc}），底图退回写死的老坐标——'
                  f'俯视图的房间/立柱位置可能跟实际场景对不上', flush=True)
    gui = (not args.no_gui) and bool(os.environ.get('DISPLAY'))
    matplotlib.use('TkAgg' if gui else 'Agg')
    import matplotlib.pyplot as plt              # 只绑定 plt，不会遮蔽模块级的 matplotlib
    # 中文字体：直接给 matplotlib 一串候选，它自己按顺序回退（原来写成
    # 每轮只塞一个再 break，等于永远用第一个，装没装都一样，只会刷
    # "findfont: Generic family 'sans-serif' not found" 的警告）。
    # 容器里装了 fonts-wqy-zenhei 才有中文，没有的话汉字会变方框，
    # 但曲线和数字照常可读。
    matplotlib.rcParams['font.sans-serif'] = [
        'WenQuanYi Zen Hei', 'Noto Sans CJK SC', 'Noto Sans CJK JP',
        'SimHei', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

    rclpy.init()
    route = [tuple(float(v) for v in tok.split(','))
             for tok in args.route.split()] if args.route else None
    mon = FormationMonitor([args.leader, args.follower],
                           args.spacing, args.cruise_agl, args.tol, route,
                           min_sep=args.min_sep)
    # 版面：上两行是 2x3 六张图（水平轨迹 / 速度曲线 / 间距曲线 ／
    # 状态时间带 / 高度曲线 / 电压曲线），最下面一条扁表放飞行数据。
    # 边距写死在 gridspec 里，draw() 不调 tight_layout——那个会按标签重算一切。
    fig = plt.figure(figsize=(15, 9.5))
    # 标题只给窗口用，不画进图里——报告的"名字"落在文件名上（运行器给的 --out）
    try:                      # 无窗口模式没有 manager，不要为它炸掉
        fig.canvas.manager.set_window_title(
            time.strftime('%Y-%m-%d %H:%M') + ' 飞行报告')
    except Exception:
        pass
    _gs = fig.add_gridspec(3, 3, height_ratios=[1.0, 1.0, 0.17],
                           hspace=0.42, wspace=0.20,
                           left=0.045, right=0.985, top=0.945, bottom=0.055)
    # 顺序按 draw() 的拆包来：轨迹 / 间距 / 高度 / 电压 / 速度 / 状态
    axes = (fig.add_subplot(_gs[0, 0]), fig.add_subplot(_gs[0, 2]),
            fig.add_subplot(_gs[1, 1]), fig.add_subplot(_gs[1, 2]),
            fig.add_subplot(_gs[0, 1]), fig.add_subplot(_gs[1, 0]))
    fig.ax_stats = fig.add_subplot(_gs[2, :])
    fig.ax_stats.axis('off')
    if gui:
        plt.ion(); plt.show(block=False)

    stop = {'v': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))
    t_begin = time.monotonic()
    next_draw = 0.0
    print(f'[监视] 订阅 /{args.leader}/uwb/pose_abs 和 /{args.follower}/uwb/pose_abs，'
          f'{"有" if gui else "无"}窗口模式，报告将存到 {args.out}', flush=True)
    try:
        done_since = None       # 结束条件从什么时候开始连续成立
        while not stop['v'] and time.monotonic() - t_begin < args.timeout:
            rclpy.spin_once(mon, timeout_sec=0.05)
            now = time.monotonic() - t_begin
            if now >= next_draw:
                next_draw = now + 0.5           # 2 Hz 重绘，够流畅也不抢 CPU
                draw(fig, axes, mon)
                if gui:
                    # 2026-10-14：这里原来是 plt.pause(0.001)，用户反馈"监控图表
                    # 老是置顶"。plt.pause() 内部每次都会调 show(block=False)，
                    # TkAgg 下那一句会 deiconify + 把窗口抬到最前并抢焦点——2 Hz
                    # 重绘就是每 0.5 秒抢一次，没法在它上面干别的活。
                    # 换成"只重绘 + 只抽事件"，两者都不碰窗口层级。
                    try:
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()
                    except Exception:
                        pass        # 窗口被用户关掉之后照常跑无窗口逻辑
            # 结束判据要**连续成立** END_CONFIRM_S 秒才算数。
            # 2026-09-30：任务2 里 NX02 是**真的降落**到物资点去抓灭火弹的，
            # 而那一刻 NX01 正在 G 点悬停等它——"一架已降落、另一架原地悬停"
            # 当场成立，监视就在任务半道出报告退出了（用户发现窗口自己关了）。
            # 抓取那十几秒撑不满确认窗口，真正的结束能撑满。
            if args.end_on_all_land:
                ended = mon._all_airborne_seen and all(mon.landed[n] for n in mon.names)
            else:
                ended = mon.mission_done()
            if ended and now > 20.0:
                if done_since is None:
                    done_since = now
                elif now - done_since >= args.end_confirm:
                    who = '、'.join(n for n in mon.names if mon.landed[n])
                    why = (f'{who} 都已降落' if args.end_on_all_land
                           else f'{who} 已降落，其余在原地悬停')
                    print(f'[监视] 任务结束（{why}，条件连续成立 '
                          f'{args.end_confirm:.0f} 秒），出报告', flush=True)
                    break
            else:
                done_since = None
    finally:
        draw(fig, axes, mon)
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        fig.savefig(args.out, dpi=130)
        print(f'[监视] 报告已存 {args.out}', flush=True)
        csv_path = os.path.splitext(args.out)[0] + '.csv'
        try:
            with open(csv_path, 'w', encoding='utf-8') as f:
                f.write('t,gap_route,gap_line,x1,y1,x2,y2\n')
                for i, t in enumerate(mon.gap_t):
                    x1, y1, x2, y2 = mon.gap_xy[i]
                    # 没在编队的那些点 gap_route 是 NaN，写成**空**而不是 "nan"：
                    # 空在 awk/pandas 里都当缺失，写 "nan" 的话 awk 的 $2>40
                    # 会按字符串比、判成真（踩过）。
                    g = mon.gap_d[i]
                    gs_ = f'{g:.4f}' if g == g else ''
                    f.write(f'{t:.3f},{gs_},{mon.gap_line[i]:.4f},'
                            f'{x1:.4f},{y1:.4f},{x2:.4f},{y2:.4f}\n')
            with open(os.path.splitext(args.out)[0] + '_diag.csv', 'w', encoding='utf-8') as f:
                f.write('t,lag,gap_node,trim,s_cmd,s_proj,total_len,sep_line,v_ff,ff_used\n')
                for i, t in enumerate(mon.diag_t):
                    f.write(f'{t:.3f},{mon.diag_lag[i]:.4f},'
                            f'{mon.diag_gap[i]:.4f},{mon.diag_trim[i]:.4f},'
                            f'{mon.diag_scmd[i]:.4f},{mon.diag_sproj[i]:.4f},'
                            f'{mon.diag_total[i]:.4f},{mon.diag_sepline[i]:.4f},'
                            f'{mon.diag_vff[i]:.4f},{mon.diag_ffused[i]:.0f}\n')
            print(f'[监视] 原始数据已存 {csv_path}（+_diag.csv）', flush=True)
        except Exception as exc:                       # 存不下不影响报告
            print(f'[监视] CSV 没存成：{exc}', flush=True)
        for line in summarize(mon):
            print('   ' + line, flush=True)
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编队飞行实时监视 + 落地后自动出 PNG 报告图。

飞行中弹一个窗口，三张图实时刷新：
  ① 俯视轨迹图——场地边界/三根立柱/两个起降点当底图，两机轨迹折线 + 当前位置
     （带机头方向的三角标）+ 两机连线，连线中点标实时间距；
  ② 高度-时间曲线——两机各一条，叠一条巡航高度参考虚线；
  ③ 间距-时间曲线——叠目标间距虚线和入列容差带。

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
       python3 /opt/host_scripts/编队监视.py --out /logs/formation.png'

没有图形界面（DISPLAY 没给、或 xhost 没放行）时自动退回无窗口模式：不弹窗，
照常记录，结束时仍然出 PNG。
"""
import argparse
import math
import os
import signal
import sys
import time

import matplotlib
import matplotlib.patches      # 模块级导入：放 main() 里会把 matplotlib 变成局部名

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

# 场地底图。2026-09-29 之前这几个常量是**写死的 fire_drill_room 坐标**，
# 换成样题场景之后俯视图画的还是老房子（房间画在原点居中、立柱在 (4.5,7)
# 那套、起降点在 (±2,-9.5)），轨迹叠在错误的底图上，看着像飞机飞到墙外去了。
# 现在改成从布局 yaml 读，读不到才退回这组老值（并打一行提示，不静默）。
ROOM_X, ROOM_Y = 20.0, 25.0
ROOM_ORIGIN = 'center'               # 'center'=原点在房间中心；'southwest'=在西南角
PILLARS = [(4.5, 7.0), (-4.5, 7.0), (0.0, 0.0)]
PILLAR_HALF = 0.5
PADS = {'NX01': (2.0, -9.5), 'NX02': (-2.0, -9.5)}
CYLINDERS = []                       # [(x, y, r), ...]
TERRAIN = None                       # (x, y, size_x, size_y)


def load_scene(path):
    """从布局 yaml 覆盖上面那组底图常量。找不到文件就保持原值。"""
    global ROOM_X, ROOM_Y, ROOM_ORIGIN, PILLARS, PILLAR_HALF, PADS, CYLINDERS, TERRAIN
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
        if q.get('twin_east'):       # 正东孪生柱，底图上要画成两个方块
            PILLARS.append((float(q['x']) + 2 * PILLAR_HALF, float(q['y'])))
    PADS = {p['id'].upper(): (float(p['x']), float(p['y']))
            for p in L['takeoff_landing_pads']}
    c = L.get('obstacle_cylinder')
    CYLINDERS = [(float(c['x']), float(c['y']), float(c['diameter']) / 2)] if c else []
    t = L.get('terrain_module')
    TERRAIN = (float(t['x']), float(t['y']), float(t['size_x']), float(t['size_y'])) if t else None
COLORS = {'NX01': '#d62728', 'NX02': '#1f77b4'}   # 长机红、僚机蓝

AIRBORNE_Z = 1.0        # 高于这个算"在空中"（判"两机都起飞过"用）
HOVER_DONE_S = 20.0     # 没降落的飞机原地悬停这么久也算任务结束（长机在 A 点选题）
HOVER_DONE_M = 0.8      # 这段时间内水平位移不超过这个算"原地"
LANDED_AGL = 0.3        # 低于这个高度算落地
LANDED_HOLD = 3.0       # 且要持续这么久
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
        """投影到航线折线上，返回弧长坐标（钳在 [0, 全长] 内）。

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
            return 0.0
        best_d2 = min(c[0] for c in cands)
        if last_s is None:
            return min(cands, key=lambda c: c[0])[1]
        tol = (math.sqrt(best_d2) + self.ALIAS_TOL_M) ** 2
        near = [c for c in cands if c[0] <= tol]
        return min(near, key=lambda c: abs(c[1] - last_s))[1]


class FormationMonitor(Node):
    def __init__(self, names, spacing, cruise_agl, tol, route=None):
        super().__init__('formation_monitor')
        self.names = names
        self.route = RoutePath(route) if route else None
        self.spacing = spacing
        self.cruise_agl = cruise_agl
        self.tol = tol
        self.t0 = time.monotonic()
        self.wall0 = time.time()              # 跟 t0 同一时刻的墙上时间，换算起飞时刻用
        self.track = {n: {'t': [], 'x': [], 'y': [], 'z': [], 'yaw': []} for n in names}
        # ---- 飞行统计（2026-09-30 用户要求：动画上显示起飞时刻、编队时长、里程、均速）----
        self.takeoff_t = {n: None for n in names}   # 首次高于 AIRBORNE_Z 的时刻（相对 t0）
        self.land_t = {n: None for n in names}      # 判定降落的时刻
        self.dist_m = {n: 0.0 for n in names}       # 水平飞行里程（累加）
        self._last_xy = {n: None for n in names}    # 上一帧位置，算增量用
        self.gap_t, self.gap_d = [], []      # 沿航线的间距（主指标）
        self.gap_line = []                    # 直线距离，只作参考对照
        self.gap_xy = []                      # 算这个间距时两机的原始 xy，存 CSV 用
        self._all_airborne_seen = False       # 是否出现过"两机同时在空中"
        self.diag_t, self.diag_lag = [], []   # 跟随节点自报的落后量
        self.diag_trim = []                   # 跟随距离的偏置补偿量（data[5]，可能没有）
        self.diag_gap = []                    # 节点自报的实际间距 = 长机轨迹全长 - 僚机投影（data[4]）
        # 两机上一次的航线弧长，投影消歧义用。初值取 0.0 而不是 None：飞行总是
        # 从航线第一个点开始，这个先验是确定的。用 None 的话第一帧没有参考、
        # 只能任选，闭合航线上会选中末端（实测锁在 66 m 上把间距顶到 62）。
        self._last_s = [0.0, 0.0]
        # 长机离开航线起点多远才开始记间距。编队开始之前这个指标没有意义——
        # 僚机停在自己起降点时，按"沿航线弧长"算可能是几十米（本场地僚机的
        # 起降点正好落在航线最后一段上，实测读数 61 m，数学上没错但没意义）。
        self.GAP_START_MOVE_M = 1.0
        self._gap_started = False
        self._low_since = {n: None for n in names}
        self.landed = {n: False for n in names}
        for n in names:
            self.create_subscription(PoseStamped, f'/{n}/uwb/pose_abs',
                                     self._make_cb(n), 10)
        # 僚机跟随回路自报的落后量——外面自己拿两机位置算弧长跟节点内部的
        # 基准不一样（折线、采样率、向后延伸段都不同），要判断控制好坏得看它自己的数
        self.create_subscription(Float64MultiArray, f'/{names[1]}/formation_diag',
                                 self._on_diag, 10)

    def _make_cb(self, name):
        def cb(msg):
            p, q = msg.pose.position, msg.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            d = self.track[name]
            t = time.monotonic() - self.t0
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

    def _on_diag(self, msg):
        if len(msg.data) >= 1:
            self.diag_t.append(time.monotonic() - self.t0)
            self.diag_lag.append(float(msg.data[0]))
            self.diag_trim.append(float(msg.data[5]) if len(msg.data) >= 6 else 0.0)
            # data[4] 是节点自己算的"长机轨迹全长 - 僚机投影弧长"，也就是控制回路
            # 眼里的真实间距。画 3.5+lag 是不对的：lag 是相对**补偿后**的跟随距离
            # 算的（lag = 实际间距 - fd_eff），3.5+lag 会恒比实际间距高出一个 trim。
            self.diag_gap.append(float(msg.data[4]) if len(msg.data) >= 5 else float('nan'))

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
        # 编队还没开始（长机还没离开航线起点）就不记——见 GAP_START_MOVE_M
        if not self._gap_started:
            if self.route is None or not self.route.ok():
                self._gap_started = True       # 没航线就没这个问题，直接开记
            else:
                start = (self.route.xs[0], self.route.ys[0])
                if math.dist(pts[0], start) < self.GAP_START_MOVE_M:
                    return
                self._gap_started = True

        # 两机都投影到**规划航线**上，间距 = 弧长差（同段内即直线距离，
        # 跨段即经过航点那几条线段之和）。没给航线就退回直线距离。
        if self.route is not None and self.route.ok():
            s0 = self.route.project(*pts[0], last_s=self._last_s[0])
            s1 = self.route.project(*pts[1], last_s=self._last_s[1])
            self._last_s = [s0, s1]
            gap = self.route.gap_between(s0, s1)
        else:
            gap = math.dist(pts[0], pts[1])
        self.gap_t.append(t)
        self.gap_d.append(gap)
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


def draw(fig, axes, mon):
    ax_xy, ax_z, ax_gap = axes
    for ax in axes:
        ax.clear()

    # ---- ① 俯视轨迹 ----
    ax_xy.set_title('水平轨迹（俯视）')
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
        gap_now = mon.gap_d[-1] if mon.gap_d else math.dist((x1, y1), (x2, y2))
        ax_xy.annotate(f'{gap_now:.2f} m',
                       ((x1 + x2) / 2, (y1 + y2) / 2),
                       fontsize=9, color='#222',
                       bbox=dict(fc='white', ec='none', alpha=0.7, pad=1.5))
    ax_xy.set_aspect('equal'); ax_xy.grid(alpha=0.3)
    ax_xy.set_xlabel('x (m)'); ax_xy.set_ylabel('y (m)')
    ax_xy.legend(loc='upper right', fontsize=8)
    # 飞行统计（2026-09-30 用户要求：动画上直接显示起飞时刻/编队时长/里程/均速）。
    # 贴在俯视图左下角——equal aspect 在这里本来就留白，不遮轨迹。
    txt = '\n'.join(stats_lines(mon))
    if txt:
        ax_xy.text(0.02, 0.02, txt, transform=ax_xy.transAxes,
                   va='bottom', ha='left', fontsize=7.5,
                   bbox=dict(fc='white', ec='#bbb', alpha=0.85, pad=3))

    tmax = max([d['t'][-1] for d in mon.track.values() if d['t']] or [0.0])
    tlo = 0.0                                   # 全程，不滚动

    # ---- ② 高度 ----
    ax_z.set_title('高度-时间（绝对高度，抬起=飞过仿地模块）')
    for n in mon.names:
        d = mon.track[n]
        if d['t']:
            ax_z.plot(d['t'], d['z'], '-', color=COLORS[n], lw=1.4, label=n)
    # 这条曲线是 uwb/pose_abs 的 z——仿真里它来自 uwb_ground_truth_node，
    # 是**绝对高度**；而定高钉的是测距雷达的**离地高度**。所以飞过仿地模块
    # 上空时曲线会整体抬起来一个模块高度再落回去，那是仿地效果本身，不是超调。
    # 虚线只是"平地上离地 2.0 m 对应的绝对高度"，仅供对照。
    ax_z.axhline(mon.cruise_agl, ls='--', color='#888', lw=1.0,
                 label=f'平地基准 {mon.cruise_agl:.1f} m')
    ax_z.set_xlim(tlo, max(tmax, tlo + 5)); ax_z.grid(alpha=0.3)
    ax_z.set_ylabel('z (m)'); ax_z.legend(loc='lower right', fontsize=8)

    # ---- ③ 间距 ----
    # 2026-09-28（用户要求）：这张图只留两条线——目标间距 + 节点自报间距。
    # 之前叠的"按航线投影算的间距""直线距离""补偿后的有效跟随距离"都撤掉，
    # 数据照常记进 CSV，要复盘再翻 CSV。
    ax_gap.set_title('两机间距-时间')
    ax_gap.axhline(mon.spacing, ls='--', color='#888', lw=1.2,
                   label=f'目标 {mon.spacing:.1f} m')
    if mon.diag_t:
        ax_gap.plot(mon.diag_t, mon.diag_gap,
                    '-', color='#ff7f0e', lw=1.4, label='节点自报间距')
    ax_gap.set_xlim(tlo, max(tmax, tlo + 5)); ax_gap.grid(alpha=0.3)
    ax_gap.set_xlabel('t (s)'); ax_gap.set_ylabel('间距 (m)')
    ax_gap.legend(loc='upper right', fontsize=8)
    fig.tight_layout()


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
        lines.append(f'沿航线间距：中位 {st.median(mon.gap_d):.2f} m，'
                     f'最小 {min(mon.gap_d):.2f}，最大 {max(mon.gap_d):.2f}')
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
    ap.add_argument('--out', default='/logs/formation_report.png')
    ap.add_argument('--timeout', type=float, default=1800.0)
    ap.add_argument('--route', default='',
                    help='规划航线（世界坐标，"x,y x,y ..."，第一个是长机起飞点）。'
                         '给了才按"沿航线弧长"算间距，不给退回直线距离。')
    ap.add_argument('--layout', default='',
                    help='布局 yaml（画俯视底图用：房间/立柱/起降点/障碍/仿地模块）。'
                         '不给就用脚本里写死的 fire_drill_room 那组老坐标。')
    ap.add_argument('--no-gui', action='store_true', help='只出PNG，不弹窗')
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
                           args.spacing, args.cruise_agl, args.tol, route)
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
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
                    plt.pause(0.001)
            # 结束判据要**连续成立** END_CONFIRM_S 秒才算数。
            # 2026-09-30：任务2 里 NX02 是**真的降落**到物资点去抓灭火弹的，
            # 而那一刻 NX01 正在 G 点悬停等它——"一架已降落、另一架原地悬停"
            # 当场成立，监视就在任务半道出报告退出了（用户发现窗口自己关了）。
            # 抓取那十几秒撑不满确认窗口，真正的结束能撑满。
            if mon.mission_done() and now > 20.0:
                if done_since is None:
                    done_since = now
                elif now - done_since >= args.end_confirm:
                    who = '、'.join(n for n in mon.names if mon.landed[n])
                    print(f'[监视] 任务结束（{who} 已降落，其余在原地悬停，'
                          f'条件连续成立 {args.end_confirm:.0f} 秒），出报告', flush=True)
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
                    f.write(f'{t:.3f},{mon.gap_d[i]:.4f},{mon.gap_line[i]:.4f},'
                            f'{x1:.4f},{y1:.4f},{x2:.4f},{y2:.4f}\n')
            with open(os.path.splitext(args.out)[0] + '_diag.csv', 'w', encoding='utf-8') as f:
                f.write('t,lag,gap_node,trim\n')
                for i, t in enumerate(mon.diag_t):
                    f.write(f'{t:.3f},{mon.diag_lag[i]:.4f},'
                            f'{mon.diag_gap[i]:.4f},{mon.diag_trim[i]:.4f}\n')
            print(f'[监视] 原始数据已存 {csv_path}（+_diag.csv）', flush=True)
        except Exception as exc:                       # 存不下不影响报告
            print(f'[监视] CSV 没存成：{exc}', flush=True)
        for line in summarize(mon):
            print('   ' + line, flush=True)
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

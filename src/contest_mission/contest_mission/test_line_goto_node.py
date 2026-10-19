#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""line_goto_node 的几何/速度剖面自检。不起 ROS，只测 LinePath 和 SpeedProfile，
外加一个"把整条航线走完"的离散积分仿真（死锁/停留/过冲都在这儿暴露）。"""
import math, sys
from contest_mission.line_goto_node import LinePath, SpeedProfile

FAIL = []
def chk(cond, msg):
    print(('  OK   ' if cond else '  FAIL ') + msg)
    if not cond:
        FAIL.append(msg)

# 本场真实航线 A->B->C->G->E->F->G->D（G 出现两次，折线自交）
ROUTE = [(3.0, 3.0), (3.0, 22.0), (10.0, 22.0), (10.0, 12.0),
         (17.0, 12.0), (17.0, 22.0), (10.0, 12.0), (17.0, 3.0)]
START = (2.6, 2.4)
AGL = [2.0] * (len(ROUTE) + 1)

path = LinePath([START] + ROUTE, AGL)
print('航线：%d 个点（含起点），总长 %.2f m' % (len(path.pts), path.total()))
chk(len(path.pts) == 9, '去重后 9 个点（G 重复但不相邻，不该被去掉）')

# ---- 1. 直线性：任意 s 处的点必须精确落在折线上 ----
def dist_to_polyline(x, y, pts):
    best = 1e9
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]; x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
        best = min(best, math.hypot(x - (x0 + t * dx), y - (y0 + t * dy)))
    return best

worst = 0.0
N = 20000
for k in range(N + 1):
    s = path.total() * k / N
    x, y, _z, (ux, uy) = path.point_at(s)
    worst = max(worst, dist_to_polyline(x, y, path.pts))
    chk(abs(math.hypot(ux, uy) - 1.0) < 1e-9, '方向是单位向量') if k == 0 else None
print('  %d 个采样点，离折线最大偏差 = %.3e m' % (N + 1, worst))
chk(worst < 1e-9, '输出位置精确落在折线上（数值级，不是"接近"）')

# ---- 2. 速度剖面 ----
CRUISE, ACCEL, DV = 0.8, 0.5, 0.4
prof = SpeedProfile(path, CRUISE, ACCEL, DV)
print('  断点限速 =', [round(v, 3) for v in prof.v])
chk(prof.v[0] == 0.0, '起点速度 = 0')
chk(prof.v[-1] == 0.0, '终点速度 = 0（抵达时速度为零）')
chk(max(prof.v) <= CRUISE + 1e-12, '处处不超巡航限速')
for i in range(len(prof.v) - 1):
    ds = path.arc[i + 1] - path.arc[i]
    reach = math.sqrt(prof.v[i] ** 2 + 2 * ACCEL * ds) + 1e-9
    brake = math.sqrt(prof.v[i + 1] ** 2 + 2 * ACCEL * ds) + 1e-9
    chk(prof.v[i + 1] <= reach, '段%d 加速可达' % i) if prof.v[i+1] > reach else None
    chk(prof.v[i] <= brake, '段%d 刹得住' % i) if prof.v[i] > brake else None
chk(all(prof.v[i + 1] <= math.sqrt(prof.v[i] ** 2 + 2 * ACCEL * (path.arc[i + 1] - path.arc[i])) + 1e-9
        for i in range(len(prof.v) - 1)), '前向可达性全段成立')
chk(all(prof.v[i] <= math.sqrt(prof.v[i + 1] ** 2 + 2 * ACCEL * (path.arc[i + 1] - path.arc[i])) + 1e-9
        for i in range(len(prof.v) - 1)), '后向刹车性全段成立（会提前减速）')
# 拐点限速必须满足 2v sin(θ/2) <= corner_dv_max
for i in range(1, len(path.pts) - 1):
    ax, ay = path.dirs[i - 1]; bx, by = path.dirs[i]
    th = math.acos(max(-1.0, min(1.0, ax * bx + ay * by)))
    dv = 2 * prof.v[i] * math.sin(th / 2)
    chk(dv <= DV + 1e-6 or prof.v[i] >= CRUISE - 1e-9,
        '拐点%d 转角%.0f° 速度增量 %.3f <= %.2f' % (i, math.degrees(th), dv, DV))

# ---- 3. 走完全程的离散积分（死锁、过冲、加速度越界都在这暴露） ----
def run(stop_at=None, dwell=0.0, dt=0.02, cruise=CRUISE, accel=ACCEL):
    p = SpeedProfile(path, cruise, accel, DV, stop_at=stop_at)
    s, v, t = 0.0, 0.0, 0.0
    max_a, dwell_hits, vs = 0.0, 0, []
    dwell_until = None; done = set()
    while s < path.total() - 1e-9 and t < 600.0:
        eff_dt = dt
        if dwell_until is not None:
            if t < dwell_until: eff_dt = 0.0
            else: dwell_until = None
        elif stop_at and dwell > 0:
            for i, f in enumerate(stop_at):
                if f and i not in done and s >= path.arc[i] - 0.05:
                    done.add(i); dwell_until = t + dwell; eff_dt = 0.0; dwell_hits += 1
                    break
        ceil = p.ceiling(s)
        v_new = max(0.0, max(min(v + accel * eff_dt, ceil), v - accel * eff_dt))
        if eff_dt > 0: max_a = max(max_a, abs(v_new - v) / eff_dt)
        v = v_new
        s = min(path.total(), s + v * eff_dt)
        vs.append(v)
        t += dt
    # 到终点之后再跑几拍，让速度按限幅收敛到 0（节点里是靠到达判据的
    # arrival_v_eps 兜住，这里要测"真的能收到 0"）
    for _ in range(200):
        ceil = p.ceiling(s)
        v_new = max(0.0, max(min(v + accel * dt, ceil), v - accel * dt))
        max_a = max(max_a, abs(v_new - v) / dt)
        v = v_new
        vs.append(v)
        if v <= 1e-9: break
    return t, s, v, max_a, dwell_hits, vs

t, s_end, v_end, max_a, _h, vs = run()
print('  连续飞完：%.1f s，终点 s=%.3f/%.3f，终点速度 %.4f m/s，实测最大加速度 %.3f m/s²'
      % (t, s_end, path.total(), v_end, max_a))
chk(s_end >= path.total() - 1e-6, '走到航线终点（没死锁）')
chk(v_end <= 1e-3, '终点速度为 0')
chk(max_a <= ACCEL + 1e-6, '全程加速度不超 accel_mps2（缓慢加速）')
chk(max(vs) <= CRUISE + 1e-9, '全程不超巡航限速')
# 起步缓慢：0 -> cruise 至少要 cruise/accel 秒
first_full = next((i for i, v in enumerate(vs) if v >= CRUISE - 1e-6), None)
chk(first_full is None or first_full * 0.02 >= CRUISE / ACCEL - 0.05,
    '起步到满速用了 >= %.2f s（缓慢加速）' % (CRUISE / ACCEL))
# 速度连续：相邻两拍差不超过 accel*dt
chk(all(abs(vs[i + 1] - vs[i]) <= ACCEL * 0.02 + 1e-9 for i in range(len(vs) - 1)),
    '速度剖面逐拍连续（无跳变）')
# 中途不停：连续模式下速度不该归零
chk(min(vs[5:-5]) > 1e-3, '整条航线中途速度不归零（没在每个航点停）')

# ---- 4. stop_at + 停留：该停的地方要停住、停完要能起来（死锁回归） ----
stops = [False] + [True] * len(ROUTE)
t2, s2, v2, _a2, hits, vs2 = run(stop_at=stops, dwell=1.0)
print('  逐点停留：%.1f s，s=%.3f/%.3f，触发停留 %d 次' % (t2, s2, path.total(), hits))
chk(s2 >= path.total() - 1e-6, 'stop_at 全开也能走到终点（停留后不死锁）')
chk(hits >= len(ROUTE) - 1, '每个标记的航点都停了（%d 次）' % hits)
chk(t2 > t, '逐点停留比连续飞慢')

# ---- 5. 自交航线的投影消歧 ----
gx, gy = 10.0, 12.0          # G，在航线里出现两次
s_first = path.arc[4]        # 第一次经过 G
s_second = path.arc[7]       # 第二次经过 G
p1 = path.project(gx, gy, s_first)
p2 = path.project(gx, gy, s_second)
print('  G 点两次投影：带 last_s=%.2f -> %.2f ；带 last_s=%.2f -> %.2f'
      % (s_first, p1, s_second, p2))
chk(abs(p1 - s_first) < 1.0, '带第一次的 last_s，投影落在第一个 G 附近')
chk(abs(p2 - s_second) < 1.0, '带第二次的 last_s，投影落在第二个 G 附近')
chk(abs(p1 - p2) > 5.0, '两次投影确实区分开了（不带消歧就会 snap 到同一个）')

# ---- 6. 极短段：后向扫描必须让它别冲过头 ----
tiny = LinePath([(0.0, 0.0), (0.3, 0.0), (0.3, 0.3)], [2.0, 2.0, 2.0])
pt = SpeedProfile(tiny, CRUISE, ACCEL, DV)
chk(pt.v[1] <= math.sqrt(2 * ACCEL * 0.3) + 1e-9, '0.3 m 的段限速被刹车约束压住')
chk(pt.v[-1] == 0.0, '极短航线终点仍然是 0')

print()
if FAIL:
    print('%d 项未通过：' % len(FAIL))
    for m in FAIL: print('  - ' + m)
    sys.exit(1)
print('全部通过')

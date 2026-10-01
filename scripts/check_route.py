#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航线安全检查：算每一段离**所有**障碍的余量，飞之前跑。

    python3 scripts/check_route.py --route "3,3 3,22 17,22 17,16"
    python3 scripts/check_route.py --route "17,16 17,3 3,3" --direct

两个问题分开答：

  **直飞安不安全**（`goto_direct`，不经规划器、没有避障）——看直线余量。
  **规划器要绕的话两侧够不够宽**——直线被挡住时，量障碍两侧各剩多少净宽。
  这条才是 2026-10-01 撞墙的直接原因：A(3,3)->B(3,22) 上的避障圆柱，东侧
  一路敞开、西侧墙到柱面只有 2.75 m，去掉两边各 0.8 m 膨胀只剩 1.15 m 净宽。
  规划器两侧都可能选，选西侧时贴到了 x=0.4，擦墙掉高摔在 (0.55, 8.64)。

**为什么要有这个工具**：同一天我手算过一次余量，障碍列表是手写死的三组立柱，
漏了 `obstacle_cylinder` / `terrain_module` / 四面墙，于是把那条正穿 6 米高
柱子的航段判成"干净"并改成了直飞。障碍要从布局文件里**全部**读出来，
少读一类就等于没算。
"""
import argparse
import math
import os
import sys

DEFAULT_LAYOUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              'src', 'contest_mission', 'config', 'sample_room_layout.yaml')
INFLATE_M = 0.8      # ego_planner 的膨胀半径，算净宽时两边各扣这么多
SAMPLES = 801


def load_obstacles(path):
    """布局里**所有**会挡路的东西。新增障碍种类时必须同步加到这里。"""
    import yaml
    d = yaml.safe_load(open(path, encoding='utf-8'))
    obs = []
    c = d.get('obstacle_cylinder')
    if c:
        obs.append((f"避障圆柱({c['x']},{c['y']})",
                    lambda p, c=c: math.hypot(p[0] - c['x'], p[1] - c['y']) - c['diameter'] / 2))
    size = float(d.get('pillar_size', 1.0))
    for pl in d.get('pillars', []):
        x, y, hw = pl['x'], pl['y'], size / 2
        b = (x - hw, x + hw + (size if pl.get('twin_east') else 0.0), y - hw, y + hw)
        obs.append((f"{pl['id']}", lambda p, b=b: math.hypot(
            max(b[0] - p[0], 0.0, p[0] - b[1]), max(b[2] - p[1], 0.0, p[1] - b[3]))))
    t = d.get('terrain_module')
    if t:
        b = (t['x'] - t['size_x'] / 2, t['x'] + t['size_x'] / 2,
             t['y'] - t['size_y'] / 2, t['y'] + t['size_y'] / 2)
        obs.append((f"仿地模块(高{t['size_z']}m)", lambda p, b=b: math.hypot(
            max(b[0] - p[0], 0.0, p[0] - b[1]), max(b[2] - p[1], 0.0, p[1] - b[3]))))
    r = d['room']
    obs.append(('墙', lambda p, r=r: min(p[0], r['size_x'] - p[0], p[1], r['size_y'] - p[1])))
    return obs, d


def clearance(p, obs):
    best, who = 1e9, ''
    for nm, f in obs:
        v = f(p)
        if v < best:
            best, who = v, nm
    return best, who


def corridor(a, b, obs, t_block):
    """直线在 t_block 处被挡——沿垂线往两侧找，各剩多少净宽。"""
    vx, vy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(vx, vy) or 1.0
    px, py = -vy / n, vx / n                      # 垂直于航段
    cx, cy = a[0] + vx * t_block, a[1] + vy * t_block
    out = []
    for sign in (+1, -1):
        free_start = None
        for i in range(1, 2001):                  # 最远找 20 m，1 cm 一步
            q = (cx + px * sign * i * 0.01, cy + py * sign * i * 0.01)
            d, _ = clearance(q, obs)
            if d > INFLATE_M and free_start is None:
                free_start = i * 0.01
            elif d <= INFLATE_M and free_start is not None:
                out.append(i * 0.01 - free_start)
                break
        else:
            out.append(0.0 if free_start is None else 20.0 - free_start)
    return out[0], out[1]        # (左侧净宽, 右侧净宽)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--route', required=True, help='世界坐标航点："x,y x,y ..."')
    ap.add_argument('--layout', default=DEFAULT_LAYOUT)
    ap.add_argument('--direct', action='store_true',
                    help='这条航线打算**直飞**（不避障）。余量不足就以非零码退出。')
    ap.add_argument('--margin', type=float, default=1.0,
                    help='直飞要求的最小余量（米），默认 1.0')
    args = ap.parse_args()

    obs, d = load_obstacles(args.layout)
    pts = [tuple(float(v) for v in w.split(',')) for w in args.route.split()]
    if len(pts) < 2:
        print('航线至少要两个点', file=sys.stderr)
        return 2

    print(f'布局 {os.path.basename(args.layout)}：房间 {d["room"]["size_x"]}x'
          f'{d["room"]["size_y"]}，障碍 {len(obs)} 类（{"、".join(n for n, _ in obs)}）')
    print(f'判据：直飞要求余量 ≥ {args.margin:.1f} m；净宽按膨胀 {INFLATE_M} m 两边各扣\n')
    bad = 0
    for a, b in zip(pts, pts[1:]):
        worst, who, t_at = 1e9, '', 0.0
        for i in range(SAMPLES):
            t = i / (SAMPLES - 1)
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            v, nm = clearance(p, obs)
            if v < worst:
                worst, who, t_at = v, nm, t
        leg = math.dist(a, b)
        print(f'({a[0]:g},{a[1]:g}) -> ({b[0]:g},{b[1]:g})   长 {leg:5.1f} m')
        verdict = '✅ 可直飞' if worst >= args.margin else (
            '⛔ 直线穿过障碍' if worst < 0 else '⚠ 余量不足，不可直飞')
        print(f'    直线最小余量 {worst:6.2f} m  （{who}）   {verdict}')
        if worst <= INFLATE_M:
            l, r = corridor(a, b, obs, t_at)
            tight = min(l, r)
            note = '  ⚠ 窄的那侧不足 1.5 m，规划器选中它就有擦碰风险' if tight < 1.5 else ''
            print(f'    规划器需绕行：两侧净宽 {l:.2f} m / {r:.2f} m{note}')
        if worst < args.margin:
            bad += 1
        print()
    if args.direct:
        if bad:
            print(f'❌ {bad} 段不满足直飞要求，**不要开 direct**')
            return 1
        print('✅ 全部航段满足直飞要求')
    else:
        print('提示：这是按"直飞"口径算的余量。走规划器时直线被挡是正常的，'
              '它会绕——要看的是上面那行"两侧净宽"。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

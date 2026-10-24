#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画「大赛样题场景俯视示意图」——坐标全部读自布局 yaml 与选手程序 venue.py。

    ./scripts/draw_sample_scene.py                 # 覆盖项目根与 docker_sim 下那两张同名 png
    ./scripts/draw_sample_scene.py --out /tmp/x.png --no-sync

为什么要有这个脚本：上一版示意图（2026-10-12）的生成脚本后来被删了，只剩
scripts/__pycache__/画样题场景示意图.cpython-310.pyc 这个 .pyc，图改不动也
对不上新场景。这份重写的放回 scripts/ 里，改完 yaml / venue.py 重跑一次即可。

⚠️ 这张图是**按 yaml 画的**，不是从 Gazebo 读的真值。改完 yaml 还得重新生成
world（docker/scripts/gen_sample_room_world.py）并重建 sim-world 镜像；图上
对不等于仿真里对，真值看 scenario_reset_node 启动时打印的那几行。

⚠️ 图上楼号是**仿真编号**。真机 contestant_real/venue.py 仍用旧编号（1# 与 3#
互换），所以同一栋楼在两边叫法不同，图上已标注，别只看一个号就去改坐标。
"""
import argparse
import os
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Wedge, FancyArrow
from matplotlib.lines import Line2D
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT = os.path.join(HERE, 'src/contest_mission/config/sample_room_layout.yaml')

# 孪生柱方向 -> 单位向量，跟 gen_sample_room_world.py / scenario_reset_node 一致。
TWIN_UNIT = {'+x': (1.0, 0.0), '-x': (-1.0, 0.0), '+y': (0.0, 1.0), '-y': (0.0, -1.0)}
# 仿真编号 -> 真机（旧）编号。2026-10-11 仿真把 1#/3# 互换了，真机那份 venue.py
# 没跟着换，两套编号至今并存（见记忆 project_building_numbering_two_systems）。
REAL_NO = {'pillar_1': '3#', 'pillar_2': '2#', 'pillar_3': '1#'}

C_ROOM, C_ROUTE, C_PATROL = '#333333', '#2f9e44', '#e8590c'
C_PILLAR, C_FIRE, C_GFIRE = '#8c6d52', '#d6336c', '#f06595'
C_SUPPLY, C_TERRAIN, C_PAD, C_CAM = '#7048e8', '#e8b000', '#1864ab', '#0b7285'

HFOV_DEG = 80.0           # 仿真前视相机水平视场（真机 82°，画图取仿真值）


def pick_cjk_font():
    """挑一个系统里确实有的中文字体，挑不到就随它去（图还能出，中文会变方块）。"""
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'WenQuanYi Zen Hei',
                 'WenQuanYi Micro Hei', 'Droid Sans Fallback', 'AR PL UMing CN'):
        if name in have:
            return name
    return None


def twin_of(p):
    """本柱 + 孪生柱的中心坐标；旧键 twin_east 等价 +x。"""
    d = str(p.get('twin_dir') or ('+x' if p.get('twin_east') else '')).lower()
    u = TWIN_UNIT.get(d)
    return (d, None) if u is None else (d, (p['x'] + u[0], p['y'] + u[1]))


def cam_wedge(ax, x, y, yaw_deg, reach, label=None):
    """观察位的相机视场扇形：半张角 = 水平视场一半。"""
    ax.add_patch(Wedge((x, y), reach, yaw_deg - HFOV_DEG / 2, yaw_deg + HFOV_DEG / 2,
                       facecolor=C_CAM, alpha=0.10, edgecolor=C_CAM,
                       linewidth=0.8, linestyle=':', zorder=1))


def draw(L, out_png):
    rx, ry = L['room']['size_x'], L['room']['size_y']
    size = L['pillar_size']
    hw = size / 2.0
    wp = {w['id']: (w['x'], w['y']) for w in L['route_waypoints']}

    fig, ax = plt.subplots(figsize=(11.0, 13.2))
    ax.set_xlim(-2.4, 21.8)
    ax.set_ylim(-3.4, 26.8)
    ax.set_aspect('equal')
    ax.set_xticks(range(0, int(rx) + 1, 2))
    ax.set_yticks(range(0, int(ry) + 1, 2))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(True, color='#eeeeee', linewidth=0.6, zorder=0)
    ax.tick_params(labelsize=9)

    # ---- 房间 ----
    ax.add_patch(Rectangle((0, 0), rx, ry, fill=False, edgecolor=C_ROOM,
                           linewidth=3.0, zorder=3))
    ax.plot(0, 0, 'o', color='#000000', ms=6, zorder=5)
    ax.text(0.25, -0.95, '原点 (0,0)：房间西南角', fontsize=9.5, fontweight='bold')

    # ---- 编队航线（第 1 轮，A-B-C-G-E-F-G-D）----
    order = [w['id'] for w in L['route_waypoints']]
    rpts = [(w['x'], w['y']) for w in L['route_waypoints']]
    ax.plot([p[0] for p in rpts], [p[1] for p in rpts], '--', color=C_ROUTE,
            linewidth=2.2, dashes=(6, 4), zorder=4)
    for wid, (x, y) in zip(order, rpts):
        ax.plot(x, y, 'o', color=C_ROUTE, ms=7, zorder=6)
    for wid, dx, dy, ha in (('A', -0.5, -0.9, 'right'), ('B', -0.5, 0.5, 'right'),
                            ('C', 0.5, 0.5, 'left'), ('D', 0.5, -0.9, 'left'),
                            ('G', 0.5, 0.25, 'left'), ('F', -0.4, 0.45, 'right')):
        x, y = wp[wid]
        ax.text(x + dx, y + dy, f'航点{wid} ({x:g},{y:g})', fontsize=9.5,
                color='#2b8a3e', ha=ha, fontweight='bold')
    ax.text(wp['F'][0] - 0.4, wp['F'][1] + 0.05, '（虚拟点，无实体）', fontsize=8.2,
            color='#2b8a3e', ha='right')

    # ---- 立柱（楼）----
    for p in L['pillars']:
        d, twin = twin_of(p)
        cells = [(p['x'], p['y'])] + ([twin] if twin else [])
        for cx, cy in cells:
            ax.add_patch(Rectangle((cx - hw, cy - hw), size, size, facecolor=C_PILLAR,
                                   edgecolor='#5c3d22', linewidth=1.2, zorder=5))
        if twin:   # 单元分隔线压在两柱贴合的那条缝上
            if d in ('+x', '-x'):
                seam_x = (p['x'] + twin[0]) / 2.0
                ax.plot([seam_x, seam_x], [p['y'] - hw, p['y'] + hw], '-',
                        color='#3b2616', linewidth=1.4, zorder=6)
            else:
                seam_y = (p['y'] + twin[1]) / 2.0
                ax.plot([p['x'] - hw, p['x'] + hw], [seam_y, seam_y], '-',
                        color='#3b2616', linewidth=1.4, zorder=6)
        xs = [c[0] for c in cells]
        ys = [c[1] for c in cells]
        no = p['id'].replace('pillar_', '') + '#'
        ax.text(sum(xs) / len(xs), max(ys) + hw + 0.28,
                f'{no}楼（真机 {REAL_NO.get(p["id"], "?")}）  孪生柱在{"正东" if d == "+x" else "正南"}',
                fontsize=9.8, color='#5c3d22', ha='center', fontweight='bold')
        ax.text(sum(xs) / len(xs), max(ys) + hw + 0.9,
                '  '.join(f'({c[0]:g},{c[1]:g})' for c in cells),
                fontsize=8.6, color='#7f5539', ha='center')

    # ---- 高层火情候选窗口：参与随机的楼 × 2 层 × 2 单元，全贴 -y 面 ----
    rnd = L['fire_apriltag_random']
    heights = [f - L['floor_rings']['spacing_m'] / 2.0 for f in rnd['floors']]
    pmap = {p['id']: p for p in L['pillars']}
    for pid in rnd['buildings']:
        p = pmap[pid]
        _, twin = twin_of(p)
        for cx, _cy in [(p['x'], p['y'])] + ([twin] if twin else []):
            ax.plot(cx, p['y'] - hw - 0.08, 's', mfc='none', mec=C_FIRE, mew=1.8,
                    ms=9, zorder=7)
    ax.text(5.1, 18.5, f'2# 高层火情候选 4 处\n2 层 × 2 单元，贴 −y 面\n'
                       f'h={"/".join(f"{h:g}" for h in heights)} m',
            fontsize=8.8, color=C_FIRE, ha='right', va='top', linespacing=1.5)
    ax.text(15.0, 18.5, f'3# 高层火情候选 4 处\n2 层 × 2 单元，贴 −y 面\n'
                        f'h={"/".join(f"{h:g}" for h in heights)} m',
            fontsize=8.8, color=C_FIRE, ha='left', va='top', linespacing=1.5)

    # ---- 1# 楼：只拍照、不参与火情随机 ----
    p1 = pmap['pillar_1']
    ax.text(p1['x'] - hw - 0.45, p1['y'] - 0.5,
            '1# 楼不参与火情随机，只需一张交付照片。\n'
            '孪生柱在正南 ⇒ 东立面 2.0 m 宽、离 E 3.5 m，\n'
            'E 点机头 180° 一帧拍全两个单元。\n'
            '（火情贴 −y 面，若这栋入选会埋进南侧孪生柱）',
            fontsize=8.6, color='#7f5539', ha='right', va='center', linespacing=1.6,
            zorder=9, bbox=dict(boxstyle='round,pad=0.22', fc='white', ec='none', alpha=0.78))

    # ---- 任务 3 巡检顺序：G -> E -> M -> N ----
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'venue_sim', os.path.join(HERE, 'contestant_sim/venue.py'))
    venue = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(venue)
    stations = list(venue.STATIONS)

    pts = [wp['G']] + [s[1] for s in stations]
    ax.plot([p[0] for p in pts], [p[1] for p in pts], '-.', color=C_PATROL,
            linewidth=1.8, zorder=4)
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        dx, dy = x1 - x0, y1 - y0
        n = (dx ** 2 + dy ** 2) ** 0.5
        ax.add_patch(FancyArrow(mx, my, dx / n * 0.45, dy / n * 0.45, width=0.03,
                                head_width=0.42, head_length=0.45, color=C_PATROL,
                                zorder=6, length_includes_head=True))

    for i, (name, (x, y), pname, yaw, check) in enumerate(stations, start=1):
        reach = 4.2 if pname == 'E' else 3.8
        cam_wedge(ax, x, y, yaw, reach)
        ax.plot(x, y, '^', color=C_CAM, ms=11, zorder=7)
        ax.text(x, y + 0.42, f'{i}', fontsize=9, color='#ffffff', ha='center',
                va='center', fontweight='bold', zorder=8,
                bbox=dict(boxstyle='circle,pad=0.18', fc=C_PATROL, ec='none'))
        tail = '拍照 + 查火情' if check else '只拍照'
        txt = f'观察位 {pname} ({x:g},{y:g})\n机头 {yaw:g}° → {name}，{tail}'
        if pname == 'M':
            ax.text(x - 0.5, y - 0.4, txt, fontsize=9, color=C_CAM, ha='right',
                    va='top', linespacing=1.5, fontweight='bold', zorder=9)
        elif pname == 'N':
            ax.text(x + 0.6, y - 0.5, txt, fontsize=9, color=C_CAM, ha='left',
                    va='top', linespacing=1.5, fontweight='bold', zorder=9)
        else:   # E 站：标注挪到楼的南边空地，左右都是占着的（西边是 1# 楼、
                #        东边是同一条线上的地面火情区间），只有下面这块是空的
            ax.plot([x - 0.25, x - 1.3], [y - 0.35, y - 1.6], '-', color=C_CAM,
                    linewidth=0.8, zorder=8)
            ax.text(x - 1.5, y - 2.0, txt, fontsize=9, color=C_CAM, ha='center',
                    va='top', linespacing=1.5, fontweight='bold', zorder=9)

    # ---- 地面火情随机区间（G-E 段上）----
    gf = L['ground_fire_random']
    (ax0, ay0), (ax1, _ay1) = wp[gf['between'][0]], wp[gf['between'][1]]
    m0 = gf.get('margin_start_m', gf['margin_m'])
    m1 = gf['margin_m']
    lo, hi = sorted((ax0 + (-m0 if ax0 > ax1 else m0), ax1 + (m1 if ax0 > ax1 else -m1)))
    ax.plot([lo, hi], [ay0, ay0], '-', color=C_GFIRE, linewidth=7, alpha=0.55, zorder=3)
    ax.text((lo + hi) / 2.0 + 1.1, ay0 - 0.75,
            f'地面火情随机区间 AprilTag ID2\nx∈[{lo:g},{hi:g}]（距 G ≥{m0:g} m，距 E ≥{m1:g} m）',
            fontsize=8.8, color='#c2255c', ha='center', va='top', linespacing=1.5)

    # ---- 物资点 ----
    sp = L['supply_point']
    ax.add_patch(Rectangle((sp['x'] - sp['size_x'] / 2, sp['y'] - sp['size_y'] / 2),
                           sp['size_x'], sp['size_y'], facecolor=C_SUPPLY, alpha=0.85,
                           edgecolor='#4a2fa8', linewidth=1.2, zorder=5))
    ax.text(sp['x'], sp['y'] + sp['size_y'] / 2 + 0.3,
            f'物资点 ({sp["x"]:g},{sp["y"]:g}) AprilTag ID{sp["apriltag_id"]}　'
            f'{sp["size_x"]:g}×{sp["size_y"]:g} m（坐标为中心）',
            fontsize=9, color=C_SUPPLY, ha='center', fontweight='bold')

    # ---- 仿地模块（梯形坡：外框=下底，内框=平顶）----
    t = L['terrain_module']
    ax.add_patch(Rectangle((t['x'] - t['size_x'] / 2, t['y'] - t['size_y'] / 2),
                           t['size_x'], t['size_y'], facecolor=C_TERRAIN, alpha=0.35,
                           edgecolor='#a07800', linewidth=1.2, zorder=4))
    ax.add_patch(Rectangle((t['x'] - t['size_x'] / 2, t['y'] - t['top_width_y'] / 2),
                           t['size_x'], t['top_width_y'], facecolor=C_TERRAIN,
                           edgecolor='#7a5c00', linewidth=1.0, zorder=5))
    ax.text(t['x'] - t['size_x'] / 2 - 0.4, t['y'],
            f'仿地模块 ({t["x"]:g},{t["y"]:g})\n'
            f'{t["size_x"]:g} m(X)×下底 {t["size_y"]:g} m×高 {t["size_z"]:g} m\n'
            f'上底 {t["top_width_y"]:g} m，航线 G→D 正中穿过',
            fontsize=8.8, color='#8a6d00', ha='right', va='center', linespacing=1.5)

    # ---- 起降点 ----
    for pad, who in zip(L['takeoff_landing_pads'], ('侦察机', '任务机')):
        ax.add_patch(Rectangle((pad['x'] - 0.25, pad['y'] - 0.25), 0.5, 0.5,
                               fill=False, edgecolor=C_PAD, linewidth=2.0, zorder=6))
        ax.text(pad['x'], pad['y'], 'H', fontsize=8.5, color=C_PAD, ha='center',
                va='center', fontweight='bold', zorder=7)
        ax.text(pad['x'], pad['y'] - 0.55,
                f'{pad["id"].upper()} 起降点（{who}）\n({pad["x"]:g},{pad["y"]:g})',
                fontsize=8.8, color=C_PAD, ha='center', va='top', linespacing=1.4)

    # ---- 编号说明（两套编号并存，图上最容易看错的一点）----
    ax.text(10.0, 24.4,
            '楼号为**仿真**编号；真机 contestant_real/venue.py 仍用旧编号：1# ↔ 3# 互换、2# 不变，\n'
            '所以同一趟巡检在真机那边写作 3#→2#→1#。火情随机始终跟物理楼走（北墙那两栋）。'
            .replace('**', ''),
            fontsize=9, color='#444444', ha='center', va='center', linespacing=1.6,
            bbox=dict(boxstyle='round,pad=0.5', fc='#fbfbf5', ec='#cccccc'))

    handles = [
        Line2D([], [], color=C_ROUTE, ls='--', lw=2.2, label='第 1 轮编队航线 A-B-C-G-E-F-G-D'),
        Line2D([], [], color=C_PATROL, ls='-.', lw=1.8, label='任务 3 巡检顺序 G→E→M→N'),
        Line2D([], [], color=C_CAM, marker='^', ls='', ms=10, label='观察位（扇形=相机水平视场 80°）'),
        Line2D([], [], color=C_FIRE, marker='s', mfc='none', mew=1.8, ls='', ms=9,
               label='高层火情候选窗口（每轮随机取 1）'),
        Line2D([], [], color=C_GFIRE, lw=7, alpha=0.55, label='地面火情随机区间（每轮随机取 1）'),
        Line2D([], [], color=C_PILLAR, marker='s', ls='', ms=10, label='立柱（楼：本柱+孪生柱，各 1×1 m，高 6 m）'),
        Line2D([], [], color=C_SUPPLY, marker='s', ls='', ms=10, label='物资点'),
        Line2D([], [], color=C_TERRAIN, marker='s', ls='', ms=10, label='仿地模块'),
        Line2D([], [], color=C_PAD, marker='s', mfc='none', mew=2, ls='', ms=10, label='起降点'),
    ]
    ax.legend(handles=handles, loc='lower center', bbox_to_anchor=(0.5, -0.005),
              ncol=3, fontsize=8.6, framealpha=0.95, borderpad=0.7,
              columnspacing=1.2, handlelength=2.4)

    ax.set_title('大赛样题场景俯视示意图（2026-10-19 版：1# 楼孪生柱移至正南，1# 改在 E 点拍）',
                 fontsize=13.5, fontweight='bold', pad=14)
    ax.text(0.0, -1.35,
            f'房间 {rx:g}×{ry:g}×{L["room"]["height"]:g} m　坐标读自 '
            f'src/contest_mission/config/sample_room_layout.yaml + contestant_sim/venue.py',
            fontsize=8.2, color='#888888', ha='left')

    fig.tight_layout()
    fig.savefig(out_png, dpi=130, facecolor='white')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description='按布局 yaml 画样题场景俯视示意图')
    ap.add_argument('--layout', default=LAYOUT)
    ap.add_argument('--out', default=os.path.join(HERE, '样题场景示意图.png'))
    ap.add_argument('--no-sync', action='store_true',
                    help='不要把图同步到项目根目录那份同名副本')
    args = ap.parse_args()

    font = pick_cjk_font()
    if font:
        plt.rcParams['font.sans-serif'] = [font]
    else:
        print('!! 系统里没找到中文字体，图上中文会变方块')
    plt.rcParams['axes.unicode_minus'] = False

    with open(args.layout, encoding='utf-8') as f:
        layout = yaml.safe_load(f)
    draw(layout, args.out)
    print(f'已生成 {args.out}')

    # 项目根目录历来放一份同名副本（给人直接看，不用进 docker_sim）
    twin = os.path.join(os.path.dirname(HERE), os.path.basename(args.out))
    if not args.no_sync and os.path.abspath(twin) != os.path.abspath(args.out):
        shutil.copy2(args.out, twin)
        print(f'已同步 {twin}')


if __name__ == '__main__':
    main()

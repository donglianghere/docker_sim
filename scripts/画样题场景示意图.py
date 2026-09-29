#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 sample_room_layout.yaml 画样题场景俯视示意图（PNG）。

跟 docker/scripts/gen_sample_room_world.py 读的是**同一份 yaml**，所以示意图
和 Gazebo 世界不会各说各话——改布局只改 yaml，world 和这张图各自重新生成。
（手画一张图然后靠人记住"改了坐标要同步改图"，是迟早对不上的做法。）

用法：
    python3 scripts/画样题场景示意图.py \\
        --layout src/contest_mission/config/sample_room_layout.yaml \\
        --out 样题场景示意图.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import yaml

matplotlib.rcParams["font.sans-serif"] = [
    "WenQuanYi Zen Hei", "Noto Sans CJK SC", "Noto Sans CJK JP",
    "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

C_WALL = "#444444"
C_PILLAR = "#B78C5C"      # 浅褐色，跟世界里立柱的 RGBA (0.72,0.55,0.36) 对应
C_CYL = "#1f77b4"
C_TERRAIN = "#e8c31e"
C_ROUTE = "#2ca02c"
C_PAD = "#333333"
C_FIRE = "#d62728"
C_SUPPLY = "#7f4fbf"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--layout", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    L = yaml.safe_load(open(args.layout, encoding="utf-8"))

    R = L["room"]
    sx, sy = R["size_x"], R["size_y"]
    fig, ax = plt.subplots(figsize=(9.5, 11.5))

    # ---- 房间 ----
    ax.add_patch(Rectangle((0, 0), sx, sy, fill=False, ec=C_WALL, lw=3))
    ax.text(sx / 2, -1.1, f"房间 {sx:.0f} × {sy:.0f} m，墙高 {R['height']:.0f} m"
                          f"（天花板 z={R['height']:.0f}，隐形）",
            ha="center", va="top", fontsize=10, color=C_WALL)
    ax.plot(0, 0, marker="o", ms=7, color="k", zorder=5)
    ax.annotate("原点 (0,0)\n房间西南角", (0, 0), textcoords="offset points",
                xytext=(8, 8), fontsize=9, fontweight="bold")

    # ---- 航线 ----
    wps = L["route_waypoints"]
    xs = [w["x"] for w in wps] + [wps[0]["x"]]
    ys = [w["y"] for w in wps] + [wps[0]["y"]]
    ax.plot(xs, ys, "--", color=C_ROUTE, lw=2, zorder=2, label="巡检航线")
    for w in wps:
        ax.plot(w["x"], w["y"], "o", color=C_ROUTE, ms=9, zorder=6)
        ax.annotate(f"航点{w['id']}\n({w['x']:.0f},{w['y']:.0f})",
                    (w["x"], w["y"]), textcoords="offset points", xytext=(9, 6),
                    fontsize=9, color=C_ROUTE, fontweight="bold")

    # ---- 立柱（含正东孪生柱）----
    S = L["pillar_size"]
    for p in L["pillars"]:
        cols = [(p["id"], p["x"])] + ([(p["id"] + "_e", p["x"] + S)]
                                      if p.get("twin_east") else [])
        for pid, px in cols:
            ax.add_patch(Rectangle((px - S / 2, p["y"] - S / 2), S, S,
                                   fc=C_PILLAR, ec="k", lw=0.8, zorder=4))
        # 标注挂在本柱上，注明它是"一对"
        ax.annotate(f"{p['id'].replace('pillar_', '')}#立柱 ({p['x']:.1f},{p['y']:.1f})\n"
                    f"+正东孪生柱，合 {2*S:.0f}×{S:.0f} m",
                    (p["x"] - S / 2, p["y"] + S / 2), textcoords="offset points",
                    xytext=(-6, 8), fontsize=8.5, color=C_PILLAR,
                    fontweight="bold", ha="right")

    # ---- 高层着火点（贴在某根柱子的某个面）----
    fam = L["fire_apriltag_marker"]
    fp = next(q for q in L["pillars"] if q["id"] == fam["pillar_id"])
    d = fam["mount_standoff_m"]
    off = {"-y": (0, -d), "+y": (0, d), "-x": (-d, 0), "+x": (d, 0)}[fam["face"].lower()]
    fx, fy = fp["x"] + off[0], fp["y"] + off[1]
    ax.plot(fx, fy, marker="s", ms=9, color=C_FIRE, zorder=7)
    ax.annotate(f"高层着火点 h={L['fire_apriltag_height_m']}m\n"
                f"贴 {fam['pillar_id'].replace('pillar_','')}# 的 {fam['face']} 面",
                (fx, fy), textcoords="offset points", xytext=(10, -22),
                fontsize=8.5, color=C_FIRE, fontweight="bold")

    # ---- 障碍圆柱 ----
    c = L["obstacle_cylinder"]
    ax.add_patch(Circle((c["x"], c["y"]), c["diameter"] / 2, fc=C_CYL, ec="k", zorder=4))
    ax.annotate(f"障碍圆柱 ({c['x']:.0f},{c['y']:.0f})\nφ{c['diameter']}m，压在航线上",
                (c["x"], c["y"]), textcoords="offset points", xytext=(-10, 10),
                fontsize=8.5, color=C_CYL, fontweight="bold", ha="right")

    # ---- 仿地模块（梯形，画下底轮廓+上底）----
    t = L["terrain_module"]
    x0, x1 = t["x"] - t["size_x"] / 2, t["x"] + t["size_x"] / 2
    yb0, yb1 = t["y"] - t["size_y"] / 2, t["y"] + t["size_y"] / 2
    yt0, yt1 = t["y"] - t["top_width_y"] / 2, t["y"] + t["top_width_y"] / 2
    ax.add_patch(Rectangle((x0, yb0), t["size_x"], t["size_y"],
                           fc=C_TERRAIN, ec="k", lw=0.8, alpha=0.5, zorder=3))
    ax.add_patch(Rectangle((x0, yt0), t["size_x"], yt1 - yt0,
                           fc=C_TERRAIN, ec="k", lw=0.8, zorder=4))
    ax.annotate(f"仿地模块 ({t['x']:.0f},{t['y']:.1f})\n"
                f"梯形坡 高{t['size_z']}m，航线穿过",
                (x1, t["y"]), textcoords="offset points", xytext=(-8, 16),
                fontsize=8.5, color="#a8860b", fontweight="bold", ha="right")

    # ---- 地面目标 ----
    for key, color, label in ((("supply_point"), C_SUPPLY, "物资点"),
                              (("ground_fire_point"), C_FIRE, "地面火情点")):
        g = L[key]
        ax.add_patch(Circle((g["x"], g["y"]), 0.4, fc=color, ec="k", alpha=0.85, zorder=5))
        ax.annotate(f"{label} ({g['x']:.0f},{g['y']:.0f})\nAprilTag ID{g['apriltag_id']}",
                    (g["x"], g["y"]), textcoords="offset points", xytext=(10, 6),
                    fontsize=8.5, color=color, fontweight="bold")

    # ---- 起降点 ----
    for pad in L["takeoff_landing_pads"]:
        ax.add_patch(Rectangle((pad["x"] - 0.25, pad["y"] - 0.25), 0.5, 0.5,
                               fc="w", ec=C_PAD, lw=1.6, zorder=6))
        ax.text(pad["x"], pad["y"], "H", ha="center", va="center",
                fontsize=8, fontweight="bold", zorder=7)
        ax.annotate(f"{pad['id'].upper()} 起降点\n({pad['x']:.0f},{pad['y']:.0f})",
                    (pad["x"], pad["y"]), textcoords="offset points", xytext=(6, -26),
                    fontsize=8.5, color=C_PAD, fontweight="bold")

    ax.set_xlim(-3.5, sx + 3.5)
    ax.set_ylim(-3.0, sy + 2.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)  →东")
    ax.set_ylabel("y (m)  →北")
    ax.set_xticks(range(0, int(sx) + 1, 2))
    ax.set_yticks(range(0, int(sy) + 1, 2))
    ax.grid(alpha=0.25, ls=":")
    ax.set_title("大赛样题场景俯视示意图（原点=房间西南角）", fontsize=13, fontweight="bold")
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"[画样题场景示意图] {args.out}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把场地布置画成一张 SVG 示意图，坐标直接读布局 yaml（单一权威来源）。

    ./scripts/draw_layout.py                      # 输出 SVG
    ./scripts/draw_layout.py --jpeg               # 同时输出同名 .jpg
    ./scripts/draw_layout.py --out /tmp/x.svg --route "5,-9.5 5,9.5 -7,9.5 -7,-9.5"

改完 src/contest_mission/config/fire_drill_room_layout.yaml 之后重跑一次就能
看到新布局，不用手工画、也不会跟 yaml 对不上。编队航线不在 yaml 里（那是选手
程序的参数，不是场地道具），用 --route 传，默认值跟《双机全流程示例.py》一致。

⚠️ 这张图是**按 yaml 画的**，不是从 Gazebo 读的真值。改完 yaml 还要同步
world 补丁并重建 sim-world 镜像，图上是对的不等于仿真里就是对的——真值要看
sim-world 容器启动日志里 scenario_reset_node 打印的那一行，或订阅
/plug/model_states_plug。
"""
import argparse
import math
import os
import shutil
import subprocess
import tempfile

import yaml

SC, MX, MY, LEG_W = 33.0, 78, 96, 300
INFLATE = 0.6          # 规划器障碍物膨胀半径（仿真值）
ORBIT_R = 3.0          # 高楼绕飞半径，跟《高楼火情示例.py》一致
LANE = 2.52            # 弓字行距，2.5m 高度 + 0.5m 标志 + 20% 重叠算出来的


def load_layout(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def draw(layout, route, out_path):
    room_x, room_y = layout['room']['size_x'] / 2, layout['room']['size_y'] / 2
    X0, X1, Y0, Y1 = -room_x, room_x, -room_y, room_y
    W = int((X1 - X0) * SC) + 2 * MX + LEG_W
    H = int((Y1 - Y0) * SC) + 2 * MY

    def px(x):
        return MX + (x - X0) * SC

    def py(y):
        return MY + (Y1 - y) * SC

    pads = layout['takeoff_landing_pads']
    pillars = layout['pillars']
    side = layout['pillar_size']
    keepout = side / 2 * math.sqrt(2) + INFLATE
    fire = layout['ground_fire_point']
    supply = layout['supply_point']
    cyl = layout['obstacle_cylinder']
    terr = layout['terrain_module']

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" font-family="sans-serif">',
         f'<rect width="{W}" height="{H}" fill="#fff"/>']
    for x in range(int(X0), int(X1) + 1):
        o.append(f'<line x1="{px(x):.1f}" y1="{py(Y0):.1f}" x2="{px(x):.1f}" y2="{py(Y1):.1f}" stroke="#f0f0f0"/>')
    for y in range(int(Y0), int(Y1) + 1):
        o.append(f'<line x1="{px(X0):.1f}" y1="{py(y):.1f}" x2="{px(X1):.1f}" y2="{py(y):.1f}" stroke="#f0f0f0"/>')
    for x in range(int(X0), int(X1) + 1, 5):
        o.append(f'<text x="{px(x):.1f}" y="{py(Y0)+24:.1f}" font-size="13" fill="#888" text-anchor="middle">x={x}</text>')
    for y in range(int(Y0) + 2, int(Y1), 5):
        o.append(f'<text x="{px(X0)-12:.1f}" y="{py(y)+5:.1f}" font-size="13" fill="#888" text-anchor="end">y={y}</text>')
    o.append(f'<rect x="{px(X0):.1f}" y="{py(Y1):.1f}" width="{(X1-X0)*SC:.1f}" height="{(Y1-Y0)*SC:.1f}" '
             f'fill="none" stroke="#444" stroke-width="5"/>')

    # 弓字搜索行（按航线围成的矩形推算，只为看覆盖关系）
    ys = [p[1] for p in route]
    xs = [p[0] for p in route]
    lanes, y = [], min(ys) + LANE / 2
    while y < max(ys):
        lanes.append(y)
        y += LANE
    for yy in lanes:
        o.append(f'<line x1="{px(min(xs)):.1f}" y1="{py(yy):.1f}" x2="{px(max(xs)):.1f}" y2="{py(yy):.1f}" '
                 f'stroke="#a5c9ff" stroke-width="1.4" stroke-dasharray="7 6"/>')

    pts = ' '.join(f'{px(x):.1f},{py(y):.1f}' for x, y in route)
    o.append(f'<polyline points="{pts}" fill="none" stroke="#2f9e44" stroke-width="3.5"/>')
    o.append(f'<line x1="{px(route[-1][0]):.1f}" y1="{py(route[-1][1]):.1f}" '
             f'x2="{px(pads[0]["x"]):.1f}" y2="{py(pads[0]["y"]):.1f}" stroke="#2f9e44" '
             f'stroke-width="2" stroke-dasharray="9 7"/>')
    for i, (x, y) in enumerate(route, start=1):
        o.append(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="7" fill="#2f9e44"/>')
        dx, anchor = (14, 'start') if x < 0 else (-14, 'end')
        o.append(f'<text x="{px(x)+dx:.1f}" y="{py(y)+(20 if y<0 else -12):.1f}" font-size="13" '
                 f'fill="#2b8a3e" text-anchor="{anchor}">航点{i} ({x:g}, {y:g})</text>')

    # 仿地模块：实心矩形=下底轮廓，内部虚线框=平顶（两者之间就是两侧的坡）
    o.append(f'<rect x="{px(terr["x"]-terr["size_x"]/2):.1f}" y="{py(terr["y"]+terr["size_y"]/2):.1f}" '
             f'width="{terr["size_x"]*SC:.1f}" height="{terr["size_y"]*SC:.1f}" fill="#b08968" stroke="#7f5539" stroke-width="2"/>')
    top_w = terr.get('top_width_y', terr['size_y'])
    o.append(f'<rect x="{px(terr["x"]-terr["size_x"]/2):.1f}" y="{py(terr["y"]+top_w/2):.1f}" '
             f'width="{terr["size_x"]*SC:.1f}" height="{top_w*SC:.1f}" fill="#8c6d52" stroke="#5c3d22" '
             f'stroke-width="1.5" stroke-dasharray="5 4"/>')
    o.append(f'<text x="{px(terr["x"]-terr["size_x"]/2)-10:.1f}" y="{py(terr["y"])+5:.1f}" font-size="13" '
             f'fill="#7f5539" text-anchor="end">仿地模块 ({terr["x"]:g}, {terr["y"]:g}) 长{terr["size_x"]:g}×'
             f'下底{terr["size_y"]:g}×高{terr["size_z"]:g} m</text>')
    o.append(f'<text x="{px(terr["x"]+terr["size_x"]/2)+8:.1f}" y="{py(terr["y"])+5:.1f}" font-size="11.5" '
             f'fill="#7f5539">梯形：上底{top_w:g} m，两侧45°坡</text>')

    r = cyl['diameter'] / 2
    o.append(f'<circle cx="{px(cyl["x"]):.1f}" cy="{py(cyl["y"]):.1f}" r="{(r+INFLATE)*SC:.1f}" fill="#fff4e6" '
             f'stroke="#fd7e14" stroke-width="1.5" stroke-dasharray="5 4"/>')
    o.append(f'<circle cx="{px(cyl["x"]):.1f}" cy="{py(cyl["y"]):.1f}" r="{r*SC:.1f}" fill="#fd7e14"/>')
    o.append(f'<text x="{px(cyl["x"])-30:.1f}" y="{py(cyl["y"])+5:.1f}" font-size="13" fill="#d9480f" '
             f'text-anchor="end">障碍圆柱 ({cyl["x"]:g}, {cyl["y"]:g}) r={r:g}</text>')

    for p in pillars:
        x, y, name = p['x'], p['y'], p['id'].replace('pillar_', '') + '#'
        o.append(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="{ORBIT_R*SC:.1f}" fill="none" stroke="#c77dff" '
                 f'stroke-width="1.5" stroke-dasharray="8 7"/>')
        o.append(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="{keepout*SC:.1f}" fill="#f3e8ff" fill-opacity="0.65" '
                 f'stroke="#9775fa" stroke-width="1.2"/>')
        o.append(f'<rect x="{px(x-side/2):.1f}" y="{py(y+side/2):.1f}" width="{side*SC:.1f}" height="{side*SC:.1f}" fill="#6f42c1"/>')
        o.append(f'<text x="{px(x):.1f}" y="{py(y)-keepout*SC-8:.1f}" font-size="14" fill="#5f3dc4" '
                 f'text-anchor="middle" font-weight="bold">{name}立柱 ({x:g}, {y:g})</text>')
    o.append(f'<text x="{px(pillars[-1]["x"]):.1f}" y="{py(pillars[-1]["y"]-ORBIT_R)+18:.1f}" font-size="11.5" '
             f'fill="#9775fa" text-anchor="middle">绕飞圈 r={ORBIT_R:g} m　禁入圈 r={keepout:.2f} m'
             f'（半对角{side/2*math.sqrt(2):.2f}+膨胀{INFLATE:g}）</text>')

    tag = layout['fire_apriltag_marker']
    o.append(f'<text x="{px(tag["default_x"]):.1f}" y="{py(tag["default_y"])-keepout*SC-26:.1f}" font-size="11.5" '
             f'fill="#e8590c" text-anchor="middle">高层着火点贴在 2# 朝向 1# 的那一面，'
             f'{layout["fire_apriltag_height_m"]:g} m 高</text>')

    def marker(x, y, color, label, size=0.5):
        o.append(f'<rect x="{px(x-size/2):.1f}" y="{py(y+size/2):.1f}" width="{size*SC:.1f}" height="{size*SC:.1f}" fill="{color}"/>')
        o.append(f'<text x="{px(x)+14:.1f}" y="{py(y)+5:.1f}" font-size="13.5" fill="{color}" font-weight="bold">{label}</text>')
    marker(fire['x'], fire['y'], '#e03131', f'地面着火点 ({fire["x"]:g}, {fire["y"]:g})')
    marker(supply['x'], supply['y'], '#7048e8', f'物资点 ({supply["x"]:g}, {supply["y"]:g})')
    for p, color, who in zip(pads, ('#0b7285', '#1864ab'), ('侦察机', '任务机')):
        o.append(f'<rect x="{px(p["x"]-0.25):.1f}" y="{py(p["y"]+0.25):.1f}" width="{0.5*SC:.1f}" height="{0.5*SC:.1f}" '
                 f'fill="none" stroke="{color}" stroke-width="3"/>')
        o.append(f'<text x="{px(p["x"]):.1f}" y="{py(p["y"])+30:.1f}" font-size="12.5" fill="{color}" '
                 f'text-anchor="middle" font-weight="bold">{p["id"].upper()} 起降点（{who}）</text>')
        o.append(f'<text x="{px(p["x"]):.1f}" y="{py(p["y"])+45:.1f}" font-size="12" fill="{color}" '
                 f'text-anchor="middle">({p["x"]:g}, {p["y"]:g})</text>')

    lx, ly = px(X1) + 40, MY + 10
    o.append(f'<text x="{lx}" y="{ly-24}" font-size="16" font-weight="bold" fill="#222">图例</text>')
    items = [('#2f9e44', '编队航线（虚线=返航）'), ('#a5c9ff', f'弓字搜索行（行距 {LANE:.2f} m，{len(lanes)} 行）'),
             ('#6f42c1', f'立柱 {side:g}×{side:g} m（淡紫=禁入圈，虚线=绕飞圈）'), ('#fd7e14', '障碍圆柱'),
             ('#b08968', '仿地模块'), ('#e03131', '地面着火点'), ('#7048e8', '物资点'), ('#0b7285', '起降点')]
    for i, (c, t) in enumerate(items):
        yy = ly + i * 26
        o.append(f'<rect x="{lx}" y="{yy-11}" width="16" height="14" fill="{c}"/>')
        o.append(f'<text x="{lx+24}" y="{yy}" font-size="12.5" fill="#333">{t}</text>')
    o.append(f'<text x="{lx}" y="{ly+len(items)*26+22}" font-size="12.5" fill="#555">'
             f'房间 {layout["room"]["size_x"]:g}×{layout["room"]["size_y"]:g} m，高 {layout["room"]["height"]:g} m</text>')
    o.append(f'<text x="{lx}" y="{ly+len(items)*26+42}" font-size="12.5" fill="#555">坐标读自 fire_drill_room_layout.yaml</text>')
    o.append(f'<text x="{MX}" y="{MY-34:.0f}" font-size="19" font-weight="bold" fill="#222">场地布置示意（世界坐标，单位米）</text>')
    o.append('</svg>')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(o))


def to_jpeg(svg_path, jpeg_path, width=1400, quality=92):
    """SVG -> JPEG。宿主机上没有 rsvg-convert/inkscape/ImageMagick，就用 Chrome
    无头模式截图成 PNG，再用 PIL 转 JPEG（Chrome 的 --screenshot 只出 PNG）。
    缺工具时不让整个脚本失败——SVG 本身已经写出来了，只提示一句。
    """
    chrome = next((c for c in ('google-chrome', 'chromium', 'chromium-browser')
                   if shutil.which(c)), None)
    if chrome is None:
        print('!! 没找到 chrome/chromium，跳过 JPEG（SVG 已生成）')
        return False
    try:
        from PIL import Image
    except ImportError:
        print('!! 没装 PIL（python3-pil），跳过 JPEG（SVG 已生成）')
        return False

    with tempfile.TemporaryDirectory() as tmp:
        png = os.path.join(tmp, 'layout.png')
        cmd = [chrome, '--headless', '--disable-gpu', '--hide-scrollbars',
               f'--screenshot={png}', f'--window-size={width},{int(width * 1.15)}',
               '--default-background-color=ffffffff', f'file://{os.path.abspath(svg_path)}']
        r = subprocess.run(cmd, capture_output=True, timeout=120)
        if not os.path.exists(png):
            print(f'!! Chrome 截图失败，跳过 JPEG：{r.stderr.decode()[-300:]}')
            return False
        img = Image.open(png)
        # 去掉四周的白边，只留图本身
        bbox = img.convert('RGB').point(lambda v: 0 if v > 250 else 255).convert('L').getbbox()
        if bbox:
            img = img.crop(bbox)
        img.convert('RGB').save(jpeg_path, 'JPEG', quality=quality)
    print(f'已生成 {jpeg_path}')
    return True


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description='按布局 yaml 画场地示意图')
    ap.add_argument('--layout', default=os.path.join(here, 'src/contest_mission/config/fire_drill_room_layout.yaml'))
    ap.add_argument('--route', default='7,-9.5 7,9.5 -7,9.5 -7,-9.5', help='编队航线 "x,y x,y ..."')
    ap.add_argument('--out', default=os.path.join(here, '场地布置示意.svg'))
    ap.add_argument('--jpeg', action='store_true', help='同时输出同名 .jpg')
    args = ap.parse_args()
    route = [tuple(float(v) for v in tok.split(',')) for tok in args.route.split()]
    draw(load_layout(args.layout), route, args.out)
    print(f'已生成 {args.out}')
    if args.jpeg:
        to_jpeg(args.out, os.path.splitext(args.out)[0] + '.jpg')


if __name__ == '__main__':
    main()

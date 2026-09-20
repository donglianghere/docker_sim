#!/usr/bin/env python3
"""生成起降点标识贴图——50厘米粗体黑色"H"字母+外包围黑色圆图案，白底。

对应2026-09-09用户对场景重新设计的要求："每架飞机的起降点为同一个
位置，用一个50cm的粗体黑色H字母加外包围黑色圆图案标识"（经典停机坪
"H"标志的样式）。用PIL纯几何绘制（矩形+圆环），不依赖系统字体渲染
"H"这个字——"H"本身就是2根竖杠+1根横杠，用矩形直接画比找字体+调
基线对齐更精确可控，也避免了这台机器/容器里字体缺失的风险。

两架飞机（NX01/NX02）用的是同一张贴图（用户原话"每架飞机的起降点为
同一个位置"里的"同一个"是指"起飞点=降落点合一"，不是"两架机共用
一个点"——两机各自还是有自己独立的起降点坐标，只是贴图样式相同，
不需要区分）。
"""
from pathlib import Path

from PIL import Image, ImageDraw

SIZE = 512
BG_COLOR = (255, 255, 255)
FG_COLOR = (0, 0, 0)


def generate_helipad_marker(size: int = SIZE) -> Image.Image:
    img = Image.new("RGB", (size, size), BG_COLOR)
    draw = ImageDraw.Draw(img)

    # 外包围黑色圆图案：圆环（不是实心圆盘），线宽跟"H"的笔画粗细呼应。
    ring_margin = size * 0.04
    ring_width = size * 0.05
    draw.ellipse(
        [ring_margin, ring_margin, size - ring_margin, size - ring_margin],
        outline=FG_COLOR, width=int(ring_width),
    )

    # 粗体"H"：两根竖杠+一根横杠，用矩形拼出来，比例参照常见停机坪H标志
    # （竖杠占大部分高度，横杠在竖直方向居中，笔画粗细跟圆环线宽同一
    # 量级，视觉上"粗体"）。
    stroke = size * 0.11
    v_height = size * 0.5
    v_top = (size - v_height) / 2
    v_bottom = v_top + v_height
    left_x = size * 0.28
    right_x = size * 0.72

    draw.rectangle([left_x - stroke / 2, v_top, left_x + stroke / 2, v_bottom], fill=FG_COLOR)
    draw.rectangle([right_x - stroke / 2, v_top, right_x + stroke / 2, v_bottom], fill=FG_COLOR)
    bar_y = size / 2
    draw.rectangle(
        [left_x - stroke / 2, bar_y - stroke / 2, right_x + stroke / 2, bar_y + stroke / 2],
        fill=FG_COLOR,
    )
    return img


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "patches"
    out_dir.mkdir(parents=True, exist_ok=True)
    img = generate_helipad_marker()
    out_path = out_dir / "mighty_contest_helipad_marker.png"
    img.save(out_path)
    print(f"[OK] {out_path} size={img.size}")


if __name__ == "__main__":
    import sys
    main()

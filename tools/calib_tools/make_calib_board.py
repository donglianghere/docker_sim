#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成棋盘格标定板（PDF/SVG/PNG），配合 calib_camera.py 使用

用法:
  python3 make_calib_board.py                          # A4横向, 9x6内角点, 20mm方格
  python3 make_calib_board.py --square 25 --paper A3
  python3 make_calib_board.py --board 11x8 --square 15

--board 给的是**内角点数**(列x行), 不是方块数: 9x6 内角点 = 10x7 个方块。
这个数要和 calib_camera.py 的 --board 完全一致。

为什么默认 9x6: 一维奇数一维偶数, 棋盘朝向唯一可判, 不会出现 180° 旋转歧义。

打印要求(最关键):
  1. 必须按 100% / 实际大小 打印, 不要"适合页面"/"缩放到边距", 否则方格尺寸全错。
  2. 打完用卡尺或钢尺量几个方格的实际边长, 把量到的值填给 calib_camera.py 的 --square。
     打印机有 1-2% 缩放是常事, 量一下比相信标称值可靠。
  3. 贴在硬纸板/亚克力板上, 必须平整。翘曲会直接污染畸变系数, 这是最常见的标定翻车原因。
"""
import argparse
import os

import cv2
import numpy as np

PAPERS = {"A4": (297.0, 210.0), "A3": (420.0, 297.0), "A5": (210.0, 148.0),
          "LETTER": (279.4, 215.9)}
MM2PT = 72.0 / 25.4


def layout(board, square, paper):
    """返回 (页面mm, 棋盘左上角mm, 方块数)。棋盘在页面居中。"""
    cols, rows = board
    nx, ny = cols + 1, rows + 1          # 方块数 = 内角点数 + 1
    bw, bh = nx * square, ny * square
    pw, ph = PAPERS[paper]
    if bw > pw or bh > ph:
        raise SystemExit(
            "棋盘 %.0fx%.0fmm 放不进 %s (%.0fx%.0fmm)\n"
            "  把 --square 调小, 或换 --paper A3" % (bw, bh, paper, pw, ph))
    quiet = min((pw - bw) / 2, (ph - bh) / 2)
    if quiet < square * 0.6:
        print("提示: 四周留白只有 %.1fmm (不足一格), 边缘角点可能不好检出；"
              "建议 --square 调小或换大纸" % quiet)
    return (pw, ph), ((pw - bw) / 2, (ph - bh) / 2), (nx, ny)


def black_squares(origin, square, counts):
    """左上角那格为黑, 返回所有黑格的 (x, y, w, h)，单位 mm，y 从页面顶部算。"""
    ox, oy = origin
    nx, ny = counts
    out = []
    for i in range(ny):
        for j in range(nx):
            if (i + j) % 2 == 0:
                out.append((ox + j * square, oy + i * square, square, square))
    return out


def write_pdf(path, page, rects, texts):
    """手写最小 PDF: 无第三方依赖, 尺寸精确到 pt, 不经过任何栅格化。"""
    pw, ph = page[0] * MM2PT, page[1] * MM2PT
    body = ["0 0 0 rg"]
    for x, y, w, h in rects:                      # PDF 原点在左下, y 要翻过来
        body.append("%.4f %.4f %.4f %.4f re f" % (
            x * MM2PT, ph - (y + h) * MM2PT, w * MM2PT, h * MM2PT))
    for x, y, sz, txt in texts:
        safe = txt.replace("\\", r"\\").replace("(", r"\(").replace(")", r"\)")
        body.append("BT /F1 %.2f Tf %.4f %.4f Td (%s) Tj ET" % (
            sz, x * MM2PT, ph - y * MM2PT, safe))
    content = "\n".join(body).encode("latin-1")

    objs = [
        b"<</Type/Catalog/Pages 2 0 R>>",
        b"<</Type/Pages/Kids[3 0 R]/Count 1>>",
        ("<</Type/Page/Parent 2 0 R/MediaBox[0 0 %.4f %.4f]"
         "/Contents 4 0 R/Resources<</Font<</F1 5 0 R>>>>>>" % (pw, ph)).encode(),
        b"<</Length " + str(len(content)).encode() + b">>\nstream\n" + content + b"\nendstream",
        b"<</Type/Font/Subtype/Type1/BaseFont/Helvetica>>",
    ]
    out = bytearray(b"%PDF-1.4\n")
    offs = []
    for i, o in enumerate(objs, 1):
        offs.append(len(out))
        out += ("%d 0 obj\n" % i).encode() + o + b"\nendobj\n"
    xref = len(out)
    out += ("xref\n0 %d\n" % (len(objs) + 1)).encode() + b"0000000000 65535 f \n"
    for off in offs:
        out += ("%010d 00000 n \n" % off).encode()
    out += ("trailer\n<</Size %d/Root 1 0 R>>\nstartxref\n%d\n%%%%EOF\n"
            % (len(objs) + 1, xref)).encode()
    with open(path, "wb") as f:
        f.write(bytes(out))


def write_svg(path, page, rects, label):
    pw, ph = page
    parts = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<svg xmlns="http://www.w3.org/2000/svg" width="%gmm" height="%gmm" '
             'viewBox="0 0 %g %g">' % (pw, ph, pw, ph),
             '<rect width="%g" height="%g" fill="#fff"/>' % (pw, ph)]
    for x, y, w, h in rects:
        parts.append('<rect x="%g" y="%g" width="%g" height="%g" fill="#000"/>' % (x, y, w, h))
    parts.append('<text x="6" y="%g" font-family="sans-serif" font-size="3.5" '
                 'fill="#000">%s</text>' % (ph - 4, label))
    parts.append("</svg>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def write_png(path, page, rects, label, dpi):
    pw, ph = page
    px = lambda mm: int(round(mm / 25.4 * dpi))
    img = np.full((px(ph), px(pw)), 255, np.uint8)
    for x, y, w, h in rects:
        img[px(y):px(y + h), px(x):px(x + w)] = 0
    cv2.putText(img, label, (px(6), px(ph) - px(4)), cv2.FONT_HERSHEY_SIMPLEX,
                dpi / 300.0 * 0.9, 0, max(1, int(dpi / 300.0 * 2)), cv2.LINE_AA)
    cv2.imwrite(path, img)
    return img


def main():
    ap = argparse.ArgumentParser(description="生成棋盘格标定板")
    ap.add_argument("--board", default="9x6", help="内角点数 列x行, 默认 9x6")
    ap.add_argument("--square", type=float, default=20.0, help="方格边长 mm, 默认 20")
    ap.add_argument("--paper", default="A4", choices=list(PAPERS))
    ap.add_argument("--dpi", type=int, default=300, help="PNG 分辨率, 默认 300")
    ap.add_argument("--out", default=os.path.expanduser("~/camera_calib/board"))
    a = ap.parse_args()

    try:
        cols, rows = (int(v) for v in a.board.lower().split("x"))
    except ValueError:
        raise SystemExit("--board 格式应是 列x行, 例如 9x6")
    board = (cols, rows)

    page, origin, counts = layout(board, a.square, a.paper)
    rects = black_squares(origin, a.square, counts)
    label = ("chessboard inner corners %dx%d  square %.1fmm  %s  PRINT AT 100%% "
             "- then MEASURE the square and pass it to --square"
             % (cols, rows, a.square, a.paper))

    os.makedirs(a.out, exist_ok=True)
    base = os.path.join(a.out, "chessboard_%dx%d_%gmm_%s" % (cols, rows, a.square, a.paper))
    write_pdf(base + ".pdf", page, rects, [(6, page[1] - 4, 8, label)])
    write_svg(base + ".svg", page, rects, label)
    img = write_png(base + ".png", page, rects, label, a.dpi)

    print("标定板已生成 (打印用 PDF, 屏幕看用 PNG):")
    for ext in ("pdf", "svg", "png"):
        p = base + "." + ext
        print("  %-5s %s  (%.0f KB)" % (ext, p, os.path.getsize(p) / 1024))
    print("规格: 内角点 %dx%d, 方块 %dx%d, 方格 %.1fmm, 棋盘 %.0fx%.0fmm, 纸张 %s %.0fx%.0fmm"
          % (cols, rows, counts[0], counts[1], a.square,
             counts[0] * a.square, counts[1] * a.square, a.paper, page[0], page[1]))
    print("四周留白: %.1fmm" % min(origin))
    print("\n标定时用: python3 calib_camera.py front --board %dx%d --square <实测边长>"
          % (cols, rows))

    # 自检: 用 calib_camera 的检测器回验这张图案, 内角点数必须正好对上
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from calib_camera import find_corners
    small = cv2.resize(img, (1280, int(round(1280 * img.shape[0] / img.shape[1]))),
                       interpolation=cv2.INTER_AREA)
    ok, corners = find_corners(small, board)
    print("\n自检(缩到1280宽再检测): %s, 检出角点 %d 个(应为 %d)"
          % ("检出" if ok else "未检出", 0 if corners is None else len(corners), cols * rows))
    return 0 if ok and corners is not None and len(corners) == cols * rows else 1


if __name__ == "__main__":
    raise SystemExit(main())

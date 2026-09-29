#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""合并多次标定采集的图, 重新标定成一份更可靠的内参

单次 20 张的 fx 统计波动约 2%, 合并到 60-100 张后折半差异降到 1% 以内。
每个目录用它自己 calib.json 里的 square(方格尺寸只改外参尺度, 不影响 fx),
所以不同 --square 采的图可以放心混在一起。

用法:
  python3 merge_calib.py '~/camera_calib/cam0_front_*'
  python3 merge_calib.py '~/camera_calib/cam0_front_20260927_17*' --fix-k3
"""
import argparse, glob, json, os, sys
import cv2, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calib_camera import (make_object_points, find_corners, board_tilt_deg,
                          calibrate, fx_stability, save_results, print_report,
                          save_pose_plot)

ap = argparse.ArgumentParser(description="合并多次采集重新标定")
ap.add_argument("patterns", nargs="+", help="结果目录(支持 glob), 引号括起来防 shell 展开")
ap.add_argument("--board", default="9x6")
ap.add_argument("--square", type=float, default=30.0, help="目录里没有 calib.json 时的兜底方格尺寸")
ap.add_argument("--model", choices=["pinhole", "fisheye"], default="pinhole")
ap.add_argument("--fix-k3", action="store_true", help="固定 k3=0, 让畸变系数可复现(对 fx 几乎无影响)")
ap.add_argument("--name", default=None, help="输出用的相机名, 默认取第一个目录的名字")
ap.add_argument("--out", default=None, help="输出目录, 默认 ~/camera_calib/<name>_merged")
a = ap.parse_args()

cols, rows = (int(v) for v in a.board.lower().split("x"))
board = (cols, rows)
dirs = []
for pat in a.patterns:
    dirs += sorted(glob.glob(os.path.expanduser(pat)))
dirs = [d for d in sorted(set(dirs)) if os.path.isdir(os.path.join(d, "images"))]
if not dirs:
    sys.exit("没找到任何含 images/ 的结果目录")

objpoints, imgpoints, tilts, size = [], [], [], None
for d in dirs:
    cj = os.path.join(d, "calib.json")
    sq = json.load(open(cj))["square_size_mm"] if os.path.exists(cj) else a.square
    objp = make_object_points(board, sq)
    n0 = len(imgpoints)
    for f in sorted(glob.glob(os.path.join(d, "images", "*.jpg"))):
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        if size is None:
            size = (g.shape[1], g.shape[0])
        elif (g.shape[1], g.shape[0]) != size:
            print("  尺寸不一致, 跳过", f)
            continue
        ok, c = find_corners(g, board)
        if ok:
            objpoints.append(objp.copy())
            imgpoints.append(c)
            tilts.append(board_tilt_deg(c, board, size[0], size[1]))
    print("  %-46s square=%-4.0f 收到 %d 张" % (os.path.basename(d), sq, len(imgpoints) - n0))

if len(imgpoints) < 5:
    sys.exit("有效图不足 5 张")
print("\n合计 %d 张, 图像尺寸 %dx%d, 开始标定..." % (len(imgpoints), size[0], size[1]))

# --fix-k3 需要直接调 calibrateCamera 带 flags, 其余走统一入口
if a.fix_k3 and a.model == "pinhole":
    rms, K, D, rv, tv = cv2.calibrateCamera(objpoints, imgpoints, size, None, None,
                                            flags=cv2.CALIB_FIX_K3)
    D = D.ravel()
    errs = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rv[i], tv[i], K, D)
        p = np.asarray(imgpoints[i], np.float64).reshape(-1, 2)
        q = np.asarray(proj, np.float64).reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum((p - q) ** 2, axis=1)))))
    rms = float(rms)
else:
    rms, K, D, errs = calibrate(objpoints, imgpoints, size, a.model)

stab = fx_stability(objpoints, imgpoints, size, a.model)
name = a.name or os.path.basename(dirs[0]).rsplit("_", 2)[0]
out = os.path.expanduser(a.out) if a.out else os.path.expanduser(
    "~/camera_calib/%s_merged%d" % (name, len(imgpoints)))
os.makedirs(out, exist_ok=True)
res = save_results(out, name, size, K, D, rms, errs, a.model, board,
                   a.square, len(imgpoints), tilts=tilts, stability=stab)
res["merged_from"] = dirs
json.dump(res, open(os.path.join(out, "calib.json"), "w"), indent=2, ensure_ascii=False)
if save_pose_plot(os.path.join(out, "poses.png"), objpoints, imgpoints, K, D, a.model):
    print("已生成姿态分布图 poses.png")
print_report(res, out)
print("（合并自 %d 个目录，未复制原图；camera_info.yaml 可直接用）" % len(dirs))

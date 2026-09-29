#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMX219 相机内参一键标定（棋盘格）

用法:
  python3 calib_camera.py front                    # 前视 cam0 (8080)
  python3 calib_camera.py down                     # 下视 cam1 (8081)
  python3 calib_camera.py front --board 11x8 --square 20 --shots 25
  python3 calib_camera.py down --model fisheye     # 广角/鱼眼镜头

按键: 空格=手动采一张  回车=结束并计算  r=清空重采  q=放弃退出
默认自动采集: 检测到棋盘且姿态与上一张差异足够大时自动收一张。

为什么从 MJPEG 流取图而不是自己开相机:
  一路 IMX219 同一时刻只能有一个 Argus 会话, 另开会话会把正在跑的
  video_stream_node/yolo_detector_node 一起拖死成 camera read timeout。
  流里的帧就是节点拿到的同一帧, 标定用完全够(角点检测对 JPEG 不敏感)。

内参和分辨率绑定: 本脚本在 1280x720 下标定, 结果只对这个 sensor mode 有效。
IMX219 换分辨率会换 sensor mode(裁切/binning 都变), 视场角随之改变,
不能把内参按比例缩放套到别的分辨率上, 必须重标。
"""
import argparse
import json
import os
import random
import sys
import time
from datetime import datetime

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjpg_view import MjpegReader

CAMS = {
    "front": ("http://192.168.2.101:8080/stream.mjpg", "cam0_front"),
    "down":  ("http://192.168.2.101:8081/stream.mjpg", "cam1_down"),
}


# ---------- 标定核心(与采集解耦, 便于单独测试) ----------

def make_object_points(board, square):
    """棋盘格内角点的 3D 坐标(z=0), 顺序与 OpenCV 角点顺序一致。"""
    cols, rows = board
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square)
    return objp


def find_corners(gray, board):
    """优先用 SB 版(更鲁棒且自带亚像素), 不可用/失败再退回传统版。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(
            gray, board, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return True, corners
    ok, corners = cv2.findChessboardCorners(
        gray, board,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if not ok:
        return False, None
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    return True, corners


def save_pose_plot(path, objpoints, imgpoints, K, D, model="pinhole"):
    """画所有标定板的 3D 姿态分布(参考 GML Camera Calibration Toolbox 的 3D 视图)。

    倾角/远近这些数字指标看不出的问题, 这张图一眼能看出来: 板是不是都互相平行、
    是不是都挤在同一个距离上 —— 那就是姿态退化的长相, fx 一定不可信。
    左图 3D 全景, 右图俯视(XZ), 俯视图最能看出距离分布和倾斜程度。
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError:
        return None

    quads, tilts = [], []
    for o, i in zip(objpoints, imgpoints):
        dist = None if model == "fisheye" else D
        ok, rv, tv = cv2.solvePnP(o, np.asarray(i, np.float64).reshape(-1, 1, 2), K, dist)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rv)
        X, Y = float(o[:, 0].max()), float(o[:, 1].max())
        box = np.array([[0, 0, 0], [X, 0, 0], [X, Y, 0], [0, Y, 0]], np.float64)
        quads.append(((R @ box.T + tv).T) / 1000.0)      # mm -> m
        tilts.append(np.degrees(np.arccos(min(1.0, abs(R[2, 2])))))
    if not quads:
        return None

    allp = np.vstack(quads)
    fig = plt.figure(figsize=(13, 5.5))
    ax = fig.add_subplot(121, projection="3d")
    cmap = plt.get_cmap("viridis")
    tmax = max(max(tilts), 1e-6)
    ax.add_collection3d(Poly3DCollection(
        quads, alpha=0.4, edgecolor="k", linewidths=0.3,
        facecolors=[cmap(t / tmax) for t in tilts]))
    ax.scatter([0], [0], [0], c="red", s=80, marker="^", label="camera")
    for z in (0.3, 0.6):                                  # 光轴参考线
        ax.plot([0, 0], [0, 0], [0, z], "r--", lw=0.8)
    ax.set_xlim(allp[:, 0].min(), allp[:, 0].max())
    ax.set_ylim(allp[:, 1].min(), allp[:, 1].max())
    ax.set_zlim(0, allp[:, 2].max() * 1.05)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z depth (m)")
    ax.set_title("board poses (color = tilt, %d views)" % len(quads))
    ax.legend(loc="upper left", fontsize=8)

    ax2 = fig.add_subplot(122)
    for q, t in zip(quads, tilts):
        ax2.plot(q[:, 2], q[:, 0], "-", color=cmap(t / tmax), lw=1.2, alpha=0.8)
    ax2.plot(0, 0, "r^", ms=10)
    ax2.set_xlabel("Z depth (m)"); ax2.set_ylabel("X (m)")
    ax2.set_title("top view: slanted line = tilted board;  spread in Z = distance variety")
    ax2.grid(alpha=0.3)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, tmax))
    fig.colorbar(sm, ax=ax2, label="tilt (deg)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def board_tilt_deg(corners, board, img_w, img_h):
    """估板面相对像平面的倾角(度)。0=正对镜头。

    这里用假定焦距 fx≈图像宽度(等效 HFOV≈53°, 正好接近 IMX219 720p 的实测值)。
    倾角估计对 fx 误差很不敏感(fx 差 20% 时倾角偏差仅几度), 用来判断"姿态够不够
    斜"完全够用, 而且采集阶段本来就还没有内参。
    """
    objp = make_object_points(board, 1.0)          # 单位尺度, 只求朝向
    f = float(img_w)
    K = np.array([[f, 0, img_w / 2.0], [0, f, img_h / 2.0], [0, 0, 1.0]])
    ok, rvec, _ = cv2.solvePnP(objp, corners.reshape(-1, 1, 2).astype(np.float64),
                               K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return 0.0
    R, _ = cv2.Rodrigues(rvec)
    return float(np.degrees(np.arccos(min(1.0, abs(R[2, 2])))))


def fx_stability(objpoints, imgpoints, size, model, seed=0):
    """折半交叉检验: 随机分两半各自标定, 比较 fx。

    姿态退化(全部近似正对)时, fx 与距离耦合, 单次标定的 RMS 仍然很低 ——
    RMS 检测不出这种错误, 但两半会给出明显不同的 fx。返回 (fx1, fx2, 相对差%)。
    """
    n = len(objpoints)
    if n < 12:
        return None
    idx = list(range(n))
    random.Random(seed).shuffle(idx)
    half = n // 2
    out = []
    for part in (idx[:half], idx[half:]):
        try:
            _, K, _, _ = calibrate([objpoints[i] for i in part],
                                   [imgpoints[i] for i in part], size, model)
            out.append(float(K[0, 0]))
        except cv2.error:
            return None
    return out[0], out[1], abs(out[0] - out[1]) / max(out) * 100.0


def _fe_flag(name):
    """fisheye 的 CALIB_* 常量位置随版本变过: OpenCV 4 在 cv2.fisheye 下,
    OpenCV 5 挪到了 cv2 顶层。两处都找一遍。"""
    for src in (cv2.fisheye, cv2):
        v = getattr(src, name, None)
        if v is not None:
            return v
    raise AttributeError("找不到 fisheye 标志位 %s" % name)


def _view_rms(observed, projected):
    """单张图的 RMS 重投影误差(px)。两边形状可能是 (N,1,2)/(1,N,2)/(N,2),
    统一拍平成 (N,2) 再算, 不用 cv2.norm——它会按通道数推断类型而报 mismatch。"""
    a = np.asarray(observed, dtype=np.float64).reshape(-1, 2)
    b = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))


def calibrate(objpoints, imgpoints, size, model="pinhole"):
    """返回 (rms, K, D, per_view_err)。size=(w,h)"""
    if model == "fisheye":
        n = len(objpoints)
        op = [o.reshape(1, -1, 3).astype(np.float64) for o in objpoints]
        ip = [c.reshape(1, -1, 2).astype(np.float64) for c in imgpoints]
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
        tvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            op, ip, size, K, D, rvecs, tvecs,
            flags=_fe_flag("CALIB_RECOMPUTE_EXTRINSIC") | _fe_flag("CALIB_FIX_SKEW"),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
        errs = []
        for i in range(n):
            proj, _ = cv2.fisheye.projectPoints(op[i], rvecs[i], tvecs[i], K, D)
            errs.append(_view_rms(ip[i], proj))
        return float(rms), K, D.ravel(), errs

    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
    errs = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        errs.append(_view_rms(imgpoints[i], proj))
    return float(rms), K, D.ravel(), errs


# ---------- 结果输出 ----------

def write_camera_info(path, name, size, K, D, model):
    """ROS camera_info yaml(手写, 不依赖 pyyaml), 可直接喂给需要内参的节点。"""
    w, h = size
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    dmodel = "equidistant" if model == "fisheye" else "plumb_bob"
    fmt = lambda v: ", ".join("%.9g" % x for x in v)
    with open(path, "w") as f:
        f.write("image_width: %d\nimage_height: %d\ncamera_name: %s\n" % (w, h, name))
        f.write("camera_matrix:\n  rows: 3\n  cols: 3\n  data: [%s]\n" % fmt(K.ravel()))
        f.write("distortion_model: %s\n" % dmodel)
        f.write("distortion_coefficients:\n  rows: 1\n  cols: %d\n  data: [%s]\n" % (len(D), fmt(D)))
        f.write("rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n")
        f.write("projection_matrix:\n  rows: 3\n  cols: 4\n  data: [%s]\n"
                % fmt([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]))


def save_results(outdir, name, size, K, D, rms, errs, model, board, square, shots,
                 tilts=None, stability=None):
    os.makedirs(outdir, exist_ok=True)
    write_camera_info(os.path.join(outdir, "camera_info.yaml"), name, size, K, D, model)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    # 水平/垂直视场角, 用来和镜头标称值对一下, 差太多说明标歪了
    fov_x = 2 * np.degrees(np.arctan(size[0] / (2 * fx)))
    fov_y = 2 * np.degrees(np.arctan(size[1] / (2 * fy)))
    res = {
        "camera": name, "model": model, "image_size": list(size),
        "board_inner_corners": list(board), "square_size_mm": square,
        "num_images": shots, "rms_reprojection_px": rms,
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "fov_x_deg": fov_x, "fov_y_deg": fov_y,
        "camera_matrix": K.tolist(), "distortion_coefficients": list(map(float, D)),
        "per_view_reprojection_px": errs,
        "calibrated_at": datetime.now().isoformat(timespec="seconds"),
    }
    if tilts:
        tl = np.array(tilts, dtype=float)
        res["tilt_deg"] = {"median": float(np.median(tl)), "max": float(tl.max()),
                           "min": float(tl.min()), "n_gt20": int((tl > 20).sum()),
                           "n_gt30": int((tl > 30).sum()), "all": [round(v, 2) for v in tl]}
    if stability:
        res["fx_stability"] = {"fx_a": stability[0], "fx_b": stability[1],
                               "diff_pct": stability[2]}
    with open(os.path.join(outdir, "calib.json"), "w") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    return res


def print_report(res, outdir):
    q = "好" if res["rms_reprojection_px"] < 0.5 else (
        "可接受" if res["rms_reprojection_px"] < 1.0 else "偏差大, 建议重标")
    print("\n" + "=" * 58)
    print("标定完成: %s   模型=%s   用图 %d 张" % (res["camera"], res["model"], res["num_images"]))
    print("RMS 重投影误差: %.4f px  (<0.5 好, <1.0 可接受)  --> %s" % (res["rms_reprojection_px"], q))
    print("fx=%.3f  fy=%.3f  cx=%.3f  cy=%.3f" % (res["fx"], res["fy"], res["cx"], res["cy"]))
    print("视场角: 水平 %.2f°  垂直 %.2f°   (和镜头标称值比一下)" % (res["fov_x_deg"], res["fov_y_deg"]))
    print("畸变系数: %s" % ", ".join("%.6f" % v for v in res["distortion_coefficients"]))
    worst = max(res["per_view_reprojection_px"])
    print("单张最差重投影: %.4f px%s" % (worst, "  (>1px 的那张可以删掉重算)" if worst > 1.0 else ""))
    t = res.get("tilt_deg") or {}
    if t:
        print("板倾角分布: 中位 %.1f°  最大 %.1f°  >20°的 %d 张 / 共 %d 张"
              % (t["median"], t["max"], t["n_gt20"], res["num_images"]))
    st = res.get("fx_stability")
    if st:
        flag = "稳定" if st["diff_pct"] < 2 else ("偏大, 结果可疑" if st["diff_pct"] < 5
                                                 else "不可信, 必须重采")
        print("折半交叉检验 fx: %.1f vs %.1f  相差 %.1f%%  --> %s"
              % (st["fx_a"], st["fx_b"], st["diff_pct"], flag))
    bad = []
    if t and t["max"] < 20:
        bad.append("倾角不足(最大仅 %.1f°)" % t["max"])
    if res.get("size_buckets", 3) < 2:
        bad.append("远近距离没变化(只覆盖 1 档)")
    if st and st["diff_pct"] >= 5:
        bad.append("折半 fx 相差 %.1f%%" % st["diff_pct"])
    if abs(res["distortion_coefficients"][-1]) > 5:
        bad.append("k3=%.1f 数值异常" % res["distortion_coefficients"][-1])
    if bad:
        print("")
        print("!! 标定退化警告: %s" % "; ".join(bad))
        print("   RMS 低不代表内参对 —— 板近似正对镜头时, 焦距和距离会耦合,")
        print("   '焦距大+距离远'与'焦距小+距离近'拟合出的图像几乎一样, RMS 都很低。")
        print("   重采一轮: 把板明显**倾斜**举(左右各转 30-45°, 上下各仰俯 30-45°),")
        print("   不要只是平移。至少要有 5-8 张倾角 >30° 的。")
    print("结果目录: %s" % outdir)
    print("  camera_info.yaml  ROS 格式内参")
    print("  calib.json        完整结果")
    print("  images/           采集原图(可重新标定)")
    print("  undistort.jpg     去畸变前后对比")
    print("  poses.png        标定板姿态分布(板都平行/都同距离=退化)")
    print("=" * 58)


# ---------- 采集 ----------

def grid_cell(center, size, n=3):
    x, y = center
    w, h = size
    return min(int(x / w * n), n - 1), min(int(y / h * n), n - 1)


def main():
    ap = argparse.ArgumentParser(description="IMX219 相机内参一键标定")
    ap.add_argument("cam", choices=list(CAMS) + ["url"], help="front=前视8080  down=下视8081")
    ap.add_argument("--url", default=None, help="cam=url 时指定流地址")
    ap.add_argument("--board", default="9x6", help="棋盘格内角点数 列x行, 默认 9x6")
    ap.add_argument("--square", type=float, default=25.0, help="方格边长 mm, 默认 25")
    ap.add_argument("--shots", type=int, default=40, help="采集张数, 默认 40 (20张统计波动约2%%, 40张后收敛)")
    ap.add_argument("--model", choices=["pinhole", "fisheye"], default="pinhole")
    ap.add_argument("--manual", action="store_true", help="关掉自动采集, 只用空格手动采")
    ap.add_argument("--out", default=os.path.expanduser("~/camera_calib"))
    a = ap.parse_args()

    try:
        cols, rows = (int(v) for v in a.board.lower().split("x"))
    except ValueError:
        sys.exit("--board 格式应是 列x行, 例如 9x6")
    board = (cols, rows)

    if a.cam == "url":
        if not a.url:
            sys.exit("cam=url 时必须给 --url")
        url, name = a.url, "custom"
    else:
        url, name = CAMS[a.cam]
        url = a.url or url

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(a.out, "%s_%s" % (name, stamp))
    imgdir = os.path.join(outdir, "images")
    os.makedirs(imgdir, exist_ok=True)
    sys.stdout.reconfigure(line_buffering=True)

    objp = make_object_points(board, a.square)
    objpoints, imgpoints, covered, tilts, sizebkt = [], [], set(), [], set()
    last_corners, last_t = None, 0.0
    size = None

    reader = MjpegReader(url)
    reader.start()
    print("标定 %s   流: %s" % (name, url))
    print("棋盘格内角点 %dx%d, 方格 %.1fmm, 目标 %d 张, 模型 %s"
          % (cols, rows, a.square, a.shots, a.model))
    print("自动采集: %s   空格=手动采  回车=算  r=清空  q=放弃" % ("关" if a.manual else "开"))
    print("采集要点: 标定板占画面 1/3 以上, 覆盖画面各个区域, 带上 ±30° 倾角, 别全部正对镜头")

    win = "calib %s  [SPACE]shot [ENTER]calc [r]reset [q]quit" % name
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    last_seq, frame, warned = -1, None, ""
    abort = False

    try:
        while True:
            seq, jpeg = reader.latest()
            if jpeg is not None and seq != last_seq:
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    last_seq, frame = seq, img
                    if size is None:
                        size = (img.shape[1], img.shape[0])
                        print("图像尺寸: %dx%d" % size)
                    elif (img.shape[1], img.shape[0]) != size:
                        print("警告: 尺寸变了 %dx%d, 跳过该帧" % (img.shape[1], img.shape[0]))
                        continue
            elif frame is None:
                if reader.err and reader.err != warned:
                    warned = reader.err
                    print("[等待流]", reader.err)
                if (cv2.waitKey(30) & 0xFF) in (ord('q'), 27):
                    abort = True
                    break
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, board)

            view = frame.copy()          # 采集要存干净原图, HUD 只画在副本上
            if found:
                cv2.drawChessboardCorners(view, board, corners, True)

            take = False
            now = time.time()
            if found and not a.manual and now - last_t > 0.8:
                if last_corners is None:
                    take = True
                else:
                    # 姿态差异: 角点平均位移, 太小说明还是上一张那个姿态
                    d = float(np.mean(np.linalg.norm(
                        corners.reshape(-1, 2) - last_corners.reshape(-1, 2), axis=1)))
                    take = d > 40.0

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                abort = True
                break
            if k == 32 and found:
                take = True
            if k == ord('r'):
                objpoints, imgpoints, covered = [], [], set()
                last_corners = None
                for fn in os.listdir(imgdir):
                    os.remove(os.path.join(imgdir, fn))
                print("已清空, 重新采集")
            if k in (13, 10):
                break

            if take:
                tilt = board_tilt_deg(corners, board, size[0], size[1])
                tilts.append(tilt)
                objpoints.append(objp.copy())
                imgpoints.append(corners)
                last_corners, last_t = corners, now
                pts = corners.reshape(-1, 2)
                c = pts.mean(axis=0)
                covered.add(grid_cell(c, size))
                # size 维度: 板占画面的比例分 3 档(远/中/近), 对齐 ROS
                # camera_calibration 的 size 进度条 —— 距离不变会让 fx 和深度更难解耦
                span = float(np.linalg.norm(pts.max(0) - pts.min(0))) / size[0]
                sizebkt.add(0 if span < 0.30 else (1 if span < 0.45 else 2))
                fn = os.path.join(imgdir, "%02d.jpg" % len(imgpoints))
                cv2.imwrite(fn, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                print("采集 %d/%d  中心(%.0f,%.0f)  9宫格 %d/9  远近档 %d/3  倾角 %.0f°  倾斜(>20°) %d张"
                      % (len(imgpoints), a.shots, c[0], c[1], len(covered), len(sizebkt),
                         tilt, sum(1 for v in tilts if v > 20)))

            n = len(imgpoints)
            ntilt = sum(1 for v in tilts if v > 20)
            live_tilt = board_tilt_deg(corners, board, size[0], size[1]) if found else 0.0
            cv2.putText(view, "shots %d/%d  cells %d/9  size %d/3  tilt %.0fdeg  >20deg:%d  %s"
                        % (n, a.shots, len(covered), len(sizebkt), live_tilt, ntilt,
                           "FOUND" if found else "no board"),
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if found else (0, 165, 255), 2, cv2.LINE_AA)
            if n >= a.shots:
                cv2.putText(view, "enough, press ENTER to calibrate", (8, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(win, view)

            enough_tilt = sum(1 for v in tilts if v > 20) >= 3
            enough_size = len(sizebkt) >= 2          # 至少两个远近档
            if (n >= a.shots and len(covered) >= 5
                    and ((enough_tilt and enough_size) or n >= a.shots * 2)):
                print("已采够 %d 张且覆盖 %d/9 个区域, 自动开始计算" % (n, len(covered)))
                break

            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    abort = True
                    break
            except cv2.error:
                abort = True
                break
    except KeyboardInterrupt:
        abort = True
    finally:
        reader.stop()
        cv2.destroyAllWindows()

    if abort and len(imgpoints) < 5:
        print("已放弃, 采集到 %d 张(不足 5 张无法标定)" % len(imgpoints))
        return 1
    if len(imgpoints) < 5:
        print("只有 %d 张, 至少要 5 张才标定" % len(imgpoints))
        return 1

    print("\n用 %d 张图计算中..." % len(imgpoints))
    rms, K, D, errs = calibrate(objpoints, imgpoints, size, a.model)
    stab = fx_stability(objpoints, imgpoints, size, a.model)
    res = save_results(outdir, name, size, K, D, rms, errs, a.model, board, a.square,
                       len(imgpoints), tilts=tilts, stability=stab)
    res["size_buckets"] = len(sizebkt)
    json.dump(res, open(os.path.join(outdir, "calib.json"), "w"),
              indent=2, ensure_ascii=False)

    # 去畸变前后对比, 肉眼确认标定是否合理(直线应该变直, 边缘不该被撕开)
    sample = cv2.imread(os.path.join(imgdir, "01.jpg"))
    if sample is not None:
        if a.model == "fisheye":
            und = cv2.fisheye.undistortImage(sample, K, D.reshape(4, 1), Knew=K)
        else:
            und = cv2.undistort(sample, K, D)
        cv2.imwrite(os.path.join(outdir, "undistort.jpg"), np.hstack([sample, und]))

    if save_pose_plot(os.path.join(outdir, "poses.png"), objpoints, imgpoints,
                      K, D, a.model):
        print("已生成姿态分布图 poses.png")
    print_report(res, outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())

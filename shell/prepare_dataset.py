#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把标注好的图片整理成 YOLO 训练集：去重 + 按段划分 + 生成 data.yaml

用法:
  python3 prepare_dataset.py ~/图片 -o ~/dataset
  python3 prepare_dataset.py ~/图片 -o ~/dataset --val 0.2 --hamming 5 --gap 10

做两件手工容易做错的事:
1) 去重: 连拍的相邻帧几乎一样, 留着只会让 val 虚高。用 dhash 判重(汉明距离
   <=--hamming 视为重复), 只保留每组的第一张。
2) 按"段"划分 train/val: 同一次连拍的图必须整段进同一边。随机 shuffle 会把
   同一瞬间的图分到两边 -> 数据泄漏, mAP 好看但上机不行。文件名里的时间戳
   相差超过 --gap 秒就算新的一段。

只收有对应 .txt 标注的图。classes.txt(labelImg 生成)会变成 data.yaml 的 names。
"""
import argparse
import os
import random
import shutil
import sys
from collections import Counter, defaultdict
from datetime import datetime

import cv2
import numpy as np

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


def dhash(path, size=8):
    """差值哈希: 缩到 (size+1)x size, 横向比邻居, 得 size*size 位。"""
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    small = cv2.resize(img, (size + 1, size), interpolation=cv2.INTER_AREA)
    bits = small[:, 1:] > small[:, :-1]
    return int("".join("1" if b else "0" for b in bits.ravel()), 2)


def stamp_of(path):
    """从 20260927_104106_416.jpg 这种名字取时间, 取不到就用文件 mtime。"""
    base = os.path.splitext(os.path.basename(path))[0]
    for fmt, n in (("%Y%m%d_%H%M%S_%f", 3), ("%Y%m%d_%H%M%S", 2)):
        parts = base.split("_")
        if len(parts) >= n:
            try:
                return datetime.strptime("_".join(parts[:n]), fmt).timestamp()
            except ValueError:
                pass
    return os.path.getmtime(path)


def link_or_copy(src, dst):
    try:
        os.link(src, dst)          # 硬链接, 不占额外空间
    except OSError:
        shutil.copy2(src, dst)


def main():
    ap = argparse.ArgumentParser(description="整理 YOLO 训练集")
    ap.add_argument("src", help="图片+标注所在目录")
    ap.add_argument("-o", "--out", required=True, help="输出数据集目录")
    ap.add_argument("--val", type=float, default=0.2, help="验证集比例, 默认 0.2")
    ap.add_argument("--hamming", type=int, default=5, help="去重阈值, 0=不去重")
    ap.add_argument("--gap", type=float, default=10.0, help="超过几秒算新的一段")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    # 必须转绝对路径: data.yaml 里的 path 是相对的话, 换个目录跑训练就找不到数据
    src = os.path.abspath(os.path.expanduser(a.src))
    out = os.path.abspath(os.path.expanduser(a.out))
    imgs = sorted(os.path.join(src, f) for f in os.listdir(src)
                  if f.lower().endswith(IMG_EXT))
    if not imgs:
        sys.exit("%s 里没有图片" % src)

    # 只要有标注的
    paired, unlabeled = [], []
    for p in imgs:
        txt = os.path.splitext(p)[0] + ".txt"
        (paired if os.path.exists(txt) else unlabeled).append(p)
    print("图片 %d 张, 有标注 %d 张, 没标注 %d 张%s"
          % (len(imgs), len(paired), len(unlabeled),
             "(已跳过)" if unlabeled else ""))
    if not paired:
        sys.exit("一张都没标注。先用 labelImg 标完再跑这个脚本")

    # 去重
    kept = []
    if a.hamming > 0:
        hashes = []
        dup = 0
        for p in paired:
            h = dhash(p)
            if h is None:
                print("  读不出, 跳过:", p)
                continue
            if any(bin(h ^ k).count("1") <= a.hamming for k in hashes):
                dup += 1
                continue
            hashes.append(h)
            kept.append(p)
        print("去重: 丢弃 %d 张近重复, 保留 %d 张" % (dup, len(kept)))
    else:
        kept = paired

    # 分段
    kept.sort(key=stamp_of)
    segs, cur, last = [], [], None
    for p in kept:
        t = stamp_of(p)
        if last is not None and t - last > a.gap:
            segs.append(cur)
            cur = []
        cur.append(p)
        last = t
    if cur:
        segs.append(cur)
    print("分成 %d 段 (每段图数: %s)"
          % (len(segs), ", ".join(str(len(s)) for s in segs[:12])
             + (" ..." if len(segs) > 12 else "")))

    # 整段分到 train 或 val
    rnd = random.Random(a.seed)
    order = list(range(len(segs)))
    rnd.shuffle(order)
    target = len(kept) * a.val
    val_idx, n = set(), 0
    for i in order:
        if n >= target and val_idx:
            break
        val_idx.add(i)
        n += len(segs[i])
    split = {"train": [], "val": []}
    for i, s in enumerate(segs):
        split["val" if i in val_idx else "train"].extend(s)
    if not split["train"] or not split["val"]:
        sys.exit("段太少, 分不出 train/val。多采几段(不同时间/场景)再来")

    # 落盘
    for sub in ("train", "val"):
        for kind in ("images", "labels"):
            d = os.path.join(out, kind, sub)
            os.makedirs(d, exist_ok=True)
            for fn in os.listdir(d):
                os.remove(os.path.join(d, fn))
    cls_count = {"train": Counter(), "val": Counter()}
    for sub, files in split.items():
        for p in files:
            link_or_copy(p, os.path.join(out, "images", sub, os.path.basename(p)))
            txt = os.path.splitext(p)[0] + ".txt"
            link_or_copy(txt, os.path.join(out, "labels", sub,
                                           os.path.basename(txt)))
            with open(txt) as f:
                for line in f:
                    if line.strip():
                        cls_count[sub][int(line.split()[0])] += 1

    # 类别名
    names = None
    for cand in ("classes.txt", "classes.names"):
        p = os.path.join(src, cand)
        if os.path.exists(p):
            names = [l.strip() for l in open(p) if l.strip()]
            break
    if names is None:
        mx = max(list(cls_count["train"]) + list(cls_count["val"]) + [0])
        names = ["class%d" % i for i in range(mx + 1)]
        print("没找到 classes.txt, 类别名先用占位, 记得手改 data.yaml")

    yml = os.path.join(out, "data.yaml")
    with open(yml, "w", encoding="utf-8") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n" % out)
        for i, n_ in enumerate(names):
            f.write("  %d: %s\n" % (i, n_))

    print("\ntrain %d 张 / val %d 张 (val 占 %.0f%%)"
          % (len(split["train"]), len(split["val"]),
             100.0 * len(split["val"]) / len(kept)))
    print("每类实例数:")
    for i, n_ in enumerate(names):
        tr, va = cls_count["train"][i], cls_count["val"][i]
        warn = ""
        if tr + va == 0:
            warn = "   <- 一个都没有"
        elif va == 0:
            warn = "   <- val 里没有, 评估看不出这类"
        elif tr < 50:
            warn = "   <- train 太少, 建议 >=200"
        print("  %d %-20s train %5d  val %5d%s" % (i, n_, tr, va, warn))
    print("\n数据集: %s\ndata.yaml: %s" % (out, yml))
    print("接着训练:\n  yolo detect train model=yolo26n.pt data=%s imgsz=640 "
          "epochs=150 batch=16 device=0" % yml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

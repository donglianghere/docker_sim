#!/usr/bin/env python3
"""生成2026大赛任务场景用的AprilTag标记图片（DICT_APRILTAG_36H11）。

见《2026大赛任务系统开发执行方案.md》阶段1.4：ID 0=物资点、ID 1=高层
着火点标识（贴在模拟高层建筑的2#立柱上）、ID 2=地面火情标识。本机
opencv-python 4.11自带这个字典，不需要联网下载，直接cv2.aruco.
generateImageMarker()生成即可。输出用于patches/目录下的PNG资产文件
（mighty_contest_apriltag_id{0,1,2}.png），被docker/Dockerfile.
sim-world直接COPY进worlds/media/materials/textures/，供worlds/media/
materials/scripts/contest_markers.material引用。

⚠️ 2026-09-09场景重新设计：ID1/ID2这两个"火情"相关标识**不再共址**
（原来的设计是两者贴在同一根立柱脚下、物理位置一致，只是接近方式
不同）——用户明确要求地面火情标识独立放在(0,0,0)这个固定坐标，
跟2#立柱(0,8,0)贴的ID1是两个不同位置的独立目标，阶段4(前视环绕2#
立柱识别ID1)和阶段5(下视地面搜索识别ID2)现在找的是两个真正分开的
物理点，不是同一个点的两种接近方式。1#/3#两根立柱保持无tag，纯几何
候选目标。

用法：python3 scripts/gen_contest_apriltags.py [输出目录，默认patches/]
"""
import sys
from pathlib import Path

import cv2
import numpy as np

MARKERS = {
    0: "mighty_contest_apriltag_id0.png",  # 物资点（下视相机识别，编码物资类型）
    1: "mighty_contest_apriltag_id1.png",  # 高层着火点标识（前视相机识别，贴在2#立柱上）
    2: "mighty_contest_apriltag_id2.png",  # 地面火情标识（下视相机识别，独立固定坐标(0,0,0)，
                                            # 2026-09-09起不再跟ID1共址，见上面说明）
}
SIZE = 512
BORDER_RATIO = 0.15  # 白边比例，给相机检测留对比边距（AprilTag标准做法）


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "patches"
    out_dir.mkdir(parents=True, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    for tag_id, filename in MARKERS.items():
        tag_img = cv2.aruco.generateImageMarker(aruco_dict, tag_id, SIZE)
        border = int(SIZE * BORDER_RATIO)
        canvas = np.full((SIZE + 2 * border, SIZE + 2 * border), 255, dtype=np.uint8)
        canvas[border:border + SIZE, border:border + SIZE] = tag_img
        out_path = out_dir / filename
        cv2.imwrite(str(out_path), canvas)

        corners, ids, _ = detector.detectMarkers(canvas)
        ok = ids is not None and len(ids) == 1 and ids[0][0] == tag_id
        print(f"[{'OK' if ok else 'FAIL'}] {out_path}  decoded_ids={ids}")
        if not ok:
            raise SystemExit(f"生成的{filename}解码校验失败，检查cv2.aruco版本/字典是否匹配")


if __name__ == "__main__":
    main()

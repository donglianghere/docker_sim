#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""弓字搜索地面火情：扫编队航线围成的四边形，下视相机看到火点就停下悬停。

    python3 地面火情搜索示例.py --namespace NX01 --teammate NX02

主线程按弓字航线逐点飞；后台线程全程盯着下视相机，一看到火点就打断当前
航段——必须这样，一条航段要飞十几秒，等它飞完再检查早就飞过火点了。

火点坐标用 sdk.locate_target() 取：飞行栈按相机内参、拍照时刻的位姿和地面
高度算出的火点实际位置，不是飞机自己的位置（火点在画面边缘时两者能差 1.5 米）。

关键节点的声音：起飞、降落由 takeoff()/land() 自动播，这里只播发现火情、
通报火情、任务完成三条（手动再播起降会重复）。
"""
import argparse
import threading
import time

from contest_sdk import DroneSDK
from contest_sdk.exceptions import DetectionTimeoutError, GotoUnreachableError

# 搜索区域：编队航线四个航点围成的四边形（世界坐标）
AREA = [(7.0, -10.0), (7.0, 10.0), (-8.0, 10.0), (-8.0, -10.0)]
CRUISE_AGL_M = 2.5          # 飞高一点覆盖更宽；必须低于 规划器天花板-0.1-dist0（见launch注释）
GROUND_FIRE = 'apriltag:2'
FIRE_MARKER_SIZE_M = 0.5    # 地面火点标志尺寸：航线间距要扣掉它，保证目标能完整入画
OVERLAP = 0.2               # 扣掉目标尺寸之后，再留 20% 给定位误差
HOVER_S = 10.0              # 找到后在火点上方悬停多久，再返航降落

# 题目给的 3 根立柱（坐标已知，可以写进程序）。r 是方立柱的半对角线（边长
# 0.6 米 -> 0.42），余量必须不小于规划器的障碍物膨胀半径（仿真0.6、真机0.8）。
# 障碍圆柱、仿地模块坐标未知，不写——航点落进去时 goto() 会自己判定不可达。
PILLARS = [(4.0, 4.0, 0.42), (0.0, 8.0, 0.42), (-5.0, 2.0, 0.42)]
CLEARANCE_M = 0.8


class FireWatcher:
    """后台线程：只认下视相机，看到火点就算出火点坐标、打断当前航段。"""

    def __init__(self, sdk):
        self.sdk = sdk
        self.found = threading.Event()
        self.fire_local = None          # 火点的局部坐标（x, y）
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            try:
                self.sdk.wait_for_detection(GROUND_FIRE, timeout=5.0, camera='down')
            except DetectionTimeoutError:
                continue
            # 趁火点还在画面里、边飞边定位：飞行栈按拍照时刻的位姿算，已经
            # 补偿了飞行中的检测延迟。只取 3 帧，求快——悬停后还会再精测一次。
            try:
                fire = self.sdk.locate_target(GROUND_FIRE, timeout=2.0, samples=3)
                self.fire_local = (fire.x, fire.y)
            except DetectionTimeoutError:
                # 火点刚擦过画面边缘就出去了，退而求其次用飞机当时的位置
                x, y, _ = self.sdk.get_local_position()
                self.fire_local = (x, y)
            self.found.set()
            self.sdk.play_sound_light('侦察机发现地面火情')
            # 连打断两次：主线程可能正好在两个航点之间，第一次打断落空，
            # 紧接着又发出了下一个航点
            for _ in range(2):
                self.sdk.cancel_goto()
                time.sleep(0.5)
            return


def build_route(sdk):
    xs = [p[0] for p in AREA]
    ys = [p[1] for p in AREA]
    route = sdk.generate_ground_scan_waypoints(
        room_min_x=min(xs), room_max_x=max(xs),
        room_min_y=min(ys), room_max_y=max(ys),
        altitude_agl=CRUISE_AGL_M, target_size_m=FIRE_MARKER_SIZE_M, overlap_ratio=OVERLAP)
    return sdk.pull_waypoints_out_of_circles(route, PILLARS, CLEARANCE_M)


def search(sdk, watcher):
    route = build_route(sdk)
    print(f'[{sdk.namespace}] 弓字搜索开始，共 {len(route)} 个航点', flush=True)
    for i, (wx, wy, wz) in enumerate(route, start=1):
        if watcher.found.is_set():
            break
        print(f'[{sdk.namespace}] 航点 {i}/{len(route)}: ({wx:.1f}, {wy:.1f})', flush=True)
        try:
            sdk.goto(*sdk.world_to_local(wx, wy, wz))
        except GotoUnreachableError:
            # 航点落在障碍物里（未知坐标的那些），跳过——火点不会在障碍物底下
            print(f'[{sdk.namespace}] 航点 {i} 不可达，跳过', flush=True)
    return watcher.found.is_set()


def hover_over_fire(sdk, watcher):
    # 1. 直线飞到算出来的火点正上方，高度不变。刚飞过、只有一两米，用直线飞。
    fx, fy = watcher.fire_local
    _, _, z = sdk.get_local_position()
    sdk.goto_direct(fx, fy, z)

    # 2. 在正上方再精测一次：火点在画面中央，畸变和姿态误差影响最小，飞机也
    #    停稳了，比飞行中擦边看到时准。偏差超过 0.1 米就再挪一下。
    try:
        fire = sdk.locate_target(GROUND_FIRE, timeout=5.0, samples=10)
        if abs(fire.x - fx) > 0.1 or abs(fire.y - fy) > 0.1:
            sdk.goto_direct(fire.x, fire.y, z)
        fx, fy, how = fire.x, fire.y, f'精测（{fire.samples}帧，离散 {fire.spread_m:.2f}m）'
    except DetectionTimeoutError:
        how = '飞行中测得'           # 正上方没看到，用发现时算的坐标
    wx, wy, _ = sdk.local_to_world(fx, fy, 0.0)
    print(f'[{sdk.namespace}] 悬停在地面火情上方，火点世界坐标 ({wx:.2f}, {wy:.2f})，{how}',
          flush=True)
    sdk.play_sound_light('侦察机通报地面火情')
    time.sleep(HOVER_S)


def return_and_land(sdk):
    pad = sdk.local_to_world(0.0, 0.0, 0.0)
    home = sdk.world_to_local(pad[0], pad[1], CRUISE_AGL_M)
    try:
        sdk.goto(*home)             # 远距离回程走规划器，有避障
    except GotoUnreachableError:
        pass
    sdk.goto_direct(*home)          # 最后一段收准，落点精度高一个量级
    sdk.land()                      # 自动播"侦察机降落"


def main():
    ap = argparse.ArgumentParser(description='弓字搜索地面火情')
    ap.add_argument('--namespace', default='NX01')
    ap.add_argument('--teammate', default='NX02')
    args = ap.parse_args()

    sdk = DroneSDK(namespace=args.namespace, role='recon', teammate_namespace=args.teammate)
    watcher = FireWatcher(sdk)
    try:
        sdk.takeoff()               # 自动播"侦察机起飞"
        watcher.start()
        found = search(sdk, watcher)
        watcher.stop()
        if found:
            sdk.cancel_goto()
            hover_over_fire(sdk, watcher)
        else:
            print(f'[{sdk.namespace}] 整个区域扫完，没有发现地面火情', flush=True)
        return_and_land(sdk)
        if found:
            sdk.play_sound_light('侦察机任务完成')
        print(f'[{sdk.namespace}] 结束', flush=True)
    finally:
        watcher.stop()
        sdk.shutdown()


if __name__ == '__main__':
    main()

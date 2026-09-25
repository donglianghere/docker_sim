#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿地飞行：开一个开关，飞机就按"恒定离地高度"飞，地形一抬高它跟着抬。

    bash 运行仿真.sh --task 仿地飞行示例.py --single
    bash 运行仿真.sh --task 仿地飞行示例.py --single --agl 1.0

关键点：**这个程序里没有仿地模块的坐标**。赛场上那块模块放在哪事先不知道，所以
不能靠"飞到某个坐标就抬高"这种写法。真正要做的只有一件事——把高度钉住：

    sdk.set_fixed_altitude(高度)   # 开
    ...正常 goto() 飞你的航线...
    sdk.set_fixed_altitude_off()   # 关

为什么钉住高度就是仿地：本项目默认 `LOCALIZATION_SOURCE=uwb_imu`，那套定位方案的
z 本来就是**离地高度**（`uwb_imu_fusion_node` 把 测距雷达range×cos(roll)×cos(pitch)
喂给飞控当高度观测，x/y 才用 UWB，见该文件头）。所以命令高度恒定 = 离地高度恒定，
地面抬高时飞控自己把飞机顶上去，选手这边不需要读雷达、也不需要知道地形在哪。

为什么还需要这个开关：规划器是高频重规划的，轨迹高度本身在波动（实测同一段航线
里轨迹 z 在 0.97~2.33 米之间跑），地形起伏那点变化会被这种波动淹没，飞机不会稳定
跟着地面走；钉死之后才是稳定的仿地行为。切进来时飞行栈按 0.6 m/s 过渡，不会阶跃。

⚠️ 定位源不是 uwb_imu（比如 gt/dlio，z 是绝对高度）时，这个开关只是"定高飞行"，
没有仿地效果。

这一版航线横穿整个场地南侧（模块就在这条线上，但程序并不知道），往返各飞一次，
日志里打"离地/绝对高度"两列，能直接看出经过台面时绝对高度抬起来、离地高度基本
不变。
"""
import argparse
import time

from contest_sdk import DroneSDK
from contest_sdk.exceptions import DetectionTimeoutError, GotoUnreachableError

# 横穿场地南侧的一条航线（世界坐标）。跟仿地模块无关——它在哪都行，这条线上
# 有起伏就会起伏，没有就平飞。
# 沿 x=7 南北向穿过场地南半部分：正好垂直穿过仿地模块（程序并不知道它在哪，
# 这条线只是"场地里有地形的那一带"）。y 只到 -3，不去碰 (7,0) 那根障碍圆柱。
LEG = [(7.0, -9.0), (7.0, -3.0)]
FOLLOW_AGL_M = 2.0          # 要保持的离地高度（坡道高0.5米，离顶面1.5米，规划器不会绕）
SAMPLE_S = 0.5              # 飞行中每隔这么久记一条"离地/绝对高度"，只为看效果


def watch_height(sdk, seconds, label):
    """飞行中按固定间隔记录离地高度和绝对高度——看仿地效果的唯一依据。"""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            agl = sdk.get_agl(timeout=1.0)
        except DetectionTimeoutError:
            agl = None
        _, _, z = sdk.get_local_position()
        print(f'[{sdk.namespace}] {label} 离地 '
              + (f'{agl:.2f}' if agl is not None else '  ? ')
              + f' m，绝对高度 {z:.2f} m', flush=True)
        time.sleep(SAMPLE_S)


def main():
    ap = argparse.ArgumentParser(description='仿地飞行（恒定离地高度）')
    ap.add_argument('--namespace', default='NX01')
    ap.add_argument('--role', default='leader')      # 运行脚本会传，本程序不用
    ap.add_argument('--teammate', default='NX02')
    ap.add_argument('--agl', type=float, default=FOLLOW_AGL_M, help='要保持的离地高度（米）')
    ap.add_argument('--no-follow', action='store_true',
                    help='关掉仿地、按绝对高度飞同一条航线，用来对比')
    args = ap.parse_args()

    sdk = DroneSDK(namespace=args.namespace, role='recon', teammate_namespace=args.teammate)
    try:
        sdk.takeoff(height_m=args.agl)              # 直接起到要保持的离地高度

        if args.no_follow:
            print(f'[{sdk.namespace}] 对比模式：不开仿地，按绝对高度 {args.agl:.2f} m 飞',
                  flush=True)
        else:
            # 定高的 z 是局部系的，世界高度要先换算（见 SDK set_fixed_altitude 说明）
            sdk.set_fixed_altitude(sdk.world_to_local(0.0, 0.0, args.agl)[2])

        route = LEG + LEG[::-1][1:]                 # 去一趟再原路回来
        for i, (wx, wy) in enumerate(route, start=1):
            print(f'[{sdk.namespace}] 航点 {i}/{len(route)}: ({wx:.1f}, {wy:.1f})', flush=True)
            try:
                sdk.goto(*sdk.world_to_local(wx, wy, args.agl))
            except GotoUnreachableError as exc:
                print(f'[{sdk.namespace}] 这个航点不可达（{exc}），跳过', flush=True)
                continue
            watch_height(sdk, 1.0, f'航点{i}到位')

        sdk.set_fixed_altitude_off()                # 返航段不要仿地
        pad = sdk.local_to_world(0.0, 0.0, 0.0)
        try:
            sdk.goto(*sdk.world_to_local(pad[0], pad[1], args.agl))
        except GotoUnreachableError:
            pass
        sdk.goto_direct(0.0, 0.0, args.agl)
        sdk.land()                                  # 自动播"侦察机降落"
        print(f'[{sdk.namespace}] 仿地飞行结束', flush=True)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()

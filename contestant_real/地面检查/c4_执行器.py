#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面检查 4：执行器（发射器 + 双爪夹具）。**不起飞、不解锁。**

动作和核对都在 SDK 里，这个程序只决定"按命令行调哪一个"：

    sdk.check_actuators()   核飞控参数（Set 映射/行程/COM_PREARM_MODE/频率）
    sdk.grip()              抓取        sdk.grip(release=True)  打开爪子
    sdk.shoot()             发射（推到发射位再复位）
    sdk.set_servos()        手动把某一路停在指定 PWM（试行程用）

用法::

    ./c4_执行器.sh --params             # 只核飞控参数，一下都不动
    ./c4_执行器.sh --open               # 打开爪子
    ./c4_执行器.sh --grip               # 抓取
    ./c4_执行器.sh --shoot              # 发射
    ./c4_执行器.sh --launcher 1500      # 发射机构停在指定 PWM
    ./c4_执行器.sh --ns NX01 --params   # 换机号

接线、行程、动作方向都在 SDK 的 `SERVO_CONFIG` 里，不在这个程序里
（两机相同：爪子 MAIN7+MAIN9 共用 Actuator Set1，**PWM 900~2100，
900=抓紧 / 2100=打开**；发射器 MAIN5 在 Set2，**PWM 800~2200，
2200=发射位 / 800=装填复位**。两路都已在 NX02 上实测通过，2026-10-15；
**NX01 侧还没实测过**）。

⚠️ 先 `--params` 核对、再动执行器：接错口会驱动别的输出，而且不报错——
指令照样被飞控 ACCEPTED。
"""
import argparse
import sys

from contest_sdk import DroneSDK

NEEDS_AIRCRAFT = True          # 要连飞机（执行器指令走 mavros）


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='执行器地面检查（不起飞、不解锁）')
    ap.add_argument('--ns', default='NX02', help='飞机命名空间（默认 NX02，夹具在任务机上）')
    ap.add_argument('--params', action='store_true', help='只核飞控参数，一下都不动')
    ap.add_argument('--open', action='store_true', help='打开爪子')
    ap.add_argument('--grip', action='store_true', help='抓取')
    ap.add_argument('--shoot', action='store_true', help='发射（推到发射位再复位）')
    ap.add_argument('--launcher', type=int, metavar='PWM',
                    help='发射机构停在指定 PWM（手动试行程，不自动复位）')
    args = ap.parse_args(argv)
    if args.open and args.grip:
        print('!! --open 和 --grip 只能给一个', file=sys.stderr)
        return 2

    sdk = DroneSDK(namespace=args.ns,
                   role='supply' if args.ns == 'NX02' else 'recon',
                   teammate_namespace=args.ns)
    try:
        sdk.progress(f'夹爪：{ {n: sp.output for n, sp in sdk.grippers.items()} }'
                     f'   发射机构：{ {n: sp.output for n, sp in sdk.launchers.items()} }')
        if not sdk.check_actuators():
            sdk.progress('参数没全对——**一下都不动**。用 QGC 或 ros2 param set 改好再测：'
                         'PWM_MAIN_FUNCn = 300+Set号、舵机那组 PWM_MAIN_TIMn = 50、'
                         'COM_PREARM_MODE = 2')
            return 1
        if args.params:
            sdk.progress('（--params：只核参数，不动执行器）')
            return 0

        did = False
        if args.open or args.grip:
            sdk.progress('⚠️ 确认爪子附近没有手，爪子里没夹着东西（打开会掉）')
            sdk.grip(release=args.open, label='打开爪子' if args.open else '抓取')
            did = True
        if args.shoot:
            sdk.progress('⚠️ 确认机构前方无人（NX02 已实测通过，NX01 侧还没验过）')
            sdk.shoot(label='发射')
            did = True
        if args.launcher is not None:
            if not sdk.launchers:
                sdk.progress('这架飞机没有配置发射机构，跳过 --launcher')
            else:
                sdk.set_servos({n: args.launcher for n in sdk.launchers})
            did = True
        if not did:
            sdk.progress('没给动作（--open/--grip/--shoot/--launcher），只核了参数')
        return 0
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    sys.exit(main())

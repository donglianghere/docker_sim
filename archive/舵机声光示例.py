#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""舵机 + 声光反馈示例：不起飞，只动 NX02 上的两个舵机，同时播报对应语音。

    bash 运行真机.sh --task 舵机声光示例.py --single --leader NX02 --follower NX01
    bash 运行仿真.sh --task 舵机声光示例.py --single --leader NX02 --follower NX01

飞控设置了 COM_PREARM_MODE=2，所以上锁状态下舵机也能动（电机不会转）。
⚠️ 运行前把手从抓取机构里拿开。
"""
import argparse
import time

from contest_sdk import DroneSDK

GRAB_PWM = 800    # 抓取（抓紧）位置——2026-09-21 NX02实测确认
DROP_PWM = 2000   # 投放（松开）位置——2026-09-21 NX02实测确认
HOLD_S = 3.0      # 每个动作之后停多久，让舵机转到位、语音播完


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--namespace', default='NX02')
    ap.add_argument('--role', default='follower')
    ap.add_argument('--teammate', default='NX01')
    args = ap.parse_args()

    sdk = DroneSDK(namespace=args.namespace, role='supply', teammate_namespace=args.teammate)
    try:
        if not sdk.servos:
            print(f'[{sdk.namespace}] 这架飞机没有配置舵机，舵机在 NX02 上，用 --leader NX02 运行')
            return
        all_servos = sorted(sdk.servos)

        # 声光只是往常驻程序发一条请求、立刻返回，所以跟舵机指令几乎同时生效
        sdk.play_sound_light('任务机抓取灭火弹')
        sdk.set_servos({s: GRAB_PWM for s in all_servos})
        time.sleep(HOLD_S)

        sdk.play_sound_light('任务机投放灭火弹')
        sdk.set_servos({s: DROP_PWM for s in all_servos})
        time.sleep(HOLD_S)

        print(f'[{sdk.namespace}] 舵机声光示例结束', flush=True)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()

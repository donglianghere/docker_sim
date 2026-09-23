#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机灭火全流程：先地面火情、后高楼火情，一次跑完。

    bash 运行仿真.sh --task 双机灭火任务示例.py

一份代码两架飞机各起一个容器，按 --role 分工（运行脚本会传）：

  leader（NX01，侦察机）：起飞 -> 弓字搜索地面火情、通报 -> 回起飞点**悬停**
      -> 逐栋绕飞找高层火情、对准、通报、发射破窗弹 -> 回起飞点 -> 降落

  follower（NX02，任务机）：等地面火情通报 -> 起飞取灭火弹、投到火点 -> 回起飞点
      **降落** -> 等高层火情通报 -> 起飞、飞到瞄准位置、发射灭火弹 -> 回起飞点降落

三条规则（用户 2026-09-23 要求）：
  · 每个任务结束两架都要回到自己的起飞点；
  · 侦察机回到起飞点后**不降落**，直接接着做下一个任务；任务机每个任务都
    **必须降落**；
  · 侦察机要**等任务机降落之后**才开始下一个任务——任务机每落一次就发一条
    事件通知侦察机，侦察机在起飞点悬停着等这条事件。

任务段的具体实现直接复用两个单任务示例，不再抄一遍：地面火情来自
《地面火情搜索示例.py》，高楼火情来自《高楼火情示例.py》（跟本文件放在同一个
文件夹里，容器里一起挂到 /workspace）。想单独调某一半就直接跑那个示例。

两架同时在天上时不用刻意错开位置，机间避让由飞控栈负责。
"""
import argparse

import 地面火情搜索示例 as 地面
import 高楼火情示例 as 高楼
from contest_sdk import DroneSDK
from contest_sdk.exceptions import GotoUnreachableError, TeammateUnreachableError

RETURN_AGL_M = 地面.CRUISE_AGL_M     # 回起飞点的高度，跟地面搜索巡航高度一致
SUPPLY_LANDED_EVENT = '任务机本阶段已降落'
WAIT_SUPPLY_S = 900.0                # 侦察机等任务机降落最多等多久


def return_home_hover(sdk):
    """回到起飞点上方悬停（不降落）。起飞点就是自己局部系的原点。"""
    print(f'[{sdk.namespace}] 返回起飞点', flush=True)
    home = (0.0, 0.0, RETURN_AGL_M)
    try:
        sdk.goto(*home)             # 远距离回程走规划器，有避障
    except GotoUnreachableError:
        pass
    sdk.goto_direct(*home)          # 最后一段收准，下一个任务从同一个点出发
    # 机头恢复成起飞时的朝向：上一个任务把它锁在了对准火点的方向上
    sdk.set_yaw_mode_constant(sdk.pretakeoff_yaw or sdk.get_current_yaw())


def wait_supply_landed(sdk, 通知):
    """在起飞点悬停着等任务机落地。等不到也继续往下做，但要说清楚。"""
    print(f'[{sdk.namespace}] 在起飞点等任务机完成本阶段并降落…', flush=True)
    if 通知.wait(WAIT_SUPPLY_S):
        print(f'[{sdk.namespace}] 任务机已降落，开始下一个任务', flush=True)
    else:
        print(f'[{sdk.namespace}] 等了 {WAIT_SUPPLY_S:.0f} 秒没等到任务机降落，继续下一个任务',
              flush=True)


def run_recon(sdk):
    """侦察机：两个任务连着做，中间回起飞点悬停等任务机降落，自己不降落。"""
    # 接收器提前注册：任务机可能比侦察机先落地（见 run_supply 里的说明）
    任务机已降落 = 高楼.Notice()
    sdk.on_teammate_event(SUPPLY_LANDED_EVENT, 任务机已降落.on_event)

    sdk.takeoff()                   # 自动播"侦察机起飞"

    found_ground = 地面.recon_search_and_report(sdk)
    return_home_hover(sdk)
    wait_supply_landed(sdk, 任务机已降落)

    found_high = 高楼.recon_orbit_and_fire(sdk)
    return_home_hover(sdk)

    sdk.land()                      # 全部任务做完才降落，自动播"侦察机降落"
    if found_ground or found_high:
        sdk.play_sound_light('侦察机任务完成')
    print(f'[{sdk.namespace}] 地面火情{"已" if found_ground else "未"}发现，'
          f'高层火情{"已" if found_high else "未"}发现', flush=True)


def run_supply(sdk, teammate):
    """任务机：两个任务各自等通报、各自起降（每次都要落回起飞点）。

    两个通报的接收器都**在最开始就注册**：侦察机做完地面任务会接着做高楼任务，
    它发高层通报时任务机很可能还在做地面任务；而可靠事件通道是先回 ACK 再查
    处理函数，没注册的事件会被确认然后丢弃，发送方还以为送到了（实测丢过一次，
    任务机就一直等在那儿）。提前注册后，早到的通报会先存着，轮到那一段直接取。
    """
    报告地面 = 地面.listen_for_report(sdk)
    报告高层 = 高楼.listen_for_report(sdk)

    地面.run_supply(sdk, teammate, 报告地面)   # 取弹投放 -> 返航降落
    通知已降落(sdk, teammate)                  # 侦察机等这条才开始高楼任务
    高楼.run_supply(sdk, teammate, 报告高层)   # 瞄准发射 -> 返航降落
    通知已降落(sdk, teammate)


def 通知已降落(sdk, teammate):
    """告诉侦察机"本阶段我已经落地了"。送不到不影响自己往下走——侦察机那边
    等不到也会继续（见 wait_supply_landed）。"""
    try:
        sdk.send_to_teammate(SUPPLY_LANDED_EVENT, timeout_s=30.0)
    except TeammateUnreachableError as exc:
        print(f'[{sdk.namespace}] 降落通知没送达 {teammate}：{exc}', flush=True)


def main():
    ap = argparse.ArgumentParser(description='双机灭火全流程（地面火情 + 高楼火情）')
    ap.add_argument('--namespace', default='NX01')
    ap.add_argument('--role', default='leader', choices=['leader', 'follower'])
    ap.add_argument('--teammate', default='NX02')
    args = ap.parse_args()

    is_recon = args.role == 'leader'
    sdk = DroneSDK(namespace=args.namespace,
                   role='recon' if is_recon else 'supply',
                   teammate_namespace=args.teammate)
    try:
        if is_recon:
            run_recon(sdk)
        else:
            run_supply(sdk, args.teammate)
        print(f'[{sdk.namespace}] 全流程结束', flush=True)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机全流程（高层用定点巡检）：编队飞行 -> 地面火情 -> 高层火情，一次跑完。

    bash 运行仿真.sh --task 双机全流程巡检版示例.py

跟《双机全流程示例.py》的唯一区别是**第三段换成了定点巡检版**：不再绕着立柱飞，
而是到三条边的中点上按两个高度、两个朝向定点看（见《高层火情巡检版示例.py》），
实测这一段从 8 分钟缩到 4 分钟，而且不会出现"绕着 A 楼却看见贴在 B 楼上的标志"
这种归属混乱。绕飞版仍然保留，想用就跑那个全流程示例。

一份代码两架飞机各起一个容器，按 --role 分工（运行脚本会传）：

  leader（NX01，侦察机/长机）
      起飞 -> ① 带僚机飞编队航线 -> 回起飞点悬停、等任务机降落
           -> ② 弓字搜索地面火情、通报 -> 回起飞点悬停、等任务机降落
           -> ③ 定点巡检高层火情 -> 判是否正对、必要时挪到正对位置 -> 通报
              -> 等任务机到待命点 -> 发射破窗弹 -> 回起飞点 -> 降落

  follower（NX02，任务机/僚机）
      ① 起飞跟队，航线飞完回起飞点降落
      ② 收到地面火情通报 -> 起飞取灭火弹（边瞄准边降落+抓取）-> 飞火点投放 -> 降落
      ③ 收到高层火情通报 -> 起飞到待命点报到 -> 等破窗 -> 进场发射灭火弹
         -> 原地等 5 秒让侦察机先返航 -> 降落

三条规则（用户 2026-09-23 要求）：
  · 每个任务结束两架都要回到自己的起飞点；
  · 侦察机在起飞点**上空悬停等待**，全部任务做完才降落；任务机每段都**必须降落**；
  · 侦察机要**等任务机降落之后**才开始下一个任务。

各段实现直接复用单任务示例（同一个文件夹，容器里一起挂到 /workspace），这里只
负责串起来：编队飞行示例.py / 地面火情搜索示例.py / 高层火情巡检版示例.py
"""
import argparse

import 地面火情搜索示例 as 地面
import 编队飞行示例 as 编队
import 高层火情巡检版示例 as 巡检
import 高楼火情绕飞版示例 as 高楼      # 只用它的 Notice/listen_for_report 这些公共件
from contest_sdk import DroneSDK
from contest_sdk.exceptions import GotoUnreachableError, TeammateUnreachableError

# 编队航线（世界坐标），用户指定。注意 x=7 这条边会经过障碍圆柱 (7, 0) 和
# 仿地模块 (7, -6)（长边 3 米，占 x∈[5.5, 8.5]），长机到那儿会被规划器带着
# 绕一下，僚机跟轨迹也会跟着扭——不是故障。
ROUTE = [(7.0, -9.5), (7.0, 9.5), (-7.0, 9.5), (-7.0, -9.5)]
SPACING_M = 3.5                      # 僚机沿轨迹的跟随间距下限
RETURN_AGL_M = 地面.CRUISE_AGL_M     # 侦察机回起飞点等待的高度
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
    # 机头恢复成起飞时的朝向：上一个任务可能把它锁在了对准火点的方向上
    sdk.set_yaw_mode_constant(sdk.pretakeoff_yaw or sdk.get_current_yaw())


def wait_supply_landed(sdk, 通知):
    """在起飞点悬停着等任务机落地。等不到也继续往下做，但要说清楚。"""
    print(f'[{sdk.namespace}] 在起飞点等任务机完成本阶段并降落…', flush=True)
    if 通知.wait(WAIT_SUPPLY_S):
        print(f'[{sdk.namespace}] 任务机已降落，开始下一个任务', flush=True)
    else:
        print(f'[{sdk.namespace}] 等了 {WAIT_SUPPLY_S:.0f} 秒没等到任务机降落，继续下一个任务',
              flush=True)
    通知.received.clear()           # 复位，下一个任务还要再等一次
    通知.data = {}


def 通知已降落(sdk, teammate):
    """告诉侦察机"本阶段我已经落地了"。送不到不影响自己往下走。"""
    try:
        sdk.send_to_teammate(SUPPLY_LANDED_EVENT, timeout_s=30.0)
    except TeammateUnreachableError as exc:
        print(f'[{sdk.namespace}] 降落通知没送达 {teammate}：{exc}', flush=True)


def run_recon(sdk):
    """侦察机：三段连着做，中间在起飞点上空等任务机降落，自己最后才落地。"""
    # 三个收件箱都提前注册：僚机就位、任务机降落、任务机到待命点这三条事件都
    # 可能在侦察机还没走到对应那一步时就到达，而可靠事件通道是先回 ACK 再查
    # 处理函数，没注册的事件会被确认后丢弃（实测踩过）。
    任务机已降落 = 高楼.Notice()
    sdk.on_teammate_event(SUPPLY_LANDED_EVENT, 任务机已降落.on_event)
    任务机就位 = 高楼.Notice()
    sdk.on_teammate_event(巡检.STANDBY_EVENT, 任务机就位.on_event)
    僚机就位 = 编队.listen_standby(sdk)

    sdk.takeoff()                   # 自动播"侦察机起飞"，全程只起飞这一次

    # ① 编队飞行
    编队.leader_route(sdk, ROUTE, 僚机就位)
    return_home_hover(sdk)
    wait_supply_landed(sdk, 任务机已降落)

    # ② 地面火情
    found_ground = 地面.recon_search_and_report(sdk)
    return_home_hover(sdk)
    wait_supply_landed(sdk, 任务机已降落)

    # ③ 高层火情（定点巡检版）
    found_high = 巡检.recon_inspect_and_fire(sdk, 任务机就位)
    return_home_hover(sdk)

    sdk.land()                      # 三段都做完才降落，自动播"侦察机降落"
    sdk.play_sound_light('侦察机任务完成')
    print(f'[{sdk.namespace}] 地面火情{"已" if found_ground else "未"}发现，'
          f'高层火情{"已" if found_high else "未"}发现', flush=True)


def run_supply(sdk, teammate):
    """任务机：三段各自起降，每次落地后通知侦察机。"""
    # 两个火情通报的接收器提前注册，理由同上
    报告地面 = 地面.listen_for_report(sdk)
    报告高层 = 高楼.listen_for_report(sdk)

    编队.follower(sdk, SPACING_M)                 # 跟队飞完 -> 回起飞点降落
    通知已降落(sdk, teammate)

    地面.run_supply(sdk, teammate, 报告地面)      # 取弹投放 -> 返航降落
    通知已降落(sdk, teammate)

    巡检.run_supply(sdk, teammate, 报告高层)      # 待命点报到 -> 破窗后灭火 -> 降落
    通知已降落(sdk, teammate)


def main():
    ap = argparse.ArgumentParser(description='双机全流程（编队 + 地面火情 + 高层火情巡检版）')
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

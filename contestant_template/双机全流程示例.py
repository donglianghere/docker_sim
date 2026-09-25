#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机全流程：编队飞行 -> 地面火情 -> 高楼火情，一次跑完。

    bash 运行仿真.sh --task 双机全流程示例.py

一份代码两架飞机各起一个容器，按 --role 分工（运行脚本会传）：

  leader（NX01，侦察机/长机）
      ① 起飞带僚机飞编队航线 -> 回起飞点降落、等任务机降落
      ② 起飞弓字搜索地面火情、通报 -> 回起飞点降落、等任务机降落
      ③ 起飞逐栋绕飞高楼、对准着火点、通报、发射破窗弹 -> 回起飞点降落

  follower（NX02，任务机/僚机）
      ① 起飞跟队，航线飞完回起飞点降落
      ② 收到地面火情通报 -> 起飞取灭火弹 -> 精准降落物资点抓取 -> 飞火点投放 -> 降落
      ③ 收到高层火情通报 -> 起飞 -> 飞瞄准位置对准 -> 发射灭火弹 -> 降落

三条规则（用户 2026-09-23 定、2026-09-24 改了第二条）：
  · 每个任务结束两架都要回到自己的起飞点；
  · 两架**每段任务结束都降落**（原来是侦察机悬停等着，改成落地等——干等着
    没意义，天上少一架也更安全）；
  · 侦察机要**等任务机降落之后**才起飞做下一个任务。

各任务段的实现直接复用三个单任务示例（同一个文件夹，容器里一起挂到
/workspace），这里只负责串起来：
  编队飞行示例.py / 地面火情搜索示例.py / 高楼火情示例.py
想单独调某一段就直接跑对应的那个示例。

两架同时在天上时不用刻意错开位置，机间避让由飞控栈负责。
"""
import argparse

import 任务工具 as 工具

import 地面火情搜索示例 as 地面
import 编队飞行示例 as 编队
import 高楼火情绕飞版示例 as 高楼
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


def return_home_and_land(sdk):
    """回到起飞点上方，然后降落。起飞点就是自己局部系的原点。

    用户 2026-09-24 改的规则：原来是"回起飞点悬停着等任务机降落、全程只起飞
    一次"，改成**每段任务结束就落地**，下一段等任务机也落回来之后再起飞。理由
    是干等期间没事可做，落地等更合理——天上少一架飞机，也不用一直耗电占空域。
    """
    print(f'[{sdk.namespace}] 返回起飞点', flush=True)
    home = (0.0, 0.0, RETURN_AGL_M)
    try:
        # 转场段定高：这一段最长（从场地另一头飞回来），不钉住的话规划器高频
        # 重规划会把轨迹高度压得很低——用户 2026-09-24 实测高层段返航时看到过
        with sdk.fixed_altitude(RETURN_AGL_M):
            sdk.goto(*home)         # 远距离回程走规划器，有避障
    except GotoUnreachableError:
        pass
    sdk.goto_direct(*home)          # 最后一段收准，下一次起飞还是这个点
    # 机头恢复成起飞时的朝向：上一个任务可能把它锁在了对准火点的方向上，
    # 带着那个朝向落地、再起飞，下一个任务的画面朝向就不可预期了
    sdk.set_yaw_mode_constant(sdk.pretakeoff_yaw or sdk.get_current_yaw())
    工具.land_or_confirm(sdk)       # 自动播"侦察机降落"


def wait_supply_landed(sdk, 通知):
    """在起飞点悬停着等任务机落地。等不到也继续往下做，但要说清楚。"""
    print(f'[{sdk.namespace}] 已降落，在起飞点等任务机完成本阶段并降落…', flush=True)
    if 通知.wait(WAIT_SUPPLY_S):
        print(f'[{sdk.namespace}] 任务机已降落，开始下一个任务', flush=True)
    else:
        print(f'[{sdk.namespace}] 等了 {WAIT_SUPPLY_S:.0f} 秒没等到任务机降落，继续下一个任务',
              flush=True)
    通知.received.clear()           # 复位，下一个任务还要再等一次
    通知.data = {}


def 通知已降落(sdk, teammate):
    """告诉侦察机"本阶段我已经落地了"。送不到不影响自己往下走——侦察机那边
    等不到也会继续（见 wait_supply_landed）。"""
    try:
        sdk.send_to_teammate(SUPPLY_LANDED_EVENT, timeout_s=30.0)
    except TeammateUnreachableError as exc:
        print(f'[{sdk.namespace}] 降落通知没送达 {teammate}：{exc}', flush=True)


def run_recon(sdk):
    """侦察机：三个任务连着做，中间在起飞点上空等任务机降落，自己最后才落地。"""
    # 两个收件箱都提前注册：僚机就位、任务机降落这两条事件都可能在侦察机还没
    # 走到对应那一段时就到达，而可靠事件通道是先回 ACK 再查处理函数，没注册的
    # 事件会被确认后丢弃（实测踩过）。
    任务机已降落 = 高楼.Notice()
    sdk.on_teammate_event(SUPPLY_LANDED_EVENT, 任务机已降落.on_event)
    僚机就位 = 编队.listen_standby(sdk)

    # 每段任务各起飞一次，且**直接起到该段的巡航高度**——省掉"起飞到1米再爬"
    # 那一次纯垂直规划（2026-09-25 加）。自动播"侦察机起飞"。
    sdk.takeoff(height_m=编队.CRUISE_AGL_M)

    # ① 编队飞行：全程唯一开定高的一段。本项目默认 LOCALIZATION_SOURCE=uwb_imu，
    # 那套定位的 z 就是离地高度，所以"钉住高度"= 仿地飞行（见 SDK 里
    # set_fixed_altitude 的说明）。开关持续生效，用 try/finally 保证异常时也关掉。
    # ⚠️ 只对长机生效：僚机走 formation_follower_node 直发 position_cmd，
    # 不经过 traj_server。
    # 定高的 z 必须跟 goto() 同一套局部坐标系，不能直接传世界高度（见 SDK 说明）
    with sdk.fixed_altitude(sdk.world_to_local(0.0, 0.0, 编队.CRUISE_AGL_M)[2]):
        编队.leader_route(sdk, ROUTE, 僚机就位)
    return_home_and_land(sdk)
    wait_supply_landed(sdk, 任务机已降落)

    # ② 地面火情（任务机已落地，起飞做下一段）
    sdk.takeoff(height_m=地面.CRUISE_AGL_M)
    found_ground = 地面.recon_search_and_report(sdk)
    return_home_and_land(sdk)
    wait_supply_landed(sdk, 任务机已降落)

    # ③ 高楼火情
    sdk.takeoff(height_m=高楼.ORBIT_AGL_M)
    found_high = 高楼.recon_orbit_and_fire(sdk)
    return_home_and_land(sdk)       # 最后一段，落地即收工
    sdk.play_sound_light('侦察机任务完成')
    print(f'[{sdk.namespace}] 地面火情{"已" if found_ground else "未"}发现，'
          f'高层火情{"已" if found_high else "未"}发现', flush=True)


def run_supply(sdk, teammate):
    """任务机：三个任务各自起降，每次落地后通知侦察机。"""
    # 两个火情通报的接收器也提前注册，理由同上（侦察机可能在本机还在做上一个
    # 任务时就发来了下一个任务的通报）
    报告地面 = 地面.listen_for_report(sdk)
    报告高层 = 高楼.listen_for_report(sdk)

    编队.follower(sdk, SPACING_M)              # 跟队飞完 -> 回起飞点降落
    通知已降落(sdk, teammate)

    地面.run_supply(sdk, teammate, 报告地面)   # 取弹投放 -> 返航降落
    通知已降落(sdk, teammate)

    高楼.run_supply(sdk, teammate, 报告高层)   # 瞄准发射 -> 返航降落
    通知已降落(sdk, teammate)


def main():
    ap = argparse.ArgumentParser(description='双机全流程（编队 + 地面火情 + 高楼火情）')
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

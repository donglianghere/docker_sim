# -*- coding: utf-8 -*-
"""双机基础动作测试：起飞1.2m -> 悬停5秒 -> 前飞1.5m -> 悬停5秒 -> 回垫降落。

真机第一次通电、换场地、换参数之后先跑这个，再跑任务程序。它只验**最基础的
四件事**，不碰航线、不碰视觉、不碰避障：

  1. 两机都能解锁起飞到指定高度（1.2 m）
  2. 悬停稳不稳（位置保持，不漂不摆）
  3. 能不能按机头方向走一个确定的小位移（1.5 m，经规划器）
  4. 能不能正常降落上锁

两机跑的是同一段动作，各自独立执行，**没有编队、没有跨机握手** —— 所以
一架有问题不会连带另一架，便于单独判断是哪台的事。

**降落回各自的起降垫**（不是停在前方 1 米处）：末尾用 return_home()，它会
飞回自己的起飞点、按落点坐标核对、核对通过才播降落声光。1 米的回程短于
RETURN_HOME_FAR_M(2.0)，所以直接走精修段，不经规划器也不需要论证直线余量。
它内部用 land_or_confirm()，兜住"贴地了但 PX4 不报 Landing detected"那个老坑。

前飞**走规划器**（goto -> waypoint_queue -> ego_planner，有避障）。

⚠️ 但回程仍是直线：1.5 m 短于 RETURN_HOME_FAR_M(2.0)，return_home 不走远距离
那条规划器分支、直接进精修段。所以起飞点正前方那 1.5 米还是要空出来。

⚠️ **"前"是机身此刻的实际朝向**，不是地图上某个固定方向——takeoff 保持起飞前
的真实 yaw，两机摆放朝向不同就会各自朝不同方向飞。程序会把实测航向角打进
日志（`当前机头 +xx.x°`），起飞后先核一眼这个数再放手。

看什么：
  · 监视窗口里两条轨迹都应该是"原地 -> 前移1.5米 -> 退回原地"，悬停段应该是
    个点而不是一团；落点应该回到起飞点上
  · 日志末尾 return_home 会报落点是否在起降垫容差内——这一项是硬判据
  · 声光应该响四次：两机起飞各一次、两机降落各一次
用法： ./run_real.sh basic_test
"""
import math
import time

from contest_sdk import DroneSDK

TAKEOFF_AGL_M = 1.2      # 起飞高度。基础测试取低一点，出问题时余量大
HOVER_S = 5.0            # 每段悬停时长
FORWARD_M = 1.5          # 前飞距离


def _sequence(sdk, who, takeoff_event, land_event):
    """两机共用的动作序列。who 只用于日志，声光事件两机不同。"""
    sdk.progress(f'=== 基础动作测试（{who}）===')
    sdk.progress(f'起飞 {TAKEOFF_AGL_M} m -> 悬停 {HOVER_S:.0f}s -> '
                 f'前飞 {FORWARD_M} m -> 悬停 {HOVER_S:.0f}s -> 降落')

    sdk.takeoff(height_m=TAKEOFF_AGL_M)
    sdk.announce(takeoff_event)

    # 悬停就是"不发新指令"：px4ctrl 会保持最后一个设定点。这里不调 hold_at，
    # 因为那要世界坐标、依赖原点已锁；基础测试要尽量少依赖。
    sdk.progress(f'悬停 {HOVER_S:.0f} 秒（位置保持，看漂不漂）')
    time.sleep(HOVER_S)

    # 不用 sdk.step_forward()：那个内部走 goto_direct（直飞、不经规划器）。
    # 这里要**走规划器**，所以自己算落脚点再用 goto()——goto 走
    # waypoint_queue -> ego_planner_bridge -> ego_planner，有避障。
    #
    # 方向：yaw 是局部系航向角，0 弧度指局部 +x，所以 (cos, sin) 就是机头方向。
    # 这和 SDK 的 goto()/set_yaw_mode_constant() 是同一套约定。**注意"前"是
    # 机身此刻的实际朝向**（takeoff 会保持起飞前的真实 yaw），不是地图上某个
    # 固定方向——两机摆放朝向不同，就会各自朝不同方向飞。所以这里把实测
    # 航向角打出来，起飞后先看一眼这个数对不对，再让它走。
    cx, cy, cz = sdk.get_local_position()
    yaw = sdk.get_current_yaw()
    tx = cx + FORWARD_M * math.cos(yaw)
    ty = cy + FORWARD_M * math.sin(yaw)
    sdk.progress(f'当前机头 {math.degrees(yaw):+.1f}°（局部系，0°=+x）')
    sdk.progress(f'沿机头前飞 {FORWARD_M} m：局部 ({cx:.2f},{cy:.2f}) -> ({tx:.2f},{ty:.2f})')
    # 定高用**局部系 z**、而且等于本段 goto 的 z——不等的话永远判不到点。
    with sdk.fixed_altitude(cz):
        sdk.goto(tx, ty, cz)

    sdk.progress(f'再悬停 {HOVER_S:.0f} 秒')
    time.sleep(HOVER_S)

    # 声光交给 return_home 的 sound= 自己播，**不要在这里 announce**：
    # 它要先按落点坐标核对是否真落在起降垫上，核对通过才播（SDK 文件头有说明）。
    sdk.progress('回起降垫降落')
    ok = sdk.return_home(sound=land_event)
    sdk.progress(f'=== {who} 测试结束，落点{"在" if ok else "**不在**"}起降垫容差内 ===')


def leader(sdk: DroneSDK):
    """侦察机 NX01。"""
    _sequence(sdk, '侦察机', '侦察机起飞', '侦察机降落')


def follower(sdk: DroneSDK):
    """任务机 NX02。"""
    _sequence(sdk, '任务机', '任务机起飞', '任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=leader, follower=follower, description=__doc__)

# -*- coding: utf-8 -*-
"""双机基础动作测试：起飞 -> 悬停5秒 -> 前飞1米 -> 悬停5秒 -> 降落。

真机第一次通电、换场地、换参数之后先跑这个，再跑任务程序。它只验**最基础的
四件事**，不碰航线、不碰视觉、不碰避障：

  1. 两机都能解锁起飞到指定高度
  2. 悬停稳不稳（位置保持，不漂不摆）
  3. 能不能按机头方向走一个确定的小位移
  4. 能不能正常降落上锁

两机跑的是同一段动作，各自独立执行，**没有编队、没有跨机握手** —— 所以
一架有问题不会连带另一架，便于单独判断是哪台的事。

**降落回各自的起降垫**（不是停在前方 1 米处）：末尾用 return_home()，它会
飞回自己的起飞点、按落点坐标核对、核对通过才播降落声光。1 米的回程短于
RETURN_HOME_FAR_M(2.0)，所以直接走精修段，不经规划器也不需要论证直线余量。
它内部用 land_or_confirm()，兜住"贴地了但 PX4 不报 Landing detected"那个老坑。

⚠️ 前飞和回程都是直线（step_forward 走 goto_direct，1 米回程走精修段），
**不避障**。所以起飞点正前方那一米必须是空的。

看什么：
  · 监视窗口里两条轨迹都应该是"原地 -> 前移1米 -> 退回原地"，悬停段应该是
    个点而不是一团；落点应该回到起飞点上
  · 日志末尾 return_home 会报落点是否在起降垫容差内——这一项是硬判据
  · 声光应该响四次：两机起飞各一次、两机降落各一次
用法： ./run_real.sh basic_test
"""
import time

from contest_sdk import DroneSDK

TAKEOFF_AGL_M = 1.2      # 起飞高度。基础测试取低一点，出问题时余量大
HOVER_S = 5.0            # 每段悬停时长
FORWARD_M = 1.0          # 前飞距离


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

    sdk.progress(f'沿机头前飞 {FORWARD_M} m')
    sdk.step_forward(FORWARD_M, what='前方1米')

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

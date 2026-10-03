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

⚠️ **降落点在起飞点正前方约 1 米**，不是起降垫上。跑之前确认那一米范围内
没人没障碍。要让它回起降垫降落，把末尾的 sdk.land() 换成
sdk.return_home(direct=True)。

⚠️ 前飞用 step_forward()，它内部走 goto_direct（直飞，不经规划器）。1 米的
空旷位移这样最直接；但也因此**周围必须是空的**，它不会避障。

看什么：
  · 监视窗口里两条轨迹都应该是"原地 -> 前移1米 -> 原地"，悬停段应该是个点
    而不是一团
  · 声光应该响四次：两机起飞各一次、两机降落各一次
  · 日志里不该有 LandTimeoutError（贴地时测距仪卡在最小量程、PX4 不报
    Landing detected 的老问题，见 archive/utils.py 的说明）

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

    sdk.progress('就地降落（注意：这里不是起降垫，是起飞点前方约 1 米）')
    sdk.land()
    sdk.announce(land_event)
    sdk.progress(f'=== {who} 测试结束 ===')


def leader(sdk: DroneSDK):
    """侦察机 NX01。"""
    _sequence(sdk, '侦察机', '侦察机起飞', '侦察机降落')


def follower(sdk: DroneSDK):
    """任务机 NX02。"""
    _sequence(sdk, '任务机', '任务机起飞', '任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=leader, follower=follower, description=__doc__)

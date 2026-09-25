#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务层的小兜底：不让单点故障打死整轮任务。

这个文件不是"示例"，是被各个示例共用的几个函数。

为什么需要 `land_or_confirm()`（2026-09-25 实测踩的坑）：
一次降落里，飞机真值 z=0.05 米、**已经实实在在落在地上了**，但 PX4 没报
"Landing detected"（贴地时测距雷达读数卡在 0.28~0.34 米的最小量程上，PX4 认为它
还在空中），pt4ctrl 的 AUTO_LAND 只在 `ExtendedState==LANDED_STATE_ON_GROUND` 时
才发解锁指令（`PX4CtrlFSM.cpp:319`），于是一直没解锁；`sdk.land()` 等 30 秒没等到
`armed=false` 就抛 `LandTimeoutError`，僚机程序**直接退出**，长机在那边一直等它，
整轮任务就此结束。同一轮里长机的雷达读数下到 0.13 米就正常解锁了——所以这是个
时好时坏的边缘条件，不是必现故障，但代价是整轮任务。

判据不靠"多等几秒"，而是**看飞机自己到底落没落地**：离地高度低于 LANDED_AGL_M
且基本不动，就认为已落地、继续往下做。真机上同样成立（同一颗雷达、同一条话题）。
"""
import time

from contest_sdk.exceptions import DetectionTimeoutError, LandTimeoutError

LANDED_AGL_M = 0.45      # 低于这个离地高度就认为已经贴地（雷达最小量程约 0.3 米）
LANDED_MOVE_M = 0.10     # 这段时间内位移小于这个值才算"停住了"
LANDED_WATCH_S = 2.0


def land_or_confirm(sdk) -> bool:
    """降落；`land()` 超时时再自己确认一次是不是其实已经落地了。

    Returns:
        True=已落地（正常解锁，或超时但确认贴地静止）；False=确实还在空中。
    """
    try:
        sdk.land()
        return True
    except LandTimeoutError as exc:
        print(f'[{sdk.namespace}] 降落没等到解锁确认（{exc}），自己核一下是不是已经落地',
              flush=True)

    x0, y0, z0 = sdk.get_local_position()
    time.sleep(LANDED_WATCH_S)
    x1, y1, z1 = sdk.get_local_position()
    moved = ((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2) ** 0.5
    try:
        agl = sdk.get_agl(timeout=2.0)
    except DetectionTimeoutError:
        agl = z1                      # 读不到雷达就拿里程计的 z 顶上（uwb_imu 下它就是离地高度）

    if agl <= LANDED_AGL_M and moved <= LANDED_MOVE_M:
        print(f'[{sdk.namespace}] 已确认落地：离地 {agl:.2f} m、{LANDED_WATCH_S:.0f} 秒内'
              f'只动了 {moved:.2f} m（飞控没解锁，但飞机确实在地上），继续后续任务', flush=True)
        return True
    print(f'[{sdk.namespace}] 没能确认落地：离地 {agl:.2f} m、位移 {moved:.2f} m，还在空中',
          flush=True)
    return False

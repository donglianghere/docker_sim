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
import math
import time

from contest_sdk.exceptions import ContestSdkError, DetectionTimeoutError, LandTimeoutError

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


# ---- 转场段的统一动作（用户 2026-09-25 要求：先锁 yaw，到位后等 2 秒再飞）----
TRANSFER_SETTLE_S = 2.0      # 机头到位后再稳这么久才起步
TRANSFER_YAW_TIMEOUT_S = 15.0


def transfer_to(sdk, x, y, z, face_xy=None, settle_s=TRANSFER_SETTLE_S):
    """转场到 (x, y, z)：**先把机头锁好、到位后等 settle_s 秒，再定高飞过去**。

    统一三件事，原来各段写法不一致（有的先锁 yaw 再飞、有的飞完才锁、有的一路
    对着上一个目标）：
    · 朝向：`face_xy=None` 就**锁住当前朝向**（不转、不等，用户 2026-09-25：没有
      明确朝向要求的水平转场，锁定当前 yaw 就行）；给了点就悬停转到对着那个点、
      到位后再等 settle_s 秒才起步（进场瞄准这类，到了就要能直接看目标）；
    · 高度：整段 `fixed_altitude` 钉在 z 上，规划器不能把它压下去。

    Args:
        x/y/z: 目标点，这架飞机自己的局部坐标系（跟 `goto()` 一致）。
        face_xy: 机头要对着的点（局部系）；None=对着前进方向。
    Raises:
        跟 `sdk.goto()` 一样——调用方自己决定怎么处理 GotoUnreachableError。
    """
    if face_xy is None:
        # 没有朝向要求：**锁住当前朝向**就行，不必转、也不必等（用户 2026-09-25）。
        # 锁一下是必要的——不锁的话 traj_server 可能还停在上一段的 POINT 模式，
        # 机头会在飞行途中跟着目标点慢慢转。
        sdk.set_yaw_mode_constant(sdk.get_current_yaw())
    else:
        # 有朝向要求（进场瞄准、待命点这种，到了就要能直接看目标）：悬停转到位，
        # 再稳 settle_s 秒才起步，不允许一边偏航一边飞。
        if not sdk.face_point(face_xy[0], face_xy[1], timeout=TRANSFER_YAW_TIMEOUT_S):
            print(f'[{sdk.namespace}] 转场前机头没转到位（仍继续，注意画面朝向可能不对）',
                  flush=True)
        time.sleep(settle_s)
    with sdk.fixed_altitude(z):
        sdk.goto(x, y, z)


# ---- 抵近发射（用户 2026-09-26 定的流程）--------------------------------
# 规划器管不到最后这一段：立柱膨胀边界在离柱心 1.10 m（半宽0.5+膨胀0.6），
# 零代价边界在 2.10 m（再加 dist0=1.0）。飞进膨胀区，ego-planner 的
# rebound_optimize() 会对"起点在障碍里"直接拒绝规划、FSM 原地自旋（实测过的
# 死锁）。所以最后一段必须脱离规划器、用直飞走。
#
# 整段动作的次序：站在**正对火情那条边的水平中点**上、机头朝火情 -> 通报队友
# -> 原地等队友到待命点 -> 沿"中点->火情"这条直线进到发射点 -> 发射 -> 原路退回
# **出发时那个中点** -> 退到位之后才通知队友 -> 再启动规划器返航。
# 通知放在退回之后是关键：队友一收到就会进场，这时本机必须已经让出走廊。
#
# 进/退方向直接用"出发点->火情"这条线，**不再去推立面法线**（用户 09-26 要求）：
# 中点本来就正对着立面，这条线就是法线；而推算法线要用检测解出来的火点坐标，
# 那个坐标横向偏个 0.4 m，3 米外就把方向带歪 27°、把落点甩出 1.4 m（09-26 实测）。
FIRE_STANDOFF_M = 1.2      # 发射点：直飞进到离火情这么近（标志约 159 像素宽）
CLOSE_IN_TIMEOUT_S = 30.0
# 这个距离的下限：桨尖半径 0.38 m（Iris 电机轴 ±0.13/±0.22，加桨约 0.13）
# + 直飞到点判据 0.30 m + 余量 0.20 m ≈ 0.9 m，取 1.2 m 留更多余量。


def close_in_and_fire(sdk, fire_xy, z, fire_fn):
    """抵近发射：从当前位置直飞进到 FIRE_STANDOFF_M -> 发射 -> 原路退回出发点。

    出发点就是调用时飞机所在的位置（按流程是正对火情那条边的水平中点），退回时
    原样飞回去，进退是同一条直线——直进直出，路径上任何一点离立柱都不比终点近。

    **这中间绝不能调 `goto()`**：飞机此时在膨胀区内，规划器会拒绝规划并原地自旋。
    退回出发点之后才允许恢复用规划器。

    Args:
        fire_xy: 着火点的局部坐标 (x, y)。
        z: 保持的高度（局部系）。
        fire_fn: 发射动作，无参可调用对象（破窗弹/灭火弹各自传自己的）。
    """
    cx, cy, _ = sdk.get_local_position()
    back = (cx, cy)                       # 出发点，打完退回这儿
    dx, dy = cx - fire_xy[0], cy - fire_xy[1]
    n = math.hypot(dx, dy)
    if n < FIRE_STANDOFF_M:
        print(f'[{sdk.namespace}] 当前离火情只有 {n:.2f} m，已经比发射点还近，直接发射',
              flush=True)
        sdk.face_point(*fire_xy)
        fire_fn()
        return
    ux, uy = dx / n, dy / n
    fire_pos = (fire_xy[0] + ux * FIRE_STANDOFF_M, fire_xy[1] + uy * FIRE_STANDOFF_M)
    print(f'[{sdk.namespace}] 抵近发射：从 ({cx:.2f}, {cy:.2f}) 离火情 {n:.2f} m '
          f'-> 直飞进到 {FIRE_STANDOFF_M:.1f} m（不经过规划器）', flush=True)
    sdk.face_point(*fire_xy)              # 锁机头，直飞途中不再转
    try:
        sdk.goto_direct(fire_pos[0], fire_pos[1], z, timeout=CLOSE_IN_TIMEOUT_S)
    except ContestSdkError as exc:
        print(f'[{sdk.namespace}] 进场没走到位（{exc}），就当前位置发射', flush=True)
    fire_fn()
    print(f'[{sdk.namespace}] 发射完毕，原路直飞退回出发点 ({back[0]:.2f}, {back[1]:.2f})',
          flush=True)
    try:
        sdk.goto_direct(back[0], back[1], z, timeout=CLOSE_IN_TIMEOUT_S)
    except ContestSdkError as exc:
        print(f'[{sdk.namespace}] 退出没走到位（{exc}）——注意此时可能仍在膨胀区内，'
              f'接下来的 goto() 有卡住风险', flush=True)

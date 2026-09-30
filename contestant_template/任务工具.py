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

from contest_sdk.exceptions import (
    ActionFailedError,      # fire_launcher 用：舵机没配好时只打警告不中断
    ContestSdkError,
    DetectionTimeoutError,
    LandTimeoutError,
)

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


# ---------------------------------------------------------------------------
# 2026-09-30 从两个**已过时**的示例里搬过来的通用动作。
# 原来 descend_onto 在《地面火情搜索示例》、fire_launcher 在《高楼火情绕飞版
# 示例》里，而那两个文件的任务流程用的还是老场景 fire_drill_room 的坐标
# （SUPPLY_POINT=(-4.0,-6.0)、PILLARS/BUILDINGS=[(4.5,7),(-4.5,7),(0,0)]，
# 原点在房间中心、带负坐标），跟样题场景对不上，已经不该再被当例子用。
# 但任务2/任务3 一直从里面 import 这两个函数——等于"过时文件里扣着还在用的
# 代码"，谁哪天把过时示例删了，任务脚本当场就断。
# 这两个函数本身跟场景无关（只依赖舵机 PWM、下降步长、降落判据这类调参常量，
# 没有任何坐标），搬到这个真正的公共工具箱里才是它们该在的位置。
# ---------------------------------------------------------------------------

# --- 边瞄准边降落到底（原属《地面火情搜索示例》）---
PRECISION_LAND_S = 30.0      # 边瞄准边降落这一段的总时限，到点就交给普通降落
DESCENT_STEP_M = 0.8         # 每一步下降多少：降一点就重新解算一次目标位置
                             # 2026-09-30 用户要求 0.4 -> 0.8，物资点那次降落
                             # 从 42.9 秒降到 33.8 秒。代价是每步之间才重新对准
                             # 一次，步子迈大了中间修正机会变少，落点精度要盯住。
HANDOFF_AGL_M = 0.7          # 降到离地这么高就交给普通降落（再低下视相机看不全标志）
HANDOFF_TOL_M = 0.15         # 到交接高度附近就算到了：悬停本身有零点几十厘米的
                             # 起伏，死等"严格低于交接高度"会一直卡在上面空耗
                             # （实测卡满 30 秒）

def descend_onto(sdk, tag, what):
    """边瞄准边降落：每降一小段就把目标位置重新解算一次，直接命令"目标正上方、
    低一点"那个位置——水平修正和下降在同一条指令里完成；降到交接高度后交给
    普通降落收尾。

    为什么不用 precision_land_and_confirm()：那是"对准一点、下降一点、再对准"
    的分级下降，从 2.5 米下来要 40 秒以上，30 秒的时限内根本走不完，每次都会
    走超时兜底（2026-09-23 实测），等于精度只做了一半。这里换成连续修正，同一
    时间既在对准也在下降，30 秒够用；最后 0.7 米交给 land()，那一段本来就只能
    垂直下降，再修也无意义。
    """
    deadline = time.monotonic() + PRECISION_LAND_S
    while time.monotonic() < deadline:
        _, _, z = sdk.get_local_position()
        try:
            t = sdk.locate_target(tag, timeout=2.0, samples=3)
        except DetectionTimeoutError:
            print(f'[{sdk.namespace}] 下降中看不到{what}了，就地转普通降落', flush=True)
            break
        agl = z - t.z                       # t.z 是解算出的地面高度
        if agl <= HANDOFF_AGL_M + HANDOFF_TOL_M:
            print(f'[{sdk.namespace}] 已降到离地 {agl:.2f} m，交给普通降落', flush=True)
            break
        next_agl = max(HANDOFF_AGL_M, agl - DESCENT_STEP_M)
        print(f'[{sdk.namespace}] 对准{what} ({t.x:.2f}, {t.y:.2f}) 并降到离地 '
              f'{next_agl:.2f} m（当前 {agl:.2f} m，{t.samples}帧离散 {t.spread_m:.2f}m）',
              flush=True)
        sdk.goto_direct(t.x, t.y, t.z + next_agl)
    else:
        print(f'[{sdk.namespace}] 边瞄准边降落用满 {PRECISION_LAND_S:.0f} 秒，转普通降落',
              flush=True)
    land_or_confirm(sdk)               # 最后一段普通降落


# --- 发射机构：装填 -> 发射 -> 复位（原属《高楼火情绕飞版示例》）---
LOAD_PWM = 800               # 装填/复位
FIRE_PWM = 2000              # 发射
SERVO_TRAVEL_S = 2.0         # 舵机没有位置反馈，只能等

def fire_launcher(sdk, label):
    """发射：把发射机构的舵机推到松开位置，等它到位，再复位装填。
    没配舵机的飞机（见 sdk.servos）只打印提示，不让流程中断。"""
    if not sdk.servos:
        print(f'[{sdk.namespace}] {label}：这架飞机没有配置舵机，跳过（见 SDK 的 SERVO_CONFIG）',
              flush=True)
        return
    try:
        sdk.set_servos({s: FIRE_PWM for s in sorted(sdk.servos)})
        time.sleep(SERVO_TRAVEL_S)
        sdk.set_servos({s: LOAD_PWM for s in sorted(sdk.servos)})
        time.sleep(SERVO_TRAVEL_S)
    except (ActionFailedError, ValueError) as exc:
        print(f'[{sdk.namespace}] {label}：舵机没动（{exc}）', flush=True)

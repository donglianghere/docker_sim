#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2 单项测试：地面火情侦查 -> 取物资 -> 投放灭火弹 -> 编队返航。

流程（2026-09-29 用户指定）：

侦察机 NX01                          任务机 NX02
─────────────────────────────────    ─────────────────────────────────
起飞
航点飞行 A -> B -> C -> G -> E
  到 G 打开下视相机开始找地面火情
发现火情 -> 移动对准
通报火情坐标 ────────────────────►  收到 -> 起飞 -> 悬停 2 秒
飞回 G 点悬停等待                     飞物资点 -> 瞄准降落抓取
                                      起飞 -> 悬停 2 秒
                                      飞火情点 -> 对准通报坐标 -> 投放灭火弹
收到"已投放" ◄──────────────────── 通知已投放
就地（G 点）开始编队                   （在原地等长机发航线）
  ↓ 从 G 开始编队，NX01 在前 NX02 在后，规则跟《编队飞行示例》完全一样
  G -> D -> A，长机飞回自己起飞点上空时解散
NX01 悬停在 A 点待命                      NX02 降落在自己起降点

两个要点：

1. **编队段直接复用 `编队飞行示例` 的函数**，不另写一套。用户要求"和编队飞行
   这一段的要求一样"——复制一份代码迟早两边分叉，所以那边把**纯空中**的编队
   段拆成了 `leader_route()` / `follow_formation()`（起降不属于编队内容），
   这里直接调。协调转弯、解散判据、间距控制全都是同一份实现。

2. **所有跨机事件在脚本最开头就全部注册**。可靠事件通道是"先 ACK 再分发"，
   没注册处理函数的事件会被确认后丢弃——等用到了才注册必然丢事件。

3. **降落抓取直接复用《地面火情搜索示例》的 `descend_onto()`**，不自己写一套，
   也不用 SDK 的 `precision_land_and_confirm()`——后者分级下降太慢，30 秒时限
   内走不完（那份文件的注释里记着 2026-09-23 的实测）。
"""
import argparse
import math
import time

import 地面火情搜索示例 as 地面
import 编队飞行示例 as 编队
import 任务工具 as 工具
from contest_sdk import DroneSDK
from contest_sdk.exceptions import ActionFailedError

CRUISE_AGL_M = 编队.CRUISE_AGL_M      # 2.0，跟编队段保持同一个巡航高度
SPACING_M = 4.0

ROUTE_A = (3.0, 3.0)
ROUTE_B = (3.0, 22.0)
ROUTE_C = (17.0, 22.0)
ROUTE_G = (17.0, 16.0)
ROUTE_E = (10.0, 16.0)
ROUTE_D = (17.0, 3.0)
SUPPLY_XY = (10.0, 8.0)               # 物资点，AprilTag ID0

GROUND_FIRE = 'apriltag:2'            # 地面火情标识
SUPPLY_TAG = 'apriltag:0'

# 跨机事件
EV_FIRE = 'ground_fire_found'         # NX01 -> NX02，带火情世界坐标
EV_DROPPED = 'extinguisher_dropped'   # NX02 -> NX01

GRAB_PWM = 800                        # 抓紧（真机实测值，见 project_nx02_servos 记录）
DROP_PWM = 2000                       # 松开
SERVO_TRAVEL_S = 2.0                  # 舵机没有位置反馈，只能等
HOVER_AFTER_TAKEOFF_S = 2.0           # 用户要求：起飞后悬停 2 秒
DETECT_POLL_S = 0.5
FIRE_WAIT_S = 300.0
DROP_WAIT_S = 420.0


class _Inbox:
    """一个事件一个信箱。必须在对方可能发之前就创建（见模块 docstring 第 2 点）。"""

    def __init__(self, sdk, name):
        import threading
        self.name = name
        self.data = {}
        self._ev = threading.Event()
        sdk.on_teammate_event(name, self._on)

    def _on(self, **kw):
        self.data = kw
        self._ev.set()

    def wait(self, timeout):
        if not self._ev.wait(timeout):
            raise TimeoutError(f'等事件 {self.name} 超过 {timeout:.0f} 秒')
        return self.data


def _drive_servos(sdk, pwm, label):
    """抓取/投放机构。仿真里飞控不一定配了舵机输出，动不了就打印、继续飞完流程。"""
    try:
        sdk.set_servos({s: pwm for s in sorted(sdk.servos)})
    except (ActionFailedError, ValueError) as exc:
        print(f'[{sdk.namespace}] {label}：舵机没动（{exc}）——真机需要飞控配好 MAIN7/MAIN9',
              flush=True)
        return
    time.sleep(SERVO_TRAVEL_S)
    print(f'[{sdk.namespace}] {label}完成', flush=True)


def _goto_world(sdk, wx, wy, what, z_agl=CRUISE_AGL_M):
    """飞到某个世界坐标上方，全程锁高（走规划器，有避障）。"""
    lx, ly, lz = sdk.world_to_local(wx, wy, z_agl)
    print(f'[{sdk.namespace}] 飞往{what} ({wx:.2f}, {wy:.2f})', flush=True)
    with sdk.fixed_altitude(lz):
        sdk.goto(lx, ly, lz)


# ----------------------------------------------------------------------------
# 侦察机 NX01
# ----------------------------------------------------------------------------
def recon(sdk):
    dropped = _Inbox(sdk, EV_DROPPED)      # 先注册，后面才可能收到

    sdk.takeoff(height_m=CRUISE_AGL_M)
    # 不用显式播'起飞'：SDK 的 takeoff() 自己会按角色播一次
    # （capabilities.py 的 _play_role_sound_light），再播就是第二遍——
    # 2026-09-30 用户发现"侦查机起飞发了两遍"，间隔 22 秒正是起飞时长。

    # ---- 航点飞行 A B C G，到 G 开始下视搜索 ----
    for wp, name in ((ROUTE_A, 'A'), (ROUTE_B, 'B'), (ROUTE_C, 'C'), (ROUTE_G, 'G')):
        _goto_world(sdk, wp[0], wp[1], f'航点{name}')
    # 不需要"切相机"：前视/下视是两个独立的固定安装相机，各自一路话题，
    # 检测时用 camera='down' 选那一路就行（set_camera_view 已于 2026-09-30 删除，
    # 它发的是给一个当前模型里根本不存在的可动关节的指令，纯空转）。
    print(f'[{sdk.namespace}] 已到 G 点，开始用下视相机搜索地面火情', flush=True)

    # ---- G -> E 边飞边找 ----
    # goto 是阻塞的，所以先把目标发出去，在后台线程里飞，主线程盯检测；
    # 一发现就 cancel_goto 停在当场，避免飞过头。
    import threading
    ex, ey, ez = sdk.world_to_local(ROUTE_E[0], ROUTE_E[1], CRUISE_AGL_M)
    leg_done = threading.Event()

    def _fly_leg():
        try:
            with sdk.fixed_altitude(ez):
                sdk.goto(ex, ey, ez)
        except Exception as exc:
            print(f'[{sdk.namespace}] G->E 航段结束（{exc}）', flush=True)
        finally:
            leg_done.set()

    threading.Thread(target=_fly_leg, daemon=True).start()

    fire_local = None
    t0 = time.time()
    while time.time() - t0 < FIRE_WAIT_S:
        try:
            sdk.wait_for_detection(GROUND_FIRE, timeout=DETECT_POLL_S, camera='down')
        except Exception:
            if leg_done.is_set():
                break            # 整条 G->E 飞完都没看到
            continue
        print(f'[{sdk.namespace}] 发现地面火情，停车对准', flush=True)
        sdk.cancel_goto()
        time.sleep(1.0)          # 等刹停，别带着速度去对准
        break

    # ---- 对准 ----
    # center_on_target 让 precision_servo_node 把目标居中（只居中不下降），
    # 再用 locate_target 拿目标在地面的实际坐标。
    try:
        sdk.center_on_target(GROUND_FIRE, timeout=40.0)
        fire_local = sdk.locate_target(GROUND_FIRE, timeout=5.0)
    except Exception as exc:
        print(f'[{sdk.namespace}] 对准/解算失败（{exc}）', flush=True)

    if fire_local is None:
        raise RuntimeError('没能定位地面火情，任务2 中止')
    fx, fy = sdk.local_to_world(fire_local.x, fire_local.y, 0.0)[:2]
    print(f'[{sdk.namespace}] 地面火情世界坐标 ({fx:.2f}, {fy:.2f})', flush=True)
    sdk.play_sound_light('侦察机发现地面火情')

    # ---- 通报 ----
    sdk.send_to_teammate(EV_FIRE, x=float(fx), y=float(fy))
    print(f'[{sdk.namespace}] 已通报任务机', flush=True)

    # ---- 回 G 点悬停等待 ----
    # 2026-09-29 用户改要求：等待点就是 **G 点**，不再是"沿机头前移若干米"。
    # 好处是等待位置确定、可复现，而且编队段本来就从 G 起步，等完直接入列，
    # 不用再飞一趟。
    _goto_world(sdk, ROUTE_G[0], ROUTE_G[1], 'G点（等待位）')
    print(f'[{sdk.namespace}] 已在 G 点悬停，等任务机投放灭火弹', flush=True)

    dropped.wait(DROP_WAIT_S)
    print(f'[{sdk.namespace}] 任务机已投放灭火弹，就地开始编队返航', flush=True)

    # start_xy 要的是**世界坐标**（航线本身就是世界系）。飞机此刻就在 G，
    # 直接用 ROUTE_G——早先传 get_local_position() 的局部坐标，被当成世界坐标
    # 用，站位点算到了 3# 楼那一片、飞不过去（2026-09-29 实测）。
    编队.leader_route(sdk, [ROUTE_G, ROUTE_D], spacing_m=SPACING_M,
                      start_xy=ROUTE_G, final_xy=ROUTE_A)
    编队._hold_at_point_a(sdk, ROUTE_A)
    sdk.play_sound_light('侦察机任务完成')


# ----------------------------------------------------------------------------
# 任务机 NX02
# ----------------------------------------------------------------------------
def supply(sdk):
    # 三个事件全部先注册（见模块 docstring 第 2 点）
    fire = _Inbox(sdk, EV_FIRE)
    route_done = 编队._Inbox(sdk, 编队.ROUTE_DONE)
    route_plan = 编队._Inbox(sdk, 编队.ROUTE_PLAN)

    print(f'[{sdk.namespace}] 等侦察机通报地面火情…', flush=True)
    d = fire.wait(FIRE_WAIT_S)
    fx, fy = float(d['x']), float(d['y'])
    print(f'[{sdk.namespace}] 收到火情坐标 ({fx:.2f}, {fy:.2f})', flush=True)

    sdk.takeoff(height_m=CRUISE_AGL_M)
    # 不用显式播'起飞'：SDK 的 takeoff() 自己会按角色播一次
    # （capabilities.py 的 _play_role_sound_light），再播就是第二遍——
    # 2026-09-30 用户发现"侦查机起飞发了两遍"，间隔 22 秒正是起飞时长。
    time.sleep(HOVER_AFTER_TAKEOFF_S)

    # ---- 物资点：边瞄准边降落到底，抓取 ----
    # 直接复用《地面火情搜索示例》里验证过的 descend_onto()——它的注释写得很清楚：
    # **不要用 precision_land_and_confirm()**，那是"对准一点、下降一点、再对准"的
    # 分级下降，从 2.5 m 下来要 40 秒以上、30 秒时限内走不完，每次都走超时兜底
    # （2026-09-23 实测），等于精度只做了一半。descend_onto 是连续修正，同一时间
    # 既对准也下降，最后 0.7 m 交给普通降落。
    _goto_world(sdk, SUPPLY_XY[0], SUPPLY_XY[1], '物资点')
    地面.descend_onto(sdk, SUPPLY_TAG, '灭火弹')
    print(f'[{sdk.namespace}] 已降落在物资点，开始抓取', flush=True)
    sdk.play_sound_light('任务机抓取灭火弹')
    _drive_servos(sdk, GRAB_PWM, '抓取')

    sdk.takeoff(height_m=CRUISE_AGL_M)
    time.sleep(HOVER_AFTER_TAKEOFF_S)

    # ---- 火情点：对准通报坐标，投放 ----
    _goto_world(sdk, fx, fy, '地面火情点')
    try:
        sdk.center_on_target(GROUND_FIRE, timeout=40.0)
        print(f'[{sdk.namespace}] 已对准地面火情', flush=True)
    except Exception as exc:
        print(f'[{sdk.namespace}] 没对上火情标识（{exc}），按通报坐标投放', flush=True)
    sdk.play_sound_light('任务机投放灭火弹')
    _drive_servos(sdk, DROP_PWM, '投放')
    sdk.send_to_teammate(EV_DROPPED)
    print(f'[{sdk.namespace}] 已通知侦察机', flush=True)

    # ---- 编队返航 ----
    # 只调编队的**空中段**：起降不是编队内容（用户 2026-09-29 明确）。
    # 任务机此刻已经在空中（刚投完弹），入列 -> 跟队 -> 解散，然后自己降落。
    # 灭火完毕就地入列：此刻任务机就在长机后方约一个间距处，不用再飞站位点
    编队.follow_formation(sdk, SPACING_M, inbox=route_done, plan=route_plan,
                          goto_station=False)
    编队._land_at_pad(sdk)
    sdk.play_sound_light('任务机已降落')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--namespace', required=True)
    # 运行器（运行仿真.sh）传的是 leader/follower，这里同时接受语义化的
    # recon/supply 两个别名——长机=侦察机、僚机=任务机。
    ap.add_argument('--role', required=True,
                    choices=('leader', 'follower', 'recon', 'supply'))
    ap.add_argument('--teammate', required=True)
    ap.add_argument('--spacing', type=float, default=SPACING_M)
    args = ap.parse_args()

    sdk_role = 'recon' if args.role in ('leader', 'recon') else 'supply'
    sdk = DroneSDK(namespace=args.namespace, role=sdk_role,
                   teammate_namespace=args.teammate)
    try:
        if args.role in ('leader', 'recon'):
            recon(sdk)
        else:
            supply(sdk)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()

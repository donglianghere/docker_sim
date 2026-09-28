#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机一字纵队编队飞行。两机跑同一份代码，靠 --role 区分。

    python3 编队飞行示例.py --namespace NX01 --role leader  --teammate NX02 \
        --route "7,-9.5 7,9.5 -7,9.5 -7,-9.5" --spacing 3.5
    python3 编队飞行示例.py --namespace NX02 --role follower --teammate NX01 --spacing 3.5

航点是世界坐标 (x, y)，至少两个，按顺序飞。僚机不需要知道航线，它沿长机
实际飞过的轨迹走，沿轨迹间距不小于 --spacing（这是下限，不是要死守的值）。

长机每一段都是"**先悬停转向、转到位再前飞**"：在当前点（第一段是起飞点）把机头
转到"当前点指向下一个航点"的航向角，转到位之后整条航段锁死这个角度，到了下一个
航点再转下一段的航向。僚机不走这套——它的机头取自己航迹的切线方向（一字纵队跟随
的天然朝向，见 formation_follower_node），不需要也不应该跟长机的分段航向同步。

声光反馈：起飞/降落由 SDK 自动播（长机="侦察机"、僚机="任务机"），这里只在
落地后各补一条。需要地面站上的声光常驻程序在运行（start_sound_light_server.sh），
没运行也不影响飞行，只会打一行警告。
"""
import argparse

import 任务工具 as 工具
import math
import time

from contest_sdk import DroneSDK

# 巡航离地高度。2026-09-24 调过两次：仿地模块改成坡道后，1.5 米时飞机离顶面
# 太近（小于规划器 0.6 米的障碍物膨胀半径），规划器会把坡道当障碍绕开、看不到
# 起伏，所以先提到 2.5；后来坡道高度定为 0.5 米，2.0 米就够了（离顶面 1.5 米），
# 低一点起伏在画面上也更明显。
CRUISE_AGL_M = 2.0
# 长机等僚机就位的上限。必须大于僚机最坏情况的起飞耗时（机载等定位收敛的
# preflight_timeout_s=150秒 + 起飞动作本身60秒），否则僚机还在正常起飞、
# 长机就先判超时终止了。
STANDBY_WAIT_S = 300.0
ROUTE_DONE_WAIT_S = 600.0     # 僚机等航线飞完的上限
TURN_TIMEOUT_S = 15.0         # 每个航点转到航向角的上限（转 180° 实测十几秒）
TURN_TOL_DEG = 5.0            # 差这么多度以内算转到位
# ---- 协调转弯（2026-09-28 用户要求：拐点处不停留）----
# 原来每个航点都"先悬停把机头转到位、再前飞"。实测这是整条航线最大的一个
# 扰动源：转 90° 要 3 秒，这 3 秒里飞机在原地飘（run36 实测冲过航点 0.5 m
# 之后倒退 2.2 m 才停住），既制造了一个零速窗口，又在长机轨迹上留下一个
# 2.2 m 的回钩——而僚机是沿长机实飞轨迹跟随的，会把这个回钩原样复刻一遍。
# 现在改成：机头**连续限速**地转向当前航段方向，飞机不停；快到拐点时提前
# 起转，让转弯跨在拐角两边（真正的协调转弯）。
# 为什么不用 sdk.set_yaw_mode_velocity()（"机头跟着速度方向"）：SDK 里明确
# 警告过，那套算法急转弯时要求 yaw 瞬时大幅跳变，会跟位置/高度控制抢推力
# 分配，仿真掉高、真机炸过机。这里仍然走 set_yaw_mode_constant，只是把那个
# "constant"每拍按限速挪一点——效果相同，没有瞬时跳变。
YAW_SLEW_DPS = 60.0           # 机头最大偏航角速度（度/秒）
YAW_TICK_S = 0.1              # 偏航指令刷新周期
YAW_ANTICIPATE_M = 2.5        # 离拐点还有这么远就开始转下一段的航向
YAW_SEND_EPS = math.radians(0.5)   # 角度变化小于这个就不下发，免得刷屏
YAW_LEG_ADVANCE_M = 1.5       # 离本段终点这么近就算进入下一段（转弯线程自己推进）
READY = 'formation_standby'   # 僚机 -> 长机
ROUTE_DONE = 'route_done'     # 长机 -> 僚机
ROUTE_PLAN = 'route_plan'     # 长机 -> 僚机：整条航线（含长机起飞点），僚机用来定每段航向
IN_POSITION = 'formation_in_position'   # 僚机 -> 长机：已飞到起始站位、机头已转到第一段航向
# 起步缓加速（用户 2026-09-28 要求）：长机起步头几秒把规划器限速压低，避免
# 一上来就顶到 1.0 m/s 把僚机甩开。恢复成 None 表示用 .env 里的 V_MAX 原值。
START_SLOW_VEL_MPS = 0.25     # 2026-09-28 从 0.4 降到 0.25（用户：起步阶段长机慢点）
START_SLOW_S = 4.0            # 最少压这么久
# 2026-09-28：删掉长机照顾模式之后，起步间距从 4.93 涨到 5.95（巡航段两者
# 完全一样，std 都是 0.26——照顾模式的全部价值只在起步这一段）。
# 起步是长机**事先知道**的事件，不需要常驻回路去发现，补成一次性握手就够：
# 压住限速直到僚机落后量收到 START_SLOW_LAG_M 以内，最多压 START_SLOW_MAX_S。
START_SLOW_LAG_M = 0.3        # 僚机落后收到这个以内就开始放速度
START_SLOW_MAX_S = 12.0       # 等不到也必须恢复，别把整段航线拖死
# 放速度也要缓：从 START_SLOW_VEL_MPS 一步跳回 1.0，长机会猛加速，僚机又被甩开
# 一次（实测起步间距冲到 5 米，用户指出）。改成分档往上爬。
START_RAMP_STEP_MPS = 0.15    # 每档加多少
START_RAMP_PERIOD_S = 1.5     # 每档保持多久
# 长机"照顾模式"已删除（2026-09-28）。它原本是：僚机落后多了长机就减速、
# 落后太多就停下等。删掉的理由是**两个积分器盯同一个误差**——长机看到落后
# 就减速、僚机看到落后就提速，互相激励，间距曲线上表现为持续的高频抖动。
# 现在僚机侧的偏置补偿（bias_trim_*）自己能把稳态误差收掉，长机应该飞固定
# 剖面，不参与调节。真要让长机等僚机，正确的位置是起步那一次同步（
# IN_POSITION 事件）和起步缓加速（START_SLOW_*），那两处是**前馈**，不是回路。
# 长机的巡航限速。要跟 .env 里的 V_MAX 一致——照顾模式减速之后靠它恢复。
CRUISE_VEL_MPS = 1.0
# 不传 --route 时用的默认航线（世界坐标），跟《双机全流程示例.py》里的 ROUTE 一致。
# 加默认值是因为 运行仿真.sh 不往任务程序传额外参数，单独跑这个示例时没法给航线。
DEFAULT_ROUTE = '7,-9.5 7,9.5 -7,9.5 -7,-9.5'


def listen_standby(sdk):
    """提前注册"僚机就位"的收件箱。必须在僚机可能发事件之前调用——可靠事件
    通道是先回 ACK 再查处理函数，没注册的事件会被确认后丢弃。"""
    return _Inbox(sdk, READY, IN_POSITION)


def leader_route(sdk, route_xy, inbox=None):
    """长机的编队任务段：等僚机就位 -> 按航线飞一圈 -> 通知僚机航线已完成。

    **不起飞、不降落**，留给调用方决定，这样《双机全流程示例.py》能把编队接在
    别的任务前面，中间不落地。
    """
    if inbox is None:
        inbox = listen_standby(sdk)

    pad = _own_pad(sdk)
    waypoints = list(route_xy)
    if tuple(waypoints[-1]) != pad:
        waypoints.append(pad)           # 起飞点当最后一个航点
    # 从起飞点出发，每一段都是"上一个点 -> 这个点"
    legs = list(zip([pad] + waypoints[:-1], waypoints))

    # 把整条航线（含长机起飞点）发给僚机。两个用途：
    #   ① 僚机每段的航向用这条航线算，不能靠从长机轨迹估切线——轨迹是里程计
    #      采样点连成的，带噪声，估出来的方向在直线段上就一直在抖（2026-09-24
    #      实测，用户指出"从机中途航向角一直在变化"）；
    #   ② 僚机用它算起始站位（长机起飞点往第一段航向的反方向退 spacing）。
    # 2026-09-28 把这一步挪到等 READY **之前**：僚机要先拿到航线才能去站位，
    # 反过来写就是互等。
    try:
        sdk.send_to_teammate(ROUTE_PLAN, route=[list(p) for p in ([pad] + waypoints)])
    except Exception as exc:                # 送不到不影响自己飞，僚机退化成锁定初始朝向
        print(f'[长机] 航线没送到僚机（{exc}），僚机将保持入列时的朝向', flush=True)

    # 等僚机**飞到起始站位并把机头转到第一段航向**再起步（用户 2026-09-28 要求）。
    # 旧版只等 READY（含义是"僚机节点已接管、在自己起飞点上空保持"），长机随即
    # 起步，僚机那边要等长机轨迹够长才开始入列——这段空窗里间距以长机的速度线性
    # 拉大，而跟随算法是匀速的、没有追赶项，拉开了就再也收不回来。
    # 收不到 IN_POSITION 也照飞（退回旧行为），只是会拉开。
    print('[长机] 等僚机到起始站位并转向…', flush=True)
    try:
        inbox.wait(IN_POSITION, STANDBY_WAIT_S)
        print('[长机] 僚机已到位，起步', flush=True)
    except TimeoutError:
        print(f'[长机] 等了 {STANDBY_WAIT_S:.0f} 秒没等到僚机到位，按旧行为直接起步',
              flush=True)

    # 各航段的局部坐标（两点之差在纯平移的局部系里跟世界系一致，而
    # set_yaw_mode_constant()/get_current_yaw() 本来就是局部系的角度，
    # 全程一套坐标不用来回换算）。
    legs_local = []
    for frm, to in legs:
        fx, fy, _ = sdk.world_to_local(frm[0], frm[1], CRUISE_AGL_M)
        tx, ty, _ = sdk.world_to_local(to[0], to[1], CRUISE_AGL_M)
        legs_local.append(((fx, fy), (tx, ty)))
    # 协调转弯：机头连续转，飞机不停（2026-09-28 用户要求，见 YAW_SLEW_DPS 注释）
    yaw_state = {'stop': False, 'idx': 0, 'yaw': None, 'sent': None}
    stop_yaw = _start_coordinated_yaw(sdk, legs_local, yaw_state)

    # ---- 整条航线一次下发（2026-09-28 用户要求：拐点不要减速等待）----
    # 原来是逐段 sdk.goto()。那样每一段对 ego_planner 都是独立终点，轨迹**终点
    # 速度为零**，桥接节点又要等飞机进到 0.3 m 球内才发下一个目标，于是每个航点
    # 必然"减速到 0 → 判到点 → 从 0 重新加速"。实测四个拐点最低速度 0.01~0.09
    # m/s、速度<0.3 的时长各 2.2~5.4 秒，长机巡航速度中位只有 0.59 m/s（限速 1.0）。
    # 一次下发之后由桥接节点按顺序推进，配合它的 flythrough_radius_m（环境变量
    # WAYPOINT_FLYTHROUGH_M）中间航点提前切换，规划器从"还在动"的状态接着规划。
    _, _, tz = sdk.world_to_local(legs[0][1][0], legs[0][1][1], CRUISE_AGL_M)
    route_pts = [(tx, ty, tz) for (_f, (tx, ty)) in legs_local]
    for i, (frm, to) in enumerate(legs, start=1):
        (fx, fy), (tx, ty) = legs_local[i - 1]
        print(f'[长机] 航点 {i}/{len(legs)}: ({to[0]}, {to[1]})，'
              f'航向 {math.degrees(math.atan2(ty - fy, tx - fx)):.0f}°', flush=True)
    if START_SLOW_VEL_MPS:
        # 起步缓加速：压住限速直到僚机跟上（见 START_SLOW_LAG_M）
        _slow_start(sdk)
    # 定高飞：整条航线一个高度，所以 with 块包住整条航线而不是每段一次。
    # fixed_altitude 要用**局部系**的 z，且必须等于航点的 z，否则永远判不到点。
    if hasattr(sdk, 'goto_route'):
        with sdk.fixed_altitude(tz):
            sdk.goto_route(route_pts, timeout=ROUTE_DONE_WAIT_S)
    else:
        # 老镜像没有 goto_route：退回逐段飞（拐点会停，但能飞完）
        print('[长机] SDK 没有 goto_route，退回逐段飞（拐点会减速停顿）', flush=True)
        for tx, ty, tzz in route_pts:
            with sdk.fixed_altitude(tzz):
                sdk.goto(tx, ty, tzz)

    stop_yaw()
    sdk.send_to_teammate(ROUTE_DONE)    # 只有长机知道哪个是最后一个航点


def leader(sdk, route_xy):
    """长机：起飞 -> 编队航线 -> 返航降落（单独跑这个示例时的完整流程）。"""
    inbox = listen_standby(sdk)         # 必须在僚机可能发事件之前注册
    # 直接起飞到巡航高度，省掉"起飞到1米再 goto 爬上去"那一次纯垂直规划
    sdk.takeoff(height_m=CRUISE_AGL_M)
    leader_route(sdk, route_xy, inbox)
    _land_at_pad(sdk)
    sdk.play_sound_light('侦察机任务完成')


def follower(sdk, spacing_m):
    """僚机：起飞 -> 跟队 -> 回起飞点降落。任务机每个任务都要落地，所以这一段
    本来就自带起降，可以直接被《双机全流程示例.py》复用。"""
    inbox = _Inbox(sdk, ROUTE_DONE)
    plan = _Inbox(sdk, ROUTE_PLAN)          # 必须在长机可能发之前就注册
    sdk.takeoff(height_m=CRUISE_AGL_M)      # 直接起飞到编队巡航高度
    sdk.send_to_teammate(READY)             # "我起飞完了"，长机据此发航线

    # 2026-09-28 改：先飞到**起始站位**并把机头转到第一段航向，再接管跟随。
    # 旧版是起飞完立刻接管、报 READY，长机随即起步，僚机却要等长机轨迹够长才
    # 开始入列——这段空窗里间距以长机的速度线性拉大，而跟随算法是匀速的，
    # 拉开了就再也收不回来。
    # 顺序不能反：一旦 start_formation_follow() 接管（relay_mode=formation），
    # 设定点就由节点直接喂 pt4ctrl，这里再调 goto/face_yaw 就不管用了。
    route = []
    try:
        plan.wait(ROUTE_PLAN, 60.0)
        route = [tuple(p) for p in plan.data.get('route', [])]
    except TimeoutError:
        print('[僚机] 没收到长机航线，跳过预站位，退回旧行为', flush=True)

    if len(route) >= 2:
        _goto_start_station(sdk, route, spacing_m)

    # 这个方法返回的含义是"机载已接管、在当前位置上空保持"。
    # 高度一并下发：僚机的高度由这个节点按定高雷达保持（天然仿地），默认 1.5 米，
    # 不显式传的话长机改了巡航高度、僚机还停在默认值，编队会一高一低。
    # turn_in_place：僚机跟长机同一套动作——拐点先停下把机头转到下一段的航向，
    # 转到位再前飞（用户 2026-09-24 要求「长机、从机都一样」）
    # 航线跟着 start_formation_follow 一起下发，**不能**等接管之后再补发：
    # 节点是在"切长机"时清空轨迹缓冲区、随后第一帧长机位姿就要用航线的第一段
    # 方向把轨迹向后延伸（对齐弧长0到僚机站位），那一帧比后补的航线早到就来不及了。
    sdk.start_formation_follow(follow_distance_m=spacing_m,
                               altitude_agl_m=CRUISE_AGL_M,
                               turn_in_place=True,
                               leg_route=route or None)
    sdk.send_to_teammate(IN_POSITION)        # 长机收到这个才起步

    inbox.wait(ROUTE_DONE, ROUTE_DONE_WAIT_S)
    sdk.stop_formation_follow()
    _land_at_pad(sdk)
    sdk.play_sound_light('任务机已降落')


def _goto_start_station(sdk, route, spacing_m):
    """飞到编队起始站位：长机起飞点沿第一段航向的**反方向**退 spacing 米，
    机头转到第一段航向。这样长机一起步，队形就已经成型，不用边飞边追。

    route[0] 是长机起飞点，route[1] 是第一个航点（世界坐标）。
    """
    (lx, ly), (nx, ny) = route[0], route[1]
    heading = math.atan2(ny - ly, nx - lx)
    sx = lx - math.cos(heading) * spacing_m
    sy = ly - math.sin(heading) * spacing_m
    print(f'[僚机] 起始站位 ({sx:.2f}, {sy:.2f})，在长机起飞点后方 {spacing_m:.1f} 米，'
          f'航向 {math.degrees(heading):.0f}°', flush=True)
    tx, ty, tz = sdk.world_to_local(sx, sy, CRUISE_AGL_M)
    try:
        工具.transfer_to(sdk, tx, ty, tz)     # 锁当前朝向、定高飞过去
    except Exception as exc:
        print(f'[僚机] 站位点飞不过去（{exc}），就当前位置入列', flush=True)
    # 到位再转向：先转会让 transfer_to 锁的朝向被覆盖，白转一次
    if not sdk.face_yaw(heading, timeout=TURN_TIMEOUT_S, tolerance_deg=TURN_TOL_DEG):
        print(f'[僚机] 起始航向没转到位（目标 {math.degrees(heading):.0f}°），仍继续',
              flush=True)


def _wrap_pi(a):
    """把角度归一到 (-pi, pi]，转弯取近路用。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _start_coordinated_yaw(sdk, legs_local, state):
    """后台线程：把机头**连续限速**转到当前航段方向，拐点前提前起转。

    飞机全程不停——这是"协调转弯"跟原来"到点悬停转向"的唯一区别。
    `state['idx']` 由主循环更新成当前在飞第几段。
    返回 stop 函数。
    """
    import threading
    if not legs_local:
        return lambda: None

    def _leg_heading(i):
        (fx, fy), (tx, ty) = legs_local[i]
        return math.atan2(ty - fy, tx - fx)

    def _loop():
        while not state['stop']:
            i = max(0, min(state['idx'], len(legs_local) - 1))
            want = _leg_heading(i)
            # 航段号**自己推进**：整条航线是一次性下发的（goto_route），主循环
            # 不再逐段阻塞，没人来喂这个索引了。判据取两条的并集，避免穿越式
            # 切换时飞机切了角、始终没靠近那个航点导致索引卡住：
            #   ① 离本段终点够近；或 ② 离下一段终点比离本段终点还近。
            if i + 1 < len(legs_local):
                try:
                    px, py, _ = sdk.get_local_position()
                    tx, ty = legs_local[i][1]
                    nx, ny = legs_local[i + 1][1]
                    d_cur = math.hypot(tx - px, ty - py)
                    if d_cur < YAW_LEG_ADVANCE_M or math.hypot(nx - px, ny - py) < d_cur:
                        state['idx'] = i + 1
                    elif d_cur < YAW_ANTICIPATE_M:
                        # 提前起转：转弯跨在拐角两边，而不是到了点才开始
                        want = _leg_heading(i + 1)
                except Exception:
                    pass                      # 取不到位置就按本段航向，不影响飞行
            if state['yaw'] is None:
                try:
                    state['yaw'] = sdk.get_current_yaw()
                except Exception:
                    state['yaw'] = want
            step = math.radians(YAW_SLEW_DPS) * YAW_TICK_S
            d = _wrap_pi(want - state['yaw'])
            state['yaw'] = _wrap_pi(state['yaw'] + max(-step, min(step, d)))
            # 只在角度真的变了才下发：set_yaw_mode_constant 每次都会打一行
            # 进度日志，10 Hz 无条件下发会把任务日志刷满（实测 run41 刷了
            # 几百行"朝向模式 -> 固定角度(90°)"，把真正的报错顶出了视野）。
            if (state['sent'] is None
                    or abs(_wrap_pi(state['yaw'] - state['sent'])) > YAW_SEND_EPS):
                try:
                    sdk.set_yaw_mode_constant(state['yaw'])
                    state['sent'] = state['yaw']
                except Exception:
                    pass
            time.sleep(YAW_TICK_S)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()

    def _stop():
        state['stop'] = True
        t.join(timeout=2.0)
    return _stop


def _slow_start(sdk):
    """起步缓加速：把规划器限速压到 START_SLOW_VEL_MPS，等僚机跟上再恢复。

    恢复条件（2026-09-28 改）：僚机自报的落后量收到 START_SLOW_LAG_M 以内，
    或者到了 START_SLOW_MAX_S 上限。这是**一次性的起步握手**，不是常驻调节
    回路——起步是长机事先知道的事件，前馈就够，见 START_SLOW_LAG_M 注释。
    拿不到僚机落后量（老镜像/僚机没起跟随）时退回"固定 START_SLOW_S 秒"。

    后台线程里恢复，不挡住 goto。限速改的是 ego_planner 的 manager/max_vel，
    它影响初值多项式的时间分配（梯形剖面）——限速低，加速段就被拉长。
    SDK 探测不到这个能力（老镜像）时静默跳过，不影响飞行。
    """
    import threading
    if not hasattr(sdk, 'set_max_vel'):
        return
    try:
        old = sdk.set_max_vel(START_SLOW_VEL_MPS)
    except Exception as exc:
        print(f'[长机] 起步限速没设上（{exc}），按原速起步', flush=True)
        return
    print(f'[长机] 起步缓加速：限速 {START_SLOW_VEL_MPS} m/s，'
          f'等僚机落后收到 {START_SLOW_LAG_M} m 以内再分档放回 {old} m/s'
          f'（最少 {START_SLOW_S:.0f} 秒，最多 {START_SLOW_MAX_S:.0f} 秒）', flush=True)

    def _restore():
        t0 = time.time()
        time.sleep(START_SLOW_S)            # 这段时间内无条件压住
        why = f'{START_SLOW_S:.0f} 秒到'
        if hasattr(sdk, 'teammate_formation_lag'):
            while time.time() - t0 < START_SLOW_MAX_S:
                lag = sdk.teammate_formation_lag()
                if lag is not None and lag <= START_SLOW_LAG_M:
                    why = f'僚机已跟上（落后 {lag:+.2f} m）'
                    break
                time.sleep(0.2)
            else:
                why = f'等僚机超过 {START_SLOW_MAX_S:.0f} 秒上限'
        # 分档爬回巡航速度，别一步跳满（见 START_RAMP_STEP_MPS）
        print(f'[长机] {why}，开始分档放速度（{START_SLOW_VEL_MPS} -> {old} m/s，'
              f'每 {START_RAMP_PERIOD_S:.1f} 秒 +{START_RAMP_STEP_MPS}）', flush=True)
        v = START_SLOW_VEL_MPS
        try:
            while v < old - 1e-3:
                v = min(old, v + START_RAMP_STEP_MPS)
                sdk.set_max_vel(v)
                if v < old - 1e-3:
                    time.sleep(START_RAMP_PERIOD_S)
            print(f'[长机] 已回到巡航限速 {old} m/s（起步共用时 {time.time() - t0:.1f} 秒）',
                  flush=True)
        except Exception as exc:
            print(f'[长机] 限速没恢复（{exc}）', flush=True)

    threading.Thread(target=_restore, daemon=True).start()


def _land_at_pad(sdk):
    """回自己起飞点降落。goto_direct 直线飞、不经过规划器，落点精度高一个
    量级，但**没有避障**——只用在这种"已在起降点附近、确定无障碍"的收尾。"""
    pad = _own_pad(sdk)
    print(f'[{sdk.namespace}] 回起飞点 {pad} 降落', flush=True)
    sdk.goto_direct(*sdk.world_to_local(pad[0], pad[1], CRUISE_AGL_M))
    工具.land_or_confirm(sdk)


def _own_pad(sdk):
    """起飞点世界坐标。起飞时飞机就在起降点上，所以局部原点换算过去就是。"""
    wx, wy, _ = sdk.local_to_world(0.0, 0.0, 0.0)
    return (round(wx, 2), round(wy, 2))


class _Inbox:
    """跨机事件收件箱。回调跑在 SDK 后台线程，只能记一笔，不能在里面飞。"""

    def __init__(self, sdk, *events):
        self._seen = set()
        self.data = {}          # 最近一次收到的随事件数据（比如长机发来的航线）
        for name in events:
            sdk.on_teammate_event(name, lambda _n=name, **kw: self._on(_n, kw))

    def _on(self, name, kwargs):
        self._seen.add(name)
        self.data = kwargs

    def wait(self, event, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if event in self._seen:
                return
            time.sleep(0.2)
        raise TimeoutError(f"等队友事件 '{event}' 超过 {timeout_s:.0f} 秒未到达")


def _parse_route(text):
    points = [tuple(float(v) for v in tok.split(',')) for tok in (text or '').split()]
    if len(points) < 2 or any(len(p) != 2 for p in points):
        raise ValueError(f'--route 需要至少2个 "x,y" 格式的航点，收到 {text!r}')
    return points


def main():
    ap = argparse.ArgumentParser(description='双机一字纵队编队飞行')
    ap.add_argument('--namespace', required=True)
    ap.add_argument('--role', required=True, choices=('leader', 'follower'))
    ap.add_argument('--teammate', required=True)
    ap.add_argument('--route', default=DEFAULT_ROUTE,
                    help='"x1,y1 x2,y2 ..."，只有长机需要，默认走大赛那条环场航线')
    ap.add_argument('--spacing', type=float, default=4.0, help='沿轨迹间距下限（米）')
    args = ap.parse_args()

    # SDK 的 role 只认 recon/supply，这里映射成更直白的 leader/follower
    sdk = DroneSDK(namespace=args.namespace,
                   role='recon' if args.role == 'leader' else 'supply',
                   teammate_namespace=args.teammate)
    try:
        if args.role == 'leader':
            leader(sdk, _parse_route(args.route))
        else:
            follower(sdk, args.spacing)
        print(f'[{args.namespace}] 编队飞行结束', flush=True)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()

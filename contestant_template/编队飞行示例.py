#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机一字纵队编队飞行。两机跑同一份代码，靠 --role 区分。

    python3 编队飞行示例.py --namespace NX01 --role leader  --teammate NX02 \
        --route "3,3 3,22 17,22 17,16 10,16 14,14 17,16 17,3" --spacing 4.0
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
from contest_sdk.exceptions import GotoUnreachableError

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
# ---- 编队解散时机（2026-09-29 用户定稿）----
# 判据：**长机飞回自己的起飞点上空时解散**。
#
# 航线最后一段 D(17,3) -> A(3,3) 沿 y=3 向西，正好从长机起飞点 (8,3) 上空穿过。
# 这个判据有个很顺的性质：僚机恒落后 spacing(4) 米，长机压在 (8,3) 时僚机正好
# 在 (12,3)——**就是僚机自己的起降点**，解散那一刻它就悬在自家停机坪上方，
# 直接降落即可，不用再飞回来。
#
# 之前试过"过 D 点之后解散"：因为僚机恒落后 spacing 米，长机一到 D 就停的话
# 僚机永远过不完 D，得等长机离开 D 超过 spacing 才行，判据绕。换成起飞点之后
# 这些都不需要了。
#
# 两阶段判据（不能只判"离起飞点近"——飞机一开始就停在起飞点上）：
#   ① 先等长机离开起飞点超过 DISBAND_DEPART_M，确认航线真的开始了；
#   ② 再等它回到起飞点 DISBAND_NEAR_M 以内 -> 发解散。
DISBAND_DEPART_M = 5.0        # 离起飞点超过这么远才算"已经出发"
DISBAND_NEAR_M = 1.2          # 回到起飞点这么近 = 解散
DISBAND_POLL_S = 0.2
SELECT_TOPIC_HOVER_S = 15.0   # 长机到 A 点后悬停多久（选题占位，见 _select_topic_at_a）
                              # 悬停结束示例就返回，飞机留在空中不降落
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
WAYPOINT_HOLD_S = 2.0         # 2026-09-29 用户要求：每个航点停这么久，同时换向
# 航点握手（2026-09-30）：长机在航点转向、停满 WAYPOINT_HOLD_S 之后，**再等
# 僚机也收拢到队形位置**才起步。
#
# 为什么要这个：走停模式下间距波动的主因不是增益不够，是**两机走停节拍错开**。
# 实测 run67——长机在航点停 6~9 秒，僚机被间距上限夹住慢慢贴到 4 米；长机一
# 起步就冲到 1.0 m/s，而僚机此刻可能还在自己的原地转向里（速度被钳成 0），
# 于是间距一次性放大到 5.7 米。这是相位问题，连续调速治不了。
#
# 为什么用"落后量"一个量就够：僚机原地转向期间速度是 0、根本收不了间距，
# 所以"落后量已经收到容差内"同时蕴含了"僚机也停稳、转完了"。不需要僚机
# 额外上报状态，也不用改跟随节点。
#
# 为什么放在长机侧而不是做成双边连续律：长机的速度是 ego_planner 轨迹的一
# 部分，连续调它会触发重规划（2026-09-29 拐点 set_max_vel 降速实测"效果极差"）。
# 这里只在长机**本来就静止**的那一刻做一次离散等待，不碰规划器、不碰任何
# 速度参数；航段之间的连续调节仍然僚机独占，保持单一权限。
WAYPOINT_SYNC_ENABLED = True
WAYPOINT_SYNC_LAG_M = 0.5     # 僚机落后量收到这个以内算"已入位"
WAYPOINT_SYNC_MAX_WAIT_S = 8.0   # 等不到也必须走，别把整条航线拖死
WAYPOINT_SYNC_POLL_S = 0.1
# 被 goto() 判成"不可达"时，停的位置离航点在这个距离以内就算到了（见 _goto_waypoint）。
# 取 0.7：判定下限本身就是 0.5 米，爬行平台落在 0.5~0.7 是典型值；再大就不该当到达了。
GOTO_ACCEPT_M = 0.7
YAW_LEG_ADVANCE_M = 1.5       # 离本段终点这么近就算进入下一段（转弯线程自己推进）
YAW_LEG_DONE_RATIO = 0.98     # 或者沿本段投影进度到这个比例也算走完（切角时兜底）
READY = 'formation_standby'   # 僚机 -> 长机
ROUTE_DONE = 'route_done'     # 长机 -> 僚机
ROUTE_PLAN = 'route_plan'     # 长机 -> 僚机：整条航线（含长机起飞点），僚机用来定每段航向
IN_POSITION = 'formation_in_position'   # 僚机 -> 长机：已飞到起始站位、机头已转到第一段航向
# 起步缓加速（用户 2026-09-28 要求）：起步时把规划器限速压低，避免一上来就
# 顶到 1.0 m/s 把僚机甩开。
# 2026-09-28：删掉长机照顾模式之后，起步间距从 4.93 涨到 5.95（巡航段两者
# 完全一样，std 都是 0.26——照顾模式的全部价值只在起步这一段）。起步是长机
# **事先知道**的事件，不需要常驻回路去发现，前馈就够。
# 放速度也要缓：一步从 0.25 跳回 1.0，长机会猛加速、僚机又被甩开一次
# （实测起步间距冲到 5 米，用户指出）。所以分档往上爬。
# 分档要快：run71 用 +0.15/1.5 秒（0.5->1.0 要 7.5 秒、走 5 米）还速度，结果
# 最后一档落进终端减速段把飞机顶过了头。改成 2 秒走完，只占约 1.5 米。
START_RAMP_STEP_MPS = 0.25    # 每档加多少
START_RAMP_PERIOD_S = 1.0     # 每档保持多久
# 2026-09-30 用户指出：长机在每个航点都停下转向，**过每个航点都相当于重新
# 起步**，所以"起步缓加速"不该只在第一段用，每一段都要用；而且放速度的条件
# 应当是"僚机也过了这个航点"——只有两机同在一条航段上才谈得上平稳跟踪。
# LEG_SLOW_VEL_MPS 是"总用时 vs 间距波动"的那个旋钮：长机爬得慢，僚机跟得住，
# 但每段头 spacing 米都要爬 spacing/实际速度 秒。
# **下限不能低于 0.5**：`goto()` 的卡住检测是"5 秒内三维位移不足 0.5 米且离目标
# 还有 0.5 米以上"，即要求持续跑到 0.1 m/s 以上。而限速跟实际速度不是一回事——
# run70 实测限速 0.25 时长机只跑 0.15 m/s（18.2 秒走 2.78 米），刚起步那几秒更
# 低，直接被判成 GotoUnreachableError（leg 1 擦边活下来，leg 2 起步就挂）。
# 0.5 的限速对应实际约 0.3 m/s，5 秒走 1.5 米，留了 3 倍余量。
LEG_SLOW_VEL_MPS = 0.5        # 每段起步先把限速压到这里（0 = 关掉这个机制）
LEG_SLOW_MIN_S = 1.0          # 至少压这么久，给规划器出新轨迹的时间
# 上限必须大于"以 LEG_SLOW_VEL_MPS 走完 spacing+margin"所需的时间，否则超时
# 会在僚机过点之前就把速度放开，整个机制形同虚设：4.5 米 / 0.25 (m/s) = 18 秒。
LEG_SLOW_MAX_S = 30.0         # 等不到僚机过点也必须放速度，别把航线拖死
LEG_SLOW_MARGIN_M = 0.5       # 判"僚机已过点"时给的余量（米）
# 终端逼近段有两条互相顶着的约束，run71/run72 各撞了一次，都写在这里：
#   ① **低限速不能带进终端逼近段**：规划器的轨迹终点速度为零、收敛是渐近的，
#      限速 1.0 时最后 0.4 米就要爬 4 秒（≈0.1 m/s），已经贴着 goto() 卡住检测
#      的下限（5 秒内位移不足 0.5 米就判不可达）。压到 0.5 就必然跌破——run72
#      第 3 段整段 0.5 飞到底，停在离 C 点 0.80 米处被判不可达。
#   ② **限速变更也不能带进终端逼近段**：改限速会逼 ego_planner 重做时间分配、
#      重规划，在减速段重算刹车剖面就冲过头——run71 最后一段分档的最后一档落在
#      离 D 点 5.5 米处，飞机冲过 D 点又被拉回（local y 2.44->2.69，高度
#      2.40->1.76）。这跟 2026-09-29 "拐点 set_max_vel 降速效果极差"同根。
# 结论：限速必须在**离终点还远**的时候就还回巡航，而且要还得快。
# 这两个距离是 run73 之后收紧的：兜底距离原来给 10 米，结果四条长段里有三条
# 都被兜底提前解除、实际只压了 3~4 米（没压够 spacing 那 4.5 米），std 只收到
# 0.63（每段都压的 run71 是 0.42）。分档已经改成 2 秒走完、只占约 1.5 米，
# 6 米的兜底仍给终端逼近段留了 4.5 米余量。
LEG_SLOW_MIN_LEG_M = 8.0      # 短于这个的航段根本不压限速——没有还回去的余量
LEG_RAMP_MUST_START_M = 6.0   # 剩这么远还没等到僚机过点，就必须开始放速度
LEG_RAMP_FREEZE_REMAIN_M = 5.0   # 剩这么近一律不再动限速（兜底，正常到不了）
LEG_SLOW_POLL_S = 0.1
_LEG_SLOW = {'gen': 0}        # 换段时作废上一段还没跑完的放速度线程
# 长机"照顾模式"已删除（2026-09-28）。它原本是：僚机落后多了长机就减速、
# 落后太多就停下等。删掉的理由是**两个积分器盯同一个误差**——长机看到落后
# 就减速、僚机看到落后就提速，互相激励，间距曲线上表现为持续的高频抖动。
# 现在僚机侧的偏置补偿（bias_trim_*）自己能把稳态误差收掉，长机应该飞固定
# 剖面，不参与调节。真要让长机等僚机，正确的位置是**离散的、长机自己知道
# 时机的**那几处：入列同步（IN_POSITION 事件）、航点握手（WAYPOINT_SYNC_*）、
# 每段起步缓加速（LEG_SLOW_*）——都是前馈/一次性握手，不是常驻回路。
# 巡航速度，要跟 .env 的 V_MAX 一致。只用于日志显示和巡航高度换算；真正的
# 限速由 V_MAX 决定，_slow_leg_start 恢复时用的是 set_max_vel 返回的原值。
CRUISE_VEL_MPS = 1.0
# 不传 --route 时用的默认航线（世界坐标）。
# 2026-09-29 换成**样题场景**的 4 个航点：原点在房间西南角，坐标全为正，
# 权威来源是 src/contest_mission/config/sample_room_layout.yaml 的
# route_waypoints（那份 yaml 同时也是 world 文件的生成输入）。
# 旧的 fire_drill_room 航线 '7,-9.5 7,9.5 -7,9.5 -7,-9.5' 是中心原点那套坐标，
# 两套坐标系差了 (10, 12.5)，混用会整体偏十几米——换场景时 WORLD_ENV 和这条
# 航线必须一起换。
# 2026-09-29 四改，样题航线是 A B C G E F G D 八个点：
#   A(3,3) B(3,22) C(17,22) G(17,16) E(10,16) F(14,14) G(17,16) D(17,3)
# F 是**虚拟点**，作为从 E 返回航线的中间点：E->F->G 这条折线代替了"E 原路
# 退回 G"。原路退回有两个毛病——折返处 180° 掉头时僚机还在往前冲，间距被挤到
# 1.79 m；G-E 和 E-G 完全重合，按航线弧长投影算间距在那一段有歧义。走折线之后
# G 虽然仍出现两次，但**没有任何航段重合**（只共用 G 这一个端点），都避开了。
# G->E（y=16）仍然压在地面火情点 (13,16) 头上，覆盖不丢。
# 加默认值是因为 运行仿真.sh 不往任务程序传额外参数，单独跑这个示例时没法给航线。
DEFAULT_ROUTE = '3,3 3,22 17,22 17,16 10,16 14,14 17,16 17,3'


def listen_standby(sdk):
    """提前注册"僚机就位"的收件箱。必须在僚机可能发事件之前调用——可靠事件
    通道是先回 ACK 再查处理函数，没注册的事件会被确认后丢弃。"""
    return _Inbox(sdk, READY, IN_POSITION)


def leader_route(sdk, route_xy, inbox=None, spacing_m=4.0,
                 start_xy=None, final_xy=None, disband_after_follower_passes=None):
    """长机的编队任务段：等僚机就位 -> 按航线飞一圈 -> 通知僚机航线已完成。

    **不起飞、不降落**，留给调用方决定，这样《双机全流程示例.py》能把编队接在
    别的任务前面，中间不落地。
    """
    if inbox is None:
        inbox = listen_standby(sdk)

    pad = _own_pad(sdk)
    # start_xy/final_xy：让这段编队航线能从**任意位置**起、到**任意点**收尾，
    # 给任务2/任务3 的"任务做完从半路开始编队返航"复用（默认值就是本示例
    # 原来的行为：从起飞点出发、最后回 A 点）。解散判据仍然是"长机飞回自己
    # 起飞点上空"，跟起点终点无关。
    start = tuple(start_xy) if start_xy is not None else pad
    tail = tuple(final_xy) if final_xy is not None else tuple(route_xy[0])
    waypoints = list(route_xy)
    # 2026-09-29 样题流程：A->B->C->D 跑完之后**长机回到 A 点选题**，不是回起飞点。
    # 所以收尾航点是 route_xy[0]（A），不是 pad。僚机那边不受影响——它收到
    # ROUTE_DONE 之后各回各的起降点降落（见 follower()）。
    if tuple(waypoints[-1]) != tail:
        waypoints.append(tail)
    # 从起飞点出发，每一段都是"上一个点 -> 这个点"
    legs = list(zip([start] + waypoints[:-1], waypoints))

    # 把整条航线（含长机起飞点）发给僚机。两个用途：
    #   ① 僚机每段的航向用这条航线算，不能靠从长机轨迹估切线——轨迹是里程计
    #      采样点连成的，带噪声，估出来的方向在直线段上就一直在抖（2026-09-24
    #      实测，用户指出"从机中途航向角一直在变化"）；
    #   ② 僚机用它算起始站位（长机起飞点往第一段航向的反方向退 spacing）。
    # 2026-09-28 把这一步挪到等 READY **之前**：僚机要先拿到航线才能去站位，
    # 反过来写就是互等。
    try:
        # 2026-09-29 订正：这里原来拼的是 [pad] + waypoints。加了 start_xy 之后
        # 编队段可以从**半路**起步（任务2/任务3 是从 G 点开始编队），再拼 pad
        # 就等于告诉僚机"第一段是 起飞点->G"——那一段根本不存在。僚机据此算
        # 站位点会算到房间外（实测 (-6.22,-3.34) 不可达），分段航向也跟着错。
        # 起点跟首航点重合时不要重复塞：那样首段长度为零，僚机拿它算航向会得到
        # 一个任意值（任务2 从 G 起步、首航点也是 G，实测算出 0° 的假航向）。
        plan_pts = list(waypoints) if _same_xy(start, waypoints[0]) else [start] + list(waypoints)
        sdk.send_to_teammate(ROUTE_PLAN, route=[list(p) for p in plan_pts])
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
    _, _, tz = sdk.world_to_local(legs[0][1][0], legs[0][1][1], CRUISE_AGL_M)

    # 解散看门狗。两种判据二选一：
    #   · 默认：长机飞回**自己的起飞点**上空就解散（本示例的规则）；
    #   · disband_after_follower_passes=(wx,wy)：**僚机过了这个航点**就解散
    #     （任务3 用，用户 2026-09-30 要求"待任务机过 D 点后就解散编队"）。
    disband_state = {'stop': False, 'sent': False}
    if disband_after_follower_passes is not None:
        stop_disband = _start_disband_watch_follower_passed(
            sdk, tuple(disband_after_follower_passes), spacing_m, disband_state)
    else:
        stop_disband = _start_disband_watch(sdk, pad, spacing_m, disband_state)
    sync_wait_s = 0.0       # 航点握手总共等了多久（见 _wait_follower_settled）

    # ---- 逐段飞：每个航点停 WAYPOINT_HOLD_S 秒、同时把机头转到下一段航向 ----
    # 2026-09-29 用户要求："无论编队还是单独飞行、无论长机还是僚机，每个航点处
    # 都停顿 2 秒同时调整航向，航点之间航向不再变化。"
    #
    # 这是把 09-28 的"协调转弯 + 整条航线一次下发（goto_route）"**反过来**：
    # 那一版是为了消除拐点处失控的 4~8 秒爬行，代价是切角 1 米以上；现在改成
    # **确定性的 2 秒停顿**——停多久是自己说了算的，不再看规划器的渐近收敛脸色，
    # 而且航向在航段内恒定，观感和判读都更清楚。
    # 相应地 WAYPOINT_FLYTHROUGH_M 要设回 0：现在是**要**真正飞到航点的。
    for i, (frm, to) in enumerate(legs, start=1):
        (fx, fy), (tx, ty) = legs_local[i - 1]
        # 零长度航段直接跳过：start_xy 跟第一个航点重合时（任务2/任务3 都是从
        # G 起编队、首航点也是 G）会凑出一条 G->G 的段，atan2(0,0) 给出 0°，
        # 长机会先朝正东白转一次再去真正的下一个点（2026-09-30 实测多花十几秒）。
        if math.hypot(tx - fx, ty - fy) < 0.05:
            print(f'[长机] 航点 {i}/{len(legs)}: ({to[0]}, {to[1]}) 跟上一个点重合，跳过',
                  flush=True)
            continue
        heading = math.atan2(ty - fy, tx - fx)
        print(f'[长机] 航点 {i}/{len(legs)}: ({to[0]}, {to[1]})，'
              f'航向 {math.degrees(heading):.0f}°（停 {WAYPOINT_HOLD_S:.0f} 秒转向）',
              flush=True)
        # 停顿与转向同时进行：face_yaw 阻塞到转到位，不足 2 秒的部分补足
        t0 = time.time()
        if not sdk.face_yaw(heading, timeout=TURN_TIMEOUT_S, tolerance_deg=TURN_TOL_DEG):
            print(f'[长机] 航向没转到位（目标 {math.degrees(heading):.0f}°），仍继续前飞',
                  flush=True)
        left = WAYPOINT_HOLD_S - (time.time() - t0)
        if left > 0:
            time.sleep(left)
        # 停满、转到位之后，再等僚机也收拢到队形位置才起步（见 WAYPOINT_SYNC_*）。
        # 解散之后僚机不再跟随，落后量停更新，这时不能再等。
        if not disband_state['sent']:
            sync_wait_s += _wait_follower_settled(sdk, f'航点 {i}/{len(legs)}')
        leg_len = math.hypot(tx - fx, ty - fy)
        if LEG_SLOW_VEL_MPS and leg_len >= LEG_SLOW_MIN_LEG_M:
            # 长航段起步缓加速：压住限速直到僚机也过了这个航点（见 _slow_leg_start）。
            # (fx, fy) 就是长机此刻脚下的那个航点——本段的起点。
            _slow_leg_start(sdk, (fx, fy), (tx, ty), spacing_m,
                            f'航点 {i}/{len(legs)}')
        elif LEG_SLOW_VEL_MPS:
            print(f'[长机] 航点 {i}/{len(legs)}：本段只有 {leg_len:.1f} m '
                  f'(<{LEG_SLOW_MIN_LEG_M:.0f} m)，不压限速直接巡航'
                  f'——没有把速度还回去的余量，压了会在终端逼近段被判卡住',
                  flush=True)
        # 定高飞：航向已经锁在 heading 上，整段不再变
        with sdk.fixed_altitude(tz):
            _goto_waypoint(sdk, tx, ty, tz, f'航点 {i}/{len(legs)}')

    stop_disband()
    # 最后一段可能是"保持 LEG_SLOW_VEL_MPS 飞完"收尾的，限速还压着。编队段结束
    # 后面还有别的飞行（长机回 A 点选题、任务2/3 的后续动作），把巡航限速还回去。
    # 此刻长机已经到点停住，改限速是安全的。
    cruise = _LEG_SLOW.get('cruise')
    if cruise is not None and hasattr(sdk, 'set_max_vel'):
        try:
            sdk.set_max_vel(cruise)
        except Exception as exc:
            print(f'[长机] 巡航限速没还回去（{exc}）', flush=True)
    if WAYPOINT_SYNC_ENABLED:
        print(f'[长机] 航点握手累计等待 {sync_wait_s:.1f} 秒', flush=True)
    if not disband_state['sent']:
        # 兜底：看门狗没能判出"僚机已过点"（位置读不到/航线太短/提前结束），
        # 航线都飞完了还是要解散，不然僚机会一直跟着不回家。
        sdk.send_to_teammate(ROUTE_DONE)
        print('[长机] 航线已飞完，补发编队解散通知', flush=True)


def leader(sdk, route_xy, spacing_m=4.0):
    """长机：起飞 -> 编队航线(A B C G E G D) -> 回 A 点悬停选题。

    2026-09-29 样题流程：跟僚机不一样，长机**不回起降点、也不降落**——航线
    跑完之后回到 A 点悬停选题。僚机那边照旧回自己的起降点降落。
    """
    inbox = listen_standby(sdk)         # 必须在僚机可能发事件之前注册
    # 直接起飞到巡航高度，省掉"起飞到1米再 goto 爬上去"那一次纯垂直规划
    sdk.takeoff(height_m=CRUISE_AGL_M)
    leader_route(sdk, route_xy, inbox, spacing_m=spacing_m)
    _select_topic_at_a(sdk, route_xy[0])
    sdk.play_sound_light('侦察机任务完成')


def follow_formation(sdk, spacing_m, inbox=None, plan=None, goto_station=True):
    """**纯空中**的编队跟随段：入列 -> 跟队 -> 收到解散通知为止。

    2026-09-29 按用户要求拆出来："编队只有空中动作，起降不是编队内容"。
    起飞和降落由任务脚本自己管——任务2/任务3 里任务机是先做完取物资、投弹
    才入列的，它的起降时机跟编队没关系；本示例的 follower() 只是在这一段
    外面加了起降的一个薄壳。

    inbox/plan 允许调用方**提前注册**这两个事件：可靠事件通道是"先 ACK 再
    分发"，没注册处理函数的事件会被确认后丢弃，所以多任务脚本必须一开始就
    把所有事件注册上，不能等用到了才注册。
    """
    if inbox is None:
        inbox = _Inbox(sdk, ROUTE_DONE)
    if plan is None:
        plan = _Inbox(sdk, ROUTE_PLAN)
    sdk.send_to_teammate(READY)             # "我到位了"，长机据此发航线

    route = []
    try:
        plan.wait(ROUTE_PLAN, 60.0)
        route = [tuple(p) for p in plan.data.get('route', [])]
    except TimeoutError:
        print('[僚机] 没收到长机航线，跳过预站位，退回旧行为', flush=True)

    if goto_station and len(route) >= 2:
        _goto_start_station(sdk, route, spacing_m)
    elif not goto_station:
        # 任务2/任务3：任务机刚做完事（投弹/发射）就在长机后方不远处，直接入列。
        # 再飞一趟"航线起点后方 spacing 米"的站位点纯属绕路——实测任务2 里它
        # 已经离长机正好 4 米了，而首段朝南、站位点却在长机北边。
        print('[僚机] 就地入列（跳过预站位）', flush=True)

    sdk.start_formation_follow(follow_distance_m=spacing_m,
                               altitude_agl_m=CRUISE_AGL_M,
                               turn_in_place=True,
                               leg_route=route or None)
    sdk.send_to_teammate(IN_POSITION)        # 长机收到这个才起步

    inbox.wait(ROUTE_DONE, ROUTE_DONE_WAIT_S)
    sdk.stop_formation_follow()


def follower(sdk, spacing_m):
    """僚机完整流程 = 起飞 + 编队跟随（空中段） + 回起飞点降落。

    起降在这里，不在 follow_formation 里——编队只管空中。
    """
    inbox = _Inbox(sdk, ROUTE_DONE)
    plan = _Inbox(sdk, ROUTE_PLAN)          # 必须在长机可能发之前就注册
    sdk.takeoff(height_m=CRUISE_AGL_M)      # 直接起飞到编队巡航高度
    follow_formation(sdk, spacing_m, inbox=inbox, plan=plan)
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


def _goto_waypoint(sdk, tx, ty, tz, tag):
    """飞到航点，并兜住"终端渐近爬行被误判成不可达"这一类失败。

    根因是规划器轨迹的终点速度为零、收敛是渐近的：最后半米要爬好几秒（早先
    实测"最后 0.4 米爬了 4 秒"，约 0.1 m/s）。而 `goto()` 的卡住判据是
    "5 秒内三维位移不足 0.5 米、且离目标还有 0.5 米以上"——爬行平台一旦停在
    0.5 米出头就正好落进判定区（run76 实测停在离 G 点 0.52 米处抛异常，那条
    段根本没压限速、全程巡航，跟缓起步无关；之前几轮没撞上纯属运气）。

    这时候飞机其实**已经到点了**（0.52 m 已经在过点精度范围内），没有理由让
    整条航线崩掉。所以：停的位置离目标在 GOTO_ACCEPT_M 以内就按到达处理、
    打一行日志继续飞；超出这个范围才是真的到不了，照常抛出去。
    """
    try:
        sdk.goto(tx, ty, tz)
    except GotoUnreachableError as exc:
        d = float(getattr(exc, 'distance_m', 1e9))
        if d > GOTO_ACCEPT_M:
            raise
        print(f'[长机] {tag}：规划器终端爬行被判卡住，但离航点只有 {d:.2f} m '
              f'(≤{GOTO_ACCEPT_M:.1f} m)，按到达处理继续飞', flush=True)


def _wait_follower_settled(sdk, tag):
    """长机在航点上等僚机收拢到队形位置。返回实际等待秒数。

    判据是僚机自报的落后量（`teammate_formation_lag()` = 实际间距 − 有效跟随
    距离，>0 表示落后）。只判"不落后太多"、不判 |lag|——落后是能靠僚机自己
    追上来消掉的，太近却没法后退（参考点只前进不后退），等也白等。

    僚机节点没在跟随 / 话题还没到时 `teammate_formation_lag()` 返回 None，
    这时直接走，退回没有握手的旧行为；读到的值也可能是僚机停发之后的残值，
    所以一律带 WAYPOINT_SYNC_MAX_WAIT_S 硬超时。
    """
    if not WAYPOINT_SYNC_ENABLED:
        return 0.0
    if not hasattr(sdk, 'teammate_formation_lag'):   # 旧版 SDK
        return 0.0
    if sdk.teammate_formation_lag() is None:
        print(f'[长机] {tag}：读不到僚机落后量，不等待（退回无握手行为）', flush=True)
        return 0.0
    t0 = time.time()
    while time.time() - t0 < WAYPOINT_SYNC_MAX_WAIT_S:
        lag = sdk.teammate_formation_lag()
        if lag is None:                     # 中途停发（比如已解散）就别等了
            break
        if lag <= WAYPOINT_SYNC_LAG_M:
            waited = time.time() - t0
            print(f'[长机] {tag}：僚机已入位（落后 {lag:+.2f} m），'
                  f'等了 {waited:.1f} 秒后起步', flush=True)
            return waited
        time.sleep(WAYPOINT_SYNC_POLL_S)
    lag = sdk.teammate_formation_lag()
    waited = time.time() - t0
    print(f'[长机] {tag}：等了 {waited:.1f} 秒僚机仍落后 '
          f'{"读不到" if lag is None else f"{lag:+.2f} m"}，按超时起步', flush=True)
    return waited


def _same_xy(a, b, tol=0.3):
    """两个航点算不算同一个点（米）。"""
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])) <= tol


def _wrap_pi(a):
    """把角度归一到 (-pi, pi]，转弯取近路用。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _slow_leg_start(sdk, corner_local, end_local, spacing_m, tag):
    """**每段**起步缓加速：压住限速，等僚机也过了这个航点再分档放回巡航速度。

    为什么每段都要（2026-09-30 用户指出）：长机在每个航点都停下转向，对它自己
    来说**过每个航点都相当于重新起步**。而僚机是沿长机实飞轨迹落后 spacing 米
    的——长机站在拐点上时，僚机还在**上一段**上、还没拐弯。这时候两机不在同
    一条航线上，跟踪必然差；长机此刻一脚油门顶到 1.0 m/s，间距就一次性放大
    （run68 实测起步段峰值 7.02 m，而全程其它地方都不超过 5.86 m）。

    "僚机已过点"的判据：僚机沿轨迹落在长机后方 `spacing + lag` 米（lag 是僚机
    自报的落后量），而本段是直线，所以**长机离拐点的直线距离 ≥ spacing + lag**
    就等价于"僚机已经走过这个拐点"。这个量长机自己就能算（自己的位置 + 僚机
    自报的落后量），不需要僚机额外上报，也不用改跟随节点。

    ⚠️ 这里压的是 ego_planner 的 `manager/max_vel`，跟 2026-09-29 试过、被判
    "效果极差"的那个拐点降速是**同一个旋钮的不同用法**：那次是在飞行**中途**
    改限速、逼规划器重规划；这里改限速时长机是**停着的**，改完才下发新目标，
    走的是原来"第一段起步缓加速"那条已经验证过的路径。

    corner_local / end_local 是本段起点（长机此刻脚下的那个航点）和终点的局部
    坐标。终点用来提前放速度，见 LEG_SLOW_RELEASE_REMAIN_M——短航段（比如
    E->F 只有 4.47 米、F->G 只有 3.6 米）本来就短于 spacing，僚机不可能在长机
    到下一个拐点之前过点，这时全靠这条提前释放。
    """
    import threading
    if not hasattr(sdk, 'set_max_vel'):
        return
    _LEG_SLOW['gen'] += 1
    gen = _LEG_SLOW['gen']
    try:
        old = sdk.set_max_vel(LEG_SLOW_VEL_MPS)
    except Exception as exc:
        print(f'[长机] {tag}：起步限速没设上（{exc}），按原速起步', flush=True)
        return
    # 巡航限速只在第一次记录：第二段再调 set_max_vel 时，它返回的"旧值"已经是
    # 上一段压下去的 0.25，拿它当恢复目标的话速度就再也回不到巡航了。
    cruise = _LEG_SLOW.setdefault('cruise', old)
    cx, cy = float(corner_local[0]), float(corner_local[1])
    ex, ey = float(end_local[0]), float(end_local[1])
    need0 = spacing_m + LEG_SLOW_MARGIN_M
    print(f'[长机] {tag}：起步限速 {LEG_SLOW_VEL_MPS} m/s，'
          f'等僚机也过这个航点（长机离拐点走够 ~{need0:.1f} m）再分档放回 {cruise} m/s',
          flush=True)

    def _restore():
        t0 = time.time()
        time.sleep(LEG_SLOW_MIN_S)
        why = f'压满 {LEG_SLOW_MIN_S:.0f} 秒下限'
        while time.time() - t0 < LEG_SLOW_MAX_S:
            if _LEG_SLOW['gen'] != gen:      # 已经换段了，这条线程作废
                return
            try:
                px, py, _ = sdk.get_local_position()
            except Exception:
                break                        # 读不到位置就退回"只压下限时间"
            gone = math.hypot(px - cx, py - cy)
            remain = math.hypot(px - ex, py - ey)
            if remain <= LEG_RAMP_MUST_START_M:
                why = (f'剩余 {remain:.2f} m 已到必须放速度的距离'
                       f'（僚机还没过点，但再压下去终端逼近段会被判卡住）')
                break
            lag = (sdk.teammate_formation_lag()
                   if hasattr(sdk, 'teammate_formation_lag') else None)
            need = spacing_m + LEG_SLOW_MARGIN_M + max(0.0, lag or 0.0)
            if gone >= need:
                why = (f'僚机已过点（长机离拐点 {gone:.2f} m ≥ '
                       f'{need:.2f} m，僚机落后 '
                       f'{"读不到" if lag is None else f"{lag:+.2f} m"}）')
                break
            time.sleep(LEG_SLOW_POLL_S)
        else:
            why = f'等僚机过点超过 {LEG_SLOW_MAX_S:.0f} 秒上限'
        if _LEG_SLOW['gen'] != gen:
            return
        print(f'[长机] {tag}：{why}，开始分档放速度'
              f'（{LEG_SLOW_VEL_MPS} -> {cruise} m/s，'
              f'每 {START_RAMP_PERIOD_S:.1f} 秒 +{START_RAMP_STEP_MPS}）', flush=True)
        v = LEG_SLOW_VEL_MPS
        try:
            while v < cruise - 1e-3:
                if _LEG_SLOW['gen'] != gen:
                    return
                # 每加一档之前再查一次：爬档过程中飞机一直在往前走，进了冻结区
                # 就停在当前这一档，绝不把限速变更带进终端减速段。
                try:
                    px, py, _ = sdk.get_local_position()
                    remain = math.hypot(px - ex, py - ey)
                except Exception:
                    remain = None
                if remain is not None and remain <= LEG_RAMP_FREEZE_REMAIN_M:
                    print(f'[长机] {tag}：离本段终点还剩 {remain:.2f} m，'
                          f'分档停在 {v} m/s 不再上调', flush=True)
                    return
                v = min(cruise, v + START_RAMP_STEP_MPS)
                sdk.set_max_vel(v)
                if v < cruise - 1e-3:
                    time.sleep(START_RAMP_PERIOD_S)
            print(f'[长机] {tag}：已回到巡航限速 {cruise} m/s'
                  f'（本段起步共用时 {time.time() - t0:.1f} 秒）', flush=True)
        except Exception as exc:
            print(f'[长机] {tag}：限速没恢复（{exc}）', flush=True)

    threading.Thread(target=_restore, daemon=True).start()


def _start_disband_watch_follower_passed(sdk, wp_xy, spacing_m, state):
    """盯着**僚机**什么时候过了 wp_xy 这个航点，过了就发 ROUTE_DONE 解散编队。

    长机拿不到僚机的位置，但拿得到它自报的落后量：僚机沿长机轨迹落在长机后方
    `spacing + lag` 米。本段是直线，所以"长机离这个航点的直线距离 ≥ spacing + lag"
    就等价于僚机已经走过它——跟 `_slow_leg_start` 的判据同一套几何。

    前提是长机**已经过了**这个航点（先判长机自己过点，再判僚机跟上），不然
    刚起步时离航点也很远，会当场误触发。
    """
    import threading
    wx, wy = float(wp_xy[0]), float(wp_xy[1])
    lx, ly, _lz = sdk.world_to_local(wx, wy, CRUISE_AGL_M)

    def _loop():
        leader_passed = False
        while not state['stop'] and not state['sent']:
            try:
                cx, cy, _ = sdk.get_local_position()
            except Exception:
                time.sleep(DISBAND_POLL_S)
                continue
            d = math.hypot(cx - lx, cy - ly)
            if not leader_passed:
                if d <= DISBAND_NEAR_M:
                    leader_passed = True
                    print(f'[长机] 已过航点 ({wx:.1f}, {wy:.1f})，开始等僚机过点', flush=True)
                time.sleep(DISBAND_POLL_S)
                continue
            lag = (sdk.teammate_formation_lag()
                   if hasattr(sdk, 'teammate_formation_lag') else None)
            need = spacing_m + max(0.0, lag or 0.0)
            if d >= need:
                state['sent'] = True
                try:
                    sdk.send_to_teammate(ROUTE_DONE)
                except Exception as exc:
                    print(f'[长机] 解散通知没送到僚机（{exc}）', flush=True)
                print(f'[长机] 僚机已过航点 ({wx:.1f}, {wy:.1f})'
                      f'（长机离该点 {d:.1f} m ≥ {need:.1f} m），编队解散', flush=True)
                return
            time.sleep(DISBAND_POLL_S)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()

    def _stop():
        state['stop'] = True
        t.join(timeout=2.0)
    return _stop


def _start_disband_watch(sdk, pad_xy, spacing_m, state):
    """后台盯着长机位置，等它飞回自己起飞点上空就发 ROUTE_DONE（解散编队）。

    两阶段判据见 DISBAND_DEPART_M 那段注释——飞机一开始就停在起飞点上，
    只判"离起飞点近"会在起飞那一刻就误触发。

    state['sent'] 供调用方判断要不要补发（看门狗没等到就兜底）。
    """
    import threading
    px, py = float(pad_xy[0]), float(pad_xy[1])
    lx, ly, _lz = sdk.world_to_local(px, py, CRUISE_AGL_M)

    def _loop():
        departed = False
        while not state['stop'] and not state['sent']:
            try:
                cx, cy, _ = sdk.get_local_position()
            except Exception:
                time.sleep(DISBAND_POLL_S)
                continue
            d = math.hypot(cx - lx, cy - ly)
            if not departed:
                if d >= DISBAND_DEPART_M:
                    departed = True
            elif d <= DISBAND_NEAR_M:
                state['sent'] = True
                try:
                    sdk.send_to_teammate(ROUTE_DONE)
                except Exception as exc:
                    print(f'[长机] 解散通知没送到僚机（{exc}）', flush=True)
                print(f'[长机] 已飞回自己起飞点上空（离 {d:.1f} m），编队解散'
                      f'——此时僚机正好在自己起降点上方', flush=True)
                return
            time.sleep(DISBAND_POLL_S)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()

    def _stop():
        state['stop'] = True
        t.join(timeout=2.0)
    return _stop


def _select_topic_at_a(sdk, point_a):
    """长机在 A 点"选题"。

    2026-09-29 用户明确：长机回 A 点之后**悬停**，不降落（上一版我自作主张
    加了"就地降落"，撤掉）。

    ⚠️ **选题动作本身还没实现**——样题里"选题"具体要做什么（读标志物？
    等地面站指令？）目前没有定义，这里只做到"飞到 A 点、悬停、把状态打出来"。
    等选题的具体动作定了再把这个函数填上，调用点和飞行剖面都不用动。
    """
    ax, ay = float(point_a[0]), float(point_a[1])
    lx, ly, lz = sdk.world_to_local(ax, ay, CRUISE_AGL_M)
    print(f'[长机] 航线跑完，回 A 点 ({ax:.1f}, {ay:.1f}) 选题', flush=True)
    with sdk.fixed_altitude(lz):
        sdk.goto(lx, ly, lz)
    print(f'[长机] 已到 A 点，悬停等待选题'
          f'（选题动作待实现；示例在此悬停 {SELECT_TOPIC_HOVER_S:.0f} 秒后结束，不降落）',
          flush=True)
    time.sleep(SELECT_TOPIC_HOVER_S)
    print('[长机] 选题段结束，保持在 A 点悬停', flush=True)


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
            leader(sdk, _parse_route(args.route), spacing_m=args.spacing)
        else:
            follower(sdk, args.spacing)
        print(f'[{args.namespace}] 编队飞行结束', flush=True)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()

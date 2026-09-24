#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高层火情：按"三角形边中点定点巡检"找着火点，不绕飞。

    bash 运行仿真.sh --task 高层火情巡检版示例.py

为什么不用绕飞了（2026-09-23 赛项改了部署方式）：着火点只会贴在**两根立柱
互相正对的那一面**上，高度只有 1.5 米或 2.5 米两种。那么把三根立柱两两连线
围成一个三角形，每条边的中点正好同时正对着这条边两端的两个立面——在中点上
朝两端各看一眼，就把这条边对应的两个立面查完了；三条边六个立面，加上两个
可能的高度，**最多 12 次观察**就能覆盖所有可能的部署位置。

    12 = 3 个水平点（三条边中点）× 4 次（2 个高度 × 2 个朝向）

每个水平点只转两次机头（用户 2026-09-23 指定的动作顺序，省转向、省爬升）：
  到点 -> 对准一端立柱、检测 -> 保持朝向升/降到另一高度、检测
       -> 原地转 180° 对准另一端、检测 -> 保持朝向回到前一高度、检测
高度变化时朝向不用重新设：水平位置没变，指向同一根立柱的方位角就没变。

比绕飞省时间，也不会出现"绕着 A 楼却先看见贴在 B 楼上的标志"这种归属混乱
（绕飞版见《高楼火情绕飞版示例.py》，那个版本仍然可用，两者不冲突）。

**检测的时机是硬约束**：只在位置和朝向都到位、停稳之后开一个短窗口测一次，
测完就不再看；转动和飞行期间一律不检测（测之前还要清一次检测缓存，否则会
用到转动过程中拍的那一帧——实测踩过）。

两机发射的时序（用户 2026-09-23 要求：两发之间的间隔越短越好）：
  侦察机通报火情后**原地等**，不马上发射
  -> 任务机起飞，飞到离侦察机最近、同高度的那个巡检点待命，到位后告诉侦察机
  -> 侦察机这才发射破窗弹，发完立刻返航
  -> 任务机收到"已破窗"就出发去灭火点，对准、发射灭火弹，返航降落
这样任务机在侦察机动手之前就已经贴着目标待命了，两发之间只差一段几米的飞行。
"""
import argparse
import itertools
import math
import time

import 高楼火情绕飞版示例 as 高楼
from contest_sdk import DroneSDK
from contest_sdk.exceptions import DetectionTimeoutError, GotoUnreachableError, TeammateUnreachableError

BUILDINGS = 高楼.BUILDINGS          # 三根立柱的世界坐标，跟绕飞版共用一份
HIGH_FIRE = 高楼.HIGH_FIRE
FIRE_EVENT = 高楼.FIRE_EVENT

STANDBY_EVENT = '任务机已到待命点'   # 任务机 -> 侦察机
BREACH_EVENT = '侦察机已破窗'        # 侦察机 -> 任务机
WAIT_STANDBY_S = 300.0             # 侦察机等任务机到待命点
WAIT_BREACH_S = 120.0              # 任务机等侦察机破窗
WAIT_NOTIFY_S = 900.0              # 任务机等火情通报

INSPECT_HEIGHTS_M = (1.5, 2.5)     # 赛项规定的两个可能高度，都要查
LOOK_SETTLE_S = 1.5                # 转到位/到高度之后再稳一下，别在机头还在动时测
LOOK_TIMEOUT_S = 2.5               # 每次只给这么长的检测窗口，过了就算这个方向没有
MAX_LOOKS = 12                     # 3 个水平点 × 4 次

# ---- 判断"我是不是正对着这个立面" ----
# 标志是 0.5×0.5 米的正方形：正对时检测框接近正方形（宽高比≈1），斜着看时
# 宽度按 cos(入射角) 收缩、高度基本不变，所以 宽/高 就是 cos(入射角)。
# 为什么需要这个判据（2026-09-24 用户提出的实际情况）：只有**互相正对的那一对**
# 立柱之间才贴着标志，本场地是 1#-2# 相对，2#-3#、1#-3# 并不相对。于是飞机在
# 2#-3# 中点就从侧面（实测入射角约 64°）看到了贴在 2# 上朝着 1# 的标志——看得见，
# 但既不是正对、也不在该去的位置上。光看"看到了"分不出这两种情况，宽高比能。
FACE_ON_ASPECT = 0.80              # 宽高比≥这个算正对（cos36.9°=0.80）
AIM_STANDOFF_M = 3.0               # 正对时停在离标志多远
STANDBY_CLEARANCE_M = 3.0          # 任务机待命点至少离侦察机这么远（见 standby_point）
AFTER_FIRE_HOLD_S = 5.0            # 投弹后先在原地停这么久再返航：侦察机这会儿正在
                                   # 往起飞点飞，两机的返航路线会交叠，错开时间最省事


def inspection_order(start_xy):
    """三条边的中点，按离出发点的远近排。每个中点是一个"水平目标点"。"""
    mids = []
    for a, b in itertools.combinations(BUILDINGS, 2):
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        mids.append((math.dist(start_xy, mid), mid, (a, b)))
    mids.sort(key=lambda m: m[0])
    return [(mid, targets) for _, mid, targets in mids]


def look_once(sdk, target_world, why):
    """**确认位置和朝向都到位、停稳之后，只测一次**。没看到返回 None。

    三条纪律（用户 2026-09-23 要求，前两条都是实测踩出来的）：
    · 机头没转到位不测：那时相机指的不是规定方向，拍到什么都不算"在这个点位、
      这个朝向上看到了火情"；
    · 测之前先清检测缓存：`wait_for_detection()` 查的是"最新一条检测消息"，
      检测节点每帧都发，缓存里那条可能是机头还在转时拍的——不清缓存的话，
      "到位后再测"这个要求光靠调用顺序根本保证不了（实测就是这么在转动过程中
      检出目标的）；
    · 只给一个短窗口（LOOK_TIMEOUT_S），窗口内没有就算这个方向没有，立刻转下
      一个动作——不在飞行/转动期间留着检测一直开。
    """
    tx, ty, _ = sdk.world_to_local(target_world[0], target_world[1], 0.0)
    if not sdk.face_point(tx, ty):       # 已经对准时立刻返回，不会白转
        print(f'[{sdk.namespace}] 朝向没转到位，{why}这次不检测', flush=True)
        return None
    time.sleep(LOOK_SETTLE_S)            # 停稳再看，别在机头收尾时测
    sdk.clear_detections('front')        # 只认清空之后新拍的帧
    try:
        det = sdk.wait_for_detection(HIGH_FIRE, timeout=LOOK_TIMEOUT_S, camera='front')
    except DetectionTimeoutError:
        print(f'[{sdk.namespace}] {why}：没有', flush=True)
        return None
    print(f'[{sdk.namespace}] {why}：发现标志，画面宽 {det.bbox_width:.0f} 像素', flush=True)
    return det


def inspect_for_fire(sdk):
    """按"每个水平点只转两次朝向"的顺序巡检，返回 (检测结果, 位置, 朝向)。

    每个水平点（三角形一条边的中点）的动作固定四步、只转两次机头：
      ① 到点，对准两端立柱里转角较小的那根 -> 检测
      ② 保持朝向，升/降到另一个高度      -> 检测
      ③ 原地转 180° 对准另一根立柱        -> 检测
      ④ 保持朝向，回到前一个高度          -> 检测
    三个水平点跑完正好 12 次检测，覆盖"三条边 × 两个立面 × 两个高度"的全部
    可能部署位置。高度变化时朝向不用管：水平位置没变，指向同一根立柱的方位角
    就没变，traj_server 的 POINT 模式会自己保持。
    """
    here = sdk.local_to_world(*sdk.get_local_position())[:2]
    plan = inspection_order(here)
    print(f'[{sdk.namespace}] 高层火情巡检：{len(plan)} 个水平点 × 4 次检测 = '
          f'{len(plan) * 4} 次（每点只转两次机头）', flush=True)

    looks = 0
    for n, (mid, (a, b)) in enumerate(plan, start=1):
        # 先飞到这个水平点的第一个高度
        h1, h2 = INSPECT_HEIGHTS_M
        print(f'[{sdk.namespace}] 水平点 {n}/{len(plan)}：中点 ({mid[0]:.1f}, {mid[1]:.1f})',
              flush=True)
        try:
            sdk.goto(*sdk.world_to_local(mid[0], mid[1], h1))
        except GotoUnreachableError:
            print(f'[{sdk.namespace}] 这个水平点不可达，跳过', flush=True)
            continue

        # 第一个朝向选转角小的那根，省一次大转向
        yaw_now = sdk.get_current_yaw()
        x, y, _ = sdk.get_local_position()

        def _turn(target):
            tx, ty, _ = sdk.world_to_local(target[0], target[1], 0.0)
            want = math.atan2(ty - y, tx - x)
            return abs((want - yaw_now + math.pi) % (2 * math.pi) - math.pi)

        first, second = (a, b) if _turn(a) <= _turn(b) else (b, a)

        for step, (target, height) in enumerate(
                ((first, h1), (first, h2), (second, h2), (second, h1)), start=1):
            if looks >= MAX_LOOKS:
                break
            if step in (2, 4):              # 保持朝向，只换高度
                print(f'[{sdk.namespace}] 保持朝向，移动到 {height:.1f} m', flush=True)
                try:
                    sdk.goto(*sdk.world_to_local(mid[0], mid[1], height))
                except GotoUnreachableError:
                    print(f'[{sdk.namespace}] 这个高度不可达，跳过', flush=True)
                    continue
            looks += 1
            why = (f'第{looks}/{MAX_LOOKS}次：高 {height:.1f} m 朝 '
                   f'({target[0]:.1f}, {target[1]:.1f})')
            det = look_once(sdk, target, why)
            if det is not None:
                px, py, _ = sdk.get_local_position()
                return det, (px, py), sdk.get_current_yaw()
    return None


def incidence_deg(det):
    """由检测框的宽高比估入射角（0°=正对着立面）。框太小/没高度时返回 None。"""
    if det.bbox_height < 1.0 or det.bbox_width < 1.0:
        return None
    return math.degrees(math.acos(max(0.0, min(1.0, det.bbox_width / det.bbox_height))))


def face_on_candidates(fire_local, owner_local, buildings_local, drone_xy, det):
    """按"哪一面朝哪根柱"的两个候选，各给一个正对位置，按可信度排好序。

    候选只有两个（部署规则：贴在两柱相对的面上，法线必然指向另外某一根柱）。
    排序用测得的入射角（由检测框宽高比反推）跟各假设算出的入射角比，差小的在前。
    宽高比不可信时（框太小、或者换成 YOLO 这类只给粗框的检测器）两个候选的顺序
    就没什么意义了——**但候选本身仍然只有两个**，调用方可以一个个飞过去试，
    按"哪边看到的目标更大/置信度更高"来定，这条退路不依赖宽高比。
    """
    measured = incidence_deg(det)
    vx, vy = drone_xy[0] - fire_local[0], drone_xy[1] - fire_local[1]
    vn = math.hypot(vx, vy) or 1.0
    vx, vy = vx / vn, vy / vn
    out = []
    for bx, by in buildings_local:
        if math.dist((bx, by), owner_local) < 0.5:
            continue                       # 跳过贴标志的那根柱自己
        nx, ny = bx - owner_local[0], by - owner_local[1]
        nn = math.hypot(nx, ny) or 1.0
        nx, ny = nx / nn, ny / nn
        angle = math.degrees(math.acos(max(-1.0, min(1.0, nx * vx + ny * vy))))
        err = abs(angle - measured) if measured is not None else 0.0
        aim = (fire_local[0] + nx * AIM_STANDOFF_M, fire_local[1] + ny * AIM_STANDOFF_M)
        out.append((err, angle, (bx, by), aim))
    out.sort(key=lambda t: t[0])
    return measured, out


def _dist_to_segment(p, a, b):
    """点到线段的距离。"""
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def standby_point(aim_world, aim_z, recon_home=None):
    """任务机的待命点：三条边的中点里，**离侦察机返航路线最远**的那个。

    两条硬要求（都是 2026-09-24 实测撞出来的）：
    · 不能贴着侦察机：它纠正位置之后往往正好停在某个中点附近，选了离它 1.2 米
      的那个点时，机间回避把任务机挡在 1.25 米外进不去，goto() 判不可达；
    · 不能压在侦察机的返航路线上：破窗后侦察机要从瞄准位置飞回自己的起飞点，
      待命点如果在这条线附近，两机又会在那条走廊里互相挤。
    所以先按"离侦察机至少 STANDBY_CLEARANCE_M"筛，再取离"瞄准位置->侦察机起飞点"
    这条线段最远的一个。`recon_home` 由侦察机在通报里一起给出（它自己的起飞点），
    没给就退回"离侦察机最远"这个较弱的判据。
    """
    mids = [((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            for a, b in itertools.combinations(BUILDINGS, 2)]
    cand = [m for m in mids if math.dist(aim_world[:2], m) >= STANDBY_CLEARANCE_M] or mids
    if recon_home is None:
        mid = max(cand, key=lambda m: math.dist(aim_world[:2], m))
    else:
        mid = max(cand, key=lambda m: _dist_to_segment(m, aim_world[:2], recon_home))
        print(f'  （待命点 ({mid[0]:.2f}, {mid[1]:.2f})：离侦察机 '
              f'{math.dist(aim_world[:2], mid):.2f} m，离它的返航路线 '
              f'{_dist_to_segment(mid, aim_world[:2], recon_home):.2f} m）', flush=True)
    return (mid[0], mid[1], aim_z)


def recon_inspect_and_fire(sdk, 任务机就位=None):
    """侦察机的高层火情任务段：定点巡检 -> 对准 -> 通报任务机 -> 发射破窗弹。

    **不起飞、不返航**，跟《高楼火情绕飞版示例.py》的同名任务段保持一致的用法，
    方便《双机全流程示例.py》那种编排直接换用。

    `任务机就位` 是提前注册好的接收器（见 run_recon）；不传就不等待、通报完
    直接发射（单机调试时用）。
    """
    sdk.play_sound_light('侦察机排查高层火情')
    hit = inspect_for_fire(sdk)
    if hit is None:
        print(f'[{sdk.namespace}] {MAX_LOOKS} 个朝向都看过了，没有发现高层火情', flush=True)
        return False
    sdk.play_sound_light('侦察机发现高楼火情')

    det, pos, yaw = hit
    buildings_local = [sdk.world_to_local(bx, by, INSPECT_HEIGHTS_M[-1])[:2] for bx, by in BUILDINGS]
    solved = 高楼.locate_fire(det, pos, yaw, buildings_local)
    if solved is None:
        print(f'[{sdk.namespace}] 看到了标志但判不出它贴在哪栋楼上，就地对准', flush=True)
        fire_local, owner_local = (pos[0], pos[1]), (pos[0], pos[1])
    else:
        fire_local, owner_local = solved
        ow = sdk.local_to_world(owner_local[0], owner_local[1], 0.0)
        print(f'[{sdk.namespace}] 着火点在高楼 ({ow[0]:.1f}, {ow[1]:.1f}) 上', flush=True)

    # 现在的位置到底是不是正对着这个立面？看检测框的宽高比（见 incidence_deg）。
    # 只有相对的那一对立柱之间才贴标志，所以从"不相对"的那条边上看过去必然是斜的，
    # 虽然能看见但不在该去的位置——这一步就是把这两种情况分开。
    _, _, az = sdk.get_local_position()
    aspect = det.bbox_width / det.bbox_height if det.bbox_height >= 1 else 0.0
    if aspect >= FACE_ON_ASPECT:
        print(f'[{sdk.namespace}] 检测框宽高比 {aspect:.2f}，已经是正对着立面，不用挪位置',
              flush=True)
    else:
        print(f'[{sdk.namespace}] 检测框宽高比 {aspect:.2f}（斜看），要挪到正对位置：', flush=True)
        measured, cands = face_on_candidates(fire_local, owner_local, buildings_local, pos, det)
        if measured is not None:
            print(f'  （测得入射角 {measured:.0f}°，候选：'
                  + '；'.join(f'朝({c[2][0]:.1f},{c[2][1]:.1f})该假设下{c[1]:.0f}°' for c in cands)
                  + '）', flush=True)
        # 逐个候选飞过去看：哪个位置上标志显得更大就用哪个。这一步**不依赖宽高比**，
        # 所以换成只给粗框的检测器（比如 YOLO）时也能用——那时宽高比只决定先试
        # 哪个候选，试错本身仍然有效，最多多飞一趟。
        best = None                     # (标志宽度像素, 航点, 火点坐标)
        for k, (_, _, pillar, aim_xy) in enumerate(cands, start=1):
            print(f'[{sdk.namespace}] 试候选{k}：正对位置 ({aim_xy[0]:.2f}, {aim_xy[1]:.2f})'
                  f'（这一面朝 ({pillar[0]:.1f}, {pillar[1]:.1f})）', flush=True)
            sdk.set_yaw_mode_point(*fire_local)     # 出发前设好，路上机头就转到位
            try:
                sdk.goto(aim_xy[0], aim_xy[1], az)
            except GotoUnreachableError:
                print(f'[{sdk.namespace}] 这个正对位置不可达，换下一个候选', flush=True)
                continue
            if not sdk.face_point(*fire_local):
                print(f'[{sdk.namespace}] 到位后朝向没转好，换下一个候选', flush=True)
                continue
            time.sleep(LOOK_SETTLE_S)
            sdk.clear_detections('front')
            try:
                chk = sdk.wait_for_detection(HIGH_FIRE, timeout=LOOK_TIMEOUT_S, camera='front')
            except DetectionTimeoutError:
                print(f'[{sdk.namespace}] 这里看不到标志，换下一个候选', flush=True)
                continue
            ratio = chk.bbox_width / chk.bbox_height if chk.bbox_height >= 1 else 0.0
            print(f'[{sdk.namespace}] 候选{k}：标志宽 {chk.bbox_width:.0f} 像素、宽高比 {ratio:.2f}',
                  flush=True)
            if best is None or chk.bbox_width > best[0]:
                best = (chk.bbox_width, aim_xy)
            if ratio >= FACE_ON_ASPECT:
                print(f'[{sdk.namespace}] 这个候选就是正对面，不再试了', flush=True)
                break
        if best is not None and best[1] != aim_xy:
            # 试到最后停在别的候选上，回到看得最清楚的那个
            try:
                sdk.goto(best[1][0], best[1][1], az)
            except GotoUnreachableError:
                pass

    aimed, fire_local = 高楼.aim_at_fire(sdk, fire_local, buildings_local)
    sdk.clear_detections('front')
    try:
        check = sdk.wait_for_detection(HIGH_FIRE, timeout=3.0, camera='front')
        ratio = check.bbox_width / check.bbox_height if check.bbox_height >= 1 else 0.0
        print(f'[{sdk.namespace}] 复核：检测框宽高比 {ratio:.2f}、宽 {check.bbox_width:.0f} 像素'
              + ('（正对）' if ratio >= FACE_ON_ASPECT else '（仍然偏斜）'), flush=True)
    except DetectionTimeoutError:
        print(f'[{sdk.namespace}] 复核时看不到标志了', flush=True)

    hx, hy, hz = sdk.local_to_world(*sdk.get_local_position())
    fx, fy, _ = sdk.local_to_world(fire_local[0], fire_local[1], 0.0)
    print(f'[{sdk.namespace}] 发现高层火情：着火点约 ({fx:.2f}, {fy:.2f})，'
          f'瞄准位置 ({hx:.2f}, {hy:.2f}, {hz:.2f})', flush=True)

    sdk.play_sound_light('侦察机通报高层火情')
    try:
        px, py, _ = sdk.local_to_world(0.0, 0.0, 0.0)    # 自己的起飞点，任务机选待命点要用
        sdk.send_to_teammate(FIRE_EVENT, x=hx, y=hy, z=hz, fire_x=fx, fire_y=fy,
                             pad_x=px, pad_y=py)
    except TeammateUnreachableError as exc:
        print(f'[{sdk.namespace}] 高层火情通报没送达队友：{exc}', flush=True)

    # 原地等任务机到待命点再动手：这样它在本机发射前就已经贴着目标了
    if 任务机就位 is not None:
        print(f'[{sdk.namespace}] 在瞄准位置等任务机到待命点…', flush=True)
        if 任务机就位.wait(WAIT_STANDBY_S):
            print(f'[{sdk.namespace}] 任务机已就位，发射', flush=True)
        else:
            print(f'[{sdk.namespace}] 等了 {WAIT_STANDBY_S:.0f} 秒没等到任务机就位，先发射', flush=True)

    sdk.play_sound_light('侦察机发射破窗弹')
    高楼.fire_launcher(sdk, '发射破窗弹')
    sdk.play_sound_light('侦察机破窗完成')
    try:
        sdk.send_to_teammate(BREACH_EVENT, timeout_s=30.0)   # 任务机收到才出发
    except TeammateUnreachableError as exc:
        print(f'[{sdk.namespace}] 破窗完成没通知到队友：{exc}', flush=True)
    return True


def run_recon(sdk):
    """侦察机：起飞 -> 定点巡检 -> 等任务机就位 -> 破窗 -> 返航降落。"""
    # 接收器提前注册：任务机可能在本机还没走到"等就位"那一步时就已经到位了，
    # 而可靠事件通道是先回 ACK 再查处理函数，没注册的事件会被确认后丢弃。
    任务机就位 = 高楼.Notice()
    sdk.on_teammate_event(STANDBY_EVENT, 任务机就位.on_event)

    sdk.takeoff()                   # 自动播"侦察机起飞"
    found = recon_inspect_and_fire(sdk, 任务机就位)
    高楼.recon_return_and_land(sdk)
    if found:
        sdk.play_sound_light('侦察机任务完成')


def run_supply(sdk, teammate, 通报=None):
    """任务机：等通报 -> 起飞到待命点报到 -> 等侦察机破窗 -> 去灭火 -> 返航降落。

    `通报` 可以传一个提前注册好的接收器（`高楼.listen_for_report()`）：串着做多个
    任务时，侦察机的高层通报可能在本机还在做上一个任务时就到了，而可靠事件通道是
    先回 ACK 再查处理函数，没注册的会被确认后丢弃。不传就在这里注册（单独跑本
    示例时用）。
    """
    if 通报 is None:
        通报 = 高楼.listen_for_report(sdk)
    已破窗 = 高楼.Notice()
    sdk.on_teammate_event(BREACH_EVENT, 已破窗.on_event)

    print(f'[{sdk.namespace}] 等 {teammate} 通报高层火情…', flush=True)
    if not 通报.wait(WAIT_NOTIFY_S):
        print(f'[{sdk.namespace}] {WAIT_NOTIFY_S:.0f} 秒内没收到高层火情通报，不起飞', flush=True)
        return
    aim = (float(通报.data['x']), float(通报.data['y']), float(通报.data['z']))
    fire = (float(通报.data['fire_x']), float(通报.data['fire_y']))
    print(f'[{sdk.namespace}] 收到高层火情通报：瞄准位置 ({aim[0]:.2f}, {aim[1]:.2f}, {aim[2]:.2f})，'
          f'着火点 ({fire[0]:.2f}, {fire[1]:.2f})', flush=True)

    sdk.takeoff()                   # 自动播"任务机起飞"
    try:
        # 先到待命点：离侦察机最近、同高度的那个巡检点。先在这儿等着，等侦察机
        # 破完窗再进场，两发之间只差这一小段路。
        recon_home = ((float(通报.data['pad_x']), float(通报.data['pad_y']))
                      if 'pad_x' in 通报.data else None)
        sx, sy, sz = standby_point(aim, aim[2], recon_home)
        print(f'[{sdk.namespace}] 到待命点 ({sx:.2f}, {sy:.2f}, {sz:.2f}) 等侦察机破窗', flush=True)
        sdk.set_yaw_mode_point(*sdk.world_to_local(fire[0], fire[1], sz)[:2])
        try:
            sdk.goto(*sdk.world_to_local(sx, sy, sz))
        except GotoUnreachableError as exc:
            # 待命点只是个等待的地方，进不去就在原地等——绝不能因此跳过报到，
            # 那样侦察机会一直等我们（2026-09-24 实测过一次）
            print(f'[{sdk.namespace}] 待命点进不去（{exc}），就地等', flush=True)
        sdk.play_sound_light('任务机高层灭火已就位')
        try:
            sdk.send_to_teammate(STANDBY_EVENT, timeout_s=30.0)
        except TeammateUnreachableError as exc:
            print(f'[{sdk.namespace}] 就位通知没送达 {teammate}：{exc}', flush=True)

        if 已破窗.wait(WAIT_BREACH_S):
            print(f'[{sdk.namespace}] 侦察机已破窗，进场灭火', flush=True)
        else:
            print(f'[{sdk.namespace}] 等了 {WAIT_BREACH_S:.0f} 秒没等到破窗通知，自行进场', flush=True)

        sdk.goto(*sdk.world_to_local(*aim))
        sdk.play_sound_light('任务机到达瞄准点')
        fire_local = sdk.world_to_local(fire[0], fire[1], aim[2])[:2]
        buildings_local = [sdk.world_to_local(bx, by, aim[2])[:2] for bx, by in BUILDINGS]
        高楼.aim_at_fire(sdk, fire_local, buildings_local)

        sdk.play_sound_light('任务机发射灭火弹')
        高楼.fire_launcher(sdk, '发射灭火弹')
        # 等侦察机先走：它破窗后就开始返航，两机的返航走廊是叠在一起的
        print(f'[{sdk.namespace}] 原地等 {AFTER_FIRE_HOLD_S:.0f} 秒让侦察机先返航', flush=True)
        time.sleep(AFTER_FIRE_HOLD_S)
    except GotoUnreachableError as exc:
        print(f'[{sdk.namespace}] 航点不可达：{exc}，提前返航', flush=True)

    pad = sdk.local_to_world(0.0, 0.0, 0.0)
    sdk.set_yaw_mode_constant(sdk.pretakeoff_yaw or sdk.get_current_yaw())
    try:
        sdk.goto(*sdk.world_to_local(pad[0], pad[1], aim[2]))
    except GotoUnreachableError:
        pass
    sdk.goto_direct(0.0, 0.0, aim[2])           # 最后一段收准再落
    sdk.land()                                  # 自动播"任务机降落"
    sdk.play_sound_light('任务机已降落')


def main():
    ap = argparse.ArgumentParser(description='高层火情定点巡检（按 --role 分工）')
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
        print(f'[{sdk.namespace}] 结束', flush=True)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()

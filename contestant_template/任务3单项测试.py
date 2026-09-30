#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务3 单项测试：高层火情侦查 -> 破窗 -> 灭火 -> 编队返航。

前提（2026-09-29 用户指定）：火情**只可能**在 M(6,16) 或 N(14,16) 朝向 90°
（正北 +Y）方向的那两座楼上，高度 1.5 m 或 2.5 m。
  M(6,16) 正北 3.5 m 处是 2# 楼（(5.5,19.5)+(6.5,19.5) 两根合一栋）
  N(14,16) 正北 3.5 m 处是 1# 楼（(13.5,19.5)+(14.5,19.5)）
所以 M/N 是**观察位**，不是楼本身。

流程（2026-09-30 用户重新指定 M 点的动作顺序）：

侦察机 NX01                                任务机 NX02
──────────────────────────────────────     ──────────────────────────
起飞 -> 航点 A B C G E
  （每个航点先把机头转到下一段方向再走，航段内航向不变）
飞到 M，2 m 高度
  ① 朝正南给 3# 楼拍照回传
  ② 原地转 180° 对准正北的 2# 楼
  ③ 检测高层火情并拍照
┌ 情况①：2# 有火情
│  对准 -> 沿机头前移 1.5 m
│  通报火情 ─────────────────────────►    起飞 -> 飞 E 点待命
│  等任务机到 E ◄─────────────────────    到位通报
│  发射破窗弹 ───────────────────────►    收到"已破窗"
│  等灭火完成 ◄──────────────────────     飞到侦察机的发射点，连发 4 发灭火弹
│  ④ 灭完火后飞 N，给 1# 楼拍照
└  飞回 G
┌ 情况②：2# 没有火情
│  转去 N -> 对准 1# 楼 -> 检测并拍照（这一支 1# 当场就拍了，不再补一趟）
└  之后同情况①：通报 -> 破窗 -> 等灭火 -> 回 G
从 G 开始编队返回（规则同《编队飞行示例》），NX01 悬停 A、NX02 降落起降点

三栋楼相对观察位的方位（坐标来自 sample_room_layout.yaml）：
  3# (5.5,12.5) 在 M(6,16) 正南 3.5 m
  2# (5.5,19.5) 在 M(6,16) 正北 3.5 m
  1# (13.5,19.5) 在 N(14,16) 正北 3.5 m
火情只可能在 1#/2# 其中一栋，高度 1.5 m 或 2.5 m。M/N 是**观察位**，不是楼本身。

拍照走 SDK 的 `capture_photo()`（2026-09-30 新增），存进 /logs（挂给地面站的
目录）即视为回传。
"""
import argparse
import math
import time

import 编队飞行示例 as 编队
import 高楼火情绕飞版示例 as 高楼
from contest_sdk import DroneSDK

CRUISE_AGL_M = 编队.CRUISE_AGL_M
SPACING_M = 4.0

ROUTE_A, ROUTE_B = (3.0, 3.0), (3.0, 22.0)
ROUTE_C, ROUTE_G = (17.0, 22.0), (17.0, 16.0)
ROUTE_E, ROUTE_D = (10.0, 16.0), (17.0, 3.0)
POINT_M = (6.0, 16.0)                 # 观察位，正北 3.5 m 是 2# 楼
POINT_N = (14.0, 16.0)                # 观察位，正北 3.5 m 是 1# 楼

HIGH_FIRE = 'apriltag:1'              # 高层着火点标识
OBSERVE_AGL_M = 2.0                   # 用户指定：在 M 点悬停于 2 米高度
# 2026-09-30 用户明确了 M 点的动作顺序：先给 3# 楼拍照回传，再原地转 180°
# 对准 2# 楼查火情并拍照；灭完火之后才去 N 点给 1# 楼拍照，然后编队返航。
# 三栋楼相对观察位的方位（楼的坐标来自 sample_room_layout.yaml）：
#   3# (5.5,12.5) 在 M(6,16) 的**正南**  -> 机头 -90°
#   2# (5.5,19.5) 在 M(6,16) 的**正北**  -> 机头 +90°（从 3# 原地转 180° 过来）
#   1# (13.5,19.5) 在 N(14,16) 的**正北** -> 机头 +90°
YAW_TO_3F_DEG = -90.0                 # M 点朝 3# 楼
YAW_TO_2F_DEG = 90.0                  # M 点转 180° 朝 2# 楼
YAW_TO_1F_DEG = 90.0                  # N 点朝 1# 楼
PHOTO_DIR = '/logs/任务3照片'          # 拍照存这儿（/logs 是挂给地面站的目录）
FIRE_HEIGHTS_M = (1.5, 2.5)           # 火情只可能在这两个高度
FORWARD_BEFORE_FIRE_M = 1.5           # 对准后沿机头前移这么多再发射
EXTINGUISHER_SHOTS = 4                # 任务机连发几发灭火弹
SHOT_INTERVAL_S = 1.0

DETECT_TIMEOUT_S = 6.0                # 在一个朝向/高度上等检测多久
AT_E_WAIT_S = 300.0
BREACH_WAIT_S = 300.0
EXTINGUISH_WAIT_S = 420.0

# 跨机事件
EV_FIRE = 'high_fire_found'           # NX01 -> NX02：火情位置 + 侦察机发射点
EV_AT_E = 'supply_at_standby'         # NX02 -> NX01：我到 E 点待命了
EV_BREACHED = 'breach_done'           # NX01 -> NX02：破窗完成
EV_EXTINGUISHED = 'extinguish_done'   # NX02 -> NX01：灭火弹发射完毕


class _Inbox:
    """一个事件一个信箱。必须在对方可能发之前创建——可靠事件通道先 ACK 再分发，
    没注册处理函数的事件会被确认后丢弃。"""

    def __init__(self, sdk, name):
        import threading
        self.name, self.data = name, {}
        self._ev = threading.Event()
        sdk.on_teammate_event(name, self._on)

    def _on(self, **kw):
        self.data = kw
        self._ev.set()

    def wait(self, timeout):
        if not self._ev.wait(timeout):
            raise TimeoutError(f'等事件 {self.name} 超过 {timeout:.0f} 秒')
        return self.data


def _capture_photo(sdk, tag, camera='front'):
    """拍一张存到 PHOTO_DIR 并打印路径（"回传"= 存进挂给地面站的 /logs）。

    走 SDK 的 `capture_photo()`（2026-09-30 新增）。**不发声光事件**：声光事件
    是固定枚举（见 _sound_light_port.py 的 SOUND_LIGHT_EVENTS），表里没有
    "拍照"这一项，自造事件名会直接抛 ValueError 把整个任务打断——首次跑
    任务3 就是这么挂的。真要给拍照配声光，得先在事件表里加一项。

    拍不到不让整个任务失败：照片是交付物，不是流程前提。
    """
    x, y, _ = sdk.get_local_position()
    wx, wy = sdk.local_to_world(x, y, 0.0)[:2]
    stamp = time.strftime('%H%M%S')
    path = f'{PHOTO_DIR}/{sdk.namespace}_{stamp}_{tag}.png'
    try:
        sdk.set_camera_view(camera)
        time.sleep(0.6)                 # 关节转到位要一点时间，见 set_camera_view 说明
        sdk.capture_photo(path, camera=camera)
        print(f'[{sdk.namespace}] 拍照回传 {tag}：{path}'
              f'（位置 ({wx:.2f}, {wy:.2f})，朝向 {math.degrees(sdk.get_current_yaw()):.0f}°）',
              flush=True)
        return path
    except Exception as exc:
        print(f'[{sdk.namespace}] 拍照失败 {tag}（{exc}），流程继续', flush=True)
        return None


def _goto_world(sdk, wx, wy, what, z_agl=CRUISE_AGL_M):
    lx, ly, lz = sdk.world_to_local(wx, wy, z_agl)
    print(f'[{sdk.namespace}] 飞往{what} ({wx:.2f}, {wy:.2f})', flush=True)
    with sdk.fixed_altitude(lz):
        sdk.goto(lx, ly, lz)


def _fly_route(sdk, legs, z_agl=CRUISE_AGL_M):
    """按航点序列飞，**每个航点先把机头转到下一段的方向、停住，再走**，
    航段之间航向不再变化（2026-09-30 用户要求，跟编队飞行同一条规则）。

    legs 是 [(名字, (wx, wy)), ...]，从飞机**当前位置**出发依次飞过去。
    停顿时长/转向超时/容差都复用《编队飞行示例》的常量，两边保持一致。
    """
    cx, cy, _ = sdk.get_local_position()
    for name, (wx, wy) in legs:
        tx, ty, tz = sdk.world_to_local(wx, wy, z_agl)
        heading = math.atan2(ty - cy, tx - cx)
        print(f'[{sdk.namespace}] 飞往航点{name} ({wx:.2f}, {wy:.2f})，'
              f'航向 {math.degrees(heading):.0f}°（停 {编队.WAYPOINT_HOLD_S:.0f} 秒转向）',
              flush=True)
        t0 = time.time()
        if not sdk.face_yaw(heading, timeout=编队.TURN_TIMEOUT_S,
                            tolerance_deg=编队.TURN_TOL_DEG):
            print(f'[{sdk.namespace}] 航向没转到位，仍继续前飞', flush=True)
        left = 编队.WAYPOINT_HOLD_S - (time.time() - t0)
        if left > 0:
            time.sleep(left)
        with sdk.fixed_altitude(tz):
            sdk.goto(tx, ty, tz)
        cx, cy = tx, ty


def _inspect_here(sdk):
    """在当前位置、当前朝向找高层火情。两个可能高度各看一遍。

    火情贴在楼的立面上，高度 1.5 或 2.5——飞机悬停在 2.0 m，两个高度都在
    前视相机的视场里，所以先原地看；看不到再逐个高度升降过去看一眼。
    返回检测到的 Detection，没找到返回 None。
    """
    try:
        det = sdk.wait_for_detection(HIGH_FIRE, timeout=DETECT_TIMEOUT_S, camera='front')
        print(f'[{sdk.namespace}] 原地看到高层火情', flush=True)
        return det
    except Exception:
        pass
    cx, cy, _ = sdk.get_local_position()
    for h in FIRE_HEIGHTS_M:
        _, _, lz = sdk.world_to_local(0.0, 0.0, h)
        print(f'[{sdk.namespace}] 升到 {h:.1f} m 再看一遍', flush=True)
        sdk.goto_direct(cx, cy, lz)
        time.sleep(1.0)
        try:
            det = sdk.wait_for_detection(HIGH_FIRE, timeout=DETECT_TIMEOUT_S, camera='front')
            print(f'[{sdk.namespace}] 在 {h:.1f} m 高度看到高层火情', flush=True)
            return det
        except Exception:
            continue
    return None


def _aim_and_step_in(sdk):
    """对准火情，然后沿机头方向前移 FORWARD_BEFORE_FIRE_M，返回发射点世界坐标。"""
    try:
        sdk.center_on_target(HIGH_FIRE, timeout=40.0)
        print(f'[{sdk.namespace}] 已对准高层火情', flush=True)
    except Exception as exc:
        print(f'[{sdk.namespace}] center_on_target 没收敛（{exc}），按当前朝向继续',
              flush=True)
    cx, cy, cz = sdk.get_local_position()
    yaw = sdk.get_current_yaw()
    tx = cx + FORWARD_BEFORE_FIRE_M * math.cos(yaw)
    ty = cy + FORWARD_BEFORE_FIRE_M * math.sin(yaw)
    print(f'[{sdk.namespace}] 沿机头前移 {FORWARD_BEFORE_FIRE_M:.1f} m 到发射点', flush=True)
    sdk.goto_direct(tx, ty, cz)
    wx, wy = sdk.local_to_world(tx, ty, 0.0)[:2]
    return float(wx), float(wy), float(cz)


# ----------------------------------------------------------------------------
# 侦察机 NX01
# ----------------------------------------------------------------------------
def recon(sdk):
    at_e = _Inbox(sdk, EV_AT_E)
    done = _Inbox(sdk, EV_EXTINGUISHED)

    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.play_sound_light('侦察机起飞')
    # 巡检航点：每个点先转向下一个点再走，航段内航向恒定
    _fly_route(sdk, [('A', ROUTE_A), ('B', ROUTE_B), ('C', ROUTE_C),
                     ('G', ROUTE_G), ('E', ROUTE_E)])

    # ---- M 点（2 m 高度）----
    # 顺序是用户 2026-09-30 定的：① 先给正南的 3# 楼拍照回传；② 原地转 180°
    # 对准正北的 2# 楼，查火情并拍照；③ 有火就灭；④ 灭完再去 N 给 1# 楼拍照。
    # 注意 N 点那张照片**在灭火之后**——上一版是在破窗之后、灭火之前去拍的。
    _fly_route(sdk, [('M', POINT_M)], z_agl=OBSERVE_AGL_M)
    sdk.face_yaw(math.radians(YAW_TO_3F_DEG))
    _capture_photo(sdk, '3号楼')
    print(f'[{sdk.namespace}] 原地转 180° 对准 2# 楼', flush=True)
    sdk.face_yaw(math.radians(YAW_TO_2F_DEG))
    det = _inspect_here(sdk)
    _capture_photo(sdk, '2号楼')

    found_at = 'M'
    if det is None:
        # 2# 没有火情：按原premise（火情只可能在 1#/2# 其中一栋）转去 N 侧。
        # 这一支里 1# 的照片就在识别现场拍，不必等灭完火再拍一次。
        print(f'[{sdk.namespace}] 2# 楼没有火情，转去 N 点看 1# 楼', flush=True)
        _fly_route(sdk, [('N', POINT_N)], z_agl=OBSERVE_AGL_M)
        sdk.face_yaw(math.radians(YAW_TO_1F_DEG))
        det = _inspect_here(sdk)
        _capture_photo(sdk, '1号楼')
        found_at = 'N'
        if det is None:
            raise RuntimeError('2#、1# 两栋楼都没发现高层火情，任务3 中止')

    fire_pos = _aim_and_step_in(sdk)
    print(f'[{sdk.namespace}] 火情在 {found_at} 侧，发射点 ({fire_pos[0]:.2f}, {fire_pos[1]:.2f})',
          flush=True)
    sdk.send_to_teammate(EV_FIRE, x=fire_pos[0], y=fire_pos[1], z=fire_pos[2], at=found_at)

    # ---- 等任务机到 E 待命，再发射破窗弹 ----
    print(f'[{sdk.namespace}] 等任务机到 E 点待命…', flush=True)
    at_e.wait(AT_E_WAIT_S)
    sdk.play_sound_light('侦察机发射破窗弹')
    高楼.fire_launcher(sdk, '发射破窗弹')
    sdk.send_to_teammate(EV_BREACHED)
    print(f'[{sdk.namespace}] 破窗完成，已通知任务机', flush=True)

    # ---- 等灭火完成，从 G 开始编队返航 ----
    print(f'[{sdk.namespace}] 等任务机灭火完成…', flush=True)
    done.wait(EXTINGUISH_WAIT_S)

    # 灭完火再去 N 给 1# 楼拍照（用户 2026-09-30 指定的顺序），然后回 G 起编队。
    # 火情在 N 侧那一支里 1# 已经拍过了，不重复跑一趟。
    if found_at == 'M':
        _fly_route(sdk, [('N', POINT_N)], z_agl=OBSERVE_AGL_M)
        sdk.face_yaw(math.radians(YAW_TO_1F_DEG))
        _capture_photo(sdk, '1号楼')
    _fly_route(sdk, [('G', ROUTE_G)])
    # start_xy 要的是**世界坐标**（航线本身就是世界系）。这里飞机刚飞到 G，
    # 直接用 ROUTE_G——早先传 get_local_position() 的局部坐标，被当成世界坐标
    # 用，站位点算到了 3# 楼那一片、飞不过去（2026-09-29 实测）。
    编队.leader_route(sdk, [ROUTE_G, ROUTE_D], spacing_m=SPACING_M,
                      start_xy=ROUTE_G, final_xy=ROUTE_A)
    编队._select_topic_at_a(sdk, ROUTE_A)
    sdk.play_sound_light('侦察机任务完成')


# ----------------------------------------------------------------------------
# 任务机 NX02
# ----------------------------------------------------------------------------
def supply(sdk):
    fire = _Inbox(sdk, EV_FIRE)
    breached = _Inbox(sdk, EV_BREACHED)
    route_done = 编队._Inbox(sdk, 编队.ROUTE_DONE)
    route_plan = 编队._Inbox(sdk, 编队.ROUTE_PLAN)

    print(f'[{sdk.namespace}] 等侦察机通报高层火情…', flush=True)
    d = fire.wait(AT_E_WAIT_S)
    fx, fy, fz = float(d['x']), float(d['y']), float(d.get('z', CRUISE_AGL_M))
    print(f'[{sdk.namespace}] 收到火情：{d.get("at")} 侧，发射点 ({fx:.2f}, {fy:.2f})',
          flush=True)

    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.play_sound_light('任务机起飞')

    # ---- 到 E 点待命 ----
    _goto_world(sdk, ROUTE_E[0], ROUTE_E[1], 'E点待命位')
    sdk.send_to_teammate(EV_AT_E)
    print(f'[{sdk.namespace}] 已在 E 点待命，等侦察机破窗', flush=True)
    breached.wait(BREACH_WAIT_S)

    # ---- 到侦察机的发射点，连发 4 发灭火弹 ----
    lx, ly, _lz = sdk.world_to_local(fx, fy, CRUISE_AGL_M)
    print(f'[{sdk.namespace}] 飞往发射点 ({fx:.2f}, {fy:.2f})', flush=True)
    with sdk.fixed_altitude(fz):
        sdk.goto(lx, ly, fz)
    try:
        sdk.center_on_target(HIGH_FIRE, timeout=30.0)
        print(f'[{sdk.namespace}] 已对准高层火情', flush=True)
    except Exception as exc:
        print(f'[{sdk.namespace}] 没对上火情标识（{exc}），按通报坐标发射', flush=True)

    sdk.play_sound_light('任务机发射灭火弹')
    for i in range(1, EXTINGUISHER_SHOTS + 1):
        高楼.fire_launcher(sdk, f'发射灭火弹 {i}/{EXTINGUISHER_SHOTS}')
        if i < EXTINGUISHER_SHOTS:
            time.sleep(SHOT_INTERVAL_S)
    sdk.send_to_teammate(EV_EXTINGUISHED)
    print(f'[{sdk.namespace}] {EXTINGUISHER_SHOTS} 发灭火弹发射完毕，已通知侦察机',
          flush=True)

    # ---- 编队返航：只调空中段，起降由本脚本自己管 ----
    编队.follow_formation(sdk, SPACING_M, inbox=route_done, plan=route_plan)
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

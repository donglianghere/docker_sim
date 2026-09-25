#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机一字纵队编队飞行。两机跑同一份代码，靠 --role 区分。

    python3 编队飞行示例.py --namespace NX01 --role leader  --teammate NX02 \
        --route "7,-9.5 7,9.5 -7,9.5 -7,-9.5" --spacing 4.0
    python3 编队飞行示例.py --namespace NX02 --role follower --teammate NX01 --spacing 4.0

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
READY = 'formation_standby'   # 僚机 -> 长机
ROUTE_DONE = 'route_done'     # 长机 -> 僚机
ROUTE_PLAN = 'route_plan'     # 长机 -> 僚机：整条航线（含长机起飞点），僚机用来定每段航向
# 不传 --route 时用的默认航线（世界坐标），跟《双机全流程示例.py》里的 ROUTE 一致。
# 加默认值是因为 运行仿真.sh 不往任务程序传额外参数，单独跑这个示例时没法给航线。
DEFAULT_ROUTE = '7,-9.5 7,9.5 -7,9.5 -7,-9.5'


def listen_standby(sdk):
    """提前注册"僚机就位"的收件箱。必须在僚机可能发事件之前调用——可靠事件
    通道是先回 ACK 再查处理函数，没注册的事件会被确认后丢弃。"""
    return _Inbox(sdk, READY)


def leader_route(sdk, route_xy, inbox=None):
    """长机的编队任务段：等僚机就位 -> 按航线飞一圈 -> 通知僚机航线已完成。

    **不起飞、不降落**，留给调用方决定，这样《双机全流程示例.py》能把编队接在
    别的任务前面，中间不落地。
    """
    if inbox is None:
        inbox = listen_standby(sdk)
    inbox.wait(READY, STANDBY_WAIT_S)   # 不等僚机就起步，就不是编队了

    pad = _own_pad(sdk)
    waypoints = list(route_xy)
    if tuple(waypoints[-1]) != pad:
        waypoints.append(pad)           # 起飞点当最后一个航点
    # 从起飞点出发，每一段都是"上一个点 -> 这个点"
    legs = list(zip([pad] + waypoints[:-1], waypoints))

    # 把整条航线（含长机起飞点）发给僚机：它的每段航向要用这条航线算，不能靠
    # 从长机轨迹估切线——轨迹是里程计采样点连成的，带噪声，估出来的方向在直线段
    # 上就一直在抖（2026-09-24 实测，用户指出"从机中途航向角一直在变化"）。
    try:
        sdk.send_to_teammate(ROUTE_PLAN, route=[list(p) for p in ([pad] + waypoints)])
    except Exception as exc:                # 送不到不影响自己飞，僚机退化成锁定初始朝向
        print(f'[长机] 航线没送到僚机（{exc}），僚机将保持入列时的朝向', flush=True)

    for i, (frm, to) in enumerate(legs, start=1):
        # 用户 2026-09-24 要求：**先悬停把机头转到这一段的航向角，转到位再前飞**，
        # 整条航段锁死这个角度。用局部坐标算航向：两点之差在纯平移的局部系里跟
        # 世界系完全一致，而 set_yaw_mode_constant()/get_current_yaw() 本来就是
        # 局部系的角度，全程一套坐标不用来回换算。
        fx, fy, _ = sdk.world_to_local(frm[0], frm[1], CRUISE_AGL_M)
        tx, ty, tz = sdk.world_to_local(to[0], to[1], CRUISE_AGL_M)
        heading = math.atan2(ty - fy, tx - fx)
        print(f'[长机] 航点 {i}/{len(legs)}: ({to[0]}, {to[1]})，'
              f'航向 {math.degrees(heading):.0f}°，先转向再前飞', flush=True)
        if not sdk.face_yaw(heading, timeout=TURN_TIMEOUT_S, tolerance_deg=TURN_TOL_DEG):
            # 没转到位也照飞：机头方向不影响这一段能不能到点（编队跟随看的是轨迹），
            # 但要打出来——连续几段都转不到位就是飞行栈那条 yaw 通路出了问题
            print(f'[长机] 航向没转到位（目标 {math.degrees(heading):.0f}°），仍继续前飞',
                  flush=True)
        sdk.goto(tx, ty, tz)            # 航向锁在 heading 上，整段不再变

    sdk.send_to_teammate(ROUTE_DONE)    # 只有长机知道哪个是最后一个航点


def leader(sdk, route_xy):
    """长机：起飞 -> 编队航线 -> 返航降落（单独跑这个示例时的完整流程）。"""
    inbox = listen_standby(sdk)         # 必须在僚机可能发事件之前注册
    sdk.takeoff()
    leader_route(sdk, route_xy, inbox)
    _land_at_pad(sdk)
    sdk.play_sound_light('侦察机任务完成')


def follower(sdk, spacing_m):
    """僚机：起飞 -> 跟队 -> 回起飞点降落。任务机每个任务都要落地，所以这一段
    本来就自带起降，可以直接被《双机全流程示例.py》复用。"""
    inbox = _Inbox(sdk, ROUTE_DONE)
    plan = _Inbox(sdk, ROUTE_PLAN)          # 必须在长机可能发之前就注册
    sdk.takeoff()
    # 这个方法返回的含义是"机载已接管、在自己起飞点上空保持"，不是"已入列"。
    # 必须立刻通知长机——入列要等长机走起来，长机又在等这个通知，等入列会死锁。
    # 高度一并下发：僚机的高度由这个节点按定高雷达保持（天然仿地），默认 1.5 米，
    # 不显式传的话长机改了巡航高度、僚机还停在默认值，编队会一高一低。
    # turn_in_place：僚机跟长机同一套动作——拐点先停下把机头转到下一段的航向，
    # 转到位再前飞（用户 2026-09-24 要求「长机、从机都一样」）
    sdk.start_formation_follow(follow_distance_m=spacing_m,
                               altitude_agl_m=CRUISE_AGL_M,
                               turn_in_place=True)
    sdk.send_to_teammate(READY)
    # 航线必须在**发完 READY 之后**再等：长机要收到 READY 才发航线，反过来写就是
    # 互等（2026-09-24 实测：僚机先等航线、30 秒超时才继续，整段退化成老行为）。
    # 等不到也继续，那种情况下僚机保持入列时锁定的朝向，不影响跟队。
    # 这里等得起：长机收到 READY 后还要先在起飞点转向、再飞出 3.5 米，僚机才会动。
    try:
        plan.wait(ROUTE_PLAN, 30.0)
        sdk.set_formation_leg_route([tuple(p) for p in plan.data.get('route', [])])
    except TimeoutError:
        print('[僚机] 没收到长机航线，本段保持入列时的朝向', flush=True)

    inbox.wait(ROUTE_DONE, ROUTE_DONE_WAIT_S)
    sdk.stop_formation_follow()
    _land_at_pad(sdk)
    sdk.play_sound_light('任务机已降落')


def _land_at_pad(sdk):
    """回自己起飞点降落。goto_direct 直线飞、不经过规划器，落点精度高一个
    量级，但**没有避障**——只用在这种"已在起降点附近、确定无障碍"的收尾。"""
    pad = _own_pad(sdk)
    print(f'[{sdk.namespace}] 回起飞点 {pad} 降落', flush=True)
    sdk.goto_direct(*sdk.world_to_local(pad[0], pad[1], CRUISE_AGL_M))
    sdk.land()


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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机一字纵队编队飞行。两机跑同一份代码，靠 --role 区分。

    python3 编队飞行示例.py --namespace NX01 --role leader  --teammate NX02 \
        --route "7,-10 7,10 -8,10 -8,-10" --spacing 3.5
    python3 编队飞行示例.py --namespace NX02 --role follower --teammate NX01 --spacing 3.5

航点是世界坐标 (x, y)，至少两个，按顺序飞。僚机不需要知道航线，它沿长机
实际飞过的轨迹走，沿轨迹间距不小于 --spacing（这是下限，不是要死守的值）。

声光反馈：起飞/降落由 SDK 自动播（长机="侦察机"、僚机="任务机"），这里只在
落地后各补一条。需要地面站上的声光常驻程序在运行（start_sound_light_server.sh），
没运行也不影响飞行，只会打一行警告。
"""
import argparse
import time

from contest_sdk import DroneSDK

CRUISE_AGL_M = 1.5            # 巡航离地高度
# 长机等僚机就位的上限。必须大于僚机最坏情况的起飞耗时（机载等定位收敛的
# preflight_timeout_s=150秒 + 起飞动作本身60秒），否则僚机还在正常起飞、
# 长机就先判超时终止了。
STANDBY_WAIT_S = 300.0
ROUTE_DONE_WAIT_S = 600.0     # 僚机等航线飞完的上限
READY = 'formation_standby'   # 僚机 -> 长机
ROUTE_DONE = 'route_done'     # 长机 -> 僚机


def leader(sdk, route_xy):
    inbox = _Inbox(sdk, READY)          # 必须在僚机可能发事件之前注册
    sdk.takeoff()
    inbox.wait(READY, STANDBY_WAIT_S)   # 不等僚机就起步，就不是编队了

    pad = _own_pad(sdk)
    waypoints = list(route_xy)
    if tuple(waypoints[-1]) != pad:
        waypoints.append(pad)           # 起飞点当最后一个航点
    for i, (wx, wy) in enumerate(waypoints, start=1):
        print(f'[长机] 航点 {i}/{len(waypoints)}: ({wx}, {wy})', flush=True)
        sdk.goto(*sdk.world_to_local(wx, wy, CRUISE_AGL_M))

    sdk.send_to_teammate(ROUTE_DONE)    # 只有长机知道哪个是最后一个航点
    _land_at_pad(sdk)
    sdk.play_sound_light('侦察机任务完成')


def follower(sdk, spacing_m):
    inbox = _Inbox(sdk, ROUTE_DONE)
    sdk.takeoff()
    # 这个方法返回的含义是"机载已接管、在自己起飞点上空保持"，不是"已入列"。
    # 必须立刻通知长机——入列要等长机走起来，长机又在等这个通知，等入列会死锁。
    sdk.start_formation_follow(follow_distance_m=spacing_m)
    sdk.send_to_teammate(READY)

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
        for name in events:
            sdk.on_teammate_event(name, lambda _n=name, **kw: self._seen.add(_n))

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
    ap.add_argument('--route', default=None, help='"x1,y1 x2,y2 ..."，只有长机需要')
    ap.add_argument('--spacing', type=float, default=3.5, help='沿轨迹间距下限（米）')
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

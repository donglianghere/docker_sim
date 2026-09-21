#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编队飞行示例.py —— 双机一字纵队编队飞行的完整流程。

【这个文件做什么】
两架飞机跑**同一份代码**，只靠命令行参数 `--role` 区分谁是长机、谁是僚机：

    长机(leader)：起飞 -> 等僚机就位 -> 依次飞过你给的航点 -> 回起飞点降落
    僚机(follower)：起飞 -> 接管编队跟随 -> 通知长机可以起步 -> 沿长机
                    轨迹飞 -> 长机报"航线飞完" -> 出列 -> 回起飞点降落

僚机不需要知道航线是什么。它沿长机**实际飞过的轨迹**走，保持沿轨迹的间距
不小于你给的 `--spacing`（米）。间距是**下限**，不是要死守的目标值。

【怎么跑】
两个终端各跑一条（或者用 & 放后台）：

    python3 编队飞行示例.py --namespace NX01 --role leader  --teammate NX02 \
        --route "7,-10 7,10 -8,10 -8,-10" --spacing 3.5
    python3 编队飞行示例.py --namespace NX02 --role follower --teammate NX01 \
        --spacing 3.5

航点坐标是**世界系** (x, y)，单位米，至少两个点，多则不限，按列表顺序飞。
高度统一用下面的 CRUISE_AGL_M（离地高度）。

【这份示例覆盖了哪些能力】
  1. 选手自己输入航点（至少2个，多则不限）+ 间距参数
  2. 编队跟随走 ROS 2 Action（判定回路在机载，选手程序断了飞机也能飞完）
  3. 航线完成通知（长机飞完广播事件，僚机等这个确切信号，不靠估时间）
  4. 两机各自回到自己的起飞点并降落收尾
  5. 长机起飞成功后先悬停（悬停时长由机载起飞判定负责，见 takeoff()）
  6. 长机等僚机就位才起步（否则僚机万一没起飞，长机会自己把航线飞完）

【时序为什么是这样——这里有个会死锁的坑】
僚机的 `start_formation_follow()` 返回的含义是"机载回路已接管、飞机在自己
起飞点上空原地保持"，**不是**"已经入列到间距位置"。入列要等长机走起来，而
长机在等僚机报就位——如果僚机等入列完成才通知长机，两边互相等，谁也动不了。
所以顺序必须是：僚机先接管并立刻通知，入列过程由机载自己在飞行中完成。
"""
import argparse
import sys

from contest_sdk import DroneSDK


# ---- 你可以改的几个数 ----

#: 巡航离地高度（米）。僚机跟随时用定高雷达锁这个高度，不跟随长机的高度变化。
CRUISE_AGL_M = 1.5

#: 长机等僚机报"编队待命"的上限（秒）。僚机那边要先走完"起飞前就绪判定 ->
#: 解锁 -> 爬升稳定 -> 悬停 -> 接管编队控制权"，其中"起飞前就绪"要等定位源
#: 收敛，可能几十秒，所以这个值要留足。等不到就报错终止——不要改成"等不到
#: 就自己先飞"，那样编队没发生却又飞完了航线，比直接失败更难排查。
STANDBY_WAIT_S = 180.0

#: 僚机等长机报"航线飞完"的上限（秒）。按你的航线总长和巡航速度估，留一倍
#: 余量。这个超时只是兜底防止事件丢了永远卡住，正常是收到事件就走。
ROUTE_DONE_WAIT_S = 600.0

#: 两个自定义的跨机事件名。名字随便起，两架飞机对得上就行。
EVENT_FOLLOWER_READY = 'formation_standby'   # 僚机 -> 长机：我就位了，你可以起步
EVENT_ROUTE_DONE = 'route_done'              # 长机 -> 僚机：航线飞完了，出列吧


def leader(sdk: DroneSDK, route_xy, spacing_m: float) -> None:
    """长机：起飞 -> 等僚机就位 -> 飞航线 -> 回起飞点降落。"""
    # 收件箱要在僚机**可能发出事件之前**就注册好，否则事件到了没人收。
    inbox = _Inbox(sdk, EVENT_FOLLOWER_READY)

    sdk.takeoff()   # 内部含机载的起飞前就绪判定 + 爬升稳定判定 + 起飞后悬停
    print('[长机] 起飞完成，等僚机进入编队跟随待命…', flush=True)

    inbox.wait(EVENT_FOLLOWER_READY, STANDBY_WAIT_S)
    print('[长机] 僚机已就位，开始飞航线', flush=True)

    # 把自己的起飞点接在航线末尾当最后一个航点——僚机沿轨迹跟随，自然会被
    # 带回起飞点附近，两机再各自归位降落。
    pad_x, pad_y = _own_pad(sdk)
    waypoints = list(route_xy)
    if tuple(waypoints[-1]) != (pad_x, pad_y):
        waypoints.append((pad_x, pad_y))

    for i, (wx, wy) in enumerate(waypoints, start=1):
        print(f'[长机] 航点 {i}/{len(waypoints)}: ({wx}, {wy})', flush=True)
        lx, ly, lz = sdk.world_to_local(wx, wy, CRUISE_AGL_M)
        sdk.goto(lx, ly, lz)        # 走 ego_planner，有避障

    # 航线飞完的判定只有长机做得了——航线是长机自己定义的，僚机手里没有这个
    # 信息。判定放在信息所在的那一侧，僚机等一个确切信号就行。
    sdk.send_to_teammate(EVENT_ROUTE_DONE)
    print('[长机] 航线飞完，已通知僚机', flush=True)

    _return_home_and_land(sdk, '长机')
    print('[长机] 编队飞行结束', flush=True)


def follower(sdk: DroneSDK, spacing_m: float) -> None:
    """僚机：起飞 -> 接管编队跟随 -> 通知长机 -> 等航线飞完 -> 回起飞点降落。"""
    inbox = _Inbox(sdk, EVENT_ROUTE_DONE)

    sdk.takeoff()
    print('[僚机] 起飞完成，接管编队跟随', flush=True)

    # 走机载 FormationFollow action。返回时机载回路已经接管、飞机在自己起飞点
    # 上空原地保持，等长机起步；入列过程在飞行中由机载自己完成。
    sdk.start_formation_follow(follow_distance_m=spacing_m)
    sdk.send_to_teammate(EVENT_FOLLOWER_READY)
    print(f'[僚机] 已就位待命（沿轨迹间距下限 {spacing_m} 米），已通知长机', flush=True)

    inbox.wait(EVENT_ROUTE_DONE, ROUTE_DONE_WAIT_S)
    print('[僚机] 收到"航线飞完"，出列', flush=True)
    sdk.stop_formation_follow()

    _return_home_and_land(sdk, '僚机')
    print('[僚机] 编队飞行结束', flush=True)


def _return_home_and_land(sdk: DroneSDK, who: str) -> None:
    """回自己的起飞点并降落。

    用 `goto_direct()` 而不是 `goto()`：直线飞、不经过规划器，落点精度高一个
    量级（实测 1.05 米 -> 0.08 米）。代价是**没有避障**，只适用于"已经在起降点
    附近、两点之间确定无障碍"这种收尾场景，不要无脑推广到别的 goto 调用。
    """
    pad_x, pad_y = _own_pad(sdk)
    print(f'[{who}] 返回自己的起飞点 ({pad_x}, {pad_y}) 降落', flush=True)
    lx, ly, lz = sdk.world_to_local(pad_x, pad_y, CRUISE_AGL_M)
    sdk.goto_direct(lx, ly, lz)
    sdk.land()


def _own_pad(sdk: DroneSDK):
    """自己的起飞点世界坐标。

    起飞那一刻飞机就停在自己的起降点上，所以把"局部坐标原点"换算成世界坐标
    就是起飞点——不需要把场地坐标写死在代码里。
    """
    wx, wy, _wz = sdk.local_to_world(0.0, 0.0, 0.0)
    return (round(wx, 2), round(wy, 2))


class _Inbox:
    """最小的跨机事件收件箱。

    `on_teammate_event()` 的回调是在 SDK 的后台线程里跑的，不能在回调里直接
    做飞行动作（会和主线程抢控制权），所以回调只往集合里记一笔，主线程自己
    轮询等待。注册必须在队友可能发出事件**之前**完成。
    """

    def __init__(self, sdk: DroneSDK, *event_names: str):
        self._sdk = sdk
        self._seen = set()
        for name in event_names:
            sdk.on_teammate_event(name, self._make_cb(name))

    def _make_cb(self, name: str):
        def _cb(**_kwargs):
            self._seen.add(name)
        return _cb

    def wait(self, event_name: str, timeout_s: float) -> None:
        import time
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if event_name in self._seen:
                return
            time.sleep(0.2)
        raise TimeoutError(
            f"等队友事件 '{event_name}' 超过 {timeout_s:.0f} 秒仍未到达"
            f"——检查队友程序是否还在运行、双机消息链路是否正常。"
        )


def _parse_route(text):
    """把 "x1,y1 x2,y2 ..." 解析成 [(x1,y1), (x2,y2), ...]。"""
    points = []
    for token in (text or '').split():
        parts = token.split(',')
        if len(parts) != 2:
            raise ValueError(f'航点格式应该是 "x,y"，收到的是 {token!r}')
        points.append((float(parts[0]), float(parts[1])))
    if len(points) < 2:
        raise ValueError(
            f'航线至少需要 2 个航点，--route 只给了 {len(points)} 个。'
            f'编队飞行的意义在于"沿一条航线保持队形"，单点不构成航线。'
        )
    return points


def main() -> None:
    ap = argparse.ArgumentParser(description='双机一字纵队编队飞行示例')
    ap.add_argument('--namespace', required=True, help="这架飞机的命名空间，比如 NX01")
    ap.add_argument('--role', required=True, choices=('leader', 'follower'),
                    help='这架飞机这次当长机还是僚机')
    ap.add_argument('--teammate', required=True, help='队友那架飞机的命名空间')
    ap.add_argument('--route', default=None,
                    help='航线，格式 "x1,y1 x2,y2 ..."（世界坐标，至少2个点）。只有长机需要')
    ap.add_argument('--spacing', type=float, default=3.5,
                    help='沿轨迹的间距下限（米），默认 3.5')
    args = ap.parse_args()

    # SDK 的 role 只认 recon/supply（长机=recon，僚机=supply）；这份示例对外
    # 用 leader/follower 两个更直白的名字，这里做一次映射。
    sdk = DroneSDK(
        namespace=args.namespace,
        role='recon' if args.role == 'leader' else 'supply',
        teammate_namespace=args.teammate,
    )
    try:
        if args.role == 'leader':
            leader(sdk, _parse_route(args.route), args.spacing)
        else:
            follower(sdk, args.spacing)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    sys.exit(main())

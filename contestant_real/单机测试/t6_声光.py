#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单机测试 6：声光反馈全链路（**不起飞**，地面上就能跑）。

默认发 3 条颜色/声音拉得开的事件（蓝→红→白），再加 1 声"完成"收尾，
**自动核对每一条是否真的播出去了**，最后出一张成败表。约 20 秒。
不需要解锁、不需要遥控器、不碰飞控、**不需要飞机在线**。

换了板子或重烧了 wav 想把 20 条全过一遍：`T6_ALL=1 ./run_test.sh t6`（约 1.5 分钟）。

跟 `shell/test_sound_light.sh` 的区别——这是本程序存在的理由：
那个脚本是 `docker exec` 进**声光容器内部**发的（它文件头自己写了，为的是不会
把 ROS 域搞错）。可那恰好**绕过了选手程序真正走的那条路**：程序是从
contestant-sdk 容器、经 ROS 图把 `std_msgs/String` 发到 `/sound_light/request`
的。域号、CycloneDDS 对等体、容器网络任何一处不对，那个脚本都照样响，而真飞
的时候一条都到不了——这种"假就绪"今晚就撞过一次。本程序走的就是真实路径。

**怎么判成功**：订 `/sound_light/status`（常驻程序 1Hz 发的 JSON），看每发一条
之后 `played` 有没有 +1、`last_played` 是不是这一条。所以它能区分三种失败：
  · 请求根本没到常驻程序（played 不涨，而且一开始就收不到 status）→ 域号/网络
  · 到了但被丢弃（dropped 涨）→ 排队超过 15 秒过期，发太密
  · 送到串口了但装置不响 → **软件侧查不出来**，见下面 ⚠️

**收尾会播一条"侦察机任务完成"（绿灯+声音12）当提示音**，不静音——人在旁边靠
这一声判断整轮跑完了。另外 `dry_run=True` 或 `connected=False` 是**硬失败**：串口
没打开的话每条都会"确认播出"、最后给你一个 20/20 的绿屏而全程一声没响，那种通过
毫无意义。

⚠️ **装置不响先查板子有没有电。** CH340 是 USB 供电的转换器，板子断电时串口照样
能打开、写入照样成功、status 里 `connected` 照样是 true、`played` 照样涨——软件
侧完全看不出异常。2026-10-03 为此把整条软件链路查了一遍才发现是没供电。所以本
程序全绿**只证明到串口这一段是通的**，灯色和声音得你自己看/听。

节奏：常驻程序有两个时间约束——最小间隔 2.5 秒（发更密会排队而不是立刻播）、
过期 15 秒（排队超过 15 秒的请求直接丢）。所以每条是"等确认 + 再等满 3.2 秒"，
默认 3 条约 20 秒、全 20 条约 1.5 分钟；有条目确认不了的话每条会多花 6 秒。
"""
import os
import sys
import json
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_sdk import DroneSDK
from contest_sdk._sound_light_port import SOUND_LIGHT_EVENTS, STATUS_TOPIC

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#: 默认只发这几条——颜色和声音都拉开，看一眼听一下就能判断装置正常
#: （2026-10-04 用户要求：有几条就够，不用 20 条全来）。
#: 想把 20 条全过一遍（比如换了板子、重烧了 wav）：`T6_ALL=1 ./run_test.sh t6`
QUICK_EVENTS = (
    '侦察机起飞',            # 蓝 (50,150,255) + 声音1
    '侦察机发现地面火情',    # 红 (255,0,0)     + 声音5
    '侦察机破窗完成',        # 白 (255,255,255) + 声音10
)

#: **本测试不需要飞机**——声光板子在地面站上，整条链路是
#: 选手容器 -> /sound_light/request -> 常驻程序 -> 串口 -> 板子，一步都不经过飞机。
#: run_test.sh 读这个标志来决定跳过"ping 飞机 / 查 control_server /
#: 查 flight_up+vision_up / 收窄 MAVLink / 起监视窗口"这几段前置。
#: 做成程序里的声明而不是让运行器按文件名猜——文件名改了就失效，而且新测试
#: 得回去改运行器的表（这个教训见 run_real.sh 里"相机不在这里配"那段）。
NEEDS_AIRCRAFT = False

#: 发出一条之后最多等多久确认它播出去了。常驻程序最小间隔 2.5 秒，给一倍余量。
CONFIRM_WAIT_S = 6.0
#: 两条之间的间隔。必须 > 常驻程序的最小间隔 2.5 秒，否则会排队、确认逻辑会错位。
GAP_S = 3.2
#: 开头等第一条 status 的超时。等不到就是常驻程序没跑或者域号不对，直接判死。
STATUS_WAIT_S = 10.0


class _StatusWatcher:
    """订 /sound_light/status，随时能读到最新一份状态。

    自己建一个 Node 而不是用 sdk 内部那个：选手程序不该碰 SDK 的私有属性，
    而且这样 spin 的节奏完全由本程序控制，不跟 SDK 自己的执行器抢。
    """

    def __init__(self) -> None:
        self._node = Node('t6_sound_light_status_probe')
        self._latest = None
        self._node.create_subscription(String, STATUS_TOPIC, self._on, 10)

    def _on(self, msg: String) -> None:
        try:
            self._latest = json.loads(msg.data)
        except (ValueError, TypeError):
            pass                      # 坏 JSON 就当没收到，不要把测试搞崩

    def spin_once(self, timeout_s: float = 0.1) -> None:
        rclpy.spin_once(self._node, timeout_sec=timeout_s)

    def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def wait_first(self, timeout_s: float):
        end = time.monotonic() + timeout_s
        while self._latest is None and time.monotonic() < end:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        return self._latest

    @property
    def latest(self):
        return self._latest

    def destroy(self) -> None:
        self._node.destroy_node()


def test(sdk: DroneSDK):
    sdk.progress('=== 测试 6：声光反馈全链路（不起飞）===')
    watcher = _StatusWatcher()
    try:
        _run(sdk, watcher)
    finally:
        watcher.destroy()


def _run(sdk: DroneSDK, watcher: '_StatusWatcher') -> None:
    st = watcher.wait_first(STATUS_WAIT_S)
    if st is None:
        sdk.progress(f'!! {STATUS_WAIT_S:.0f} 秒内收不到 {STATUS_TOPIC} 的状态')
        sdk.progress('   声光常驻程序没在跑，或者它跟本程序不在同一个 ROS 域。')
        sdk.progress('   查：docker ps | grep sound-light')
        sdk.progress('       docker inspect -f "{{range .Config.Env}}{{println .}}{{end}}" '
                     'contestant-sound-light | grep ROS_DOMAIN_ID')
        sdk.progress('   真机要域 20、仿真要域 21。运行器本来会自动掰（ensure_sound_light），'
                     '收不到说明那一步也没成。')
        raise RuntimeError('声光常驻程序不可达，测试无法进行')

    sdk.progress(f'常驻程序在线：串口={st.get("port")}  connected={st.get("connected")}  '
                 f'dry_run={st.get("dry_run")}')
    sdk.progress(f'  起始计数：played={st.get("played")} dropped={st.get("dropped")} '
                 f'queued={st.get("queued")}  最近播放={st.get("last_played")!r}')
    # 这两条是**硬失败**，不是警告。本程序的意义就是"装置真的响"——串口都没打开
    # 的话，下面每条都会"确认播出"（played 照样涨），最后打出一个 20/20 的绿屏，
    # 而全程一声没响。那样的通过毫无意义，所以在这儿就断掉。
    if st.get('dry_run'):
        sdk.progress('!! dry_run=True —— 常驻程序没有真正打开串口，装置不可能响。')
        sdk.progress('   起声光栈时没给串口，或者给错了。重起：')
        sdk.progress('     ./start_sound_light_server.sh --real /dev/ttyUSB0')
        raise RuntimeError('常驻程序处于 dry_run，装置不会响，测试无意义')
    if not st.get('connected'):
        sdk.progress('!! connected=False —— 串口打不开，装置不可能响。')
        sdk.progress('   查：板子插没插、brltty 有没有抢 CH340')
        sdk.progress('     journalctl -k | grep -i brltty')
        sdk.progress('     ls -l /dev/serial/by-id/   # 应有 usb-1a86_USB_Serial-*')
        raise RuntimeError('声光串口未连接，装置不会响，测试无意义')

    # 默认只发 QUICK_EVENTS 那几条；T6_ALL=1 才把 20 条全过一遍。
    full = os.environ.get('T6_ALL', '').strip() not in ('', '0', 'false', 'False')
    names = list(SOUND_LIGHT_EVENTS) if full else list(QUICK_EVENTS)
    total = len(names)
    sdk.progress(f'开始逐条发，共 {total} 条'
                 + ('（T6_ALL=1，全 20 条）' if full
                    else f'（默认精简版；要全 20 条用 T6_ALL=1）')
                 + f'，每条间隔 {GAP_S:.1f} 秒。请看着装置。')

    results = []
    dropped0 = int(st.get('dropped') or 0)
    for i, name in enumerate(names, 1):
        r, g, b, snd, loop, gap_ms = SOUND_LIGHT_EVENTS[name]
        before = int((watcher.latest or {}).get('played') or 0)
        sdk.progress(f'[{i:2d}/{total}] 发送 {name}'
                     f'  （应为 RGB({r},{g},{b}) + 声音{snd}）')
        sdk.announce(name)

        ok = False
        end = time.monotonic() + CONFIRM_WAIT_S
        while time.monotonic() < end:
            watcher.spin_once()
            cur = watcher.latest or {}
            if int(cur.get('played') or 0) > before and cur.get('last_played') == name:
                ok = True
                break
        results.append((name, ok))
        if not ok:
            cur = watcher.latest or {}
            sdk.progress(f'         ✗ {CONFIRM_WAIT_S:.0f} 秒内没确认播出'
                         f'（played={cur.get("played")} last_played={cur.get("last_played")!r}'
                         f' queued={cur.get("queued")} dropped={cur.get("dropped")}）')
        # 确认完再等满间隔：常驻程序的最小间隔是从**上一条真正播出**算的，
        # 所以这里不能把等确认花掉的时间算进间隔里。
        watcher.pump(GAP_S)

    # 收尾**必须发声**，不能静音：这条就是"测完了"的提示音，人在旁边靠它判断
    # 整轮跑完了。原来这里调的是 mute_sound_light()，结果程序最后一个动作是把
    # 装置熄灯静音——测试完反而是安静的，那就没法用耳朵确认收尾。
    DONE_EVENT = '侦察机任务完成'     # 绿 (0,255,0) + 声音12，20 条里最适合当"完成"
    watcher.pump(GAP_S)               # 先等满最小间隔，别让这条被排队
    sdk.progress(f'收尾提示音：{DONE_EVENT}（应为绿灯 + 声音12）')
    sdk.announce(DONE_EVENT)
    watcher.pump(CONFIRM_WAIT_S)

    fin = watcher.latest or {}
    good = [n for n, k in results if k]
    bad = [n for n, k in results if not k]
    sdk.progress('=== 结果 ===')
    sdk.progress(f'  确认播出 {len(good)}/{total}')
    if bad:
        sdk.progress(f'  没确认的 {len(bad)} 条：{"、".join(bad)}')
    sdk.progress(f'  计数变化：played {st.get("played")} -> {fin.get("played")}，'
                 f'dropped {dropped0} -> {fin.get("dropped")}')
    if int(fin.get('dropped') or 0) > dropped0:
        sdk.progress('  !! dropped 涨了：有请求排队超过 15 秒被丢弃，说明发得太密'
                     '或者串口写阻塞。把 GAP_S 调大再试。')
    sdk.progress('  ⚠️ 全绿只证明"请求送到串口"这一段通了。灯色和声音对不对、'
                 '装置有没有电，只有你能看到——没响先查电。')
    if bad:
        raise RuntimeError(f'{len(bad)}/{total} 条没确认播出')


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

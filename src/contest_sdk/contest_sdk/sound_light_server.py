#!/usr/bin/env python3
"""声光反馈常驻程序：常驻选手容器、独占声光板串口，随时待命接收两架
飞机任务进程发来的声光请求，按《声光反馈程序接口.xlsx》编码后写串口。

启动方式（SDK自带，镜像里装好`contest_sdk`就能直接跑）::

    python3 -m contest_sdk.sound_light_server            # 真实串口
    python3 -m contest_sdk.sound_light_server --dry-run  # 没接板子，只打印

部署上用`start_sound_light_server.sh`起一个`--restart unless-stopped`的
独立容器跑它，见该脚本说明。

**为什么需要一个常驻程序，而不是各任务进程自己开串口**（2026-09-17
架构讨论的结论，见`DEBUG_JOURNAL.md`同日条目）：地面站只有一块声光板，
两架飞机的任务进程（`--role recon`/`--role supply`）是同一台电脑上的两个
独立进程。两边各自开串口会互相触发板子的DTR复位（见`_sound_light_port.py`
模块头"真机踩坑"），写入还会交错。所以串口只由这一个进程持有，任务
进程只往`/sound_light/request`话题发请求（`DroneSDK.play_sound_light()`
默认就是这么做的）。顺带的好处：任务进程重启不会再重开串口、不会再吃
一次2秒复位窗口。

**仲裁规则**（`SoundLightScheduler`）：
- 请求按到达顺序排队，逐条发送；两条之间至少间隔`min_gap_s`秒（默认
  2.5秒，2026-09-16真机联调时确认"每条间隔2.5秒"能完整听清）。板子收到
  新指令会立刻切换、不会等上一条播完，不排队的话双机几乎同时触发的两条
  事件，前一条会被后一条瞬间覆盖。循环次数>1的请求按循环次数放大间隔。
- 静音请求插队：清空队列、立刻发送。
- 排队超过`max_age_s`秒还没发出去的请求直接丢弃（比如板子掉线几分钟后
  恢复，不应该把积压的一串过期播报一口气放完）。
- 队列上限`max_queue`条，满了丢最旧的。
- 串口打不开/掉线不退出：每隔`RECONNECT_INTERVAL_S`秒重试，恢复后继续
  处理队列（常驻程序的本分就是"随时待命"，外设插拔不应该让它挂掉）。
"""
import argparse
import collections
import json
import os
import signal
import threading
import time
from dataclasses import dataclass
from typing import Callable, Deque, Optional, Tuple

from contest_sdk._sound_light_port import (
    DEFAULT_SOUND_LIGHT_BAUDRATE,
    DEFAULT_SOUND_LIGHT_PORT,
    DEFAULT_SOUND_LIGHT_TIMEOUT_S,
    MUTE_COMMAND_ARGS,
    REQUEST_TOPIC,
    STATUS_TOPIC,
    SoundLightPort,
    encode_command,
    parse_request,
)

DEFAULT_MIN_GAP_S = 2.5
DEFAULT_MAX_AGE_S = 15.0
DEFAULT_MAX_QUEUE = 20
RECONNECT_INTERVAL_S = 3.0
STATUS_PERIOD_S = 5.0


def _log(text: str) -> None:
    print(f"[sound_light {time.strftime('%H:%M:%S')}] {text}", flush=True)


@dataclass
class _Request:
    label: str
    args: Tuple[int, int, int, int, int, int]
    source: str
    received_at: float

    @property
    def is_mute(self) -> bool:
        return self.args == MUTE_COMMAND_ARGS


class SoundLightScheduler:
    """排队/间隔/过期/重连逻辑，不依赖ROS——`submit()`由订阅回调调用，
    `run()`在主线程里跑。`port`为`None`表示dry-run（只打印不写串口）。
    `clock`可注入，方便单元测试不真的等。
    """

    def __init__(
        self,
        port: Optional[SoundLightPort],
        min_gap_s: float = DEFAULT_MIN_GAP_S,
        max_age_s: float = DEFAULT_MAX_AGE_S,
        max_queue: int = DEFAULT_MAX_QUEUE,
        on_status_change: Callable[[], None] = lambda: None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._port = port
        self._min_gap_s = min_gap_s
        self._max_age_s = max_age_s
        self._max_queue = max_queue
        self._on_status_change = on_status_change
        self._clock = clock

        self._cond = threading.Condition()
        self._queue: Deque[_Request] = collections.deque()
        self._stop = False
        self._next_send_at = 0.0
        self._next_reconnect_at = 0.0
        self._port_error: Optional[str] = None
        self._last_played: Optional[str] = None
        self._played_count = 0
        self._dropped_count = 0

    # ---- 订阅回调侧 ----

    def submit(self, text: str) -> None:
        try:
            label, args, source = parse_request(text)
        except ValueError as exc:
            _log(f"忽略非法请求 {text!r}：{exc}")
            return
        req = _Request(label, args, source, self._clock())
        with self._cond:
            if req.is_mute:
                self._queue.clear()
                self._queue.append(req)
            else:
                if len(self._queue) >= self._max_queue:
                    old = self._queue.popleft()
                    self._dropped_count += 1
                    _log(f"队列已满（{self._max_queue}条），丢弃最旧的请求：{old.label}")
                self._queue.append(req)
            self._cond.notify()
        _log(f"收到请求：{label}（来自{source or '未知'}，排队{len(self._queue)}条）")

    def stop(self) -> None:
        with self._cond:
            self._stop = True
            self._cond.notify()

    def status(self) -> dict:
        with self._cond:
            return {
                'port': None if self._port is None else self._port.port,
                'dry_run': self._port is None,
                'connected': self._port is None or (self._port.is_open and self._port_error is None),
                'error': self._port_error,
                'queued': len(self._queue),
                'played': self._played_count,
                'dropped': self._dropped_count,
                'last_played': self._last_played,
            }

    # ---- 主循环侧 ----

    def run(self) -> None:
        self._try_connect()
        while True:
            req = self._next_request()
            if req is None:
                return
            if req is _RETRY_CONNECT:
                self._try_connect()
                continue
            self._play(req)

    def run_once_for_test(self) -> None:
        """单元测试用：处理当前能处理的一条（不阻塞等待）。"""
        req = self._next_request(block=False)
        if req is _RETRY_CONNECT:
            self._try_connect()
        elif req is not None:
            self._play(req)

    def _next_request(self, block: bool = True):
        with self._cond:
            while True:
                if self._stop:
                    return None
                now = self._clock()
                self._drop_expired(now)
                wait_s = None
                if self._port is not None and self._port_error is not None:
                    # 串口不可用：到点重试连接，队列先留着（过期的会被上面丢掉）
                    if now >= self._next_reconnect_at:
                        return _RETRY_CONNECT
                    wait_s = self._next_reconnect_at - now
                elif self._queue:
                    head = self._queue[0]
                    if head.is_mute or now >= self._next_send_at:
                        return self._queue.popleft()
                    wait_s = self._next_send_at - now
                if not block:
                    return None
                # 有队列时最多睡到"最早那条过期"，保证过期丢弃按时生效
                if self._queue:
                    expire_in = self._queue[0].received_at + self._max_age_s - now
                    wait_s = expire_in if wait_s is None else min(wait_s, expire_in)
                # 最长睡1秒：保证SIGTERM触发的stop()总能在1秒内被注意到
                self._cond.wait(timeout=1.0 if wait_s is None else min(max(wait_s, 0.01), 1.0))

    def _drop_expired(self, now: float) -> None:
        while self._queue and now - self._queue[0].received_at > self._max_age_s:
            old = self._queue.popleft()
            self._dropped_count += 1
            _log(f"请求排队超过{self._max_age_s:.0f}秒未能发出，丢弃：{old.label}")

    def _try_connect(self) -> None:
        if self._port is None:
            return
        try:
            self._port.ensure_open()
        except OSError as exc:
            self._mark_port_error(exc)
            return
        with self._cond:
            recovered = self._port_error is not None
            self._port_error = None
        _log(f"串口{self._port.port}已{'恢复' if recovered else '打开'}，待命中")
        self._on_status_change()

    def _mark_port_error(self, exc: BaseException) -> None:
        first = self._port_error is None
        with self._cond:
            self._port_error = repr(exc)
            self._next_reconnect_at = self._clock() + RECONNECT_INTERVAL_S
        if first:
            _log(f"串口{self._port.port}不可用：{exc!r}；每{RECONNECT_INTERVAL_S:.0f}秒重试一次")
            self._on_status_change()

    def _play(self, req: _Request) -> None:
        command = encode_command(*req.args)
        if self._port is not None:
            try:
                self._port.send(command)
            except OSError as exc:
                self._port.close()
                self._mark_port_error(exc)
                # 放回队首，等重连成功后再发（超时了会被过期逻辑丢掉）
                with self._cond:
                    self._queue.appendleft(req)
                return
        _, _, _, _, repeat, interval_ms = req.args
        hold_s = 0.0 if req.is_mute else (
            self._min_gap_s * max(repeat, 1) + interval_ms / 1000.0 * max(repeat - 1, 0)
        )
        with self._cond:
            self._next_send_at = self._clock() + hold_s
            self._played_count += 1
            self._last_played = req.label
        tag = '[dry-run] ' if self._port is None else ''
        _log(f"{tag}发送：{req.label} -> {command.strip()}（来自{req.source or '未知'}）")
        self._on_status_change()

    def shutdown(self) -> None:
        """退出前熄灯静音+关串口（尽力而为）。"""
        if self._port is None or not self._port.is_open:
            return
        try:
            self._port.send(encode_command(*MUTE_COMMAND_ARGS))
        except OSError:
            pass
        self._port.close()


_RETRY_CONNECT = object()


def _parse_args(argv=None) -> argparse.Namespace:
    env = os.environ.get
    p = argparse.ArgumentParser(description='声光反馈常驻程序（独占声光板串口，订阅/sound_light/request）')
    p.add_argument('--port', default=env('SOUND_LIGHT_PORT', DEFAULT_SOUND_LIGHT_PORT),
                   help='串口设备，默认取SOUND_LIGHT_PORT环境变量，否则/dev/ttyUSB0；'
                        '建议用/dev/serial/by-id/...稳定路径')
    p.add_argument('--baudrate', type=int, default=DEFAULT_SOUND_LIGHT_BAUDRATE)
    p.add_argument('--min-gap', type=float, default=float(env('SOUND_LIGHT_MIN_GAP_S', DEFAULT_MIN_GAP_S)),
                   help='两条指令之间的最小间隔（秒），0=不限')
    p.add_argument('--max-age', type=float, default=float(env('SOUND_LIGHT_MAX_AGE_S', DEFAULT_MAX_AGE_S)),
                   help='请求排队超过这么多秒仍未发出就丢弃')
    p.add_argument('--dry-run', action='store_true', default=env('SOUND_LIGHT_DRY_RUN', '') == '1',
                   help='不打开串口，只打印将要发送的指令（仿真/没接板子时用）')
    return p.parse_args(argv)


def main(argv=None) -> None:
    args = _parse_args(argv)

    # 延迟import：上面的调度逻辑不依赖ROS，单元测试可以不装rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from contest_sdk._rclpy_runtime import RclpyRuntime

    runtime = RclpyRuntime('sound_light')
    node = runtime.node

    status_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    status_pub = node.create_publisher(String, STATUS_TOPIC, status_qos)

    scheduler: Optional[SoundLightScheduler] = None

    def publish_status() -> None:
        if scheduler is not None:
            status_pub.publish(String(data=json.dumps(scheduler.status(), ensure_ascii=False)))

    port = None if args.dry_run else SoundLightPort(args.port, args.baudrate, DEFAULT_SOUND_LIGHT_TIMEOUT_S)
    scheduler = SoundLightScheduler(
        port, min_gap_s=args.min_gap, max_age_s=args.max_age, on_status_change=publish_status,
    )

    request_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
        depth=50, durability=DurabilityPolicy.VOLATILE,
    )
    node.create_subscription(String, REQUEST_TOPIC, lambda msg: scheduler.submit(msg.data), request_qos)
    node.create_timer(STATUS_PERIOD_S, publish_status)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: scheduler.stop())

    _log(
        f"启动：port={'(dry-run)' if port is None else args.port}，最小间隔{args.min_gap}s，"
        f"过期{args.max_age}s，订阅{REQUEST_TOPIC}，状态{STATUS_TOPIC}"
    )
    try:
        scheduler.run()
    finally:
        _log('退出：熄灯静音、关闭串口')
        scheduler.shutdown()
        runtime.shutdown()


if __name__ == '__main__':
    main()

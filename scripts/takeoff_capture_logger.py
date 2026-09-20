#!/usr/bin/env python3
"""起飞瞬间专项抓取：CPU频率(另见宿主机tegrastats)之外的全部关键遥测——
DLIO里程计、两路IMU(Mid-360雷达IMU + 飞控合成IMU)、vision_pose(喂给PX4
EKF的位姿输入)的频率与冻结/跳变检测，以及"其他可能影响起飞的东西"：
mavros/state武装状态切换、mavros/extended_state着陆状态切换、
mavros/statustext（PX4自己上报的告警/错误文本，CAN总线错误、failsafe等
都走这条）、mavros/battery（电池电压在螺旋桨大电流负载下是否掉压）、
mavros/estimator_status（EKF健康标志位）、/diagnostics（mavros聚合的
全局诊断，非OK级别才记）。

背景：本项目此前多次在真机ulog/rosbag离线分析里实锤过DLIO里程计"冻结几秒
→跳变一大截"这种灾难性发散（见DEBUG_JOURNAL.md 2026-09-04系列记录），
但都是事后离线分析。这次趁真机已连接、即将起飞，直接在起飞瞬间做实时
在线抓取，机会难得，尽量把所有此前怀疑跟发散有关的信号一次性录全。

用法（在flight-stack容器内跑，namespace从参数读）：
  python3 takeoff_capture_logger.py NX02
输出：追加写到 /logs/<namespace>/takeoff_capture_<启动时间戳>.log
（/logs是docker-compose挂载到宿主机runtime_logs/的volume，容器销毁/
重建数据不会丢）；如果/logs不存在，退回/tmp，不报错。
"""

import os
import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State, ExtendedState, StatusText
try:
    from mavros_msgs.msg import EstimatorStatus
    HAVE_ESTIMATOR_STATUS = True
except ImportError:
    HAVE_ESTIMATOR_STATUS = False
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray


def log_path_for(ns: str) -> str:
    logs_dir = f'/logs/{ns}'
    ts = time.strftime('%Y%m%d_%H%M%S')
    if os.path.isdir('/logs'):
        os.makedirs(logs_dir, exist_ok=True)
        return f'{logs_dir}/takeoff_capture_{ts}.log'
    return f'/tmp/takeoff_capture_{ts}.log'


class FreqTracker:
    """滑动窗口频率统计 + 冻结/跳变检测（专门盯DLIO那种"位置连续冻结
    几秒后突然跳一大截"的发散模式）。"""

    def __init__(self, name, jump_thresh_m=2.0, freeze_thresh_s=1.0):
        self.name = name
        self.count_window = 0
        self.last_pos = None
        self.freeze_since = None
        self.last_freeze_log_t = None
        self.jump_thresh_m = jump_thresh_m
        self.freeze_thresh_s = freeze_thresh_s
        self.total = 0
        self.last_t = None

    def on_msg(self, t, pos=None):
        self.count_window += 1
        self.total += 1
        events = []
        if pos is not None:
            if self.last_pos is not None:
                d = math.dist(pos, self.last_pos)
                if d < 1e-4:
                    if self.freeze_since is None:
                        self.freeze_since = t
                        self.last_freeze_log_t = None
                    elif t - self.freeze_since > self.freeze_thresh_s and \
                            (self.last_freeze_log_t is None or t - self.last_freeze_log_t > 1.0):
                        events.append(f"FROZEN {t - self.freeze_since:.2f}s @ {pos}")
                        self.last_freeze_log_t = t
                else:
                    if d > self.jump_thresh_m:
                        events.append(f"JUMP {d:.2f}m {self.last_pos}->{pos}")
                    self.freeze_since = None
                    self.last_freeze_log_t = None
            self.last_pos = pos
        self.last_t = t
        return events

    def tick_and_reset(self, window_s):
        hz = self.count_window / window_s
        self.count_window = 0
        return hz


class RollingStats:
    """窗口内min/mean/max，每个tick重置——用来看振动幅度(IMU加速度/角速度
    的std代理，这里用极差近似)、点云每帧点数分布、点云处理延迟这些"这一
    窗口内波动有多大"的问题，不需要精确std，min/mean/max足够看出异常。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.sum = 0.0
        self.sumsq = 0.0
        self.vmin = None
        self.vmax = None

    def add(self, v):
        self.n += 1
        self.sum += v
        self.sumsq += v * v
        self.vmin = v if self.vmin is None else min(self.vmin, v)
        self.vmax = v if self.vmax is None else max(self.vmax, v)

    def summary_and_reset(self):
        if self.n == 0:
            s = "n=0"
        else:
            mean = self.sum / self.n
            var = max(0.0, self.sumsq / self.n - mean * mean)
            std = math.sqrt(var)
            s = f"n={self.n} mean={mean:.3f} std={std:.3f} min={self.vmin:.3f} max={self.vmax:.3f}"
        self.reset()
        return s


class TakeoffCaptureLogger(Node):
    def __init__(self, ns: str):
        super().__init__(f'{ns.lower()}_takeoff_capture_logger')
        self.ns = ns
        self.t0 = time.monotonic()
        path = log_path_for(ns)
        self.log_file = open(path, 'a', buffering=1)
        self._log(f"# takeoff_capture_logger started ns={ns} t0(monotonic)={self.t0} path={path}")
        print(f"[takeoff_capture_logger] logging to {path}", flush=True)

        self.armed = None
        self.landed_state = None

        best_effort = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        reliable_small = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                                     history=HistoryPolicy.KEEP_LAST)

        self.trackers = {
            'dlio_odom': FreqTracker('dlio_odom'),
            'mid360_imu': FreqTracker('mid360_imu'),
            'fcu_imu': FreqTracker('fcu_imu'),
            'vision_pose': FreqTracker('vision_pose'),
            'pointcloud': FreqTracker('pointcloud'),
        }

        # 振动代理：两路IMU的加速度模长/角速度模长窗口统计
        self.mid360_accel_stats = RollingStats()
        self.mid360_gyro_stats = RollingStats()
        self.fcu_accel_stats = RollingStats()
        self.fcu_gyro_stats = RollingStats()
        # 每帧点云点数 + 处理延迟(now - header.stamp，判断是否积压)
        self.pc_count_stats = RollingStats()
        self.pc_latency_stats = RollingStats()
        self.pc_low_count_thresh = 500  # 低于这个数就单独记一条，具体阈值供事后再调

        self.create_subscription(Odometry, f'/{ns}/dlio/odom_node/odom',
                                  self._dlio_odom_cb, best_effort)
        self.create_subscription(Imu, f'/{ns}/mid360/imu',
                                  self._mid360_imu_cb, best_effort)
        self.create_subscription(Imu, f'/{ns}/mavros/imu/data_raw',
                                  self._fcu_imu_cb, best_effort)
        self.create_subscription(PoseWithCovarianceStamped, f'/{ns}/mavros/vision_pose/pose_cov',
                                  self._vision_pose_cb, best_effort)
        self.create_subscription(PointCloud2, f'/{ns}/mid360_PointCloud2',
                                  self._pointcloud_cb, best_effort)

        self.create_subscription(State, f'/{ns}/mavros/state',
                                  self._state_cb, reliable_small)
        self.create_subscription(ExtendedState, f'/{ns}/mavros/extended_state',
                                  self._extstate_cb, reliable_small)
        self.create_subscription(StatusText, f'/{ns}/mavros/statustext/recv',
                                  self._statustext_cb, best_effort)
        self.create_subscription(BatteryState, f'/{ns}/mavros/battery',
                                  self._battery_cb, best_effort)
        self.create_subscription(DiagnosticArray, '/diagnostics',
                                  self._diag_cb, reliable_small)
        if HAVE_ESTIMATOR_STATUS:
            self.create_subscription(EstimatorStatus, f'/{ns}/mavros/estimator_status',
                                      self._estimator_cb, best_effort)

        self.last_diag_level = {}
        self.window_s = 2.0
        self.create_timer(self.window_s, self._tick)

    def _t(self):
        return time.monotonic() - self.t0

    def _log(self, line):
        self.log_file.write(f"{self._t():.3f} {line}\n")

    # ---- 高频话题：DLIO/两路IMU/vision_pose ----
    def _dlio_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        events = self.trackers['dlio_odom'].on_msg(self._t(), (p.x, p.y, p.z))
        for e in events:
            self._log(f"[DLIO_ODOM] {e}")

    def _mid360_imu_cb(self, msg: Imu):
        self.trackers['mid360_imu'].on_msg(self._t())
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.mid360_accel_stats.add(math.sqrt(a.x**2 + a.y**2 + a.z**2))
        self.mid360_gyro_stats.add(math.sqrt(g.x**2 + g.y**2 + g.z**2))

    def _fcu_imu_cb(self, msg: Imu):
        self.trackers['fcu_imu'].on_msg(self._t())
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.fcu_accel_stats.add(math.sqrt(a.x**2 + a.y**2 + a.z**2))
        self.fcu_gyro_stats.add(math.sqrt(g.x**2 + g.y**2 + g.z**2))

    def _vision_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        events = self.trackers['vision_pose'].on_msg(self._t(), (p.x, p.y, p.z))
        for e in events:
            self._log(f"[VISION_POSE] {e}")

    def _pointcloud_cb(self, msg: PointCloud2):
        self.trackers['pointcloud'].on_msg(self._t())
        count = msg.width * msg.height
        self.pc_count_stats.add(count)
        try:
            latency = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            self.pc_latency_stats.add(latency)
        except Exception:
            latency = None
        if count < self.pc_low_count_thresh:
            lat_str = f"{latency:.3f}s" if latency is not None else "?"
            self._log(f"[POINTCLOUD] LOW_COUNT points={count} latency={lat_str}")

    # ---- 事件型话题：状态切换/告警，只在变化时记 ----
    def _state_cb(self, msg: State):
        if msg.armed != self.armed:
            self._log(f"[STATE] armed {self.armed}->{msg.armed} mode={msg.mode} connected={msg.connected}")
            self.armed = msg.armed

    def _extstate_cb(self, msg: ExtendedState):
        if msg.landed_state != self.landed_state:
            names = {0: 'UNDEFINED', 1: 'ON_GROUND', 2: 'IN_AIR', 3: 'TAKEOFF', 4: 'LANDING'}
            self._log(f"[EXT_STATE] landed_state {self.landed_state}->{msg.landed_state}"
                      f" ({names.get(msg.landed_state, '?')})")
            self.landed_state = msg.landed_state

    def _statustext_cb(self, msg: StatusText):
        self._log(f"[STATUSTEXT] severity={msg.severity} text={msg.text!r}")

    def _battery_cb(self, msg: BatteryState):
        # 电压/电流本身较慢变化，2s tick里附带打印，这里只catch明显掉压骤降
        if not hasattr(self, '_last_batt_v'):
            self._last_batt_v = msg.voltage
        if self._last_batt_v - msg.voltage > 1.0:
            self._log(f"[BATTERY] voltage SAG {self._last_batt_v:.2f}V->{msg.voltage:.2f}V "
                      f"current={msg.current:.2f}A")
        self._last_batt_v = msg.voltage
        self._last_batt = msg

    def _diag_cb(self, msg: DiagnosticArray):
        for s in msg.status:
            if s.level != 0:  # 0=OK
                key = s.name
                if self.last_diag_level.get(key) != s.level:
                    self._log(f"[DIAG] level={s.level} name={s.name!r} msg={s.message!r}")
                self.last_diag_level[key] = s.level

    def _estimator_cb(self, msg):
        flags = getattr(msg, 'flags', None)
        if not hasattr(self, '_last_est_flags') or self._last_est_flags != flags:
            self._log(f"[ESTIMATOR_STATUS] flags={flags}")
            self._last_est_flags = flags

    # ---- 周期性频率汇总 ----
    def _tick(self):
        parts = []
        for name, tr in self.trackers.items():
            hz = tr.tick_and_reset(self.window_s)
            parts.append(f"{name}={hz:.1f}Hz(n={tr.total})")
        batt = ''
        if hasattr(self, '_last_batt'):
            batt = f" battery={self._last_batt.voltage:.2f}V/{self._last_batt.current:.2f}A"
        self._log(f"[TICK] {' '.join(parts)}{batt} armed={self.armed} landed_state={self.landed_state}")
        self._log(f"[VIB] mid360_accel({self.mid360_accel_stats.summary_and_reset()}) "
                  f"mid360_gyro({self.mid360_gyro_stats.summary_and_reset()}) "
                  f"fcu_accel({self.fcu_accel_stats.summary_and_reset()}) "
                  f"fcu_gyro({self.fcu_gyro_stats.summary_and_reset()})")
        self._log(f"[PC_STATS] count({self.pc_count_stats.summary_and_reset()}) "
                  f"latency({self.pc_latency_stats.summary_and_reset()})")


def main():
    if len(sys.argv) < 2:
        print("用法: python3 takeoff_capture_logger.py <NAMESPACE>", file=sys.stderr)
        sys.exit(1)
    ns = sys.argv[1]
    rclpy.init()
    node = TakeoffCaptureLogger(ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.write(f"# stopped t={node._t():.3f}\n")
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""4路相机长驻记录器（sim-world容器内）：每5秒一行，每路：帧数、仿真时间帧率、stamp最大间隔、
重复stamp数、最后一帧均值/标准差/分辨率/编码；另记两机高度和RTF。每300秒存一张每路快照(PNG)。"""
import time, zlib, struct, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped

CAMS = [(ns, c) for ns in ('NX01', 'NX02') for c in ('front', 'down')]
def png(path, arr):
    h, w = arr.shape[:2]; raw = b''.join(b'\x00' + arr[y].tobytes() for y in range(h))
    ch = lambda t, d: struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) + ch(b'IDAT', zlib.compress(raw, 6)) + ch(b'IEND', b''))
class L(Node):
    def __init__(self):
        super().__init__('cam4_logger')
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.st = {k: [] for k in CAMS}; self.lastimg = {}
        for k in CAMS:
            ns, c = k
            self.create_subscription(Image, f'/{ns}/{ns}_{c}_camera/image_raw', lambda m, k=k: self.on_img(k, m), q)
        self.clk = []; self.z = {}
        self.create_subscription(Clock, '/clock', lambda m: self.clk.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), qos_profile_sensor_data)
        for ns in ('NX01', 'NX02'):
            self.create_subscription(PoseStamped, f'/{ns}/mavros/local_position/pose', lambda m, ns=ns: self.z.__setitem__(ns, m.pose.position.z), qos_profile_sensor_data)
        self.t0 = time.time(); self.last = self.t0; self.next_snap = 0
        self.create_timer(5.0, self.report)
    def on_img(self, k, m):
        self.st[k].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); self.lastimg[k] = m
    def report(self):
        now = time.time(); dt = now - self.last; self.last = now; el = now - self.t0
        rtf = 'na'
        if len(self.clk) >= 2 and self.clk[-1][0] > self.clk[0][0]:
            rtf = f'{(self.clk[-1][1] - self.clk[0][1]) / (self.clk[-1][0] - self.clk[0][0]):.2f}'
        sim = self.clk[-1][1] if self.clk else float('nan'); self.clk = self.clk[-1:]
        out = [f'{time.strftime("%H:%M:%S")} wall+{el:.0f}s sim={sim:.1f} rtf={rtf} z01={self.z.get("NX01", float("nan")):.2f} z02={self.z.get("NX02", float("nan")):.2f}']
        snap = el >= self.next_snap
        for k in CAMS:
            s = self.st[k]; self.st[k] = []; tag = f'{k[0]}_{k[1]}'
            if len(s) >= 2:
                span = s[-1] - s[0]; gaps = [b - a for a, b in zip(s, s[1:])]; dup = sum(1 for g in gaps if g <= 0)
                m = self.lastimg[k]; a = np.frombuffer(m.data, np.uint8)
                out.append(f'{tag}: n={len(s)} sim_hz={(len(s) - 1) / span if span > 0 else 0:.1f} max_gap={max(gaps):.2f}s dup={dup} img={m.width}x{m.height}/{m.encoding} mean={a.mean():.1f} std={a.std():.1f}')
                if snap and m.encoding == 'rgb8':
                    png(f'/tmp/cam4_snap_{tag}_{int(el):05d}s.png', a.reshape(m.height, m.width, 3))
            else:
                out.append(f'{tag}: n={len(s)}')
        if snap: self.next_snap = el + 300
        print(' | '.join(out), flush=True)
rclpy.init(); rclpy.spin(L())

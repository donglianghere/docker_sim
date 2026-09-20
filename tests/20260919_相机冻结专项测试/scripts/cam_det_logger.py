#!/usr/bin/env python3
"""4路相机 + 两机vision/detections 长驻记录器（sim-world容器内跑）。
每5秒一行：各路相机帧数/仿真帧率/最大间隔；各机检测消息数+检测到的class_id集合；两机高度。"""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
CAMS = [(ns, c) for ns in ('NX01', 'NX02') for c in ('front', 'down')]
class L(Node):
    def __init__(self):
        super().__init__('cam_det_logger')
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.st = {k: [] for k in CAMS}
        for k in CAMS:
            ns, c = k
            self.create_subscription(Image, f'/{ns}/{ns}_{c}_camera/image_raw', lambda m, k=k: self.st[k].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), q)
        self.det = {ns: [0, set()] for ns in ('NX01', 'NX02')}
        for ns in ('NX01', 'NX02'):
            self.create_subscription(Detection2DArray, f'/{ns}/vision/detections', lambda m, ns=ns: self.on_det(ns, m), 10)
        self.clk = []; self.z = {}
        self.create_subscription(Clock, '/clock', lambda m: self.clk.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), qos_profile_sensor_data)
        for ns in ('NX01', 'NX02'):
            self.create_subscription(PoseStamped, f'/{ns}/mavros/local_position/pose', lambda m, ns=ns: self.z.__setitem__(ns, m.pose.position.z), qos_profile_sensor_data)
        self.t0 = time.time(); self.create_timer(5.0, self.report)
    def on_det(self, ns, m):
        self.det[ns][0] += 1
        for d in m.detections:
            if d.results: self.det[ns][1].add(d.results[0].hypothesis.class_id)
    def report(self):
        now = time.time(); el = now - self.t0
        rtf = 'na'
        if len(self.clk) >= 2 and self.clk[-1][0] > self.clk[0][0]:
            rtf = f'{(self.clk[-1][1]-self.clk[0][1])/(self.clk[-1][0]-self.clk[0][0]):.2f}'
        sim = self.clk[-1][1] if self.clk else float('nan'); self.clk = self.clk[-1:]
        out = [f'{time.strftime("%H:%M:%S")} wall+{el:.0f}s sim={sim:.1f} rtf={rtf} z01={self.z.get("NX01",float("nan")):.2f} z02={self.z.get("NX02",float("nan")):.2f}']
        for k in CAMS:
            s = self.st[k]; self.st[k] = []
            if len(s) >= 2:
                span = s[-1]-s[0]; gaps = [b-a for a, b in zip(s, s[1:])]
                out.append(f'{k[0]}_{k[1]}: n={len(s)} sim_hz={(len(s)-1)/span if span>0 else 0:.1f} max_gap={max(gaps):.2f}s')
            else:
                out.append(f'{k[0]}_{k[1]}: n={len(s)}')
        for ns in ('NX01', 'NX02'):
            c, ids = self.det[ns]; self.det[ns] = [0, set()]
            out.append(f'{ns}_det: msgs={c} ids={sorted(ids) if ids else "-"}')
        print(' | '.join(out), flush=True)
rclpy.init(); rclpy.spin(L())

#!/usr/bin/env python3
"""相机图像接收探针：argv= name qos(best|reliable) topic [delay_s]
每5秒打印收到帧数/最后一帧距今秒数；同时打印自己进程的Cyclone socket丢包。"""
import sys, os, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
name, qos_kind, topic = sys.argv[1], sys.argv[2], sys.argv[3]
delay = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
if delay: time.sleep(delay)
def drops():
    inodes = set()
    for fd in os.listdir(f'/proc/{os.getpid()}/fd'):
        try: t = os.readlink(f'/proc/{os.getpid()}/fd/{fd}')
        except OSError: continue
        if t.startswith('socket:['): inodes.add(t[8:-1])
    out = []
    for line in open('/proc/net/udp').read().splitlines()[1:]:
        f = line.split()
        if f[9] in inodes and f[-1] != '0': out.append(f'{f[1].split(":")[1]}:{f[-1]}')
    return ','.join(out) or '-'
rclpy.init(args=['--ros-args', '-r', f'__node:=probe_{name}'])
n = Node(f'probe_{name}')
q = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
               reliability=ReliabilityPolicy.RELIABLE if qos_kind == 'reliable' else ReliabilityPolicy.BEST_EFFORT)
st = {'n': 0, 'total': 0, 'last': 0.0}
def cb(m):
    st['n'] += 1; st['total'] += 1; st['last'] = time.time()
n.create_subscription(Image, topic, cb, q)
t0 = time.time()
def rep():
    now = time.time()
    since = f'{now-st["last"]:.0f}s' if st['last'] else 'never'
    print(f'{time.strftime("%H:%M:%S")} probe={name} qos={qos_kind} t+{now-t0:.0f}s n5={st["n"]} total={st["total"]} last_frame={since} drops={drops()}', flush=True)
    st['n'] = 0
n.create_timer(5.0, rep)
rclpy.spin(n)

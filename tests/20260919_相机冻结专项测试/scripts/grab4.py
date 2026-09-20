import sys, time, zlib, struct, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
def png(path, arr):
    h, w = arr.shape[:2]; raw = b''.join(b'\x00' + arr[y].tobytes() for y in range(h))
    ch = lambda t, d: struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) + ch(b'IDAT', zlib.compress(raw, 6)) + ch(b'IEND', b''))
rclpy.init(); n = Node('grab4'); got = {}
tag = sys.argv[1]
for ns in ('NX01', 'NX02'):
    for c in ('front', 'down'):
        k = f'{ns}_{c}'
        n.create_subscription(Image, f'/{ns}/{ns}_{c}_camera/image_raw', lambda m, k=k: got.setdefault(k, m), qos_profile_sensor_data)
t = time.time()
while len(got) < 4 and time.time() - t < 15: rclpy.spin_once(n, timeout_sec=0.2)
for k, m in got.items():
    png(f'/tmp/grab_{tag}_{k}.png', np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3))
    print(k, m.header.stamp.sec, 'ok')

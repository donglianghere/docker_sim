#!/usr/bin/env python3
"""起飞→飞到世界坐标(x,y,z)上空→悬停（进程退出后靠pt4ctrl AUTO_HOVER保持）。
argv: ns world_x world_y world_z"""
import sys
from contest_sdk import DroneSDK
ns, wx, wy, wz = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
mate = 'NX02' if ns == 'NX01' else 'NX01'
sdk = DroneSDK(namespace=ns, role='camtest', teammate_namespace=mate)
try:
    sdk.takeoff()
    lx, ly, lz = sdk.world_to_local(wx, wy, wz)
    print(f'{ns} world({wx},{wy},{wz}) -> local({lx:.2f},{ly:.2f},{lz:.2f})', flush=True)
    sdk.goto(lx, ly, lz)
    print(f'ARRIVED {ns}', flush=True)
finally:
    sdk.shutdown()

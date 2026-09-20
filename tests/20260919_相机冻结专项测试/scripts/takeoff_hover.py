#!/usr/bin/env python3
"""原地起飞到takeoff_height(1.0m)后退出，飞机由pt4ctrl AUTO_HOVER保持悬停。argv[1]=NX01/NX02"""
import sys
from contest_sdk import DroneSDK
ns = sys.argv[1]; mate = 'NX02' if ns == 'NX01' else 'NX01'
sdk = DroneSDK(namespace=ns, role='camtest', teammate_namespace=mate)
try:
    sdk.takeoff()
    print(f'TAKEOFF_OK {ns}', flush=True)
finally:
    sdk.shutdown()

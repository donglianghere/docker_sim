# -*- coding: utf-8 -*-
# 用法（仿真需已启动且两机PX4报Ready for takeoff）：
#   docker run --rm --network host -v $PWD/scripts:/w contestant-sdk:latest \
#       python3 -u /w/verify_goto_unreachable.py
"""验证：航点落在障碍圆柱里时 goto() 尽快抛 GotoUnreachableError，跳过后能继续飞。

障碍圆柱在世界坐标 (7,0)，直径0.5，仿真膨胀0.6。依次飞：
  A (5,-2)  正常点
  B (7, 0)  障碍圆柱正中心 —— 期望几秒内抛 GotoUnreachableError（不是等60秒）
  C (5, 2)  正常点 —— 期望跳过 B 之后照常飞到
"""
import time

from contest_sdk import DroneSDK
from contest_sdk.exceptions import GotoUnreachableError

sdk = DroneSDK(namespace='NX01', role='recon', teammate_namespace='NX02')
Z = 1.5
results = []
try:
    sdk.takeoff()
    for name, wx, wy in (('A', 5.0, -2.0), ('B', 7.0, 0.0), ('C', 5.0, 2.0)):
        t0 = time.monotonic()
        try:
            sdk.goto(*sdk.world_to_local(wx, wy, Z))
            results.append((name, 'reached', time.monotonic() - t0, None))
        except GotoUnreachableError as e:
            results.append((name, 'UNREACHABLE', time.monotonic() - t0, e.distance_m))
        print(f'>>> {results[-1]}', flush=True)
    pad = sdk.local_to_world(0.0, 0.0, 0.0)
    sdk.goto_direct(*sdk.world_to_local(pad[0], pad[1], Z))
    sdk.land()
finally:
    print('\n===== 结果 =====', flush=True)
    for name, st, dt, d in results:
        extra = f'，停在离目标 {d:.2f} 米处' if d is not None else ''
        print(f'  {name}: {st:12s} 用时 {dt:5.1f} 秒{extra}', flush=True)
    sdk.shutdown()

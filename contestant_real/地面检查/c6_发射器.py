#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面检查 6：发射机构验证——连发 3 次，每发之间隔 3 秒。**不起飞、不解锁。**

用法::

    ./c6_发射器.sh              # 默认 NX02（2026-10-15 实测通过）
    ./c6_发射器.sh NX01         # 侦察机也要发破窗弹，**还没在 NX01 上验过**

⚠️ 跑之前确认：机构前方无人。
"""
import sys

from contest_sdk import DroneSDK

NEEDS_AIRCRAFT = True          # 要连飞机（执行器指令走 mavros）

SHOTS = 3                      # 连发几次
INTERVAL_S = 3.0               # 两发之间隔多久

NS = sys.argv[1] if len(sys.argv) > 1 else 'NX02'

sdk = DroneSDK(namespace=NS, role='supply' if NS == 'NX02' else 'recon',
               teammate_namespace=NS)

sdk.shoot(SHOTS, interval_s=INTERVAL_S, label='发射')

sdk.shutdown()

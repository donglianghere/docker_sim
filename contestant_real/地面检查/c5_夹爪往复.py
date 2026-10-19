#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面检查 5：夹爪张开/闭合 3 次，相邻两次动作间隔 3 秒。**不起飞、不解锁。**

接线、PWM 行程、张开/闭合是哪个方向，全在 SDK 的 SERVO_CONFIG 里，这里不关心。
（爪子是 MAIN7+MAIN9，共用 Actuator Set1，PWM 900~2100；2026-10-15 改装后
900=抓紧、2100=打开，跟上一版相反——这个程序正是用来现场确认张/合没做反的。
两机同样设置，换机号改 namespace 即可。）

⚠️ `grip()` 自己要花 GRIP_TRAVEL_S 等舵机走到位——舵机没有位置反馈，软件读不到
角度，只能等。所以这里只补差额（INTERVAL_S 减掉它），直接写 sleep(3) 会变成
"3 秒 + 行程时间"。GRIP_TRAVEL_S 改了这个程序不用动。
"""
import time

from contest_sdk import DroneSDK

NEEDS_AIRCRAFT = True          # 要连飞机（执行器指令走 mavros）

INTERVAL_S = 3.0                                               # 想要的动作间隔
WAIT_S = max(0.0, INTERVAL_S - DroneSDK.GRIP_TRAVEL_S)         # 扣掉 grip() 自带的等待

sdk = DroneSDK(namespace='NX02', role='supply', teammate_namespace='NX02')

for i in range(1, 4):
    sdk.grip(release=True, label=f'张开 {i}/3')
    time.sleep(WAIT_S)
    sdk.grip(label=f'闭合 {i}/3')
    time.sleep(WAIT_S)

sdk.shutdown()

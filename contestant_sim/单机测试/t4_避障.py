# -*- coding: utf-8 -*-
"""单机测试 4：避障专项。

A 点起飞 -> A↔B 往返三趟 -> 回家。

**这条航段上有考核用的障碍物**，A→B 之间立着圆柱（仿真里是 (3,8) 那根 φ0.5），
所以这段**必须走规划器**、不能直飞。程序里用 fly_route/goto_world（都经 ego_planner），
全程不碰 goto_direct。

看什么：
  · 监视窗口里的轨迹应该绕开柱子，不是直线
  · 日志里不该出现反复重规划失败（"ERROR! the drone is in obstacle"）
  · 往返多趟是为了看**一致性**——偶尔绕过去不算过，三趟都稳才算

坐标来自 ../venue.py。
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_sdk import DroneSDK
from venue import ROUTE_A, ROUTE_B, CRUISE_AGL_M

LAPS = 3


def test(sdk):
    sdk.progress('=== 测试 4：A↔B 往返避障 ===')
    sdk.progress(f'A{ROUTE_A} <-> B{ROUTE_B}，共 {LAPS} 个来回')
    sdk.progress('这段有障碍物，全程走规划器——不用直飞，避障本身就是要测的东西')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    for i in range(1, LAPS + 1):
        sdk.progress(f'--- 第 {i}/{LAPS} 趟：A -> B ---')
        sdk.goto_world(ROUTE_B[0], ROUTE_B[1], CRUISE_AGL_M, what='B点')
        sdk.progress(f'--- 第 {i}/{LAPS} 趟：B -> A ---')
        sdk.goto_world(ROUTE_A[0], ROUTE_A[1], CRUISE_AGL_M, what='A点')

    sdk.progress(f'{LAPS} 个来回全部完成，返航')
    sdk.return_home(sound='侦察机任务完成')


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

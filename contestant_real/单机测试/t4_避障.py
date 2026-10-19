# -*- coding: utf-8 -*-
"""单机测试 4：避障专项。

A 点起飞 -> A↔B 往返三趟 -> 回家。

**这条航段上有考核用的障碍物**，A→B 之间立着圆柱（仿真里是 (3,8) 那根 φ0.5），
所以这段**必须走规划器**、不能直飞。程序里用 fly_route/goto_world（都经 ego_planner），
全程不碰 goto_direct。

全程限速 0.4 m/s（2026-10-09 用户指定）。改的是 `ego_planner` 的巡航限速
（`manager/max_vel`），管的正是本测试用的 `goto_world()`；`set_max_vel()` 是
**节点级**参数、不会自己复位，所以结束时在 finally 里还回原值，免得后面的
测试和返航精修带着这个限速跑。

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
#: 本测试的规划器巡航限速，米/秒。下限别再往下压：`goto()` 的卡住检测要求实际
#: 速度持续在 0.1 m/s 以上，而限速 0.25 时实测只跑出 0.15 m/s，会被判成不可达
#: （见 capabilities.py 里 LEG_SLOW_MPS 那段注释）。
MAX_VEL_MPS = 0.8


def test(sdk: DroneSDK):
    sdk.progress('=== 测试 4：A↔B 往返避障 ===')
    sdk.progress(f'A{ROUTE_A} <-> B{ROUTE_B}，共 {LAPS} 个来回')
    sdk.progress('这段有障碍物，全程走规划器——不用直飞，避障本身就是要测的东西')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    old_vel = sdk.set_max_vel(MAX_VEL_MPS)
    try:
        for i in range(1, LAPS + 1):
            sdk.progress(f'--- 第 {i}/{LAPS} 趟：A -> B ---')
            sdk.goto_world(ROUTE_B[0], ROUTE_B[1], CRUISE_AGL_M, what='B点')
            sdk.progress(f'--- 第 {i}/{LAPS} 趟：B -> A ---')
            sdk.goto_world(ROUTE_A[0], ROUTE_A[1], CRUISE_AGL_M, what='A点')

        sdk.progress(f'{LAPS} 个来回全部完成，返航')
        sdk.return_home(sound='侦察机任务完成')
    finally:
        # 读不到原值时 set_max_vel() 返回 0.0，那种情况下别拿 0 去设（会抛
        # ValueError 把真正的异常盖掉），留给下一次启动按 launch 默认值恢复。
        if old_vel > 0:
            sdk.set_max_vel(old_vel)


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

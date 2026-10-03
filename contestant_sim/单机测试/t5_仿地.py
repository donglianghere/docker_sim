# -*- coding: utf-8 -*-
"""单机测试 5：仿地飞行专项。

G 点起飞 -> G↔D 往返三趟 -> 回家。

**这条航段下方有高低起伏**（仿真里 (17,9.5) 有 terrain_module），正好用来看
"仿地"效果：飞机应当保持**离地高度**恒定，也就是地形升高时飞机跟着升高，
而不是保持海拔高度不变、贴着地形削过去。

怎么判断过没过：
  · 监视窗口的高度曲线：地形段应该看到飞机高度跟着抬起来
  · 对照组在程序里：第 3 趟改用 fixed_altitude 定高飞一遍，两者高度曲线
    一对比，仿地有没有生效一眼就看出来

⚠ 定高那一段用的是**局部系 z**，而且必须等于该段 goto 的 z，否则永远判不到点。
  这是记录在案的坑，程序里按正确写法来。

坐标来自 ../venue.py。
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_sdk import DroneSDK
from venue import ROUTE_G, ROUTE_D, CRUISE_AGL_M

LAPS = 3


def test(sdk):
    sdk.progress('=== 测试 5：G↔D 往返仿地 ===')
    sdk.progress(f'G{ROUTE_G} <-> D{ROUTE_D}，共 {LAPS} 个来回')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    for i in range(1, LAPS + 1):
        if i < LAPS:
            sdk.progress(f'--- 第 {i}/{LAPS} 趟：仿地模式（保持离地 {CRUISE_AGL_M} m）---')
            sdk.goto_world(ROUTE_D[0], ROUTE_D[1], CRUISE_AGL_M, what='D点')
            sdk.goto_world(ROUTE_G[0], ROUTE_G[1], CRUISE_AGL_M, what='G点')
        else:
            # 对照组：定高飞一趟。高度曲线跟前两趟一比，仿地有没有生效很直观。
            sdk.progress(f'--- 第 {i}/{LAPS} 趟：定高对照（不跟地形起伏）---')
            _, _, z = sdk.world_to_local(ROUTE_D[0], ROUTE_D[1], CRUISE_AGL_M)
            with sdk.fixed_altitude(z):
                sdk.goto_world(ROUTE_D[0], ROUTE_D[1], CRUISE_AGL_M, what='D点(定高)')
                sdk.goto_world(ROUTE_G[0], ROUTE_G[1], CRUISE_AGL_M, what='G点(定高)')

    sdk.progress('仿地测试完成，对比监视窗口的高度曲线：'
                 '前两趟应随地形起伏，第三趟应是平的')
    sdk.return_home(sound='侦察机任务完成')


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

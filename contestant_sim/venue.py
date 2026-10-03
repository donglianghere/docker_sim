# -*- coding: utf-8 -*-
"""场地参数——**仿真**。

四个选手程序的代码两边逐字节相同，差异全部收在这一个文件里。
真机那份在 `contestant_real/venue.py`，改坐标只改那边，程序一行都不用动。

坐标系：来自 `src/contest_mission/config/sample_room_layout.yaml`——
房间 20×25 米，世界原点在**西南角**，坐标全为正（x∈[0,20]，y∈[0,25]）。

⚠ 这里写的是**世界系**坐标，而 `goto()` 吃的是飞机**自己的局部系**。
  SDK 的 `world_to_local()` 负责换算，依赖起飞点原点锁定那条 TF。
"""

# ---- 航线点位 ----
ROUTE_A = (3.0, 3.0)      # 起降区
ROUTE_B = (3.0, 22.0)
ROUTE_C = (17.0, 22.0)
ROUTE_G = (17.0, 16.0)    # 编队集结 / 解散
ROUTE_E = (10.0, 16.0)
ROUTE_F = (14.0, 14.0)
ROUTE_D = (17.0, 3.0)

# ---- 观察位 ----
POINT_M = (6.0, 16.0)     # 看 3# / 2# 楼
POINT_N = (14.0, 16.0)    # 看 1# 楼

# ---- 物资点 ----
SUPPLY_XY = (10.0, 8.0)

# ---- 目标标识 ----
SUPPLY_TAG  = 'apriltag:0'
HIGH_FIRE   = 'apriltag:1'
GROUND_FIRE = 'apriltag:2'

# ---- 朝向 ----
# 火情只贴在楼的 -Y 面 -> 正对楼面就是机头朝正北。
FACADE_YAW_DEG = 90.0
# (标签, 观察位, 观察位名, 机头朝向°, 这站要不要查火情)
# 按 3#->2#->1# 走。3# 只拍照——火情只可能在 1#/2#。2# 跟 3# 同在 M 点，原地转 180°。
STATIONS = (('3#楼', POINT_M, 'M', -90.0, False),
            ('2#楼', POINT_M, 'M',  90.0, True),
            ('1#楼', POINT_N, 'N',  90.0, True))

# ---- 高度 ----
CRUISE_AGL_M  = 2.0
OBSERVE_AGL_M = 2.0

# ---- 非场地参数（两边相同，放这里只为集中）----
# ⚠ 弹丸的要求，不是场地参数，别按场地改
FORWARD_BEFORE_FIRE_M = 1.5
SHOTS = 4

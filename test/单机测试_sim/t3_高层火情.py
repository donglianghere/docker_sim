# -*- coding: utf-8 -*-
"""单机测试 3：高层火情瞄准专项。

M 点（或 N 点）起飞 -> 拍照 -> 前视精确对准高层火情 -> 回家。
默认用 M 点，加 --n 用 N 点。**具体看的是哪栋楼不写死在这里**，按观察位名
从 venue.STATIONS 查——楼号是场地参数，仿真/真机两边编号可以不一样
（2026-10-11 仿真把 1# 和 3# 互换过），写死就得改两边的程序代码，而两份
程序必须逐字节相同（run_real.sh 1.8 节的闸门）。

**这个测试专查 aim_at 的前视几何**，跟 t2 是两条不同的路：前视用相机内参
自己算角度和距离（2026-10-03 起从 camera_info 读，真机 2026-10-15 起四路
共用一组标定值 **fx≈735.46 / cx≈616.22 / cy≈376.47**，水平 FOV 82°——换
摄像头之前是 fx≈1200/FOV 56°，差 1.62 倍），而下视转给 precision_servo 的
像素闭环。所以 t2 过了不代表 t3 过。

⚠ 必须先 face_yaw 正对楼面再对准——aim_at 是"锁住当前朝向只做平移"的，
  朝向不对的话画面里居中了、机身却斜着。

坐标来自 ../venue.py。
"""
import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_sdk import DroneSDK
from venue import (POINT_M, POINT_N, HIGH_FIRE, FACADE_YAW_DEG,
                   CRUISE_AGL_M, OBSERVE_AGL_M, STATIONS)

#: 观察位名 -> 楼号标签，只收"要查火情"的那些站（另一站是只拍照的那栋）。
#: 查不到就退回观察位名本身，少个好看的名字而已，不让测试因为标签挂掉。
_BLDG_AT = {s[2]: s[0] for s in STATIONS if s[4]}

PHOTO_DIR = '/logs/测试3_高层火情'


def test(sdk: DroneSDK):
    # 只开需要的那一路相机（拍照 + 瞄准高层火情用前视）。仿真下是空操作。
    sdk.set_camera_mode('front')
    use_n = '--n' in sys.argv
    spot, name = (POINT_N, 'N') if use_n else (POINT_M, 'M')
    bldg = _BLDG_AT.get(name, f'{name}点那栋')
    sdk.progress(f'=== 测试 3：高层火情瞄准（{name} 点看 {bldg}）===')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress(f'飞往观察位 {name}{spot}，升到观察高度 {OBSERVE_AGL_M} m')
    sdk.goto_world(spot[0], spot[1], OBSERVE_AGL_M, what=f'{name}点')

    # 正对楼面：火情贴在楼的 -Y 面，所以机头朝正北
    # face_yaw 吃的是弧度。这一下是为了拍照时就正对楼面；后面 aim_at 还会
    # 用 face_yaw_deg 再确保一次（它内部也会转），两处不冲突。
    sdk.progress(f'机头转到 {FACADE_YAW_DEG}° 正对楼面')
    sdk.face_yaw(math.radians(FACADE_YAW_DEG))

    sdk.snapshot(f'{bldg}_巡检', camera='front', directory=PHOTO_DIR)
    sdk.progress('巡检照片已回传')

    sdk.progress(f'前视搜索高层火情 {HIGH_FIRE}')
    if sdk.look_for(HIGH_FIRE, camera='front', what='高层火情'):
        sdk.play_sound_light('侦察机发现高楼火情')
        # 前视：aim_at 自己的几何，用 camera_info 的内参
        ok = sdk.aim_at(HIGH_FIRE, camera='front',
                        face_yaw_deg=FACADE_YAW_DEG, what='高层火情')
        if ok:
            sdk.play_sound_light('侦察机通报高层火情')
            sdk.snapshot(f'{bldg}_已对准', camera='front', directory=PHOTO_DIR)
            sdk.progress('对准成功，照片已回传')
        else:
            sdk.progress('!! 没能对准——前视几何用的是 camera_info 的内参，先确认 '
                         '{ns}_front_camera/camera_info 的 fx 是 735 那组标定值；'
                         '读到 381 是订阅错话题后走的回退值，1200 是老摄像头的旧值')
    else:
        sdk.progress(f'!! {name} 点前视看不到 {HIGH_FIRE}——确认标识摆位、朝向和视觉已起')

    sdk.progress('高层火情测试完成，返航')
    sdk.return_home(sound='侦察机排查高层火情')


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

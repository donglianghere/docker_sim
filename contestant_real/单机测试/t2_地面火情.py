# -*- coding: utf-8 -*-
"""单机测试 2：地面火情对准专项。

G 点起飞 -> 飞往 E 点 -> 下视精确瞄准地面火情 -> 拍照回传 -> 回家。

**下视对准走的不是 aim_at 自己的几何**：aim_at(camera='down') 会转给
center_on_target() 的 precision_servo 闭环（像素级 P 控制，不用相机内参）。
所以这个测试验的是 precision_servo 这条链：检测话题 -> frame_id 判据 ->
precision_land_cmd -> position_cmd_relay -> pt4ctrl。

坐标来自 ../venue.py。
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_sdk import DroneSDK
from venue import ROUTE_G, ROUTE_E, GROUND_FIRE, CRUISE_AGL_M

PHOTO_DIR = '/logs/测试2_地面火情'


def test(sdk):
    # 只开需要的那一路相机（搜索 + 对准地面火情用下视）。仿真下是空操作。
    sdk.set_camera_mode('down')
    sdk.progress('=== 测试 2：地面火情对准 ===')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress(f'G{ROUTE_G} -> E{ROUTE_E}')
    sdk.goto_world(ROUTE_E[0], ROUTE_E[1], CRUISE_AGL_M, what='E点')

    sdk.progress(f'下视搜索地面火情 {GROUND_FIRE}')
    if sdk.look_for(GROUND_FIRE, camera='down', what='地面火情'):
        sdk.play_sound_light('侦察机发现地面火情')
        # 下视：转给 precision_servo 闭环
        ok = sdk.aim_at(GROUND_FIRE, camera='down', what='地面火情')
        if ok:
            sdk.play_sound_light('侦察机通报地面火情')
            sdk.snapshot('地面火情已对准', camera='down', directory=PHOTO_DIR)
            sdk.progress('对准成功，照片已回传')
        else:
            sdk.progress('!! 没能对准——查 precision_servo 这条链：'
                         '检测话题有没有、frame_id 是不是带 _camera_down_')
    else:
        sdk.progress(f'!! E 点下视看不到 {GROUND_FIRE}——先确认标识摆位和视觉已起')

    sdk.progress('地面火情测试完成，返航')
    sdk.return_home(sound='侦察机任务完成')


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

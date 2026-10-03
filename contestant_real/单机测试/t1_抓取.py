# -*- coding: utf-8 -*-
"""单机测试 1：物资抓取专项。

起飞 -> 飞到物资点 -> 边瞄边降到底 -> 抓紧 -> 起飞 -> 原地放回 -> 回家。

**抓取这一步最容易出问题的不是飞行，是舵机**：PWM 值、输出口、COM_PREARM_MODE
三件事里错一件，飞机会稳稳降在物资上但夹爪不动，而且不报错。所以程序里每一步
都把状态播出来，便于在现场对照着看。

坐标来自 ../venue.py。
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_sdk import DroneSDK
from venue import SUPPLY_XY, SUPPLY_TAG, CRUISE_AGL_M


def test(sdk):
    # 只开需要的那一路相机（物资点精降看 AprilTag，用下视）。仿真下是空操作。
    sdk.set_camera_mode('down')
    sdk.progress('=== 测试 1：物资抓取 ===')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress(f'飞往物资点 {SUPPLY_XY}，标识 {SUPPLY_TAG}')
    # fetch_from 一步做完：飞过去 -> 边瞄边降到底 -> 抓 -> 起飞回巡航高度。
    # sound 在落地后、夹爪动作前播，所以听到播报就该看见夹爪闭合。
    sdk.fetch_from(SUPPLY_XY, SUPPLY_TAG, what='灭火弹',
                   agl_m=CRUISE_AGL_M, sound='任务机抓取灭火弹')
    sdk.progress('抓取完成，已回到巡航高度')

    # 原地放回：不挪位置，只验"松开"这一下也正常。
    sdk.progress('原地降落放回，验证松开')
    sdk.land_on(SUPPLY_TAG, '物资点')
    sdk.play_sound_light('任务机投放灭火弹')
    sdk.grip(release=True, label='放回灭火弹')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('抓取测试完成，返航')
    sdk.return_home(sound='任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

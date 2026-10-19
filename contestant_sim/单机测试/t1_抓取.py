# -*- coding: utf-8 -*-
"""单机测试 1：物资抓取专项。

开爪 -> 起飞 -> 飞到物资点 -> 边瞄边降到底 -> 抓紧 -> 起飞 -> 原地放回 -> 回家。

**抓取这一步最容易出问题的不是飞行，是舵机**：PWM 值、输出口、COM_PREARM_MODE
三件事里错一件，飞机会稳稳降在物资上但夹爪不动，而且不报错。所以程序里每一步
都把状态播出来，便于在现场对照着看。

坐标来自 ../venue.py。
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_sdk import DroneSDK
from venue import SUPPLY_XY, SUPPLY_TAG, CRUISE_AGL_M


def test(sdk: DroneSDK):
    # 2026-10-16 补：起飞前先把爪子打开，跟 rounds.supply_ground 开头同款。
    # 飞机这会儿还停在地上，开爪的行程时间（GRIP_TRAVEL_S）不占飞行时间。
    # ⚠️ **fetch_from() 内部只发"抓紧"、不会先开爪** —— 上一轮结束时爪子若停在
    # 闭合位，这个程序会稳稳降在物资上、发一条"抓紧"给本来就闭着的爪子，然后
    # 抓着空气飞走，**而且不报错**（舵机没有位置反馈，软件读不到角度）。
    # 幂等：爪子本来就开着时再发一次没有副作用。
    sdk.grip(release=True, label='起飞前打开夹爪')
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
    # 2026-10-16 改成**不走精降**（原来这里是 land_on(SUPPLY_TAG, ...)）。两个理由：
    # ① 这一步的目的只是验证"松开"，东西就在爪子里、原地放下即可，不需要对准标识；
    # ② **抓着灭火弹的时候下视被弹体挡住**，根本看不到 apriltag:0（跟
    #    rounds.supply_ground 里"投放时不做视觉对准"是同一条理由）。实测这一步会把
    #    land_on 的"降到 1.2 m + 沿 X 轴左右各挪 0.5 m"整套搜索走完、白花约 20 秒，
    #    最后还是退回普通降落。
    sdk.progress('原地降落放回，验证松开（不走精降：爪子里有东西，下视被挡住）')
    sdk.land_or_confirm()
    sdk.play_sound_light('任务机投放灭火弹')
    sdk.grip(release=True, label='放回灭火弹')
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('抓取测试完成，返航')
    sdk.return_home(sound='任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=test, follower=test, description=__doc__)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""我的任务.py —— 你写飞行任务代码的地方（固定文件名，不要改名）。

改这个文件里的代码，然后运行同一个文件夹里的"运行仿真.sh"（Linux/macOS）
或"运行仿真.bat"（Windows）。不需要新建文件，也不需要写任何 docker/ROS2
相关的东西。

【两架飞机跑的是同一份这个文件】
启动脚本会把这个文件同时丢给两架飞机跑，只是给的 `--role` 不一样：
一架拿到 leader、一架拿到 follower。所以下面 `我的飞行任务()` 里用
`if role == 'leader'` 分开写两架各自要做的事。想只飞一架时，运行脚本
加 `--single` 参数。

【坐标系】
`goto(x, y, z)` 用的是**自己这架飞机的局部坐标**：以自己起飞点为原点，
x 往前、y 往左、z 往上，单位米。想用场地的世界坐标就先换算：
    sdk.goto(*sdk.world_to_local(7.0, -10.0, 1.5))

【能用的方法】（都是 contest_sdk 真实提供的）
    sdk.takeoff()                                 起飞，等到确认离地才返回
    sdk.goto(x, y, z)                             飞到一个点（有避障），等到达才返回
    sdk.goto_direct(x, y, z)                      直线飞到一个点（**无避障**），近距离收尾用
    sdk.land()                                    降落，等到确认落地才返回
    sdk.cancel_goto()                             打断正在飞的 goto()
    sdk.world_to_local(x, y, z)                   世界坐标 -> 自己的局部坐标
    sdk.local_to_world(x, y, z)                   自己的局部坐标 -> 世界坐标
    sdk.send_to_teammate(事件名, **数据)           给队友发消息，等到对方确认才返回
    sdk.on_teammate_event(事件名, 回调函数)         注册"收到队友消息"的处理函数
    sdk.start_formation_follow(follow_distance_m) 开启编队跟随（沿队友走过的轨迹飞）
    sdk.stop_formation_follow()                   出列
    sdk.wait_for_detection(class_id, timeout)     等摄像头识别到某个目标
    sdk.do_action(name)                           触发动作（抓取/投放等）
    sdk.set_mission_state(s) / get_mission_state() 记录/查询任务进度

完整的双机编队例子见同目录的"编队飞行示例.py"。
"""
import argparse

from contest_sdk import DroneSDK


def 我的飞行任务(sdk, role):
    """在这里写你的任务。role 是 'leader' 或 'follower'。"""

    if role == 'leader':
        # ---- 长机要做的事 ----
        sdk.takeoff()          # 一直等到确认离地才往下走
        sdk.goto(2, 0, 1.5)    # 改这三个数字，让飞机去你想去的地方
        sdk.land()

    else:
        # ---- 僚机要做的事 ----
        sdk.takeoff()
        sdk.goto(2, 0, 1.5)
        sdk.land()

    print(f'[{sdk.namespace}] 任务完成，飞机已安全落地。')


# ===========================================================================
# 下面这段是启动样板，一般不需要改。
# ===========================================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--namespace', default='NX01', help='自己这架飞机的编号')
    ap.add_argument('--role', default='leader', choices=('leader', 'follower'))
    ap.add_argument('--teammate', default='NX02', help='队友那架飞机的编号')
    args = ap.parse_args()

    # SDK 的 role 参数只认 recon/supply 这两个内部名字，这里做一次映射，
    # 你在上面的代码里用 leader/follower 就行。
    sdk = DroneSDK(namespace=args.namespace,
                   role='recon' if args.role == 'leader' else 'supply',
                   teammate_namespace=args.teammate)
    try:
        我的飞行任务(sdk, args.role)
    finally:
        # 不管任务成功还是出错，都要收尾——不收尾会留下后台线程，
        # 下次运行可能出现莫名其妙的问题。
        sdk.shutdown()


if __name__ == '__main__':
    main()

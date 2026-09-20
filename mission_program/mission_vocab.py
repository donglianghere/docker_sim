"""这次2026大赛contest task的任务专属词汇表（对应方案4.1节/清单D1节）。

这些事件名/动作名字符串是选手自定义的——`contest_sdk`本身不限定具体有
哪些事件/动作，`send_to_teammate()`/`do_action()`只是把字符串原样转发，
真正的含义（谁发给谁、附带什么字段）由这份程序自己定义，不污染SDK的
通用性。
"""

# ---- 跨机事件名（sdk.send_to_teammate(event, **kwargs)的event参数取值）----

GROUND_FIRE_FOUND = 'ground_fire_found'
"""RECON -> SUPPLY，附x/y（RECON下视居中悬停收敛后读到的地面火情局部
坐标，SDK的send_to_teammate内部会转换成SUPPLY能用的坐标系）。"""

HR_FIRE_LOCKED = 'hr_fire_locked'
"""RECON -> SUPPLY，附staging_x/staging_y/staging_yaw（`fire_pillar_
staging_pose`话题给的任务机等待点坐标）。"""

RECON_LAUNCH_DONE = 'recon_launch_done'
"""RECON -> SUPPLY，附aim_x/aim_y/aim_yaw（`fire_pillar_aim_pose`话题
给的、RECON自己锁定的瞄准点坐标，SUPPLY投射前先飞到这个点，不需要
重新做一遍检测/锁定）。"""

SUPPLY_READY = 'supply_ready'
"""SUPPLY -> RECON，无附加字段——SUPPLY已经飞到等待点，可以开始投射。"""


# ---- 动作名（sdk.do_action(name)的name参数取值）----

GRAB_SUPPLY = 'grab_supply'
"""抓取物资（SUPPLY角色在物资点精降后触发）。"""

DROP_SUPPLY = 'drop_supply'
"""投放物资（SUPPLY角色飞到地面火情坐标后触发，不需要落地，悬停触发
即可）。"""

HORIZONTAL_LAUNCH = 'horizontal_launch'
"""水平投射破窗（RECON/SUPPLY两个角色在各自的高层火情投射流程里都要
触发这个动作，触发方是谁由D6的流程顺序决定，不是这个常量本身的属性）。"""

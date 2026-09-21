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

# 2026-09-21新增：僚机已进入编队跟随待命（SUPPLY->RECON，无附加字段）。
#
# 为什么必须有这个事件：编队飞行的前提是"两机都起飞稳定了，长机才起步"。
# 没有这个信号时长机起飞完就直接飞航线——实测过一次僚机因为定位没收敛
# 没能解锁、长机照样把4个航点飞完了，编队根本没发生，测试等于白跑。
#
# 为什么不复用`SUPPLY_READY`：那个是D6抓取环节"物资已抓好、可以投送"的
# 语义，跟这里"编队待命"是任务流程里两个完全不同的时刻，共用一个名字会
# 让收件箱分不清是哪一次。
#
# 时序上的关键点：僚机发这个事件的时机是`start_formation_follow()`返回，
# 而那个方法等的是机载feedback报`hold`阶段（已接管控制权、在自己起飞点
# 上空原地保持），**不是**等入列完成——入列要等长机走起来，长机又在等
# 这个事件，等入列就是互相等死锁。
FORMATION_STANDBY = 'formation_standby'

# 2026-09-20新增：第一部分航线飞完（RECON->SUPPLY，无附加字段）。
#
# 为什么必须是跨机事件而不是各自判断：从机手里根本没有"航线由哪几个点
# 组成、哪个是最后一个"这个信息——航线是长机的任务程序自己定义的。原来
# 从机只能按路线几何长度估算一个等待秒数然后 time.sleep()，代码里那句
# `enroute:formation_follow_assumed_complete` 的注释自陈是"近似判定"。
# 把判定搬到信息所在的那一侧（长机），从机等一个确切信号即可。
#
# 顺带解决可移植性：原来 D3 的到位判定依赖 mission_judge/checkpoints，
# 而裁判节点只存在于 sim-world 容器里，真机上没有。"最后一个 goto 返回
# 成功"是任务程序自己的状态，仿真真机通用。
ROUTE_DONE = 'route_done'
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

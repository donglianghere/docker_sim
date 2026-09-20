#!/usr/bin/env bash
# 一个tmux session管理launch/sim-world/NX01/NX02/status五份输出——不是同一
# 屏幕里的横向窗格（屏幕小会太挤），也不是三个独立的操作系统终端窗口（不好
# 统一管理），用tmux自己的"窗口"（window，类似浏览器分页/tab，一次只占满
# 整个终端、互相之间切换着看，不是同屏拆分）：每个窗口整屏显示一个服务的
# 完整日志，窗口内部不再拆分。
# 用法: ./scripts/watch_sim.sh
set -eo pipefail

cd "$(dirname "$0")/.."
TMUX_CONF="$(pwd)/scripts/mighty_sim.tmux.conf"
SESSION="mighty_sim"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux会话 ${SESSION} 已经存在，直接attach进去"
    # 重新设一遍状态栏——覆盖老会话(比如这个功能上线之前就已经在跑、还没
    # 重新执行过watch_sim.sh的session)，让它也能补上状态栏，不用非得
    # kill-session重开一次。
    tmux set-option -t "$SESSION" status-right-length 220
    tmux set-option -t "$SESSION" status-right "#($(pwd)/scripts/render_tmux_status.sh)"
    exec tmux -f "$TMUX_CONF" attach -t "$SESSION"
fi

# 第一个窗口：sim-world日志。NX01/NX02窗口过滤掉DLIO自己的多行状态面板
# （[dlio_odom_node-N]前缀，Ang Velocity/Accel Bias/Registration/GICP等
# 一堆内部调试字段，刷新频率很高，把mighty/mavros的日志全刷没了）——只是
# 不再打印到这两个窗口，DLIO本身继续正常跑、继续发布话题，数据不受影响。
# --line-buffered让grep按行转发，不然管道里会攒一批才刷新，跟着日志实时
# 看的体验会变差。
tmux -f "$TMUX_CONF" new-session -d -s "$SESSION" -n sim-world \
    "docker compose logs -f sim-world"
tmux new-window -t "$SESSION" -n NX01 \
    "docker compose logs -f flight-stack-nx01 | grep --line-buffered -v '\\[dlio_odom_node'"
tmux new-window -t "$SESSION" -n NX02 \
    "docker compose logs -f flight-stack-nx02 | grep --line-buffered -v '\\[dlio_odom_node'"

# 第四个窗口：起飞/降落控制台，四个按钮（NX01/NX02各一套起飞+降落，
# launch_control.sh），按对应数字键立即执行，控制台上会把实际执行的
# docker exec/ros2 service call命令打印出来。docker compose up之后飞机
# 不会自动起飞爬升（只会解锁、切进OFFBOARD、悬停在起飞位置），必须在这个
# 窗口按对应的起飞键才会真正往上爬。排第四个（sim-world/NX01/NX02排前
# 三个），但attach进tmux时依然默认停在这里（下面用select-window显式切
# 过去），不会漏按。
tmux new-window -t "$SESSION" -n launch \
    "./scripts/launch_control.sh"

# 最后一个窗口：NX01/NX02集中状态面板，表格形式、原地刷新（位置/姿态
# 欧拉角/推力油门/发布频率），一个进程同时订阅两架飞机的话题重绘一张
# 固定表格，不再是两个进程各自printl交替刷屏滚动；同时后台起
# attitude_thrust_logger.py（PX4自己上报的姿态/推力目标+实际姿态，持续
# 写进/logs/<namespace>/attitude_thrust_debug.log——这是docker-compose
# 挂载到宿主机runtime_logs/的volume，容器销毁/重建数据不丢，不再是老版本
# 写在容器内/tmp、容器一删就没了，得手动docker cp才能取出来）。两个脚本
# 都不参与staging/patches那套编译流程（纯监控脚本，不是仿真逻辑的一
# 部分），每次用docker cp现改现塞，不用重新build镜像就能迭代。
# docker exec开的这些都是全新的shell进程，跟entrypoint.sh自己那个shell
# 完全独立，PLANNER=ego_planner时必须自己重新切RMW才能看见真实节点的
# 话题——完整原因、之前踩过的坑（$和"转义层级搞反、容器刚重启文件还没
# 写出来这个启动时序竞态）都写在scripts/ros2_env_setup.sh自己的注释里，
# 不在这里重复。下面每处docker exec都是先cp这个文件进去、source它，
# 不再在bash -c的单引号参数里手写这段判断——避免同一段逻辑改N遍、每次
# 都要重新绕一遍多层shell转义关系的验证。
# 2026-08-13：原来这里是一长串写死双机的`&&`命令链，NX02容器不存在时（单机
# 仿真彩排）会在第二步docker cp就失败、后面全部不执行，连NX01自己的status_
# monitor.py都起不来——抽成scripts/status_window.sh，用docker compose ps
# 探测NX02是否真的在跑，动态决定单机/双机两种启动方式，详见该脚本头注释。
tmux new-window -t "$SESSION" -n status \
    "./scripts/status_window.sh"

# 第六个窗口：双机目标点手动输入面板——世界坐标输一次，自动换算成
# NX01/NX02各自的local坐标（2026-08-12起通过origin_setter_node广播的
# world -> {ns}/map这条TF换算，含θ*旋转，不再是硬编码INIT_X纯平移），
# 同时发给双机的/term_goal。
# 常驻一个rclpy节点+常驻publisher，不是每次现拼一次性的`ros2 topic pub
# --once`——这次session里踩过好几次"--once在DDS发现完成前就退出、消息
# 实际没发出去"的坑，常驻节点从根上避开这个问题。同样不参与patches编译
# 流程，docker cp现改现塞。
tmux new-window -t "$SESSION" -n goal \
    "docker cp scripts/ros2_env_setup.sh docker_sim-flight-stack-nx01-1:/tmp/ros2_env_setup.sh \
     && docker cp scripts/dual_goal_input.py docker_sim-flight-stack-nx01-1:/tmp/dual_goal_input.py \
     && docker exec -it docker_sim-flight-stack-nx01-1 bash -c 'source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && python3 /tmp/dual_goal_input.py'"

# 第七个窗口：运行时数据"黑匣子"录制——record_rosbag.sh每5分钟滚动录一个
# bag块（关键话题：TF/双机共享轨迹/mavros状态位姿/目标点/占据栅格/UWB，
# 不含原始点云，体积太大）到/logs/rosbag/，prune_rosbag.sh只保留最近12个
# 块（约1小时），配合后台跑，超过的自动删掉，不会无限增长。事情刚发生时
# 手动跑一下scripts/save_incident.sh能把当时的几个块单独另存一份，不受
# 滚动清理影响，具体见README"运行时数据记录"一节。
tmux new-window -t "$SESSION" -n record \
    "docker cp scripts/ros2_env_setup.sh docker_sim-flight-stack-nx01-1:/tmp/ros2_env_setup.sh \
     && docker cp scripts/record_rosbag.sh docker_sim-flight-stack-nx01-1:/tmp/record_rosbag.sh \
     && docker cp scripts/prune_rosbag.sh docker_sim-flight-stack-nx01-1:/tmp/prune_rosbag.sh \
     && docker cp scripts/save_incident.sh docker_sim-flight-stack-nx01-1:/tmp/save_incident.sh \
     && docker exec docker_sim-flight-stack-nx01-1 chmod +x /tmp/record_rosbag.sh /tmp/prune_rosbag.sh /tmp/save_incident.sh \
     && docker exec -d docker_sim-flight-stack-nx01-1 bash -c '/tmp/prune_rosbag.sh >> /logs/prune_rosbag_stdout.log 2>&1' \
     && docker exec docker_sim-flight-stack-nx01-1 bash -c '/tmp/record_rosbag.sh'"

# 第八个窗口：三个容器的stdout持久化到宿主机runtime_logs/container_logs/
# ——docker compose logs本身容器一down就没了，这个宿主机侧常驻脚本额外
# tee一份、按大小自己滚动（不依赖系统logrotate）。三个服务在同一个窗口里
# 各起一个后台tail进程，窗口本身只打印这三个脚本自己的状态提示，不会被
# 三份日志的实时滚动刷屏。
tmux new-window -t "$SESSION" -n logs \
    "./scripts/tail_persist_logs.sh sim-world & \
     ./scripts/tail_persist_logs.sh flight-stack-nx01 & \
     ./scripts/tail_persist_logs.sh flight-stack-nx02 & \
     wait"

# tmux状态栏(status-right)常驻显示两机各子系统的健康位——2026-08-09起因：
# NX02的DLIO(SLAM)中途挂了，当时没人切到status窗口看，表格本身"发布频率"
# 那一列虽然会掉到0，但只有真正切进那个窗口才看得见。状态栏在任何窗口下都
# 常驻可见，不用先猜"是不是该去status窗口看看"。数据来源是status窗口里那个
# status_monitor.py每秒写一次的runtime_logs/health_status.txt，这里配置的
# 是"怎么把它读出来贴到状态栏上"，具体判定哪个子系统健康/掉线的逻辑全部在
# status_monitor.py里（文件头有大段说明），这里不重复。
# 用绝对路径(而不是相对路径)是因为tmux执行`#(...)`命令时的工作目录不一定
# 跟这个脚本自己的cwd一致（尤其是从别的目录attach进已存在session的时候），
# render_tmux_status.sh自己也会再cd一次保证稳妥，这里传绝对路径是双重保险。
tmux set-option -t "$SESSION" status-right-length 220
tmux set-option -t "$SESSION" status-right "#($(pwd)/scripts/render_tmux_status.sh)"

# 每个窗口在状态栏上固定一个颜色，一眼就能分清哪个是哪个（不用等切过去看
# 内容才知道）——跟tmux.conf里`window-status-current-style`的"当前激活"高亮
# 是两回事：这里设的是窗口未激活时的颜色，激活时会被那个高亮样式盖过去。
tmux set-window-option -t "${SESSION}:launch" window-status-style "fg=colour196"     # 红色，提醒先按这里
tmux set-window-option -t "${SESSION}:sim-world" window-status-style "fg=colour51"   # 青色
tmux set-window-option -t "${SESSION}:NX01" window-status-style "fg=colour46"        # 绿色
tmux set-window-option -t "${SESSION}:NX02" window-status-style "fg=colour214"       # 橙黄色
tmux set-window-option -t "${SESSION}:status" window-status-style "fg=colour201"     # 品红色
tmux set-window-option -t "${SESSION}:goal" window-status-style "fg=colour226"       # 黄色
tmux set-window-option -t "${SESSION}:record" window-status-style "fg=colour129"     # 紫色
tmux set-window-option -t "${SESSION}:logs" window-status-style "fg=colour245"       # 灰色

# 上面创建NX01/NX02/launch/status/goal/record/logs这7个窗口的
# new-window调用都没加-d，每次都会把当前激活窗口切过去——跑到这里，
# 实际激活的是最后创建的logs，不是launch（launch现在也不是第一个创建的
# 窗口了，排第四个，更不会自动停在这里）。显式切回来。只在这条"全新建
# session"路径上做，上面"会话已存在，直接attach"那个分支不动——那种情况
# 用户可能是主动切到别的窗口后detach的，不该强制拉回launch。
tmux select-window -t "${SESSION}:launch"

echo "== 八个tmux窗口已经建好：sim-world(青) / NX01(绿) / NX02(橙) / launch(红) / status(品红) / goal(黄) / record(紫) / logs(灰) =="
echo "== 飞机不会自动起飞，先在launch窗口(默认停在这里)按对应数字键下达起飞/降落口令 =="
echo "== NX01/NX02已经过滤掉DLIO自己的刷屏状态面板，位置/姿态/频率/内存改到status窗口集中看 =="
echo "== 状态栏(屏幕最下面一行)常驻显示两机子系统健康位，不管当前在哪个窗口都能看到：NX01/NX02各自"
echo "== \"雷位规控链\"(雷达点云/SLAM/规划器/板外控制器/MAVROS链路)，绿=正常 红=掉线 灰=启动中或不监控；"
echo "== 出现红色说明对应子系统真的停了，去status/NX01/NX02窗口查日志；事件时间线记在"
echo "== runtime_logs/incidents/health_events.log，出问题当时没看到状态栏也能事后翻 =="
echo "== 姿态/推力遥测持续记录在宿主机runtime_logs/NX01(NX02)/attitude_thrust_debug.log =="
echo "== goal窗口输入世界坐标\"x y z\"同时给双机下目标点，自动换算各自的local坐标 =="
echo "== record窗口滚动录制关键话题到runtime_logs/rosbag/(最近1小时)，出事故手动跑save_incident.sh留证 =="
echo "== logs窗口把三个容器的stdout持久化到runtime_logs/container_logs/，容器down掉也不会丢 =="
echo "== 切窗口: Shift+左右方向键(不用按前缀)，或鼠标直接点状态栏窗口名，或 Ctrl-a + 窗口号(0~7，launch排第4个=编号3)"
echo "== 鼠标滚轮可以翻看窗口内的历史输出(scrollback 5万行)"
echo "== 前缀键: Ctrl-a    detach(不关闭): Ctrl-a d    彻底关闭所有窗口和session: Ctrl-a k (会有二次确认) =="
exec tmux -f "$TMUX_CONF" attach -t "$SESSION"

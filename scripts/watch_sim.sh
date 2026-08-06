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
    exec tmux -f "$TMUX_CONF" attach -t "$SESSION"
fi

# 第一个窗口：起飞口令控制台。docker compose up之后飞机不会自动起飞爬升
# （只会解锁、切进OFFBOARD、悬停在起飞位置），必须在这个窗口按一下回车，
# 才会给两个flight-stack容器都touch /tmp/takeoff_go，飞机检测到这个文件
# 才真正开始往上爬（见patches/ros2_px4_stack_takeoff_gate.patch）。放在
# 第一个窗口，attach进tmux时默认就停在这里，不会漏按。
tmux -f "$TMUX_CONF" new-session -d -s "$SESSION" -n launch \
    "./scripts/launch_control.sh"

# NX01/NX02窗口过滤掉DLIO自己的多行状态面板（[dlio_odom_node-N]前缀，
# Ang Velocity/Accel Bias/Registration/GICP等一堆内部调试字段，刷新频率
# 很高，把mighty/mavros的日志全刷没了）——只是不再打印到这两个窗口，DLIO
# 本身继续正常跑、继续发布话题，数据不受影响。--line-buffered让grep按行
# 转发，不然管道里会攒一批才刷新，跟着日志实时看的体验会变差。
tmux new-window -t "$SESSION" -n sim-world \
    "docker compose logs -f sim-world"
tmux new-window -t "$SESSION" -n NX01 \
    "docker compose logs -f flight-stack-nx01 | grep --line-buffered -v '\\[dlio_odom_node'"
tmux new-window -t "$SESSION" -n NX02 \
    "docker compose logs -f flight-stack-nx02 | grep --line-buffered -v '\\[dlio_odom_node'"

# 最后一个窗口：NX01/NX02集中状态面板，表格形式、原地刷新（位置/姿态
# 欧拉角/推力油门/发布频率），一个进程同时订阅两架飞机的话题重绘一张
# 固定表格，不再是两个进程各自printl交替刷屏滚动；同时后台起
# attitude_thrust_logger.py（PX4自己上报的姿态/推力目标+实际姿态，持续
# 写进/logs/<namespace>/attitude_thrust_debug.log——这是docker-compose
# 挂载到宿主机runtime_logs/的volume，容器销毁/重建数据不丢，不再是老版本
# 写在容器内/tmp、容器一删就没了，得手动docker cp才能取出来）。两个脚本
# 都不参与staging/patches那套编译流程（纯监控脚本，不是仿真逻辑的一
# 部分），每次用docker cp现改现塞，不用重新build镜像就能迭代。
tmux new-window -t "$SESSION" -n status \
    "docker cp scripts/status_monitor.py docker_sim-flight-stack-nx01-1:/tmp/status_monitor.py \
     && docker cp scripts/attitude_thrust_logger.py docker_sim-flight-stack-nx01-1:/tmp/attitude_thrust_logger.py \
     && docker cp scripts/attitude_thrust_logger.py docker_sim-flight-stack-nx02-1:/tmp/attitude_thrust_logger.py \
     && docker exec -d docker_sim-flight-stack-nx01-1 bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/attitude_thrust_logger.py NX01 >> /tmp/attitude_thrust_logger_stdout.log 2>&1' \
     && docker exec -d docker_sim-flight-stack-nx02-1 bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/attitude_thrust_logger.py NX02 >> /tmp/attitude_thrust_logger_stdout.log 2>&1' \
     && docker exec docker_sim-flight-stack-nx01-1 bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/status_monitor.py NX01 NX02'"

# 第六个窗口：双机目标点手动输入面板——世界坐标输一次，自动换算成
# NX01/NX02各自的local坐标（减各自的INIT_X），同时发给双机的/term_goal。
# 常驻一个rclpy节点+常驻publisher，不是每次现拼一次性的`ros2 topic pub
# --once`——这次session里踩过好几次"--once在DDS发现完成前就退出、消息
# 实际没发出去"的坑，常驻节点从根上避开这个问题。同样不参与patches编译
# 流程，docker cp现改现塞。
tmux new-window -t "$SESSION" -n goal \
    "docker cp scripts/dual_goal_input.py docker_sim-flight-stack-nx01-1:/tmp/dual_goal_input.py \
     && docker exec -it docker_sim-flight-stack-nx01-1 bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/dual_goal_input.py'"

# 第七个窗口：运行时数据"黑匣子"录制——record_rosbag.sh每5分钟滚动录一个
# bag块（关键话题：TF/双机共享轨迹/mavros状态位姿/目标点/占据栅格/UWB，
# 不含原始点云，体积太大）到/logs/rosbag/，prune_rosbag.sh只保留最近12个
# 块（约1小时），配合后台跑，超过的自动删掉，不会无限增长。事情刚发生时
# 手动跑一下scripts/save_incident.sh能把当时的几个块单独另存一份，不受
# 滚动清理影响，具体见README"运行时数据记录"一节。
tmux new-window -t "$SESSION" -n record \
    "docker cp scripts/record_rosbag.sh docker_sim-flight-stack-nx01-1:/tmp/record_rosbag.sh \
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

echo "== 八个tmux窗口已经建好：launch(红) / sim-world(青) / NX01(绿) / NX02(橙) / status(品红) / goal(黄) / record(紫) / logs(灰) =="
echo "== 飞机不会自动起飞，先在launch窗口(默认停在这里)按回车下达起飞口令 =="
echo "== NX01/NX02已经过滤掉DLIO自己的刷屏状态面板，位置/姿态/频率/内存改到status窗口集中看 =="
echo "== 姿态/推力遥测持续记录在宿主机runtime_logs/NX01(NX02)/attitude_thrust_debug.log =="
echo "== goal窗口输入世界坐标\"x y z\"同时给双机下目标点，自动换算各自的local坐标 =="
echo "== record窗口滚动录制关键话题到runtime_logs/rosbag/(最近1小时)，出事故手动跑save_incident.sh留证 =="
echo "== logs窗口把三个容器的stdout持久化到runtime_logs/container_logs/，容器down掉也不会丢 =="
echo "== 切窗口: Shift+左右方向键(不用按前缀)，或鼠标直接点状态栏窗口名，或 Ctrl-a + 窗口号(0~7)"
echo "== 鼠标滚轮可以翻看窗口内的历史输出(scrollback 5万行)"
echo "== 前缀键: Ctrl-a    detach(不关闭): Ctrl-a d    彻底关闭所有窗口和session: Ctrl-a k (会有二次确认) =="
exec tmux -f "$TMUX_CONF" attach -t "$SESSION"

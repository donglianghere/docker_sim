#!/usr/bin/env bash
# 每秒采集一次三个容器的CPU/内存/网络I/O（docker stats）+ GPU利用率
# （nvidia-smi），写到runtime_logs/container_stats.txt，供status_monitor.py
# （跑在flight-stack-nx01容器内部，docker exec进去的）读取，渲染成
# status窗口"4 资源消耗"表格。
#
# 这个脚本必须跑在宿主机上，不能docker exec进某个容器里跑——`docker stats`/
# `nvidia-smi`看的是宿主机层面的容器/GPU状态，容器内部因为PID/cgroup
# namespace隔离，天然看不到"隔壁容器用了多少CPU"这类跨容器信息（之前
# status_monitor.py自己的内存统计只能看"这个容器自己的进程"就是这同一个
# 限制）。要在容器内部拿到这些数据，要么把/var/run/docker.sock挂进容器
# （安全性代价：等于把容器提到能操控整个宿主机docker的权限），要么像这样
# 在宿主机侧采集、通过已经挂载的共享卷(runtime_logs/ <-> /logs)把结果
# 传进容器——选后者，不引入新的特权挂载。
#
# GPU利用率是整机层面的（这套仿真里只有sim-world容器实际用到GPU：
# gzclient/rviz2的OpenGL渲染），nvidia-smi本身不按容器拆分利用率，没有
# 强行伪造"每个容器占了多少GPU"这种数据——status_monitor.py渲染这张表
# 时，GPU这一列只在sim-world那一行填数字，其它两行留空，用空值本身表达
# "这一项不适用于这一行"，不额外写说明文字（跟status_monitor.py"表格只
# 要标题+主体，不要说明性文字"的既有要求一致）。
#
# 用法：宿主机上后台常驻跑，不需要参数：
#   ./scripts/collect_container_stats.sh &
set -eo pipefail
cd "$(dirname "$0")/.."

OUT="runtime_logs/container_stats.txt"
CONTAINERS=(docker_sim-sim-world-1 docker_sim-flight-stack-nx01-1 docker_sim-flight-stack-nx02-1)

while true; do
    TMP="${OUT}.tmp"
    {
        # --no-stream：只采一个瞬时快照就退出，不是常驻监听——这个脚本
        # 自己已经在用while循环控制采集节奏，不需要docker stats自己的
        # 流式刷新。CPUPerc/MemUsage/NetIO都是docker自己算好的现成字段，
        # 不用再手动读cgroup文件算。容器没在跑时docker stats会跳过它，
        # 不会导致整条命令失败（2>/dev/null兜底吞掉这种情况的报错）。
        docker stats --no-stream --format '{{.Name}}|{{.CPUPerc}}|{{.MemUsage}}|{{.NetIO}}' \
            "${CONTAINERS[@]}" 2>/dev/null
        echo "---GPU---"
        nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total \
            --format=csv,noheader,nounits 2>/dev/null
    } > "$TMP"
    # 原子写：同目录内mv是原子操作，避免status_monitor.py读到写了一半的
    # 内容——跟render_tmux_status.sh读health_status.txt的问题、
    # origin_setter_node.py持久化文件用的是同一个防坑手法。
    mv "$TMP" "$OUT"
    sleep 1
done

#!/usr/bin/env bash
# 2026-10-18 机载补丁一键重打。**断电重启后必须重跑一次**——这些改动在容器
# 文件系统里，不在镜像里，容器一旦被重建就回到镜像内容（10-10 实测过）。
#
#   ① precision_servo_node.py：track_land 的超时判据提到 elif 链最前面。
#      原来它排在"高度到了但没对准"之后，而那条只打印日志不设 why，于是飞机
#      一悬在交接高度就反复命中它、`elapsed > tmo` 永远执行不到——实测 30 s
#      的时限跑成 70 s，SDK 侧等 70 s 比它早 6 秒放弃，servo_status 全程 None。
#   ② yolo_detector_node.py：MJPEG 预览也画 AprilTag 框（橙色）。原来只画
#      YOLO 的 dets，而 tag 是单独检测直接填进消息的，所以抓弹那段一个框都没有。
#
# ③（像素容差 10→100 px）不在这里：那个在 src/contest_sdk，已经重建进
#    contestant-sdk:latest 镜像，断电不丢。
#
# 用法：./apply.sh            两架都打
#       ./apply.sh 102        只打 NX02
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FS=docker_sim-flight-stack-hw-1
VS=docker_sim-vision-stack-1
IPS=(192.168.2.102 192.168.2.101)
[ $# -gt 0 ] && IPS=("192.168.2.$1")

for ip in "${IPS[@]}"; do
    echo "########## $ip ##########"
    scp -q -o BatchMode=yes -o ConnectTimeout=6 \
        "$HERE/patch_servo.py" "$HERE/patch_preview.py" "$HERE/restart_servo.py" \
        "nvidia@$ip:/tmp/"
    ssh -o BatchMode=yes -o ConnectTimeout=6 "nvidia@$ip" "
        set -e
        docker cp /tmp/patch_servo.py   $FS:/tmp/
        docker cp /tmp/restart_servo.py $FS:/tmp/
        docker cp /tmp/patch_preview.py $VS:/tmp/
        docker exec $FS python3 -I /tmp/patch_servo.py \
            /opt/contest_mission_ws/install/contest_mission/lib/python3.10/site-packages/contest_mission/precision_servo_node.py \
            /opt/contest_mission_ws/src/contest_mission/contest_mission/precision_servo_node.py
        docker exec $FS python3 -m py_compile /opt/contest_mission_ws/install/contest_mission/lib/python3.10/site-packages/contest_mission/precision_servo_node.py
        docker exec $VS python3 -I /tmp/patch_preview.py \
            /opt/vision_ws/src/vision_stack/vision_stack/yolo_detector_node.py
        docker exec $VS python3 -m py_compile /opt/vision_ws/src/vision_stack/vision_stack/yolo_detector_node.py
        echo '--- 重启 precision_servo_node（沿用原进程 environ，不热重启容器）---'
        docker exec $FS python3 -I /tmp/restart_servo.py
    "
done
echo
echo '✅ 打完了。yolo_detector_node 下次切相机模式时自动加载新代码，不用额外动作。'

#!/usr/bin/env bash
# 运行真机.sh —— 连真机调试：跟"运行仿真.sh"完全一样，只是加了 --real
# （ROS_DOMAIN_ID=20、DDS 走连飞机的网卡、不重建仿真）。其它参数照样能用，比如：
#     bash 运行真机.sh --single
exec bash "$(dirname "$0")/运行仿真.sh" --real "$@"

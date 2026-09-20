#!/usr/bin/env bash
# tmux "launch"窗口的入口——真正的四按钮起飞/降落控制台实现在
# launch_control.py（curses，鼠标真的能点，见该文件头部注释：按
# CONTROLLER自动区分px4ctrl/so3ctrl的可重复起降 vs ros2_px4_stack只能
# 起飞一次的限制）。这个文件只是个瘦包装：
#   1. python3自带curses，理论上不需要额外装依赖，但万一目标环境没有
#      （比如非交互式终端、curses初始化失败），兜底降级成一个纯数字菜单，
#      保证控制台至少能用，不会直接报错退出把整个launch窗口撂在那。
set -eo pipefail

cd "$(dirname "$0")/.."

if python3 scripts/launch_control.py; then
    exit 0
fi

echo ""
echo "!! curses界面没能正常启动（可能是这个终端不支持），降级成纯数字菜单 !!"
echo ""

NX01_CONTAINER=docker_sim-flight-stack-nx01-1
NX02_CONTAINER=docker_sim-flight-stack-nx02-1
GATE_FILE=/tmp/takeoff_go

get_controller() {
    docker exec "$1" printenv CONTROLLER 2>/dev/null || echo "ros2_px4_stack"
}

do_takeoff() {
    local ns="$1" container="$2" controller
    controller="$(get_controller "$container")"
    echo "== ${ns}: 下达起飞口令（CONTROLLER=${controller}） =="
    if [[ "$controller" == "px4ctrl" || "$controller" == "so3ctrl" ]]; then
        local topic="/${ns}/takeoff_land"
        echo "   实际执行: ros2 topic pub --once ${topic} quadrotor_msgs/msg/TakeoffLand \"{takeoff_land_cmd: 1}\"（连发3次）"
        # quadrotor_msgs是so3ctrl/px4ctrl自己workspace built的自定义消息包，只
        # source基础ROS2安装解析不到，会报"The passed message type is invalid"
        # （实测踩过），必须额外source px4ctrl_ws；顺带带上ros2_env_setup.sh
        # 处理PLANNER=ego_planner时的CycloneDDS切换。
        docker cp scripts/ros2_env_setup.sh "${container}:/tmp/ros2_env_setup.sh" 2>/dev/null
        docker exec "$container" bash -c \
            "source /opt/ros/humble/setup.bash && source /opt/px4ctrl_ws/install/setup.bash && source /tmp/ros2_env_setup.sh && for i in 1 2 3; do ros2 topic pub --once ${topic} quadrotor_msgs/msg/TakeoffLand \"{takeoff_land_cmd: 1}\" >/dev/null 2>&1; sleep 0.2; done" \
            && echo "   ${ns} 起飞口令已发送" || echo "   !! ${ns} 起飞口令发送失败 !!"
    else
        echo "   实际执行: docker exec ${container} touch ${GATE_FILE}"
        if docker exec "$container" touch "$GATE_FILE" 2>/dev/null; then
            echo "   ${ns} 起飞口令已下达"
            echo "   !! 注意：CONTROLLER=ros2_px4_stack这条链路只支持起飞一次，降落过就不能再靠这个键起飞 !!"
        else
            echo "   !! ${ns} 容器不存在或没起来 !!"
        fi
    fi
}

do_land() {
    local ns="$1" container="$2" controller
    controller="$(get_controller "$container")"
    echo "== ${ns}: 下达降落口令（CONTROLLER=${controller}） =="
    if [[ "$controller" == "px4ctrl" || "$controller" == "so3ctrl" ]]; then
        local topic="/${ns}/takeoff_land"
        echo "   实际执行: ros2 topic pub --once ${topic} quadrotor_msgs/msg/TakeoffLand \"{takeoff_land_cmd: 2}\"（连发3次）"
        docker cp scripts/ros2_env_setup.sh "${container}:/tmp/ros2_env_setup.sh" 2>/dev/null
        docker exec "$container" bash -c \
            "source /opt/ros/humble/setup.bash && source /opt/px4ctrl_ws/install/setup.bash && source /tmp/ros2_env_setup.sh && for i in 1 2 3; do ros2 topic pub --once ${topic} quadrotor_msgs/msg/TakeoffLand \"{takeoff_land_cmd: 2}\" >/dev/null 2>&1; sleep 0.2; done" \
            && echo "   ${ns} 降落口令已发送（只有悬停状态才会响应，正在执行任务会被拒绝）" || echo "   !! ${ns} 降落口令发送失败 !!"
    else
        local svc="/${ns}/mavros/set_mode"
        echo "   实际执行: ros2 service call ${svc} mavros_msgs/srv/SetMode \"{custom_mode: 'AUTO.LAND'}\""
        if ! docker cp scripts/ros2_env_setup.sh "${container}:/tmp/ros2_env_setup.sh" 2>/dev/null; then
            echo "   !! ${ns} 容器不存在或没起来 !!"
            return
        fi
        if docker exec "$container" bash -c \
            "source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && timeout 5 ros2 service call ${svc} mavros_msgs/srv/SetMode \"{custom_mode: 'AUTO.LAND'}\""
        then
            echo "   ${ns} 降落口令已下达（AUTO.LAND）"
        else
            echo "   !! ${ns} 降落口令下达失败 !!"
        fi
    fi
}

while true; do
    echo ""
    echo "[1] NX01起飞  [2] NX01降落  [3] NX02起飞  [4] NX02降落"
    read -r -n1 -s -p "按键选择 [1/2/3/4]（Ctrl-C退出）: " choice
    echo ""
    case "$choice" in
        1) do_takeoff NX01 "$NX01_CONTAINER" ;;
        2) do_land NX01 "$NX01_CONTAINER" ;;
        3) do_takeoff NX02 "$NX02_CONTAINER" ;;
        4) do_land NX02 "$NX02_CONTAINER" ;;
        *) echo "无效输入: '${choice}'" ;;
    esac
done

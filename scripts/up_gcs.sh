#!/usr/bin/env bash
# 一键起本机GCS（rosbridge+backend两个容器），2026-08-30新增，合并原来的
# up_gcs_local.sh（本机仿真联调专用）+ up_gcs_hw.sh（真机部署专用）两个
# 脚本——比赛场景下"仿真模式"/"真机模式"两个按钮改成了真正互斥切网络
# （改GCS的CycloneDDS配置+ROS_DOMAIN_ID+重建gcs容器，见gcs/backend/app.py
# "1. GCS自己的网络配置"一节），网络模式的选择完全交给网页按钮，这个脚本
# 不再需要区分"这次是仿真还是真机"两条分支，唯一职责是把GCS自己安全
# 拉起来。
#
# 即使 gcs/cyclonedds_gcs.xml 还是刚从 .example 复制出来的占位符状态
# 导致 rosbridge_websocket 启动即挂，也不影响这个脚本判定成功——
# gcs/entrypoint.sh 里 `python3 -m http.server 8080` 是独立的后台进程，
# rosbridge 崩溃不会拖累它一起退出，网页依然能打开、按钮依然能点，点一下
# 就能把配置改成合法值并重建修好，不需要这个脚本预先猜/校验场景。
#
# 用法: ./scripts/up_gcs.sh
# 镜像不会自动build，第一次用之前要先手动 `cd gcs && docker compose build`。
set -eo pipefail
cd "$(dirname "$0")/.."

echo "== [1/5] xhost 授权，容器里的gzclient/rviz2要连宿主机X server用 =="
# xhost只能由启动这个X会话的宿主机用户（不是容器里的root，容器和宿主机
# 是不同UID，X server靠SO_PEERCRED按UID认权限）来授权，天生绕不开这一步。
# xhost的ACL活在这次X会话里，重启/重新up容器不会清掉，只有注销/重启宿主机
# 才需要重新跑；DISPLAY不存在（比如纯SSH无X转发跑这个脚本）时xhost会报错，
# 不影响其它跟GUI无关的功能，所以不用set -e那一套硬失败。
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 \
        && echo "-- 已授权，Gazebo/RViz按钮可以直接用 --" \
        || echo "!! xhost授权失败（可能没有DISPLAY/不在图形桌面里跑这个脚本），Gazebo图形界面/RViz这两个按钮会用不了，其它功能不受影响" >&2
else
    echo "!! 没找到xhost命令（试试 sudo apt install x11-xserver-utils），跳过，Gazebo图形界面/RViz这两个按钮会用不了" >&2
fi

echo "== [2/5] 确保 gcs/cyclonedds_gcs.xml 存在（不校验/不猜内容，网络模式由网页按钮决定）=="
(
    cd gcs
    if [[ ! -f cyclonedds_gcs.xml ]]; then
        echo "-- cyclonedds_gcs.xml 不存在，从 .example 复制一份 --"
        cp cyclonedds_gcs.xml.example cyclonedds_gcs.xml
    fi
)

echo "== [3/5] 确保 gcs/.env、gcs/hw_fleet_tokens.json、gcs/gcs_network_state.json 存在 =="
(
    cd gcs
    if [[ ! -f .env ]]; then
        echo "!! gcs/.env 不存在，必须先手动创建（至少要有 HOST_REPO_PATH）:" >&2
        echo "     HOST_REPO_PATH=$(cd .. && pwd)" >&2
        echo "     GCS_ROS_DOMAIN_ID=20" >&2
        exit 1
    fi
    if ! grep -q '^GCS_ROS_DOMAIN_ID=' .env; then
        echo "-- gcs/.env 缺 GCS_ROS_DOMAIN_ID，补一行默认值(20=真机) --"
        echo "GCS_ROS_DOMAIN_ID=20" >> .env
    fi
    if [[ ! -f hw_fleet_tokens.json ]]; then
        echo "-- hw_fleet_tokens.json 不存在，从 .example 复制一份 --"
        cp hw_fleet_tokens.json.example hw_fleet_tokens.json
    fi
    if [[ ! -f gcs_network_state.json ]]; then
        echo "-- gcs_network_state.json 不存在，从 .example 复制一份 --"
        cp gcs_network_state.json.example gcs_network_state.json
    fi
)

echo "== [4/5] docker compose up -d 起 gcs + backend =="
(cd gcs && docker compose up -d)

echo "== [5/5] 验证 GCS 自己活着（不验证任何飞机话题——网络模式还没被网页按钮选择，rosbridge此刻可能没在正常工作，这是预期状态）=="
DEADLINE=$((SECONDS + 30))
until curl -sf -o /dev/null http://localhost:8000/healthz; do
    if (( SECONDS > DEADLINE )); then
        echo "!! 30秒内 backend /healthz 没响应，检查: docker logs gcs-backend-1" >&2
        exit 1
    fi
    sleep 2
done
if [[ "$(docker inspect --format '{{.State.Status}}' gcs-gcs-1 2>/dev/null)" != "running" ]]; then
    echo "!! gcs-gcs-1 容器没有处于running状态，检查: docker logs gcs-gcs-1" >&2
    exit 1
fi
echo "-- GCS 本机部分正常 --"

echo "== 完成 =="
echo "浏览器打开: http://localhost:8080"
echo "首次使用/切换场景前，请在网页顶部点'🖥️仿真模式'或'🛰️真机模式'按钮完成网络配置"
echo "（真机模式需要先在网页'GCS网络设置'面板里填好网卡IP再点按钮）"

#!/usr/bin/env bash
# 收窄飞控→机载的高频 MAVLink 流，降机载整机 CPU。
# 由 contestant_real/run_real.sh 和 run_test.sh source 后调用：
#     source "$ROOT/scripts/tune_mavlink_streams.sh"
#     tune_mavlink_streams NX01 NX02
# 依赖调用方已定义的两个全局：$IMAGE（contestant-sdk 镜像）和
# ${CONTESTANT_NET_ARGS[@]}（由 scripts/contestant_network.sh 生成，带
# ROS_DOMAIN_ID=20 + CycloneDDS 的 Peers），所以必须放在 contestant_net_args
# 之后调用。
#
# ─── 为什么要这一步 ───────────────────────────────────────────────────
# 飞控 SD 卡的启动脚本把四路报文顶到了 100Hz：
#     mavlink stream -d /dev/ttyS4 -s ODOMETRY            -r 100
#     mavlink stream -d /dev/ttyS4 -s LOCAL_POSITION_NED  -r 100
#     mavlink stream -d /dev/ttyS4 -s ATTITUDE_QUATERNION -r 100
#     mavlink stream -d /dev/ttyS4 -s HIGHRES_IMU         -r 100
# 改那四行要拆机取 SD 卡，所以改用 MAV_CMD_SET_MESSAGE_INTERVAL(511) 在
# 运行时压回来。param2 单位是微秒，20000=50Hz，-1=彻底关掉这一路。
#
# 2026-10-03 实测（整机 /proc/stat 增量，各 12 秒，重复 2-3 轮）：
#     NX01  四路100Hz 75.2%/75.3%  →  本配置 66.5%/66.8%/67.0%   -8.5pt
#     NX02  四路100Hz 75.1%        →  本配置 66.2%/65.6%         -9.2pt
# 省的几乎全在 32+31；105+331 单纯降到 50Hz 只值 0.6pt，但这两路可以直接关：
#   · ODOMETRY(331)：mavros 的 odometry 插件在这套配置下不往外发，实测
#     /{ns}/mavros/odometry/in 零发布者，报文收进来就扔。无条件关。
#   · HIGHRES_IMU(105)→/{ns}/mavros/imu/data_raw：只有 SLAM 后端
#     (DLIO/point_lio/fast_lio) 把 DLIO_IMU_TOPIC 指到 mavros/imu/* 时才需要。
#     LOCALIZATION_SOURCE=uwb_imu 整个跳过 SLAM，此时零订阅者。
#     ⚠️ 这里不读飞机的 .env 去推断，而是**直接查订阅者数**，有人订就不关——
#     配置怎么变都不会误关。（别信 flight-stack-entrypoint.sh 里 UWB_IMU_TOPIC
#     那段注释说 fusion 节点吃 mavros/imu/data_raw：现在机上的
#     uwb_imu_fusion_node 只订 {ns}/uwb/pose_abs，连 imu_topic 参数都没声明，
#     ros2 param get 返回 "Parameter not set"，那段注释已过时。）
#
# 50Hz 是下限：PX4 的 EKF2 对外部视觉要求 30-50Hz。但那是机载→飞控方向
# (mavros/vision_pose/pose_cov)，不受这四路影响，实测改完仍是 50.0Hz。
#
# 失效条件：飞控**单独**重启而 flight-stack 容器不重启，会恢复成 100Hz。
# 再跑一次运行器即可。整段可用 MAVLINK_STREAM_TUNE=0 关掉。
# ─────────────────────────────────────────────────────────────────────

tune_mavlink_streams() {
    local nss="$*"
    [ -n "$nss" ] || return 0

    if [ "${MAVLINK_STREAM_TUNE:-1}" = "0" ]; then
        echo "   （MAVLINK_STREAM_TUNE=0，跳过 MAVLink 流收窄）"
        return 0
    fi
    if [ -z "${IMAGE:-}" ]; then
        echo "!! tune_mavlink_streams: 调用方没定义 \$IMAGE，跳过" >&2
        return 0
    fi

    # 失败只警告不中止：这一步纯粹是省 CPU，不收窄也能飞。
    docker run --rm --network host \
        -v /etc/localtime:/etc/localtime:ro \
        -e TUNE_NSS="$nss" \
        "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
        bash -lc '
set -u
for ns in $TUNE_NSS; do
    # mavros 要先跟飞控握上手，否则 command 插件直接回 success=False。
    # 最多等 60 秒；运行器走到这一步时 /status 已经报过 flight_up=True，
    # 正常情况下第一次就通。
    connected=0
    for i in $(seq 1 30); do
        if timeout 4 ros2 topic echo --once "/$ns/mavros/state" 2>/dev/null \
             | grep -q "connected: true"; then
            connected=1; break
        fi
        sleep 2
    done
    if [ "$connected" != "1" ]; then
        echo "   △ $ns: 等了 60 秒 mavros 仍未连上飞控，这架跳过"
        continue
    fi

    # 2026-10-05 加入 132(DISTANCE_SENSOR)->50Hz。⚠️ 这一路**绝不能关**：
    # 它是定高/仿地的唯一高度观测源（mavros/hrlv_ez4_pub ->
    # uwb_imu_fusion_node 的 range*cos(roll)*cos(pitch) -> 喂飞控当视觉高度，
    # EKF2_HGT_REF=3）。两机飞控 SD 卡这一行不一致：实测 NX01 只有 20Hz、
    # NX02 是 49Hz，而两机 uwb/pose_abs 都 50Hz。融合在 UWB 回调里用"最后一次
    # range"算，range 慢一半就有 2~3 拍复用同值、再把 50ms 的高度变化除以 20ms
    # 的 dt_uwb，垂直速度被放大约 2.5 倍；过仿地模块(0.5m 高差)就变成大的虚假
    # 垂直速度观测，长机实测窜到 3.19m(巡航 1.46m)。
    # 当场用 511 验证过：NX01 请求 50Hz 后从 20.08Hz 立刻升到 48.2Hz，所以 20Hz
    # 是流速率配置限的、不是传感器上限。设了这一行就跟 SD 卡写什么无关。
    specs="32:20000.0 31:20000.0 331:-1.0 132:20000.0"
    # HIGHRES_IMU：查订阅者数，有人用就不关
    subs=$(timeout 12 ros2 topic info "/$ns/mavros/imu/data_raw" 2>/dev/null \
             | sed -n "s/^Subscription count: //p" | head -1)
    if [ "${subs:-unknown}" = "0" ]; then
        specs="$specs 105:-1.0"
    else
        echo "   △ $ns: imu/data_raw 有 ${subs:-?} 个订阅者，HIGHRES_IMU 保留原速率"
    fi

    ok=0; total=0
    for spec in $specs; do
        id=${spec%%:*}; iv=${spec##*:}
        total=$((total + 1))
        if timeout 15 ros2 service call "/$ns/mavros/cmd/command" \
             mavros_msgs/srv/CommandLong \
             "{broadcast: false, command: 511, confirmation: 0, param1: $id.0, param2: $iv, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}" \
             2>&1 | grep -q "success=True"; then
            ok=$((ok + 1))
        else
            echo "   △ $ns: msg $id -> $iv 设置失败"
        fi
    done
    echo "   $ns: MAVLink 流已设 $ok/$total（LOCAL_POSITION_NED/ATTITUDE_QUATERNION 降 50Hz，ODOMETRY 及按需 HIGHRES_IMU 关闭，DISTANCE_SENSOR **提到** 50Hz 供定高用）"
done
' || { echo "!! MAVLink 流收窄没跑成（不中止飞行，只是机载 CPU 会高 8-9 个百分点）" >&2
       return 0; }
}

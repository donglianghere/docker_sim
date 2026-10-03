#!/usr/bin/env bash
# 被 contestant_real/run_real.sh、run_test.sh **source** 的视觉就绪判据。
# 不是独立入口，别直接跑。
#
# 为什么重写（2026-10-03）：原来的判据是
#     ros2 topic list | grep -c "vision/detections"   >= 2
# 这是**数话题数，不是数相机路数**。两路相机（前视/下视）发的是**同一个**话题
# `/{ns}/vision/detections`，靠 header.frame_id 区分
# （`{ns}_camera_front_optical_frame` / `..._down_optical_frame`）。所以：
#   · 一架飞机只要有**任意一路**在跑，话题就存在，旧判据就放行；
#   · 真漏开了程序要用的那一路，旧判据照样放行，然后 `wait_for_detection()`
#     静默超时——**正是它文件头声称要防的那个失效模式**。实测确认过：停掉
#     NX01 的下视后，闸门仍然"数到 2 条"并放行。
#   · 而且它的提示文字写着"四条检测话题都要有发布者""两路都要起"，跟它实际
#     检查的东西不符，会误导人。
#
# 新判据：订一下话题，看 frame_id 里**有没有这个程序真正需要的那一路**。
# 既支持"只起一路省 CPU"（实测单路比双路省约 9% 整机），又真能抓住漏开。

# 程序需要哪一路相机。判据来自各程序实际调用的 SDK 方法：
#   · fetch_from/land_on/precision_land_* 走下视（SDK 默认 camera='down'）
#   · aim_at/look_for/snapshot 走前视（SDK 默认 camera='front'）
required_camera() {   # required_camera <程序文件名> -> down|front|any|none
    case "$1" in
        *groundfire*|*地面火情*|*抓取*)   echo down ;;
        *highrise*|*高层火情*)            echo front ;;
        # 综合任务两路分阶段用（地面火情→下视、高层火情→前视），运行中会切，
        # 所以这里只要求"至少有一路"，具体哪一路由程序自己切换时保证。
        *mission*)                        echo any ;;
        *formation*|*避障*|*仿地*|*basic_test*) echo none ;;
        # 不认识的程序保守处理：要求至少一路，别直接放行
        *)                                echo any ;;
    esac
}

# 检查某架飞机上需要的那一路在不在发数据。
# 调用方约定：已有 $IMAGE、$CONTESTANT_NET_ARGS、log()、die()。
check_vision() {   # check_vision <NS> <down|front|any|none>
    local ns="$1" want="$2"
    if [ "$want" = none ]; then
        log "本程序不用相机，跳过视觉检查"
        return 0
    fi
    local pat
    case "$want" in
        down)  pat='camera_down_optical' ;;
        front) pat='camera_front_optical' ;;
        any)   pat='camera_\(front\|down\)_optical' ;;
    esac
    log "检查 $ns 的视觉就绪（需要：$want）"
    # 轮询而不是死等：DDS 发现通常几秒收敛。`--no-daemon` 不能省——不加会读
    # ROS 2 daemon 的缓存，一个 RMW/域配错的 shell 也会返回看着正常的结果。
    local got
    got=$(timeout 100 docker run --rm --network host "${CONTESTANT_NET_ARGS[@]}" "$IMAGE" \
        bash -lc "source /opt/ros/humble/setup.bash 2>/dev/null
                  for i in \$(seq 1 10); do
                      c=\$(timeout 6 ros2 topic echo /$ns/vision/detections --no-daemon 2>/dev/null \
                          | grep -oE 'camera_(front|down)_optical' | sort -u | tr '\n' ',')
                      [ -n \"\$c\" ] && { echo \"\$c\"; exit 0; }
                      sleep 2
                  done
                  echo ''" 2>/dev/null | tr -d '\r' | grep -oE 'camera_[a-z]+_optical' | sort -u | tr '\n' ',')

    if [ -z "$got" ]; then
        echo "!! $ns 的 /$ns/vision/detections 收不到任何数据" >&2
        echo "   机载检测节点没起，或 vision-stack 容器没起。" >&2
        _vision_hint "$ns" "$want"
        die "视觉未就绪，已中止（不拦住的话火情相关动作会静默超时）"
    fi
    if ! printf '%s' "$got" | grep -q "$pat"; then
        echo "!! $ns 在发的是 [${got%,}]，但本程序需要 **$want**" >&2
        echo "   这一路没起。只起一路是省 CPU 的正常做法，但得起对那一路。" >&2
        _vision_hint "$ns" "$want"
        die "视觉未就绪（相机路数不对），已中止"
    fi
    log "  $ns 视觉就绪：在发 [${got%,}]"
}

# 打出补救命令。cam0=front / cam1=down 是固定对应（control_server.py 的
# CAMERA_DEFAULTS 表定死，同时也决定 MJPEG 端口 8080/8081）。
_vision_hint() {   # _vision_hint <NS> <want>
    local ns="$1" want="$2" cam
    case "$want" in down) cam=cam1 ;; front) cam=cam0 ;; *) cam=cam1 ;; esac
    local ip
    case "$ns" in NX01) ip=192.168.2.101 ;; NX02) ip=192.168.2.102 ;; *) ip="<飞机IP>" ;; esac
    echo "   起它（$cam = $([ "$cam" = cam0 ] && echo front || echo down)）：" >&2
    echo "     curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \\" >&2
    echo "          -d '{\"cam\":\"$cam\",\"mode\":\"yolo\"}' http://$ip:8890/vision/mode" >&2
    echo "   两机四路全开： ./vision_real.sh up && ./vision_real.sh status" >&2
}

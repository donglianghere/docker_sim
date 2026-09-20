#!/usr/bin/env bash
# 真机侧(L3)实验的唯一机上脚本：录一段实验数据。
#
# 用法（在真机的 flight-stack-hw 容器里跑）：
#   ./l3_record.sh <标签> <时长秒> [命名空间] [--lock]
# 例：
#   ./l3_record.sh h1_static_los 600            # H1 静态10分钟
#   ./l3_record.sh h2_base029    180            # H2 基线0.29m静止3分钟
#   ./l3_record.sh h3_run1       300 NX01 --lock # H3 先锁起点再录5分钟(手持走或飞)
#
# 标签重名保护：目标目录已存在时默认拒绝覆盖并退出，确认要丢弃旧数据才加 --force。
#
# 产物：/logs/l3/<标签>/    （/logs 是挂载到宿主机的目录，包直接落在机外可取）
# 录完会自动打印每个话题的帧数自检，当场就能判断这包能不能用，不用等回去分析。
# ⚠️ 这里不能开 set -u：ROS 的 /opt/ros/humble/setup.bash 内部会引用
# AMENT_TRACE_SETUP_FILES 等未定义变量，开了 -u 一 source 就直接退出
# （报 "AMENT_TRACE_SETUP_FILES: unbound variable"）。脚本里所有变量引用
# 都已经写成 ${VAR:-默认值} 或由参数校验保证，不依赖 -u 来兜底。
set -o pipefail

TAG="${1:?用法: l3_record.sh <标签> <时长秒> [命名空间] [--lock]}"
DUR="${2:?缺时长(秒)}"
NS="${3:-NX01}"
LOCK=""
FORCE=""
for arg in "$@"; do
    [ "$arg" = "--lock" ]  && LOCK=1
    [ "$arg" = "--force" ] && FORCE=1
done

OUT="/logs/l3/${TAG}"

# --- ROS/DDS 环境：docker exec 进来的 shell 不带节点进程里的那几个变量，
#     必须自己补齐，否则 ros2 命令看不到任何话题。取值从正在跑的节点进程里读，
#     不写死，免得跟部署配置漂移。
source /opt/ros/humble/setup.bash
source /opt/px4ctrl_ws/install/setup.bash 2>/dev/null || true
# 依次尝试几个必定在跑的节点，谁在就从谁的环境里抄
for pat in origin_setter dlio_odom_node mavros_node point_lio; do
    PID=$(pgrep -f "${pat}" | head -1)
    [ -z "${PID}" ] && continue
    [ -r "/proc/${PID}/environ" ] || continue
    eval "$(tr '\0' '\n' < /proc/${PID}/environ | grep -E '^(RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_DOMAIN_ID)=' | sed 's/^/export /')"
    echo "== DDS环境取自进程 ${pat}(pid=${PID}) =="
    break
done
echo "== DDS: RMW=${RMW_IMPLEMENTATION:-未设置} DOMAIN=${ROS_DOMAIN_ID:-未设置} =="
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
    echo "   ⚠️ 没能自动取到DDS环境（飞控栈可能还没起来）。若下面话题自检全是[缺]，"
    echo "      先确认容器里的节点在跑，再手动 export RMW_IMPLEMENTATION / CYCLONEDDS_URI"
    echo "      / ROS_DOMAIN_ID 之后重跑本脚本。"
fi

# --- 要录的话题。缺哪一路只警告不中断：H1/H2 只需要其中一部分。
TOPICS=(
  "/${NS}/dlio/odom_node/odom"              # 局部位姿，估计器输入
  "/${NS}/uwb/pose_abs"                     # 双标签融合后的绝对位置(+yaw)
  "/${NS}/uwb_a/pose_abs"                   # 标签a原始位置（H2必需）
  "/${NS}/uwb_b/pose_abs"                   # 标签b原始位置（H2必需）
  "/${NS}/origin_setter/yaw_estimate"       # 机上θ*，用来跟离线复算对照
  "/${NS}/origin_setter/yaw_sample_count"
  "/${NS}/origin_locked"
  "/${NS}/mavros/state"
  "/tf" "/tf_static"
)

echo "== 录制前话题自检 =="
LIVE=$(ros2 topic list 2>/dev/null)
MISS=0
for t in "${TOPICS[@]}"; do
    if echo "${LIVE}" | grep -qx "$t"; then
        echo "   [有] $t"
    else
        echo "   [缺] $t"
        MISS=$((MISS+1))
    fi
done
[ "${MISS}" -gt 0 ] && echo "   ⚠️ 有${MISS}个话题不存在，相关实验的分析会缺数据（H2必须要uwb_a/uwb_b两路）"

# --- 可选：录制前锁定起飞点（H3 必须做，且必须在**静止**时做）
if [ -n "${LOCK}" ]; then
    echo "== 锁定起飞点（务必确认飞机此刻完全静止：放在地上/桌面，人离开一步）=="
    ros2 service call "/${NS}/set_origin_from_uwb" std_srvs/srv/Trigger 2>&1 | tail -2
    sleep 1
fi

mkdir -p /logs/l3
# 同名标签默认**拒绝覆盖**：现场重录一次的代价远小于把上一架次的数据删掉。
# 确实要丢弃旧数据时显式加 --force。
if [ -e "${OUT}" ]; then
    if [ -n "${FORCE}" ]; then
        echo "== ${OUT} 已存在，--force 生效，删除后重录 =="
        rm -rf "${OUT}"
    else
        echo "!! ${OUT} 已存在，里面是上次用同一个标签录的数据，拒绝覆盖。"
        echo "   现有内容："
        ls -la "${OUT}" 2>/dev/null | sed 's/^/     /'
        echo "   处理方式二选一："
        echo "     · 换一个标签重跑（推荐，例如加 _b 后缀）"
        echo "     · 确认旧数据不要了，在命令末尾加 --force 重跑"
        exit 1
    fi
fi
echo "== 开始录制 ${DUR} 秒 -> ${OUT} =="
# ⚠️ 必须 -s INT：timeout 默认发 SIGTERM，而 ros2 bag record 只对 SIGINT 做
# 优雅收尾（写完 metadata.yaml 再退出），收到 SIGTERM 会一直挂着不退，表现为
# "到点了还卡在录制中"。-k 10 是保险：发完 INT 再等10秒还不退就强杀。
# ⚠️ `< /dev/null` 不能省：ros2 bag record 会读 stdin 做键盘控制（空格暂停），
# 在 `docker exec -it` 分配的 TTY 下它属于后台进程组，一读 TTY 就收到 SIGTTIN
# 被停住（ps 里状态是 T），表现为"开始录制之后再无输出、也不生成bag目录"，
# 而且 timeout 也杀不掉它。重定向 stdin 之后读到 EOF，键盘控制自动失效。
timeout -s INT -k 10 "${DUR}" ros2 bag record -o "${OUT}" "${TOPICS[@]}" \
        < /dev/null > "/tmp/l3_bag_${TAG}.log" 2>&1
echo "== 录制结束 =="

if [ ! -d "${OUT}" ]; then
    echo "!! 没有生成 ${OUT}，ros2 bag record 的日志末尾："
    tail -20 "/tmp/l3_bag_${TAG}.log" 2>/dev/null | sed 's/^/   /'
    exit 1
fi

# --- 录完立刻自检：帧数够不够、是不是每路都有数据
echo "== 数据自检 =="
ros2 bag info "${OUT}" 2>/dev/null | sed -n '/Topic information/,$p' | sed 's/^/   /'
echo
echo "自检要点："
echo "  · odom 与 uwb/pose_abs 的 Count 应当 ≈ 各自频率×时长"
echo "    （2026-09-03 真机实测：odom≈19Hz、uwb≈48Hz，即20秒约380条/960条）"
echo "    某一路为0说明该链路没通；明显低于上面的量级说明该节点在掉帧"
echo "  · H2 还需要 uwb_a / uwb_b 两路都非空"
echo "  · H3 还要看 yaw_sample_count 的末值：<50 说明飞的路径不够长，"
echo "    需要累计位移≥30米（3米见方来回约10趟），当场补录一次"
echo
echo "把 ${OUT} 整个目录拷回分析机，用 l3_analyze.py 处理。"

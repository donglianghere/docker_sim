#!/usr/bin/env bash
# 真机部署一键打包脚本，2026-08-15新增。
#
# 把 docker_sim 打包成一个 tar.gz，排除清单跟 真机部署操作清单.md 阶段2
# 的 rsync 排除清单完全一致（同一份决策，这里只是换成 tar 实现，方便离线
# 传输/U盘拷贝，不需要现场跟 Jetson 有网络连通才能"打包"这一步）：
#   - runtime_logs/                          仿真日志，跟真机无关
#   - staging/gazebo_models_external         仿真专属
#   - staging/PX4-Autopilot                  仿真SITL用，真机走独立烧录固件
#   - staging/PX4-SITL_gazebo-classic        仿真专属
#   - docker-compose.yml                     原3服务仿真版，留着容易被误操作
#   - docker/Dockerfile.sim-world            仿真专属
#   - docker/entrypoints/sim-world-entrypoint.sh   仿真专属
#   - docker/scripts/gen_iris_mid360_sdf.py  仿真专属
#   - gcs/                                   地面站专用，不该出现在Jetson上
#   - docker-compose.override.yml            本机调试个人配置（.example模板保留）
#   - scripts/                               假设双机同host的仿真运维脚本
#   - .git/                                  版本控制元数据
#
# 输出文件放在 docker_sim/ 的上一级目录（不是 docker_sim/ 内部）——放在
# 被打包目录自己里面会被 tar 在遍历过程中当成"读取途中发生变化"报错
# （这个正在被写入、体积持续增长的输出文件本身也在遍历范围内），放上一级
# 目录彻底避开这个问题。
#
# 用法：
#   ./package_hw_deploy.sh                       # 只打包，产出放上一级目录
#   ./package_hw_deploy.sh nx01:~/                # 打包后额外 scp 到目标机器
#
# 用法（部署方法，Jetson 上执行）：
#   tar -xzf docker_sim_hw_deploy_<时间戳>.tar.gz -C ~/
#   cd ~/docker_sim
#   # 按 真机部署操作清单.md 阶段3继续（3.2 上机后第二轮编辑 开始）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_NAME="$(basename "$SCRIPT_DIR")"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_NAME="docker_sim_hw_deploy_${TIMESTAMP}.tar.gz"
# 输出必须放在被打包目录之外（PARENT_DIR，不是SCRIPT_DIR）——第一版试过
# 输出到docker_sim/自己里面，哪怕加了--exclude排除输出文件名的glob，
# 实测tar仍然在自己的目录树遍历过程中把这个正在被写入、体积持续增长的
# 输出文件当成"读取途中发生变化"报错退出（exit 1，被set -e拦下来），
# 这是把输出文件放进被打包目录本身的经典坑，改成放到上一级目录彻底避开，
# 不是靠更精细的exclude规则去堵。
OUT_PATH="${PARENT_DIR}/${OUT_NAME}"

echo "== 真机部署打包 =="
echo "源目录: ${SCRIPT_DIR}"
echo "输出: ${OUT_PATH}"
echo ""

# --exclude 路径都是相对 tar 内部路径（即相对 REPO_NAME/ 之后的路径），
# 跟 tar -C "$PARENT_DIR" "$REPO_NAME" 这种调用方式配合，保证归档内顶层
# 目录名是 docker_sim/，Jetson 上 tar -xzf 之后直接得到 ~/docker_sim/，
# 不需要额外改路径。
tar \
    --exclude="${REPO_NAME}/runtime_logs" \
    --exclude="${REPO_NAME}/staging/gazebo_models_external" \
    --exclude="${REPO_NAME}/staging/PX4-Autopilot" \
    --exclude="${REPO_NAME}/staging/PX4-SITL_gazebo-classic" \
    --exclude="${REPO_NAME}/docker-compose.yml" \
    --exclude="${REPO_NAME}/docker/Dockerfile.sim-world" \
    --exclude="${REPO_NAME}/docker/entrypoints/sim-world-entrypoint.sh" \
    --exclude="${REPO_NAME}/docker/scripts/gen_iris_mid360_sdf.py" \
    --exclude="${REPO_NAME}/gcs" \
    --exclude="${REPO_NAME}/docker-compose.override.yml" \
    --exclude="${REPO_NAME}/scripts" \
    --exclude="${REPO_NAME}/.git" \
    -czf "$OUT_PATH" \
    -C "$PARENT_DIR" \
    "$REPO_NAME"

SIZE_HUMAN="$(du -h "$OUT_PATH" | cut -f1)"
sha256sum "$OUT_PATH" > "${OUT_PATH}.sha256"

echo "打包完成：${OUT_PATH}（${SIZE_HUMAN}）"
echo "校验文件：${OUT_PATH}.sha256"
echo ""

# 可选：传了目标（scp风格的 user@host:path 或 ssh-config别名:path）就顺手传过去
if [ "${1:-}" != "" ]; then
    TARGET="$1"
    echo "== 传输到 ${TARGET} =="
    scp "$OUT_PATH" "${OUT_PATH}.sha256" "$TARGET"
    echo "已传输。Jetson 上执行以下命令解包+校验："
else
    echo "未指定传输目标，只在本机生成了归档。传到 Jetson 后（scp/U盘均可），"
    echo "在 Jetson 上执行以下命令解包+校验："
fi

echo ""
echo "  sha256sum -c ${OUT_NAME}.sha256"
echo "  tar -xzf ${OUT_NAME} -C ~/"
echo "  cd ~/docker_sim"
echo ""
echo "解包后按《真机部署操作清单》阶段3继续（3.2 上机后第二轮编辑 开始，"
echo "把 HW_NETWORK_INTERFACE/FC_SERIAL_DEVICE/UWB_SERIAL_DEVICE 等占位值"
echo "换成真实接线信息）。"

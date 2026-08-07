#!/usr/bin/env bash
# 一键导出/恢复脚本：把本项目已经build好的3个docker镜像
# （mighty-base:humble / sim-world:latest / flight-stack:latest）+
# docker_sim项目配置（compose文件/Dockerfile/entrypoint/patches/scripts）
# 打包成一份可以整个拷到另一台机器上、几分钟内`docker load`+`docker compose up`
# 就能跑起来的bundle，不需要在新机器上重新跑fetch_sources.sh（要网络/代理）
# 和colcon build（要时间、要CPU）。
#
# 原理：
#   1. `docker save`把镜像的完整层栈序列化成一个标准tar（OCI/Docker镜像
#      归档格式），`docker load`能在任何装了Docker的机器上原样重建出
#      一模一样的镜像，不依赖镜像原本是怎么build出来的——这是从根上
#      跳过"重新拉源码+重新编译"的关键，恢复速度只取决于"传文件"和
#      "docker load解包"这两步，不再取决于编译/下载耗时。
#   2. mighty-base:humble是sim-world:latest和flight-stack:latest的公共
#      父镜像（两者都是`FROM mighty-base:humble`），三个镜像的公共层
#      在磁盘上物理上只存了一份（`docker system df -v`能看到
#      SHARED SIZE字段）。把三个镜像名一起传给同一次`docker save`调用，
#      产出的tar里公共层也只会写一份，比分别对三个镜像各自`docker save`
#      要节省好几GB——这是为什么下面的脚本坚持一次性save全部三个镜像，
#      而不是循环分开save。
#   3. `docker save`不加`-o`时会把tar流写到stdout，直接接一个压缩器
#      （优先zstd，没有就pigz，再没有就退化到gzip）流式压缩，不在磁盘上
#      落一份几十GB的未压缩中间tar——省磁盘、也更快（边读镜像层边压缩，
#      不用先读完整个tar再压）。
#   4. docker_sim项目配置本身是纯文本（compose/Dockerfile/脚本/patch），
#      体积很小，但`staging/`（fetch_sources.sh拉下来的第三方源码，几GB，
#      镜像里已经编译进去了，恢复现场不需要）和`runtime_logs/`（运行时
#      日志/rosbag，跟"能不能跑起来"无关）明确排除，不进bundle。
#
# 用法：
#   导出（在当前这台机器上跑）：
#     ./bundle.sh export [输出目录，默认 ./ai_uav_bundle_<时间戳>]
#   恢复（把导出目录整个拷到新机器上，在新机器上跑）：
#     ./bundle.sh restore <bundle目录> [--dest 恢复到哪里，默认当前目录]
#
# 新机器前提条件（脚本恢复不了这些，只能靠人工确认）：
#   - 已安装Docker Engine + docker compose plugin
#   - 需要Gazebo/RViz GUI的话，装了nvidia-container-toolkit（sim-world的
#     docker-compose.yml里有`driver: nvidia`的GPU预留），且能访问X server
#   - 磁盘空间：解压后镜像层落盘约需35~40GB，`docker load`过程中会短暂
#     同时占用"压缩包本身"+"解包出来的镜像层"，预留50GB以上更保险

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_SIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -n "${AI_UAV_BUNDLE_IMAGES:-}" ]; then
    # 测试/调试用：AI_UAV_BUNDLE_IMAGES="img1 img2"覆盖默认镜像列表，
    # 正常使用不需要设这个变量。
    read -ra IMAGES <<< "$AI_UAV_BUNDLE_IMAGES"
else
    IMAGES=(mighty-base:humble sim-world:latest flight-stack:latest)
fi

log() { echo "== [bundle.sh] $* =="; }
err() { echo "!! [bundle.sh] $* !!" >&2; }

pick_compressor() {
    if command -v zstd >/dev/null 2>&1; then
        echo "zstd"
    elif command -v pigz >/dev/null 2>&1; then
        echo "pigz"
    else
        echo "gzip"
    fi
}

check_disk_space() {
    local path="$1" need_gb="$2"
    local avail_kb avail_gb
    avail_kb=$(df -Pk "$path" | tail -1 | awk '{print $4}')
    avail_gb=$((avail_kb / 1024 / 1024))
    if [ "$avail_gb" -lt "$need_gb" ]; then
        err "磁盘剩余约${avail_gb}GB，建议至少${need_gb}GB，继续可能中途写满磁盘失败"
        read -r -p "仍要继续吗？(y/N) " ans
        [[ "$ans" =~ ^[Yy]$ ]] || exit 1
    else
        log "磁盘剩余约${avail_gb}GB，够用"
    fi
}

cmd_export() {
    local out_dir="${1:-$PWD/ai_uav_bundle_$(date +%Y%m%d_%H%M%S)}"
    mkdir -p "$out_dir"
    out_dir="$(cd "$out_dir" && pwd)"
    log "导出到: $out_dir"

    for img in "${IMAGES[@]}"; do
        if ! docker image inspect "$img" >/dev/null 2>&1; then
            err "镜像 $img 不存在，先跑过 docker compose build 吗？"
            exit 1
        fi
    done

    check_disk_space "$out_dir" 40

    local comp img_tar
    comp=$(pick_compressor)
    log "用 $comp 做流式压缩"

    case "$comp" in
        zstd)
            img_tar="docker_sim_images.tar.zst"
            log "docker save ${IMAGES[*]} | zstd -T0 -o $out_dir/$img_tar"
            docker save "${IMAGES[@]}" | zstd -T0 -q -o "$out_dir/$img_tar"
            ;;
        pigz)
            img_tar="docker_sim_images.tar.gz"
            log "docker save ${IMAGES[*]} | pigz > $out_dir/$img_tar"
            docker save "${IMAGES[@]}" | pigz -c > "$out_dir/$img_tar"
            ;;
        gzip)
            img_tar="docker_sim_images.tar.gz"
            log "docker save ${IMAGES[*]} | gzip > $out_dir/$img_tar (没装zstd/pigz，退化成gzip单线程，会比较慢)"
            docker save "${IMAGES[@]}" | gzip -c > "$out_dir/$img_tar"
            ;;
    esac
    log "镜像打包完成: $(du -h "$out_dir/$img_tar" | cut -f1)"

    log "打包docker_sim项目配置（排除staging/、runtime_logs/）"
    tar -czf "$out_dir/docker_sim_config.tar.gz" \
        --exclude='staging' \
        --exclude='runtime_logs' \
        -C "$(dirname "$DOCKER_SIM_DIR")" "$(basename "$DOCKER_SIM_DIR")"
    log "配置打包完成: $(du -h "$out_dir/docker_sim_config.tar.gz" | cut -f1)"

    # 把这个脚本自己也拷进bundle——目标机器上恢复时不一定已经有这份代码
    # （bundle本来就是给"另一台机器"用的，那台机器可能压根没有docker_sim
    # 这个目录），bundle必须自带restore脚本才能真正做到"一键"。
    cp "$SCRIPT_DIR/bundle.sh" "$out_dir/bundle.sh"
    chmod +x "$out_dir/bundle.sh"

    log "生成校验和"
    (cd "$out_dir" && sha256sum "$img_tar" docker_sim_config.tar.gz > SHA256SUMS)

    {
        echo "ai_uav docker_sim 导出清单"
        echo "导出时间: $(date '+%Y-%m-%d %H:%M:%S %Z')"
        echo "导出主机: $(hostname)"
        echo "镜像列表:"
        for img in "${IMAGES[@]}"; do
            local img_id img_created img_size
            img_id=$(docker image inspect "$img" --format '{{.Id}}')
            img_created=$(docker image inspect "$img" --format '{{.Created}}')
            img_size=$(docker image inspect "$img" --format '{{.Size}}')
            echo "  - $img  ID=$img_id  Created=$img_created  Size=${img_size}bytes"
        done
        echo "压缩方式: $comp"
        echo "恢复方法: 把这整个目录拷到目标机器，跑 ./bundle.sh restore ."
    } > "$out_dir/MANIFEST.txt"

    log "导出完成，目录内容："
    ls -lh "$out_dir"
    log "把整个 '$out_dir' 目录拷到目标机器（scp/rsync/U盘均可），在目标机器上跑："
    log "  cd $(basename "$out_dir") && ./bundle.sh restore ."
}

cmd_restore() {
    local bundle_dir="${1:?用法: bundle.sh restore <bundle目录> [--dest 目标目录]}"
    shift || true
    local dest="$PWD"
    if [ "${1:-}" = "--dest" ]; then
        dest="$2"
    fi
    bundle_dir="$(cd "$bundle_dir" && pwd)"
    mkdir -p "$dest"
    dest="$(cd "$dest" && pwd)"

    log "从 $bundle_dir 恢复到 $dest"

    if [ -f "$bundle_dir/SHA256SUMS" ]; then
        log "校验完整性"
        (cd "$bundle_dir" && sha256sum -c SHA256SUMS) || {
            err "校验和不匹配，文件可能在传输过程中损坏，请重新拷贝bundle目录后再试"
            exit 1
        }
    else
        err "没找到SHA256SUMS，跳过完整性校验（不建议，但继续）"
    fi

    local img_tar
    img_tar="$(ls "$bundle_dir"/docker_sim_images.tar.* 2>/dev/null | head -1)"
    if [ -z "$img_tar" ]; then
        err "bundle目录里没找到 docker_sim_images.tar.*"
        exit 1
    fi

    check_disk_space "$dest" 40

    log "docker load 镜像（$(du -h "$img_tar" | cut -f1)，视磁盘/CPU速度可能要几分钟）"
    case "$img_tar" in
        *.tar.zst) zstd -dc "$img_tar" | docker load ;;
        *.tar.gz)  gzip  -dc "$img_tar" | docker load ;;
        *) err "不认识的压缩格式: $img_tar"; exit 1 ;;
    esac

    log "校验三个镜像都已加载"
    local all_ok=1
    for img in "${IMAGES[@]}"; do
        if docker image inspect "$img" >/dev/null 2>&1; then
            log "  OK: $img"
        else
            err "  缺失: $img"
            all_ok=0
        fi
    done
    [ "$all_ok" -eq 1 ] || { err "有镜像没加载成功，检查上面docker load的输出"; exit 1; }

    log "解包项目配置到 $dest"
    tar -xzf "$bundle_dir/docker_sim_config.tar.gz" -C "$dest"

    log "恢复完成！"
    cat <<EOF

下一步：
  cd $dest/docker_sim
  ./start.sh

注意事项（脚本恢复不了，需要人工确认）：
  - 需要装好 nvidia-container-toolkit 才能跑Gazebo/RViz GUI（sim-world的
    docker-compose.yml里预留了GPU），纯无GUI跑仿真理论上不需要，但没验证过。
  - 需要能访问X server（start.sh会自动跑xhost，但这台机器得先有个X session）。
  - staging/ 目录不在这个bundle里（镜像里已经编译好了，用不上）——除非你
    想在这台新机器上重新改代码/重新build镜像，那才需要重新跑
    docker_sim/fetch_sources.sh 把源码拉回来。
EOF
}

# 只有直接执行（不是被source）才跑分发逻辑——方便测试时source这个文件、
# 单独调用里面的函数（pick_compressor/check_disk_space等）。
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-}" in
        export)
            shift
            cmd_export "$@"
            ;;
        restore)
            shift
            cmd_restore "$@"
            ;;
        *)
            echo "用法:"
            echo "  $0 export [输出目录]"
            echo "  $0 restore <bundle目录> [--dest 目标目录]"
            exit 1
            ;;
    esac
fi

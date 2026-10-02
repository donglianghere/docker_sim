#!/usr/bin/env bash
# 一键还原：把被改坏的东西恢复到提交状态。**不需要网络**。
#
#   ./reset.sh --check    # 只看哪些被改过，不动手
#   ./reset.sh            # 还原（会先列出来让你确认）
#   ./reset.sh --yes      # 不确认直接还原
#
# 思路是"不防破坏，让破坏无所谓"：选手能改什么随他，坏了一条命令回到干净状态。
# 还原的来源是**本地 .git**（89 MB，就在仓库里），不是 GitHub——赛场没网也能用。
#
# 它**不碰**未跟踪的文件（aim.jpg、src/vision_stack/、各种 .bak），那些是
# 有意不进版本库的东西，一律保留。
#
# 镜像不在 git 里，删了要从备份恢复，见本文件末尾的说明。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
CHECK=0; YES=0
for a in "$@"; do case "$a" in
    --check) CHECK=1 ;;
    --yes|-y) YES=1 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "未知参数: $a" >&2; exit 2 ;;
esac; done

echo "=============================================="
echo " 还原检查$([ "$CHECK" = 1 ] && echo "（--check：只看不动）")"
echo "=============================================="

changed="$(git diff --name-only; git diff --cached --name-only)"
changed="$(echo "$changed" | sort -u | grep -v '^$' || true)"
missing="$(git ls-files --deleted || true)"

echo "[1/3] 跟踪文件"
if [ -z "$changed$missing" ]; then
    echo "  ✅ 618 个跟踪文件全部跟提交一致，没有被改动"
else
    [ -n "$changed" ] && echo "$changed" | sed 's/^/  ⚠ 被修改：/'
    [ -n "$missing" ] && echo "$missing" | sed 's/^/  ⚠ 被删除：/'
fi

echo "[2/3] 镜像"
lost=0
for img in sim-world:latest flight-stack:latest contestant-sdk:latest; do
    if docker image inspect "$img" >/dev/null 2>&1; then
        echo "  ✅ $img"
    else
        echo "  ❌ $img 不存在"; lost=1
    fi
done

echo "[3/3] 仓库外的启动脚本（不在 git 里，坏了要手工修）"
for f in /home/robots/ai_uav/start_*.sh; do
    [ -f "$f" ] && { bash -n "$f" 2>/dev/null \
        && echo "  ✅ $(basename "$f")" || echo "  ❌ $(basename "$f") 语法已损坏"; }
done

if [ "$CHECK" = 1 ]; then echo "=============================================="; exit 0; fi

if [ -n "$changed$missing" ]; then
    if [ "$YES" != "1" ]; then
        echo
        read -r -p "把上面这些还原到提交状态？未跟踪的文件不受影响。[y/N] " ans
        case "$ans" in y|Y) ;; *) echo "没有改动任何东西。"; exit 0 ;; esac
    fi
    # 只还原跟踪文件。**不用 git clean**——那会删掉 aim.jpg、src/vision_stack/
    # 这些有意保留的未跟踪文件。
    git checkout -- . && git reset -q && echo "  已还原跟踪文件（来源：本地 .git，未联网）"
else
    echo "  跟踪文件没有需要还原的。"
fi

if [ "$lost" = "1" ]; then
    echo
    echo "⚠ 镜像缺失，git 还原不了它们。从离线备份恢复："
    echo "    docker load -i /home/robots/ai_uav/镜像备份/images.tar"
    echo "  （没有备份就只能重新 build，而 build 需要网络——赛场前务必先做备份，"
    echo "    做法见 reset.sh 文件末尾）"
fi
echo "=============================================="

# ---- 赛场前要做的一次性准备 ----
# 1) 备份镜像（35.6 GB，约十几分钟，之后就不需要网络了）：
#      mkdir -p /home/robots/ai_uav/镜像备份
#      docker save sim-world:latest flight-stack:latest contestant-sdk:latest \
#          -o /home/robots/ai_uav/镜像备份/images.tar
# 2) 清掉 build cache（实测占 318 GB，其中 288 GB 可回收）：
#      docker builder prune -af
# 3) 把整个仓库再克隆一份到别的盘/U 盘，防 .git 本身被删：
#      git clone --mirror . /media/备份/docker_sim.git

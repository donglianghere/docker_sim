#!/usr/bin/env bash
# =============================================================================
#  运行仿真.sh —— 双击我，或者在终端里输入 ./运行仿真.sh 运行
# =============================================================================
#
# 这个脚本做的事情很简单：把同一个文件夹里的"我的任务.py"丢进仿真环境里跑，
# 你会看到中文的飞行进度提示（起飞/飞行中/降落…）。
#
# 默认**同时起两架飞机**，两架跑的是同一份"我的任务.py"，只是传给它的
# --role 不一样（一架 leader、一架 follower）——你在那个文件里用
# `if role == 'leader':` 分开写两架各自要做的事。
# 只想飞一架时加 --single 参数：
#     bash 运行仿真.sh --single
#
# 你不需要看懂这个脚本里面在做什么，正常情况下你也不需要改这个文件——
# 你只需要改"我的任务.py"里的坐标数字，然后重新运行这个脚本。
#
# 用法：
#   1. 把这个文件夹（包含"运行仿真.sh"和"我的任务.py"）放在你自己的电脑上；
#   2. 先确认 Docker 已经装好、正在运行（下面脚本会自动检查，检查不过会
#      告诉你具体该怎么办）；
#   3. 双击这个文件，或者打开终端 cd 到这个文件夹后执行：
#          bash 运行仿真.sh
#      （如果双击没反应，大概率是系统没有把.sh文件关联到"用终端打开"，
#      改用终端里执行这条命令肯定管用）；
#   4. 仿真那一侧：如果这个文件夹就放在仿真项目里（同级目录能找到
#      docker-compose.yml），脚本每次运行会自动重建仿真容器，让两架飞机
#      回到各自的起降点，保证每次都从同一个初始条件开始。不想重建就加
#      --no-restart。如果你手里只有这个模板文件夹（没有仿真那套文件），
#      脚本会跳过这一步，仿真环境由赛事方/助教启动。
#
# 如果你想让这个文件夹换个地方放（比如复制到U盘/换一台电脑），整个文件夹
# 一起复制过去就行，脚本会自动找到跟它同目录的"我的任务.py"，不依赖你在
# 哪个目录下执行它。
# =============================================================================

set -eo pipefail

# ---------------------------------------------------------------------------
# 不管你是在哪个目录下执行这个脚本（双击执行 vs 命令行里cd过去再执行，
# 这两种情况下"当前目录"可能是不一样的），都先切到脚本自己所在的目录，
# 保证一会儿挂载给容器的是"这个文件夹"本身，而不是你双击时系统随便给的
# 某个目录（跟本项目里 start.sh 处理同样问题的写法一致）。
# ---------------------------------------------------------------------------
cd "$(dirname "$0")"

TASK_FILE="我的任务.py"
IMAGE_NAME="contestant-sdk:latest"
LEADER_NS="NX01"      # 长机编号
FOLLOWER_NS="NX02"    # 僚机编号
SINGLE=0              # --single：只飞长机那一架
RESTART=1             # --no-restart：不重建仿真容器，直接接着上次的状态跑

while [[ $# -gt 0 ]]; do
    case "$1" in
        --single) SINGLE=1; shift ;;
        --no-restart) RESTART=0; shift ;;
        --leader)   LEADER_NS="$2"; shift 2 ;;
        --follower) FOLLOWER_NS="$2"; shift 2 ;;
        -h|--help)
            echo "用法： bash 运行仿真.sh [--single] [--no-restart] [--leader NX01] [--follower NX02]"
            exit 0 ;;
        *) echo "不认识的参数：$1（加 --help 看用法）"; exit 2 ;;
    esac
done

echo "======================================================================"
echo " 正在检查运行环境……"
echo "======================================================================"

# ---------------------------------------------------------------------------
# 检查1：Docker 命令本身有没有装
# ---------------------------------------------------------------------------
if ! command -v docker >/dev/null 2>&1; then
    echo ""
    echo "【没有检测到 Docker】"
    echo "这台电脑上还没有安装 Docker，运行仿真任务需要先装好 Docker。"
    echo "  下载地址（Docker Desktop，Windows/macOS/Linux都可以从这里下载）："
    echo "  https://www.docker.com/products/docker-desktop/  （占位链接，具体以赛事方通知为准）"
    echo "装好之后，重新双击/运行这个脚本即可。"
    echo ""
    exit 1
fi

# ---------------------------------------------------------------------------
# 检查2：Docker 命令装了，但 Docker 服务本身有没有在运行
#   （比如Docker Desktop还没打开、或者刚开机没启动），docker info 会失败。
# ---------------------------------------------------------------------------
if ! docker info >/dev/null 2>&1; then
    echo ""
    echo "【Docker 没有启动】"
    echo "检测到已经安装了 Docker，但它现在没有在运行。"
    echo "  Windows/macOS：请先打开「Docker Desktop」这个应用程序，等它左下角图标"
    echo "  变成绿色、显示「Running」之后，再重新运行这个脚本。"
    echo "  Linux：请执行 sudo systemctl start docker 启动 Docker 服务。"
    echo ""
    exit 1
fi

# ---------------------------------------------------------------------------
# 检查3：选手运行环境镜像有没有准备好
#   （这个镜像由赛事方/助教提前准备好并分发，选手一般不需要自己构建；
#   如果没有，很可能是还没有按赛事方说明拉取/加载这个镜像）。
# ---------------------------------------------------------------------------
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo ""
    echo "【找不到运行环境镜像：$IMAGE_NAME】"
    echo "还没有准备好名为 $IMAGE_NAME 的运行环境，请按照赛事方提供的说明"
    echo "先获取这个镜像（比如用 docker load 加载赛事方提供的镜像文件，"
    echo "或者 docker pull 从赛事方指定的镜像仓库拉取），获取成功后重新运行"
    echo "这个脚本即可。"
    echo ""
    exit 1
fi

# ---------------------------------------------------------------------------
# 检查4：任务代码文件是否存在（提醒选手别改了文件名/位置）
# ---------------------------------------------------------------------------
if [[ ! -f "$TASK_FILE" ]]; then
    echo ""
    echo "【找不到任务代码文件：$TASK_FILE】"
    echo "请确认「$TASK_FILE」这个文件跟这个脚本放在同一个文件夹里，"
    echo "而且文件名没有被改动过（包括中文字符）。"
    echo ""
    exit 1
fi

if [[ "$SINGLE" == "1" ]]; then
    echo "环境检查通过，开始运行你的任务代码（$TASK_FILE，只飞 $LEADER_NS 一架）……"
else
    echo "环境检查通过，开始运行你的任务代码（$TASK_FILE，两架飞机同时跑）……"
fi
echo "======================================================================"
echo ""

# ---------------------------------------------------------------------------
# 真正的运行命令。几个参数的意思：
#   （故意不加 --rm）容器跑完保留着，这样程序崩了你还能用
#                   `docker logs contest_task_leader` 把完整输出翻出来。
#                   下次运行这个脚本时会先把上次的删掉，不会越堆越多。
#   --network host  让容器和仿真里的飞机能互相发现（技术细节你不用关心，
#                   但这一行不能删）
#   -v "$(pwd)":/workspace
#                   把当前文件夹共享给容器，所以你在"我的任务.py"里改的内容
#                   运行时一定是最新的，不需要重新打包镜像
#   --namespace/--role/--teammate
#                   告诉这个容器"你是哪架飞机、扮演什么角色、队友是谁"。
#                   两架飞机的区别**只有这三个参数**，代码是同一份。
# ---------------------------------------------------------------------------
run_one() {   # $1=容器名 $2=自己编号 $3=角色 $4=队友编号
    docker run -d --name "$1" --network host \
        -v "$(pwd)":/workspace \
        "$IMAGE_NAME" \
        python3 -u "/workspace/$TASK_FILE" \
        --namespace "$2" --role "$3" --teammate "$4" >/dev/null
}

# 上一次没清理干净的同名容器先删掉，否则 docker run 会因为重名直接失败
docker rm -f contest_task_leader contest_task_follower >/dev/null 2>&1 || true

# ---------------------------------------------------------------------------
# 重建仿真容器，让两架飞机回到各自的起降点。
#
# 为什么每次都要重建：上一次跑完飞机停在最后的落点、各种状态（定位源、
# 规划器、编队节点的参数）也都还留着。不重置的话第二次跑就不是从同一个
# 初始条件出发，调出来的结果没法比较，还可能撞上上次遗留的异常状态。
#
# 找不到 docker-compose.yml 就跳过——选手如果只把这个模板文件夹单独拷走
# （手里没有仿真环境那套文件），仿真那一侧本来就由赛事方/助教启动，这个
# 脚本只负责起"你的任务程序"。
# ---------------------------------------------------------------------------
COMPOSE_DIR="$(cd .. 2>/dev/null && pwd)"
if [[ "$RESTART" == "1" && -f "$COMPOSE_DIR/docker-compose.yml" ]]; then
    echo "正在重建仿真环境（让飞机回到起降点）……"
    ( cd "$COMPOSE_DIR" && docker compose down --timeout 20 >/dev/null 2>&1 || true )
    ( cd "$COMPOSE_DIR" && docker compose up -d >/dev/null )

    # 等两架飞机的节点真的起来了再放任务程序进去。等这条日志而不是 sleep
    # 固定秒数：仿真启动耗时随机器负载差别很大，写死秒数要么白等要么不够。
    echo "等两架飞机就绪（最多3分钟）……"
    READY_DEADLINE=$(( $(date +%s) + 180 ))
    while :; do
        ready=1
        for c in nx01 nx02; do
            docker logs "docker_sim-flight-stack-$c-1" 2>&1 \
                | grep -q 'formation_follower_node就绪' || ready=0
        done
        [[ "$ready" == "1" ]] && break
        if [[ "$(date +%s)" -ge "$READY_DEADLINE" ]]; then
            echo ""
            echo "【仿真环境没能在3分钟内就绪】"
            echo "请把下面这条命令的输出发给助教："
            echo "    cd \"$COMPOSE_DIR\" && docker compose logs --tail 50"
            echo ""
            exit 1
        fi
        sleep 5
    done
    echo "仿真环境就绪。"
elif [[ "$RESTART" == "1" ]]; then
    echo "（没找到 docker-compose.yml，跳过重建仿真——假定仿真环境已经由赛事方启动好）"
fi

set +e
run_one contest_task_leader "$LEADER_NS" leader "$FOLLOWER_NS"
CONTAINERS="contest_task_leader"
if [[ "$SINGLE" != "1" ]]; then
    run_one contest_task_follower "$FOLLOWER_NS" follower "$LEADER_NS"
    CONTAINERS="$CONTAINERS contest_task_follower"
fi

# 两架飞机的输出会交替出现。SDK 打印的每一行本来就带 [NX01]/[NX02] 编号，
# 所以这里直接透传，不再额外加前缀（加了会变成 [NX02] [NX02] 这样重复）。
for c in $CONTAINERS; do
    docker logs -f "$c" 2>&1 &
done

# 等两个容器都结束，任一非零退出就记下来
RUN_STATUS=0
for c in $CONTAINERS; do
    code=$(docker wait "$c" 2>/dev/null || echo 1)
    [[ "$code" != "0" ]] && RUN_STATUS=$code
done
wait   # 等日志流打完，避免提示信息插在日志中间
set -e

echo ""
echo "======================================================================"
if [[ $RUN_STATUS -eq 0 ]]; then
    echo " 任务代码已运行结束。"
else
    echo " 任务代码运行时出现了错误（退出码：$RUN_STATUS），请往上翻看看具体是"
    echo " 哪一行的中文提示，或者把完整输出发给助教帮忙看看。"
fi
echo "======================================================================"
# 用真实退出码结束：出错时返回非零，方便在别的脚本里判断这次跑成没成
exit $RUN_STATUS

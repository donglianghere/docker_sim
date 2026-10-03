#!/usr/bin/env bash
# 一键关闭地面站(GCS)的两个容器：gcs + backend（docker compose down，
# 停止并删除这两个容器，不删镜像）。
#
# 这个脚本本身不放在docker_sim项目目录里，所以不能靠"脚本自己在哪个目录"
# 反推项目目录（那样脚本被移到/复制到别处就会失效）——直接写死
# docker_sim/gcs的绝对路径，不管这个脚本文件放在哪、从哪个当前目录执行，
# 都固定操作同一个项目目录。
GCS_DIR="/home/robots/ai_uav/docker_sim/gcs"
set -eo pipefail

# 文件管理器里"右键 -> Run as a program"这种方式，是临时开一个终端窗口跑
# 这个脚本，脚本一结束（不管成功还是失败）终端窗口就自动关掉，没时间看
# 输出/报错——"一闪而逝"就是这么来的，跟脚本本身对不对无关。用trap在
# 脚本退出前（EXIT，覆盖正常结束/`exit 1`/`set -e`中途失败三种情况）
# 停下来等一下按键，双击运行时窗口才能停留到你看清结果再关。命令行里
# 直接跑这个脚本时也会等这一下，代价很小，不影响正常使用。
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

cd "$GCS_DIR"

docker compose down

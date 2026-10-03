#!/usr/bin/env bash
# 一键启动地面站（双击运行用的包装）。
#
# 2026-10-03 去重：原来这里有一份 32 行的独立实现，跟 scripts/up_gcs.sh
# （88 行）重复。up_gcs.sh 是严格超集，它多做的：
#   · xhost 授权（网页上 Gazebo / RViz 两个按钮要用）
#   · gcs/.env 校验（缺 HOST_REPO_PATH 直接报，缺 GCS_ROS_DOMAIN_ID 补默认值）
#   · hw_fleet_tokens.json / gcs_network_state.json 不存在时从 .example 复制
#   · 起完**验证 backend /healthz 响应 + gcs 容器 running** 才算成功
# 原来这份只检查 cyclonedds_gcs.xml 存在，那一项 up_gcs.sh 也做了（而且是
# 自动从 .example 复制，不是报错让人手动拷）。
#
# shell/ 这一层只保留末尾暂停，双击时窗口不会立刻关掉。
set -eo pipefail
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ROOT/scripts/up_gcs.sh" "$@"

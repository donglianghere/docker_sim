#!/usr/bin/env bash
# 一键启动仿真系统（双击运行用的包装）。
#
# 2026-10-03 去重：原来这里有一份 48 行的独立实现，跟 scripts/start_sim.sh
# （153 行）是两套分叉——改一边另一边不知道。现在只保留 shell/ 这一层真正
# 独有的东西：**末尾暂停**，双击运行时窗口不会立刻关掉、看得见报错。
# 逻辑全部转发给 scripts/start_sim.sh，那份更全：
#   · 重启前先清掉选手容器，再 compose down + up
#   · 等两机 PX4 自检 + flight-stack 节点就绪（默认 240 秒）
#   · **验证四路相机真的在出图**才返回，不出图直接给排查顺序
#   · 检查 gzclient / rviz2 起没起来
#
# 参数原样转发： --no-restart / --no-gui / --timeout 秒
#
# 注意：scripts/start_sim.sh 在没有 DISPLAY 时会直接退出（相机渲染必须有 X），
# 原来这份只是警告后继续。双击运行必然有 DISPLAY，不影响；纯 SSH 环境下
# 要起不带相机的仿真，直接 `docker compose up -d` 。
set -eo pipefail
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ROOT/scripts/start_sim.sh" "$@"

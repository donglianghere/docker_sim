#!/bin/bash
# 监控已在目标上空悬停的两机：相机断档 或 检测节点停摆(日志>40s未更新且相机仍出帧) 即取证并停止
set -u
S=/tmp/claude-1001/-home-robots-ai-uav/c891693b-7e4d-4e3b-9515-e475cce90261/scratchpad/target
OUT=$S/out; mkdir -p $OUT; C=docker_sim-flight-stack-nx01-1; SW=docker_sim-sim-world-1
ENVB='source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml'
log(){ echo "[$(date +%T)] $*" | tee -a $OUT/summary.log; }
TS=$(date +%s); DUR=${1:-1200}
log "== 检测节点测试开始（${DUR}s）：NX01在地面着火点(tag2)上空，NX02在物资点(tag0)上空 =="
seen=$(docker exec $C bash -c 'wc -l < /tmp/camdet.log'); next=$((TS+60))
while [ $(( $(date +%s)-TS )) -lt $DUR ]; do
  NEW=$(docker exec $C tail -n +$((seen+1)) /tmp/camdet.log 2>/dev/null); seen=$((seen+$(echo -n "$NEW" | grep -c '')))
  bad=""; stat=""
  for ns in NX01 NX02; do
    c=docker_sim-flight-stack-$(echo $ns | tr A-Z a-z)-1
    dbg=$(docker logs -t --tail 300 $c 2>&1 | grep '\[临时调试\]' | tail -1 | cut -d' ' -f1)
    if [ -n "$dbg" ]; then dage=$(( $(date +%s) - $(date -d "$dbg" +%s) )); else dage=999; fi
    cn=$(echo "$NEW" | grep . | tail -1 | tr '|' '\n' | grep " ${ns}_down:" | grep -oE 'n=[0-9]+' | cut -d= -f2)
    dm=$(echo "$NEW" | grep . | tail -1 | tr '|' '\n' | grep " ${ns}_det:" | grep -oE 'msgs=[0-9]+' | cut -d= -f2)
    stat+=" ${ns}(cam=${cn:-?} det=${dm:-?} dbg_age=${dage}s)"
    [ $dage -gt 40 ] && [ "${cn:-0}" -gt 1 ] && bad+=" $ns:检测节点停摆(日志${dage}s未更新,相机仍在出帧)"
    [ -n "${cn:-}" ] && [ "${cn:-0}" -le 1 ] && bad+=" $ns:下视相机断档"
  done
  # 逐行检查相机断档
  while IFS= read -r L; do [ -z "$L" ] && continue
    for k in NX01_front NX01_down NX02_front NX02_down; do
      seg=$(echo "$L" | tr '|' '\n' | grep " $k:"); n=$(echo "$seg" | grep -oE 'n=[0-9]+' | cut -d= -f2)
      g=$(echo "$seg" | grep -oE 'max_gap=[0-9.]+' | cut -d= -f2)
      { [ -z "$n" ] || [ "$n" -le 1 ]; } && bad+=" $k:无帧"
      [ -n "$g" ] && awk -v g=$g 'BEGIN{exit !(g>0.5)}' && bad+=" $k:间隔${g}s"
    done
  done <<< "$NEW"
  if [ -n "$bad" ]; then
    log "!! 异常 test+$(( $(date +%s)-TS ))s:$bad"; log "   $(echo "$NEW" | grep . | tail -1)"
    D=$OUT/evidence; mkdir -p $D; docker exec $C tail -60 /tmp/camdet.log > $D/camdet_tail.log
    for ns in NX01 NX02; do
      p=$(pgrep -f "lib/contest_mission/qr_apriltag_detect_node --ros-args -r __ns:=/$ns" | head -1); [ -n "$p" ] || continue
      sudo -n gdb -p $p -batch -ex "set sysroot /proc/$p/root" -ex 'set pagination off' -ex 'thread apply all bt 12' > $D/detect_bt_$ns.txt 2>&1
      for i in $(sudo -n ls -l /proc/$p/fd | grep -o 'socket:\[[0-9]*\]' | tr -dc '0-9\n'); do awk -v i=$i -v n=$ns 'NR>1 && $10==i {print n" local="$2" drops="$NF}' /proc/net/udp; done >> $D/sockets.txt
      docker exec $C bash -c "$ENVB; timeout 8 ros2 topic hz /$ns/${ns}_down_camera/image_raw 2>&1 | tail -2" > $D/fresh_hz_$ns.txt 2>&1
    done
    docker exec $SW bash -c "$ENVB; python3 /tmp/grab4.py atfail" >/dev/null 2>&1
    for k in NX01_front NX01_down NX02_front NX02_down; do docker cp $SW:/tmp/grab_atfail_$k.png $D/ >/dev/null 2>&1; done
    log "已取证并停止（容器保留，两机仍悬停），证据在 $D"; exit 2
  fi
  if [ $(date +%s) -ge $next ]; then next=$((next+60)); log "test+$(( $(date +%s)-TS ))s$stat"; fi
  sleep 5
done
docker cp $C:/tmp/camdet.log $OUT/camdet.log >/dev/null
log "== 20分钟测试结束：相机无断档，检测节点持续输出 =="

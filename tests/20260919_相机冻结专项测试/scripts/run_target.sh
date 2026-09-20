#!/bin/bash
# 双相机模式 + 两机飞到真实检测目标上空悬停 + 检测节点测试
# NX01→地面着火点(0,-2) AprilTag id=2；NX02→物资点(-4,-6) AprilTag id=0
set -u
S=/tmp/claude-1001/-home-robots-ai-uav/c891693b-7e4d-4e3b-9515-e475cce90261/scratchpad/target
DUR=${1:-900}; OUT=$S/out; mkdir -p $OUT; SW=docker_sim-sim-world-1
ENVB='source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml'
log(){ echo "[$(date +%T)] $*" | tee -a $OUT/summary.log; }
cd /home/robots/ai_uav/docker_sim
DISPLAY=:1 xhost +local:docker >/dev/null 2>&1
log "== down/up (dual camera) =="
docker compose -f docker-compose.yml -f $S/override_dualcam.yml down >/dev/null 2>&1
docker compose -f docker-compose.yml -f $S/override_dualcam.yml up -d >/dev/null 2>&1; T0=$(date +%s)
until docker exec $SW test -f /tmp/docker_sim_cyclonedds.xml 2>/dev/null; do sleep 1; done
docker cp $S/cam_det_logger.py $SW:/tmp/ >/dev/null; docker cp $S/grab4.py $SW:/tmp/ >/dev/null
docker exec -d $SW bash -c "$ENVB; python3 -u /tmp/cam_det_logger.py > /tmp/camdet.log 2>&1"
log "logger started +$(( $(date +%s)-T0 ))s"
dl=$(( $(date +%s)+400 ))
until [ "$(docker exec $SW bash -c 'grep -l "Ready for takeoff" /tmp/px4_NX0*.log 2>/dev/null | wc -l')" -ge 2 ]; do
  [ $(date +%s) -gt $dl ] && { log "ABORT: PX4未就绪"; exit 1; }; sleep 3; done
log "PX4x2 ready +$(( $(date +%s)-T0 ))s; settle 20s"; sleep 20
docker run --rm --network host -v $S:/workspace contestant-sdk:latest python3 -u /workspace/fly_hover.py NX01 0.0 -2.0 1.5 > $OUT/fly_NX01.log 2>&1 &
docker run --rm --network host -v $S:/workspace contestant-sdk:latest python3 -u /workspace/fly_hover.py NX02 -4.0 -6.0 1.5 > $OUT/fly_NX02.log 2>&1 &
wait
for ns in NX01 NX02; do grep -q "ARRIVED $ns" $OUT/fly_$ns.log || { log "ABORT: $ns 未到位: $(tail -4 $OUT/fly_$ns.log)"; exit 1; }; done
TS=$(date +%s); log "两机就位 +$((TS-T0))s: $(grep -h 'world(' $OUT/fly_NX0*.log | tr '\n' ' ')"
log "== 检测节点测试开始（${DUR}s）：NX01在地面着火点(tag2)上空，NX02在物资点(tag0)上空 =="
docker exec $SW bash -c "$ENVB; python3 /tmp/grab4.py target" >/dev/null 2>&1
for k in NX01_front NX01_down NX02_front NX02_down; do docker cp $SW:/tmp/grab_target_$k.png $OUT/ >/dev/null 2>&1; done
seen=$(docker exec $SW bash -c 'wc -l < /tmp/camdet.log'); dbgstat=("" ""); next=$((TS+60))
while [ $(( $(date +%s)-TS )) -lt $DUR ]; do
  NEW=$(docker exec $SW tail -n +$((seen+1)) /tmp/camdet.log 2>/dev/null); seen=$((seen+$(echo -n "$NEW" | grep -c '')))
  bad=""
  while IFS= read -r L; do [ -z "$L" ] && continue
    i=0
    for ns in NX01 NX02; do
      dm=$(echo "$L" | tr '|' '\n' | grep " ${ns}_det:" | grep -oE 'msgs=[0-9]+' | cut -d= -f2)
      cn=$(echo "$L" | tr '|' '\n' | grep " ${ns}_down:" | grep -oE 'n=[0-9]+' | cut -d= -f2)
      # 停摆判据：检测节点每帧都打[临时调试]（每路相机1秒节流，跟有没有检测到目标无关）。
      # 该日志超过40秒不更新、而相机仍在出帧 => 检测节点停摆。detections只作观测量不作判据。
      c=docker_sim-flight-stack-$(echo $ns | tr A-Z a-z)-1
      dbg=$(docker logs -t --since $TS $c 2>&1 | grep '\[临时调试\]' | tail -1 | cut -d' ' -f1)
      if [ -n "$dbg" ]; then dage=$(( $(date +%s) - $(date -d "$dbg" +%s) )); else dage=$(( $(date +%s)-TS )); fi
      [ $dage -gt 40 ] && [ "${cn:-0}" -gt 1 ] && bad+=" $ns:检测节点停摆(日志${dage}s未更新,相机仍在出帧)"
      [ "${cn:-0}" -le 1 ] && bad+=" $ns:下视相机断档"
      dbgstat[$i]="$ns:det_msgs=${dm:-0},dbg_age=${dage}s"
      i=$((i+1))
    done
    [ -n "$bad" ] && break
  done <<< "$NEW"
  L=$(echo "$NEW" | grep . | tail -1)
  if [ -n "$bad" ]; then
    log "!! 异常 test+$(( $(date +%s)-TS ))s:$bad"; log "   $L"
    D=$OUT/evidence; mkdir -p $D; docker exec $SW tail -40 /tmp/camdet.log > $D/camdet_tail.log
    for ns in NX01 NX02; do
      p=$(pgrep -f "lib/contest_mission/qr_apriltag_detect_node --ros-args -r __ns:=/$ns" | head -1)
      [ -n "$p" ] || continue
      sudo -n gdb -p $p -batch -ex "set sysroot /proc/$p/root" -ex 'set pagination off' -ex 'thread apply all bt 12' > $D/detect_bt_$ns.txt 2>&1
      for i in $(sudo -n ls -l /proc/$p/fd | grep -o 'socket:\[[0-9]*\]' | tr -dc '0-9\n'); do awk -v i=$i -v n=$ns 'NR>1 && $10==i {print n" local="$2" drops="$NF}' /proc/net/udp; done >> $D/sockets.txt
    done
    docker exec $SW bash -c "$ENVB; python3 /tmp/grab4.py atfail" >/dev/null 2>&1
    for k in NX01_front NX01_down NX02_front NX02_down; do docker cp $SW:/tmp/grab_atfail_$k.png $D/ >/dev/null 2>&1; done
    log "已取证并停止（容器保留），证据在 $D"; exit 2
  fi
  if [ $(date +%s) -ge $next ]; then next=$((next+60)); log "test+$(( $(date +%s)-TS ))s ${dbgstat[0]} ${dbgstat[1]} | $(echo "$L" | cut -d'|' -f1,6,7)"; echo "$L" >> $OUT/summary.log; fi
  sleep 3
done
docker cp $SW:/tmp/camdet.log $OUT/camdet.log >/dev/null
log "== 测试结束：全程相机无断档、检测节点持续输出 =="

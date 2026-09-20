#!/bin/bash
# 检测节点接收端根因实验：标准配置起仿真，在flight-stack-nx01里并行跑3个探针
set -u
S=/tmp/claude-1001/-home-robots-ai-uav/c891693b-7e4d-4e3b-9515-e475cce90261/scratchpad/detect
OUT=$S/out; mkdir -p $OUT; C=docker_sim-flight-stack-nx01-1; SW=docker_sim-sim-world-1
T=/NX01/NX01_switchable_camera/image_raw
ENVB='source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
log(){ echo "[$(date +%T)] $*" | tee -a $OUT/summary.log; }
cd /home/robots/ai_uav/docker_sim
DISPLAY=:1 xhost +local:docker >/dev/null 2>&1
log "== down/up (stock) =="
docker compose down >/dev/null 2>&1; docker compose up -d >/dev/null 2>&1; T0=$(date +%s)
until docker exec $C test -f /tmp/docker_sim_cyclonedds.xml 2>/dev/null; do sleep 1; done
docker cp $S/probe.py $C:/tmp/ >/dev/null; docker cp $S/cyclone_trace.xml $C:/tmp/ >/dev/null
# A: BEST_EFFORT + Cyclone radmin跟踪（跟检测节点同QoS）
docker exec -d $C bash -c "$ENVB; export CYCLONEDDS_URI=file:///tmp/cyclone_trace.xml; python3 -u /tmp/probe.py A best $T > /tmp/probe_A.log 2>&1"
# B: RELIABLE
docker exec -d $C bash -c "$ENVB; export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml; python3 -u /tmp/probe.py B reliable $T > /tmp/probe_B.log 2>&1"
# C: BEST_EFFORT，延后240秒启动（验证“新订阅者不受影响”）
docker exec -d $C bash -c "$ENVB; export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml; python3 -u /tmp/probe.py C best $T 240 > /tmp/probe_C.log 2>&1"
log "probes A/B/C started at +$(( $(date +%s)-T0 ))s"
while [ $(( $(date +%s)-T0 )) -lt ${1:-600} ]; do
  el=$(( $(date +%s)-T0 ))
  d=$(docker logs --since $T0 $C 2>&1 | grep '\[临时调试\]' | wc -l)
  dl=$(docker logs -t --since $T0 $C 2>&1 | grep '\[临时调试\]' | tail -1 | cut -d' ' -f1)
  ds=$([ -n "$dl" ] && echo $(( $(date +%s) - $(date -d "$dl" +%s) )) || echo -1)
  log "+${el}s detect_node: lines=$d last_frame_age=${ds}s | $(docker exec $C tail -1 /tmp/probe_A.log 2>/dev/null | cut -d' ' -f2-) | $(docker exec $C tail -1 /tmp/probe_B.log 2>/dev/null | cut -d' ' -f2-) | $(docker exec $C tail -1 /tmp/probe_C.log 2>/dev/null | cut -d' ' -f2-)"
  sleep 30
done
for f in probe_A probe_B probe_C; do docker cp $C:/tmp/$f.log $OUT/ >/dev/null 2>&1; done
docker exec $C bash -c 'ls -la /tmp/cyc_trace_probeA.log; wc -l /tmp/cyc_trace_probeA.log' | tee -a $OUT/summary.log
log "== done =="

#!/bin/bash
# 双相机模式：重建→等PX4就绪→两机原地起飞1m→30分钟监测4路相机；断档即停（容器保留）
set -u
S=/tmp/claude-1001/-home-robots-ai-uav/c891693b-7e4d-4e3b-9515-e475cce90261/scratchpad/dualcam
DUR=${1:-1800}; OUT=$S/out; mkdir -p $OUT; SW=docker_sim-sim-world-1
ENV='source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml'
log(){ echo "[$(date +%T)] $*" | tee -a $OUT/summary.log; }
CAMS="NX01_front NX01_down NX02_front NX02_down"
cd /home/robots/ai_uav/docker_sim
DISPLAY=:1 xhost +local:docker >/dev/null 2>&1
log "== down/up (dual camera override) =="
docker compose -f docker-compose.yml -f $S/override_dualcam.yml down >/dev/null 2>&1
docker compose -f docker-compose.yml -f $S/override_dualcam.yml up -d >/dev/null 2>&1; T0=$(date +%s)
until docker exec $SW test -f /tmp/docker_sim_cyclonedds.xml 2>/dev/null; do sleep 1; done
docker cp $S/cam4_logger.py $SW:/tmp/ >/dev/null
docker exec -d $SW bash -c "$ENV; python3 -u /tmp/cam4_logger.py > /tmp/cam4.log 2>&1"
log "logger started +$(( $(date +%s)-T0 ))s"
# 等两台PX4 Ready for takeoff
dl=$(( $(date +%s)+400 ))
until [ "$(docker exec $SW bash -c 'grep -l "Ready for takeoff" /tmp/px4_NX0*.log 2>/dev/null | wc -l')" -ge 2 ]; do
  [ $(date +%s) -gt $dl ] && { log "ABORT: PX4 not ready in 400s"; exit 1; }; sleep 3; done
log "PX4x2 ready +$(( $(date +%s)-T0 ))s; settle 20s"; sleep 20
log "camera SDF check: $(docker exec $SW bash -c 'for n in NX01 NX02; do echo -n "$n:"; grep -o "sensor type=.camera. name=.[A-Za-z0-9_]*" /tmp/iris_mid360_$n.sdf | tr "\n" " "; done')"
# 两机同时原地起飞
for ns in NX01 NX02; do
  docker run --rm --network host -v $S:/workspace contestant-sdk:latest python3 -u /workspace/takeoff_hover.py $ns > $OUT/takeoff_$ns.log 2>&1 &
done
wait
for ns in NX01 NX02; do grep -q "TAKEOFF_OK $ns" $OUT/takeoff_$ns.log || { log "ABORT: $ns takeoff failed: $(tail -3 $OUT/takeoff_$ns.log)"; exit 1; }; done
TS=$(date +%s); log "both airborne at +$((TS-T0))s: $(docker exec $SW tail -1 /tmp/cam4.log | cut -d'|' -f1)"
log "== 30-min camera test starts =="
seen=$(docker exec $SW bash -c 'wc -l < /tmp/cam4.log'); seen=$(echo "$seen" | tail -1)
next=$((TS+60))
while [ $(( $(date +%s)-TS )) -lt $DUR ]; do
  NEW=$(docker exec $SW tail -n +$((seen+1)) /tmp/cam4.log 2>/dev/null); seen=$((seen+$(echo -n "$NEW" | grep -c '')))
  bad=""
  while IFS= read -r L; do [ -z "$L" ] && continue
  for c in $CAMS; do
    seg=$(echo "$L" | tr '|' '\n' | grep " $c:")
    n=$(echo "$seg" | grep -oE 'n=[0-9]+' | cut -d= -f2); g=$(echo "$seg" | grep -oE 'max_gap=[0-9.]+' | cut -d= -f2)
    if [ -z "$n" ] || [ "$n" -le 1 ]; then bad+=" $c:no_frames"; elif awk -v g=$g 'BEGIN{exit !(g>0.5)}'; then bad+=" $c:gap=${g}s"; fi
  done
  [ -n "$bad" ] && break
  done <<< "$NEW"
  L=$(echo "$NEW" | grep . | tail -1)
  if [ -n "$bad" ]; then
    log "!! GAP at test+$(( $(date +%s)-TS ))s:$bad"; log "   $L"
    docker exec $SW tail -40 /tmp/cam4.log > $OUT/cam4_at_gap.log
    log "STOPPED (containers left running)"; exit 2
  fi
  if [ $(date +%s) -ge $next ]; then
    next=$((next+60)); line="test+$(( $(date +%s)-TS ))s fresh-sub:"
    for c in $CAMS; do ns=${c%_*}; cam=${c#*_}
      r=$(docker exec $SW bash -c "$ENV; timeout 6 ros2 topic hz /$ns/${ns}_${cam}_camera/image_raw 2>&1 | grep 'average rate' | tail -1 | awk '{print \$3}'")
      line+=" $c=${r:-0}"; done
    log "$line"; echo "$L" >> $OUT/summary.log
  fi
  sleep 3
done
log "== 30-min test PASSED: no gap on 4 cameras =="

#!/bin/bash
# 反复重建，直到抓到检测节点冻结；抓到即取证并停止（容器保留）
set -u
S=/tmp/claude-1001/-home-robots-ai-uav/c891693b-7e4d-4e3b-9515-e475cce90261/scratchpad/detect
OUT=$S/hunt; mkdir -p $OUT; C=docker_sim-flight-stack-nx01-1; C2=docker_sim-flight-stack-nx02-1
T=/NX01/NX01_switchable_camera/image_raw
ENVB='source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml'
log(){ echo "[$(date +%T)] $*" | tee -a $OUT/summary.log; }
cd /home/robots/ai_uav/docker_sim
for R in $(seq 1 ${1:-6}); do
  DISPLAY=:1 xhost +local:docker >/dev/null 2>&1
  log "===== round $R: down/up ====="
  docker compose down >/dev/null 2>&1; docker compose up -d >/dev/null 2>&1; T0=$(date +%s)
  until docker exec $C test -f /tmp/docker_sim_cyclonedds.xml 2>/dev/null; do sleep 1; done
  docker cp $S/probe.py $C:/tmp/ >/dev/null
  docker exec -d $C bash -c "$ENVB; python3 -u /tmp/probe.py A best $T > /tmp/probe_A.log 2>&1"
  docker exec -d $C bash -c "$ENVB; python3 -u /tmp/probe.py B reliable $T > /tmp/probe_B.log 2>&1"
  log "probes started +$(( $(date +%s)-T0 ))s"
  frozen=0
  while [ $(( $(date +%s)-T0 )) -lt 420 ]; do
    el=$(( $(date +%s)-T0 ))
    dl=$(docker logs -t --since $T0 $C 2>&1 | grep '\[临时调试\]' | tail -1 | cut -d' ' -f1)
    if [ -n "$dl" ]; then age=$(( $(date +%s) - $(date -d "$dl" +%s) )); else age=$el; fi
    pa=$(docker exec $C tail -1 /tmp/probe_A.log 2>/dev/null | cut -d' ' -f2-)
    pb=$(docker exec $C tail -1 /tmp/probe_B.log 2>/dev/null | cut -d' ' -f2-)
    [ $((el % 60)) -lt 15 ] && log "+${el}s detect_age=${age}s | $pa | $pb"
    if { [ -n "$dl" ] && [ $age -gt 25 ]; } || { [ -z "$dl" ] && [ $el -gt 150 ]; }; then
      frozen=1; D=$OUT/round${R}_frozen_at_${el}s; mkdir -p $D
      log "!! FROZEN round $R at +${el}s (detect_age=${age}s)"
      log "   probeA: $pa"; log "   probeB: $pb"
      docker exec $C tail -20 /tmp/probe_A.log > $D/probe_A.log 2>&1
      docker exec $C tail -20 /tmp/probe_B.log > $D/probe_B.log 2>&1
      p=$(pgrep -f "lib/contest_mission/qr_apriltag_detect_node --ros-args -r __ns:=/NX01" | head -1)
      sudo -n gdb -p $p -batch -ex "set sysroot /proc/$p/root" -ex 'set pagination off' -ex 'thread apply all bt 12' > $D/detect_bt.txt 2>&1
      for i in $(sudo -n ls -l /proc/$p/fd | grep -o 'socket:\[[0-9]*\]' | tr -dc '0-9\n'); do awk -v i=$i 'NR>1 && $10==i {print "detect local="$2" drops="$NF}' /proc/net/udp; done > $D/sockets.txt
      for pp in $(pgrep -f "probe.py [AB]"); do for i in $(sudo -n ls -l /proc/$pp/fd | grep -o 'socket:\[[0-9]*\]' | tr -dc '0-9\n'); do awk -v i=$i -v n=$pp 'NR>1 && $10==i {print "probe pid="n" local="$2" drops="$NF}' /proc/net/udp; done; done >> $D/sockets.txt
      docker exec $C bash -c "$ENVB; timeout 8 ros2 topic hz $T 2>&1 | tail -2" > $D/fresh_hz.txt 2>&1
      docker exec $C bash -c 'ps -o pid,stat,pcpu,etime,nlwp -p '"$p"'; cat /proc/'"$p"'/status | grep -E "Threads|voluntary"' > $D/detect_ps.txt 2>&1
      cp $D/sockets.txt /dev/stdout | tee -a $OUT/summary.log
      log "STOPPED for inspection (containers running), evidence in $D"; exit 2
    fi
    sleep 10
  done
  log "round $R: no freeze in 420s"
done
log "===== no freeze reproduced ====="

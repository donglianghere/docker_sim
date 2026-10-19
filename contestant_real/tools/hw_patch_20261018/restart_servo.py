# -*- coding: utf-8 -*-
"""原样重启 precision_servo_node：沿用它自己进程的 environ 和 cmdline，
不碰飞行栈容器（热重启容器会搞坏 UWB，只能断电恢复）。"""
import os, signal, subprocess, sys, time

NEEDLE = b'lib/contest_mission/precision_servo_node'
target = None
argv = None
for p in os.listdir('/proc'):
    if not p.isdigit():
        continue
    try:
        raw = open('/proc/%s/cmdline' % p, 'rb').read()
    except Exception:
        continue
    if NEEDLE in raw and b'--ros-args' in raw:
        parts = [a for a in raw.split(b'\0') if a]
        # 只认真实节点进程，不要 `ros2 run` 那层 wrapper
        if any(x.endswith(b'precision_servo_node') and x.startswith(b'/opt') for x in parts):
            target = int(p)
            argv = [a.decode('utf-8') for a in parts]
            break
if target is None:
    sys.exit('precision_servo_node 没在跑，放弃')

env = {}
for kv in open('/proc/%d/environ' % target, 'rb').read().split(b'\0'):
    if b'=' in kv:
        k, v = kv.split(b'=', 1)
        env[k.decode('utf-8', 'replace')] = v.decode('utf-8', 'replace')

stat = open('/proc/%d/stat' % target).read()
ppid = int(stat.split(')')[-1].split()[1])
print('节点 pid=%d，wrapper ppid=%d' % (target, ppid))
print('cmdline: %s' % ' '.join(argv))

for pid in (target, ppid):
    if pid <= 1:
        continue
    try:
        os.kill(pid, signal.SIGKILL)
        print('killed %d' % pid)
    except Exception as exc:
        print('kill %d 失败: %r' % (pid, exc))
time.sleep(2)

log = open('/tmp/precision_servo_restart.log', 'ab')
subprocess.Popen(argv, env=env, stdout=log, stderr=log,
                 start_new_session=True, cwd='/')
time.sleep(4)
alive = [p for p in os.listdir('/proc')
         if p.isdigit() and NEEDLE in open('/proc/%s/cmdline' % p, 'rb').read()
         if os.path.exists('/proc/%s/cmdline' % p)]
print('重启后存活进程: %s' % alive)

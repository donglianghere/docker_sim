# -*- coding: utf-8 -*-
"""不依赖ROS运行时，用桩件验证 takeoff_monitor_node 的判定逻辑。

重点覆盖 2026-09-14 踩过的那个坑：位置稳定窗口"是否攒够时长"必须用
独立的 first_sample_time 判断，不能用 trim 之后剩下的 history[0]——
否则判定永远不可能成立（实测表现为 takeoff 稳定卡在超时）。
"""
import math
import sys, types, importlib.util, time

def stub(name, **attrs):
    m = types.ModuleType(name); [setattr(m, k, v) for k, v in attrs.items()]; sys.modules[name] = m; return m

class TakeoffLand:
    TAKEOFF = 1; LAND = 2
    def __init__(self): self.takeoff_land_cmd = 0
class State: pass
class Odometry: pass
class _Goal: pass
class _Result:
    def __init__(self): self.success=False; self.stage=''; self.message=''; self.final_z=0.0
class _Feedback:
    def __init__(self): self.phase=''; self.armed=False; self.current_z=0.0; self.climb_m=0.0
class Takeoff:
    Goal=_Goal; Result=_Result; Feedback=_Feedback

stub('mavros_msgs'); stub('mavros_msgs.msg', State=State)
stub('nav_msgs'); stub('nav_msgs.msg', Odometry=Odometry)
stub('quadrotor_msgs'); stub('quadrotor_msgs.msg', TakeoffLand=TakeoffLand)
stub('quadrotor_msgs.action', Takeoff=Takeoff)
stub('rclpy', init=lambda *a,**k: None, shutdown=lambda *a,**k: None)
stub('rclpy.action', ActionServer=lambda *a, **k: None,
     CancelResponse=types.SimpleNamespace(ACCEPT=1), GoalResponse=types.SimpleNamespace(ACCEPT=1, REJECT=2))
stub('rclpy.callback_groups', ReentrantCallbackGroup=lambda: None)
stub('rclpy.executors', MultiThreadedExecutor=lambda: None)
stub('rclpy.qos', qos_profile_sensor_data=None)

class _Pub:
    def __init__(self): self.sent=[]
    def publish(self, m): self.sent.append(getattr(m,'takeoff_land_cmd',None))
class _Logger:
    def info(self,*a): pass
    def warn(self,*a): pass
    def error(self,*a): pass
class Node:
    def __init__(self, name): self._p={}
    def declare_parameter(self,n,v=None): self._p[n]=v
    def get_parameter(self,n): return types.SimpleNamespace(value=self._p.get(n))
    def create_publisher(self,*a,**k): return _Pub()
    def create_subscription(self,*a,**k): return None
    def get_logger(self): return _Logger()
stub('rclpy.node', Node=Node)

spec = importlib.util.spec_from_file_location(
    'tmn', '/home/robots/ai_uav/docker_sim/src/contest_mission/contest_mission/takeoff_monitor_node.py')
tmn = importlib.util.module_from_spec(spec); spec.loader.exec_module(tmn)

fails=[]
def check(name, cond, extra=''):
    print(('  OK  ' if cond else '  FAIL')+f' {name}'+(f'  [{extra}]' if extra and not cond else ''))
    if not cond: fails.append(name)

def node():
    n = tmn.TakeoffMonitorNode.__new__(tmn.TakeoffMonitorNode)
    Node.__init__(n, 'x')
    for k,v in [('stable_window_s',0.3),('min_climb_m',0.6),('pos_tolerance_m',0.3),
                ('hover_after_s',0.2),('default_timeout_s',60.0),('armed_timeout_s',30.0)]:
        n._p[k]=v
    n._armed=None; n._odom_xyz=None
    return n

print('用例1：位置稳定窗口——攒满窗口时长前不判定（防 09-14 那个坑的反向验证）')
n = node(); n._odom_xyz=(0.0,0.0,1.0)
hist=[]; fst=None
ok,hist,fst = n._pos_stable(hist,fst,0.0)
check('第一次采样不应判定成立', ok is False)
check('first_sample_time 已记录且不为 None', fst is not None)

print('用例2：攒满窗口且爬升足够 -> 判定成立')
deadline=time.monotonic()+0.5
while time.monotonic()<deadline:
    ok,hist,fst = n._pos_stable(hist,fst,0.0)
    if ok: break
    time.sleep(0.02)
check('攒满 0.3 秒窗口后判定成立', ok is True, f'ok={ok}')

print('用例3：爬升量不足（停在地面不动）-> 永不成立')
n2 = node(); n2._odom_xyz=(0.0,0.0,0.05)
h2=[]; f2=None; ok2=False
deadline=time.monotonic()+0.5
while time.monotonic()<deadline:
    ok2,h2,f2 = n2._pos_stable(h2,f2,0.0)
    if ok2: break
    time.sleep(0.02)
check('爬升 0.05m < min_climb 0.6m 时不成立', ok2 is False)

print('用例4：位置漂移超容差 -> 不成立')
n3 = node(); h3=[]; f3=None; ok3=False
deadline=time.monotonic()+0.5; i=0
while time.monotonic()<deadline:
    i+=1; n3._odom_xyz=(0.0+ (i%2)*1.0, 0.0, 1.0)   # 在 x 上来回跳 1 米
    ok3,h3,f3 = n3._pos_stable(h3,f3,0.0)
    if ok3: break
    time.sleep(0.02)
check('漂移 1m > 容差 0.3m 时不成立', ok3 is False)

print('用例5：没有里程计数据 -> 不成立且不崩')
n4 = node(); n4._odom_xyz=None
ok4,_,_ = n4._pos_stable([],None,0.0)
check('无里程计时返回 False', ok4 is False)

print('用例6：外部发布者监测')
n5 = node(); n5._external_takeoff_seen=False; n5._own_publish_count=1
m=TakeoffLand(); m.takeoff_land_cmd=TakeoffLand.TAKEOFF
n5._on_takeoff_land(m)
check('自己发的那帧被抵扣，不算外部', n5._external_takeoff_seen is False)
n5._on_takeoff_land(m)
check('第二帧抵扣不掉，判定为外部发布', n5._external_takeoff_seen is True)
m2=TakeoffLand(); m2.takeoff_land_cmd=TakeoffLand.LAND
n6 = node(); n6._external_takeoff_seen=False; n6._own_publish_count=0
n6._on_takeoff_land(m2)
check('LAND 指令不计入外部起飞', n6._external_takeoff_seen is False)

print('用例7：已在空中判定（2026-09-20补）')
n7 = node(); n7._p['already_airborne_z_m'] = 0.5
n7._armed = True; n7._odom_xyz = (0.0, 0.0, 1.29)
check('已解锁且高度1.29m -> 判定为已在空中', n7._already_airborne() is True)
n7._odom_xyz = (0.0, 0.0, 0.33)
check('已解锁但仍在地面(0.33m) -> 不算在空中', n7._already_airborne() is False)
n7._armed = False; n7._odom_xyz = (0.0, 0.0, 1.29)
check('未解锁但高度够 -> 不算在空中', n7._already_airborne() is False)
n7._armed = True; n7._odom_xyz = None
check('没有里程计数据 -> 不算在空中且不崩', n7._already_airborne() is False)

print()
print('用例8：起飞前就绪判定（2026-09-21，用户："不能飞机一出世就给起飞命令"）')
R = tmn.preflight_ready
MAX, INST = 0.08, 0.09
STILL = [(0.0, 0.0, 0.0)] * 45          # 真静止：45个零样本
check('飞控还没连上 -> 不发', R(None, 0.01, STILL, True, MAX, INST)[0] is False)
check('connected=False -> 不发', R(False, 0.01, STILL, True, MAX, INST)[0] is False)
check('还没收到里程计 -> 不发', R(True, None, None, True, MAX, INST)[0] is False)
check('采样窗口还没攒满 -> 不发', R(True, 0.01, STILL, False, MAX, INST)[0] is False)
check('窗口满且静止 -> 可以发', R(True, 0.01, STILL, True, MAX, INST)[0] is True)
check('均值达标但这一帧瞬时0.12m/s -> 等下一帧（控制器会按瞬时值拒）',
      R(True, 0.12, STILL, True, MAX, INST)[0] is False)
check('真在动(0.3m/s朝x) -> 不发',
      R(True, 0.3, [(0.3, 0.0, 0.0)] * 45, True, MAX, INST)[0] is False)
check('不通过时给出可读原因', '还没静止' in R(True, 0.3, [(0.3, 0.0, 0.0)] * 45, True, MAX, INST)[1])

print('用例9：为什么要对"矢量"平均而不是对"速度大小"平均（2026-09-21实测根因）')
import random
random.seed(20260921)
# 飞机真静止，三轴各叠加 sigma=0.07 的零均值噪声——复现 NX02 地面实测的量级
noisy = [(random.gauss(0, 0.07), random.gauss(0, 0.07), random.gauss(0, 0.07))
         for _ in range(45)]
mag_mean = sum(math.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in noisy) / len(noisy)
vec_mean = tmn.mean_speed_vector(noisy)
check(f'对"速度大小"取平均：{mag_mean:.3f}m/s，仍然高于门限{MAX}（旧做法就卡在这）',
      mag_mean > MAX)
check(f'对"速度矢量"取平均再取模：{vec_mean:.3f}m/s，低于门限（噪声互相抵消）',
      vec_mean < MAX)
check('所以静止时新判据能通过，旧判据通不过',
      R(True, 0.02, noisy, True, MAX, INST)[0] is True)
check('空样本 -> 返回无穷大，不会误判成静止',
      tmn.mean_speed_vector([]) == float('inf'))
check('真实运动不会被平均掉：45帧都朝x以0.5m/s -> 均值仍是0.5',
      abs(tmn.mean_speed_vector([(0.5, 0.0, 0.0)] * 45) - 0.5) < 1e-9)

print()
print('全部通过' if not fails else f'失败 {len(fails)} 项: {fails}')
sys.exit(1 if fails else 0)

# -*- coding: utf-8 -*-
"""不依赖ROS运行时，用桩件验证 takeoff_monitor_node 的判定逻辑。

重点覆盖 2026-09-14 踩过的那个坑：位置稳定窗口"是否攒够时长"必须用
独立的 first_sample_time 判断，不能用 trim 之后剩下的 history[0]——
否则判定永远不可能成立（实测表现为 takeoff 稳定卡在超时）。
"""
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
MAX, HOLD = 0.08, 3.0
check('飞控还没连上 -> 不发', R(None, 0.01, 100.0, 110.0, MAX, HOLD)[0] is False)
check('connected=False -> 不发', R(False, 0.01, 100.0, 110.0, MAX, HOLD)[0] is False)
check('还没收到里程计 -> 不发', R(True, None, None, 110.0, MAX, HOLD)[0] is False)
check('速度0.51m/s（NX02实测被拒时的值）-> 不发',
      R(True, 0.512840, None, 110.0, MAX, HOLD)[0] is False)
check('速度够低但刚静止0.5秒 -> 不发（瞬时低值不算收敛）',
      R(True, 0.01, 109.5, 110.0, MAX, HOLD)[0] is False)
check('持续静止3.0秒整 -> 可以发', R(True, 0.01, 107.0, 110.0, MAX, HOLD)[0] is True)
check('持续静止5秒 -> 可以发', R(True, 0.02, 105.0, 110.0, MAX, HOLD)[0] is True)
check('速度刚好等于门限且已持续够 -> 可以发',
      R(True, 0.08, 105.0, 110.0, MAX, HOLD)[0] is True)
check('门限比控制器的0.1m/s严，0.09被拦下',
      R(True, 0.09, 105.0, 110.0, MAX, HOLD)[0] is False)
check('不通过时给出可读原因', '定位源还没收敛' in R(True, 0.5, None, 110.0, MAX, HOLD)[1])

print()
print('全部通过' if not fails else f'失败 {len(fails)} 项: {fails}')
sys.exit(1 if fails else 0)

# -*- coding: utf-8 -*-
"""不依赖ROS运行时，用桩件验证 actuator_action_node 的 goal_id 幂等/结果关联行为。"""
import sys, types, importlib.util

def stub(name, **attrs):
    m = types.ModuleType(name); [setattr(m, k, v) for k, v in attrs.items()]; sys.modules[name] = m; return m

class OverrideRCIn:
    CHAN_NOCHANGE = 65535
    def __init__(self): self.channels = []
class String:
    def __init__(self): self.data = ''
class SetParametersResult:
    def __init__(self, successful=True): self.successful = successful

stub('mavros_msgs'); stub('mavros_msgs.msg', OverrideRCIn=OverrideRCIn)
stub('std_msgs'); stub('std_msgs.msg', String=String)
stub('rcl_interfaces'); stub('rcl_interfaces.msg', SetParametersResult=SetParametersResult)
stub('rclpy', init=lambda *a, **k: None, spin=lambda *a, **k: None, shutdown=lambda *a, **k: None)

PUBLISHED = {'status': [], 'override': []}
class _Pub:
    def __init__(self, kind): self.kind = kind
    def publish(self, msg): PUBLISHED[self.kind].append(msg.data if hasattr(msg, 'data') else msg.channels)
class _Timer:
    def __init__(self): self.cancelled = False
    def cancel(self): self.cancelled = True
class _Logger:
    def info(self, *a): pass
    def warn(self, *a): pass
    def error(self, *a): pass
class Node:
    def __init__(self, name): self._params = {}
    def declare_parameter(self, n, v=None): self._params[n] = v
    def get_parameter(self, n): return types.SimpleNamespace(value=self._params.get(n))
    def create_publisher(self, t, topic, q): return _Pub('override' if 'override' in topic else 'status')
    def create_timer(self, period, cb): self._tick = cb; return _Timer()
    def add_on_set_parameters_callback(self, cb): self._pcb = cb
    def get_logger(self): return _Logger()
    def get_clock(self): return types.SimpleNamespace(now=lambda: 0)
stub('rclpy.node', Node=Node)

spec = importlib.util.spec_from_file_location(
    'aan', '/home/robots/ai_uav/docker_sim/src/contest_mission/contest_mission/actuator_action_node.py')
aan = importlib.util.module_from_spec(spec); spec.loader.exec_module(aan)

def P(name, value): return types.SimpleNamespace(name=name, value=value)
def fresh():
    PUBLISHED['status'].clear(); PUBLISHED['override'].clear()
    n = aan.ActuatorActionNode(); PUBLISHED['status'].clear(); return n
def finish(n):
    n._action_start_monotonic -= 999   # 让保持时长判定为已到
    n._on_override_tick()

fails = []
def check(name, cond, extra=''):
    print(('  OK  ' if cond else '  FAIL') + f' {name}' + (f'  [{extra}]' if extra and not cond else ''))
    if not cond: fails.append(name)

print('用例1：同一个goal_id重复下发，动作只执行一次')
n = fresh()
n._pcb([P('action_name','grab_supply'), P('action_goal_id','NX02-grab_supply-1')])
first_overrides = len(PUBLISHED['override'])
n._pcb([P('action_name','grab_supply'), P('action_goal_id','NX02-grab_supply-1')])
check('重复下发不再触发新动作', len(PUBLISHED['override']) == first_overrides, f"override帧数 {first_overrides}->{len(PUBLISHED['override'])}")
check('重复下发重播running状态', PUBLISHED['status'][-1] == 'running:grab_supply:NX02-grab_supply-1', PUBLISHED['status'][-1])

print('用例2：完成后同一个goal_id再下发，重播done而不是重做')
finish(n)
check('完成状态带goal_id', PUBLISHED['status'][-1] == 'done:grab_supply:NX02-grab_supply-1', PUBLISHED['status'][-1])
ov = len(PUBLISHED['override'])
n._pcb([P('action_name','grab_supply'), P('action_goal_id','NX02-grab_supply-1')])
check('已完成的goal_id不重做动作', len(PUBLISHED['override']) == ov)
check('已完成的goal_id重播done', PUBLISHED['status'][-1] == 'done:grab_supply:NX02-grab_supply-1', PUBLISHED['status'][-1])

print('用例3：上一个动作没结束时，新goal_id被明确拒绝')
n = fresh()
n._pcb([P('action_name','grab_supply'), P('action_goal_id','g1')])
n._pcb([P('action_name','drop_supply'), P('action_goal_id','g2')])
check('新goal_id在busy时被拒绝', PUBLISHED['status'][-1] == 'rejected:drop_supply:g2:busy', PUBLISHED['status'][-1])

print('用例4：未知动作名被拒绝并带原因')
n = fresh()
n._pcb([P('action_name','no_such_action'), P('action_goal_id','g9')])
check('未知动作名被拒绝', PUBLISHED['status'][-1] == 'rejected:no_such_action:g9:unknown_action', PUBLISHED['status'][-1])

print('用例5：老调用方（不带goal_id）行为完全不变')
n = fresh()
n._pcb([P('action_name','grab_supply')])
finish(n)
check('老格式 done:<name> 保持不变', PUBLISHED['status'][-1] == 'done:grab_supply', PUBLISHED['status'][-1])
ov = len(PUBLISHED['override'])
n._pcb([P('action_name','grab_supply')])
check('老调用方重复下发仍会重新执行（原行为）', len(PUBLISHED['override']) > ov)

print()
print('全部通过' if not fails else f'失败 {len(fails)} 项: {fails}')
sys.exit(1 if fails else 0)

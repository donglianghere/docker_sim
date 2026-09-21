# -*- coding: utf-8 -*-
"""不依赖ROS运行时，验证编队轨迹跟随的采样逻辑（2026-09-20）。

重点是"沿折线回溯"而不是"直线偏移"——直角拐弯处两者结果明显不同，
抄近道会让僚机切内弯、不再走长机走过的路径，轨迹跟随就落空了。
"""
import os, sys, math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _ros_stubs  # noqa: E402

ffn = _ros_stubs.load_node_module(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'formation_follower_node.py'))

fails = []
def check(name, cond, extra=''):
    print(('  OK  ' if cond else '  FAIL') + f' {name}' + (f'  [{extra}]' if extra and not cond else ''))
    if not cond: fails.append(name)

print('用例1：直线轨迹，僚机落后3米、同高、有速度前馈')
b = ffn.LeaderPathBuffer()
for i in range(41):                      # 沿x轴 0->20m，2m/s
    b.append(i * 0.5, 0.0, 1.5, i * 0.25)
x, y, z, vx, vy, yaw = b.sample_behind(3.0)
check('x 落后长机3米', abs(x - 17.0) < 1e-6, f'x={x}')
check('y 不变', abs(y) < 1e-9)
check('z 与轨迹同高(1.5m)', abs(z - 1.5) < 1e-9)
check('速度前馈≈2m/s 指向+x', abs(vx - 2.0) < 0.05 and abs(vy) < 1e-6, f'vx={vx:.3f}')
check('yaw 指向前进方向', abs(yaw) < 1e-6)

print('用例2：直角拐弯——必须沿折线回溯，不能抄近道')
b2 = ffn.LeaderPathBuffer()
for i in range(21): b2.append(i * 0.5, 0.0, 1.5, i * 0.25)        # 沿x到(10,0)
for i in range(1, 11): b2.append(10.0, i * 0.5, 1.5, 5.0 + i * 0.25)  # 再沿y到(10,5)
x2, y2 = b2.sample_behind(3.0)[:2]
check('沿折线回溯到(10,2)', abs(x2 - 10.0) < 1e-6 and abs(y2 - 2.0) < 1e-6, f'({x2:.2f},{y2:.2f})')
check('与长机直线距离=3m，证明点落在折线上', abs(math.hypot(10.0 - x2, 5.0 - y2) - 3.0) < 1e-6)

print('用例3：长机爬升时，僚机取轨迹上该点的高度而非长机当前高度')
b3 = ffn.LeaderPathBuffer()
for i in range(41): b3.append(i * 0.5, 0.0, 1.0 + i * 0.025, i * 0.25)
check('z=1.85（轨迹点）而非2.0（长机当前）', abs(b3.sample_behind(3.0)[2] - 1.85) < 1e-6)

print('用例4：兜底行为')
b4 = ffn.LeaderPathBuffer(); b4.append(0, 0, 1.5, 0); b4.append(1, 0, 1.5, 0.5)
check('跟随距离超出缓冲区总长 -> 返回最老点', abs(b4.sample_behind(10.0)[0]) < 1e-9)
check('空缓冲区 -> None', ffn.LeaderPathBuffer().sample_behind(3.0) is None)

print('用例5：投影——僚机"走到哪了"（只看水平，不看垂直）')
b5 = ffn.LeaderPathBuffer()
for i in range(41): b5.append(i * 0.5, 0.0, 1.5, i * 0.25)   # 长机沿x走到(20,0)
check('僚机在(5,0) -> 投影弧长5.0', abs(b5.project_arc_length(5.0, 0.0) - 5.0) < 1e-6)
check('僚机偏离轨迹3米(5,3) -> 投影仍是5.0（垂直/横向偏差不计入弧长）',
      abs(b5.project_arc_length(5.0, 3.0) - 5.0) < 1e-6)
check('空缓冲区 -> None', ffn.LeaderPathBuffer().project_arc_length(0.0, 0.0) is None)

print('用例6：按弧长取点——取出来的点必须落在长机走过的折线上')
pt = b5.point_at_arc_length(5.8)
check('弧长5.8 -> (5.8, 0)', abs(pt[0] - 5.8) < 1e-6 and abs(pt[1]) < 1e-9,
      f'({pt[0]:.2f},{pt[1]:.2f})')
check('弧长超出总长 -> 钳到终点(20,0)', abs(b5.point_at_arc_length(999.0)[0] - 20.0) < 1e-6)
check('弧长负数 -> 钳到起点(0,0)', abs(b5.point_at_arc_length(-5.0)[0]) < 1e-9)

print('用例7：间距下限——参考点不越过"落后长机follow_distance"那条线')
total = b5.total_length()
check('总长20米', abs(total - 20.0) < 1e-6)
s_limit = max(0.0, total - 3.5)
check('间距下限对应弧长16.5', abs(s_limit - 16.5) < 1e-6)
check('顶到下限时参考点在(16.5,0)，不会顶到长机(20,0)屁股上',
      abs(b5.point_at_arc_length(s_limit)[0] - 16.5) < 1e-6)
check('僚机在(5,0)时沿轨迹落后量11.5米（>3.5，合规）',
      abs((s_limit - b5.project_arc_length(5.0, 0.0)) - 11.5) < 1e-6)

print('用例8：拐弯处沿折线推进，不抄近道')
b8 = ffn.LeaderPathBuffer()
for i in range(21): b8.append(i * 0.5, 0.0, 1.5, i * 0.25)
for i in range(1, 21): b8.append(10.0, i * 0.5, 1.5, 5.0 + i * 0.25)  # 长机到(10,10)
s8 = b8.project_arc_length(9.6, 0.0)
p8 = b8.point_at_arc_length(s8 + 0.8)   # 从投影点沿折线前进0.8米
# 先走到拐角(10,0)用0.4米，再沿y走0.4米 -> (10, 0.4)；抄近道会得到(10.4, 0)之类
check('沿折线绕过拐角到(10,0.4)', abs(p8[0] - 10.0) < 1e-6 and abs(p8[1] - 0.4) < 1e-6,
      f'({p8[0]:.2f},{p8[1]:.2f})')
check('跨拐角取点的 y 一定为正（抄近道时 y 恒为0）', p8[1] > 0)

print('用例9：定位噪声不能虚增轨迹长度（2026-09-21实测根因）')
import random
random.seed(20260921)
# 长机停在(0,0)一动不动，UWB按 range_noise_std=0.05m/轴 加噪，30秒@30Hz
b9 = ffn.LeaderPathBuffer()
for i in range(900):
    b9.append(random.gauss(0.0, 0.05), random.gauss(0.0, 0.05), 1.5, i / 30.0)
check(f'静止900帧后轨迹长度仍≈0（实测得到{b9.total_length():.1f}m）',
      b9.total_length() < 1.0, f'{b9.total_length():.1f}m')
check('静止时不会满足"长机已起步"判据(total>=3.5)', b9.total_length() < 3.5)
# 对照：门限设成噪声量级(0.05)时会虚增几十米——这就是原来的缺陷
b9bad = ffn.LeaderPathBuffer(min_point_gap_m=0.05)
random.seed(20260921)
for i in range(900):
    b9bad.append(random.gauss(0.0, 0.05), random.gauss(0.0, 0.05), 1.5, i / 30.0)
check(f'对照组：门限0.05米时静止也虚增出{b9bad.total_length():.0f}米（原缺陷）',
      b9bad.total_length() > 20.0, f'{b9bad.total_length():.1f}m')
# 真实移动仍然采得到：带噪声沿x走10米
b9ok = ffn.LeaderPathBuffer()
random.seed(7)
for i in range(300):
    x = i * 10.0 / 300.0
    b9ok.append(x + random.gauss(0.0, 0.05), random.gauss(0.0, 0.05), 1.5, i / 30.0)
check(f'真实走了10米时长度≈10米（得到{b9ok.total_length():.1f}m），噪声门限没有把真实运动滤掉',
      9.0 < b9ok.total_length() < 11.5, f'{b9ok.total_length():.2f}m')
check('76米航线不会被缓冲区裁掉（上限150米）',
      ffn.LeaderPathBuffer()._max_buffer_length_m >= 100.0)

print()
print('全部通过' if not fails else f'失败 {len(fails)} 项: {fails}')
sys.exit(1 if fails else 0)

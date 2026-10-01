#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 capabilities.py 生成 SDK API 参考（Markdown）。

    python3 scripts/gen_sdk_api_doc.py            # 写 src/contest_sdk/API参考.md
    python3 scripts/gen_sdk_api_doc.py --check    # 只检查分类有没有漏，不写文件

签名和一句话说明**从源码读**，不手抄——手抄的文档迟早跟代码对不上。
分类表 CATEGORIES 是人工定的：机器分不出"选手天天用"和"底层备用"的区别。
新加的公开方法如果没进分类表，生成时会报错（而不是悄悄漏掉）。
"""
import argparse
import ast
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src', 'contest_sdk', 'contest_sdk', 'capabilities.py')
OUT = os.path.join(ROOT, 'src', 'contest_sdk', 'API参考.md')

#: 第一梯队：一个典型任务基本只用这些。顺序就是任务里的先后。
COMMON = ['run', 'takeoff', 'fly_route', 'goto_world', 'hold_at', 'return_home',
          'lead_formation', 'follow_formation', 'open_inbox', 'wait_event',
          'send_to_teammate', 'announce', 'search_along', 'aim_at', 'patrol',
          'snapshot', 'fetch_from', 'release_at', 'grip', 'shoot']

CATEGORIES = [
    ('程序入口', ['run', 'shutdown']),
    ('起飞 / 降落 / 返航', [
        'takeoff', 'land', 'land_or_confirm', 'return_home', 'own_pad',
        'land_on', 'precision_land_and_confirm', 'precision_land_at']),
    ('航线飞行', [
        'fly_route', 'goto_world', 'hold_at', 'goto', 'goto_route', 'goto_direct',
        'cancel_goto', 'set_agl', 'step_forward', 'set_max_vel', 'set_direct_speed',
        'set_fixed_altitude', 'fixed_altitude', 'set_fixed_altitude_off']),
    ('朝向', [
        'face_yaw', 'face_point', 'set_yaw_mode_constant', 'set_yaw_mode_point',
        'set_yaw_mode_velocity', 'get_current_yaw']),
    ('位置与坐标', [
        'get_local_position', 'get_agl', 'world_to_local', 'local_to_world']),
    ('编队', [
        'lead_formation', 'follow_formation', 'teammate_formation_lag',
        'start_formation_follow', 'stop_formation_follow', 'set_formation_leg_route']),
    ('跨机协同', [
        'open_inbox', 'wait_event', 'wait_any_event', 'event_ready',
        'send_to_teammate', 'on_teammate_event', 'yield_spot']),
    ('视觉：识别与对准', [
        'look_for', 'wait_for_detection', 'locate_target', 'search_along', 'patrol',
        'aim_at', 'center_on_target', 'stop_precision_servo', 'clear_detections']),
    ('拍照', ['snapshot', 'capture_photo']),
    ('抓放与发射', [
        'grip', 'fetch_from', 'release_at', 'shoot',
        'servos', 'set_servo', 'set_servos', 'set_actuator', 'do_action']),
    ('声光播报', ['announce', 'play_sound_light', 'mute_sound_light', 'trigger_alarm']),
    ('杂项 / 底层', [
        'progress', 'set_mission_state', 'get_mission_state',
        'generate_orbit_waypoints', 'generate_ground_scan_waypoints',
        'pull_waypoints_out_of_circles', 'reset_aim',
        'read_fire_pillar_staging_pose', 'read_fire_pillar_aim_pose']),
]

#: 踩过坑、必须在参考里单独点名的。写在方法条目下面。
PITFALLS = {
    'open_inbox': '可靠事件通道是**先回 ACK 再查处理函数**，没注册的事件会被确认后'
                  '**丢弃**。任务一开始就把所有事件注册全，别等用到了再注册。',
    'aim_at': '`camera=\'down\'` 会自动改走 `center_on_target()`。前视那套几何把画面'
              '纵轴当成世界的高低，下视时纵轴其实是机体前后方向，照搬永远收敛不了。',
    'fixed_altitude': '**只管 `goto()`**。`goto_direct()` / 精准降落 / 僚机编队跟随都是'
                      '另一条旁路，不受它影响——那几条本来就是直接命令 z，不会飘。',
    'goto_direct': '**不经规划器、没有避障**。传当前高度就是定高平飞。限速由'
                   '`set_direct_speed()` 管，默认 0.3 m/s。',
    'set_direct_speed': '跟 `set_max_vel()` 是两条链路：那个管规划器（`goto`/`fly_route`'
                        '/`lead_formation`），这个管直飞。实测 1.0 m/s 是拐点，再高反而慢。',
    'goto_world': '`direct=True` 会关掉避障，**只在算过余量的航段上开**'
                  '（`scripts/check_route.py`）。航线上的避障本身是考核点，不能绕。',
    'land': '飞控偶尔不把 armed 置回 false，飞机明明已经在地上。任务流程里用'
            '`land_or_confirm()` 或 `return_home()`，它们兜住了这一条。',
    'snapshot': '**不要给拍照配声光事件**：声光事件是固定枚举，自造事件名会直接抛'
                '`ValueError` 把整个任务打断。',
    'center_on_target': '走 precision_servo_node，而它**只认下视相机**的检测'
                        '（源码里 `if \'_camera_down_\' not in frame_id: return`）。',
    'run': '选手程序的 main()：解析命令行、建 SDK、按角色分派、收尾。'
           '用法 `DroneSDK.run(leader=recon, follower=supply)`。',
}


def collect():
    tree = ast.parse(io.open(SRC, encoding='utf-8').read())
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'DroneSDK')
    out = {}
    for n in cls.body:
        if not isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        if n.name.startswith('_'):
            continue
        doc = ast.get_docstring(n) or ''
        # 取**第一段**（到空行为止）拼成一行，再截到第一个句号——docstring 的
        # 第一句常常跨两三个物理行，只取第一行会断在半句话上。
        para = []
        for line in doc.splitlines():
            if not line.strip():
                if para:
                    break
                continue
            para.append(line.strip())
        first = ''.join(para)
        for end in ('。', '；'):
            if end in first:
                first = first.split(end)[0] + '。'
                break
        sig = ast.unparse(n.args)
        sig = sig.replace('self, ', '').replace('self', '')
        deco = {d.id for d in n.decorator_list if isinstance(d, ast.Name)}
        out[n.name] = (sig, first, 'staticmethod' in deco or 'property' in deco,
                       'property' in deco)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()

    api = collect()
    listed = {m for _, ms in CATEGORIES for m in ms}
    missing = sorted(set(api) - listed)
    stale = sorted(listed - set(api))
    if missing or stale:
        if missing:
            print(f'★NG  这些公开方法没进分类表，补进 CATEGORIES：{missing}', file=sys.stderr)
        if stale:
            print(f'★NG  分类表里这些方法源码中已不存在：{stale}', file=sys.stderr)
        return 1
    if args.check:
        print(f'OK   {len(api)} 个公开方法全部有分类')
        return 0

    L = []
    L.append('# contest_sdk API 参考\n')
    L.append('> 由 `scripts/gen_sdk_api_doc.py` 从 `capabilities.py` 自动生成，'
             '**不要手改**。\n> 改了 SDK 之后重新跑一遍这个脚本。\n')
    L.append(f'`DroneSDK` 共 {len(api)} 个公开方法。构造：\n')
    L.append('```python\nfrom contest_sdk import DroneSDK\n\n'
             'sdk = DroneSDK(namespace=\'NX01\', role=\'recon\', teammate_namespace=\'NX02\')\n'
             '```\n')
    L.append('命令行参数的解析、按角色分派和收尾，用 `DroneSDK.run()` 就够了：\n')
    L.append('```python\nif __name__ == \'__main__\':\n'
             '    DroneSDK.run(leader=recon, follower=supply)\n```\n')

    L.append('\n## 常用的 20 个\n')
    L.append('一个典型任务基本只用到这些，按任务里出现的先后排：\n')
    L.append('| 方法 | 做什么 |')
    L.append('|---|---|')
    for m in COMMON:
        L.append(f'| [`{m}()`](#{m}) | {api[m][1]} |')

    L.append('\n## 全部方法\n')
    for cat, names in CATEGORIES:
        L.append(f'\n### {cat}\n')
        for m in names:
            sig, first, is_static, is_prop = api[m]
            head = f'`{m}`' if is_prop else f'`{m}({sig})`'
            L.append(f'<a id="{m}"></a>')
            L.append(f'- **{head}**'
                     + ('  *(staticmethod)*' if is_static and not is_prop else '')
                     + ('  *(property)*' if is_prop else ''))
            L.append(f'  {first}')
            if m in PITFALLS:
                L.append(f'  > ⚠️ {PITFALLS[m]}')
    L.append('\n---\n')
    L.append('每个方法的完整说明（为什么这么设计、踩过什么坑）在 '
             '`capabilities.py` 的 docstring 里，这里只放一句话摘要。\n')
    io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    print(f'已生成 {os.path.relpath(OUT, ROOT)}（{len(api)} 个方法，'
          f'{len(CATEGORIES)} 个分类）')
    return 0


if __name__ == '__main__':
    sys.exit(main())

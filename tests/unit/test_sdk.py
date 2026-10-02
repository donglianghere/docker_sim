# -*- coding: utf-8 -*-
"""SDK 离线单元测试：不起仿真、不要 ROS，秒级跑完。

    python3 tests/unit/test_sdk.py          # 或 python3 -m unittest discover tests/unit

**为什么要有**：改 SDK 原来的唯一关卡是整轮试飞，5~16 分钟，而且一轮只能
验一条路径。但 SDK 里大量逻辑跟飞行无关——航点跳过判据、收件箱语义、解散
通知的成败处理、几何换算——这些秒级就能验。

**测什么**：挑 2026-10-01 真出过事的那些，每条都对应一次实际故障。不为覆盖率
凑数：没出过问题又跟飞行强耦合的部分，留给试飞。

用标准库 unittest，不依赖 pytest——赛场没网，测试不该要求先装东西。
"""
import contextlib
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _ros_stub                                    # noqa: E402

cap = _ros_stub.install()
DroneSDK = cap.DroneSDK


def bare(**attrs):
    """造一个不跑 __init__ 的 DroneSDK，只带测试要用的属性。

    __init__ 会连 ROS、建发布者、注册收件箱，离线环境下都做不了，而被测的
    这些方法根本不碰它们。

    `servos` 这类在类上是 **property** 的，实例 setattr 会报
    "can't set attribute"，所以先造一个一次性子类把它们覆盖掉。
    """
    props = {k: v for k, v in attrs.items()
             if isinstance(getattr(DroneSDK, k, None), property)}
    cls = type('DroneSDK测试替身', (DroneSDK,),
               {k: property(lambda self, _v=v: _v) for k, v in props.items()}) \
        if props else DroneSDK
    sdk = cls.__new__(cls)
    sdk.namespace = 'NX01'
    sdk._progress = lambda *a, **kw: None           # 日志吞掉，测试输出保持干净
    for k, v in attrs.items():
        if k not in props:
            setattr(sdk, k, v)
    return sdk


class 航点跳过判据(unittest.TestCase):
    """ROUTE_SKIP_M。原来是 0.05 m，比悬停漂移还小，这条分支等于永远不成立：
    综合任务第 2/3 轮都从 A 点出发而 A 正是航线首点，判不出重合就对着十几厘米
    的残差向量算 atan2，航向是纯噪声（实测 -9°），飞机白转近 290°。"""

    def test_阈值取0点3(self):
        self.assertEqual(DroneSDK.ROUTE_SKIP_M, 0.3)

    def test_三个阈值的大小关系不能乱(self):
        # 跳过 < 到达容忍 < 过点半径。乱了会出现"判成到达却不跳过"这类自相矛盾
        self.assertLess(DroneSDK.ROUTE_SKIP_M, DroneSDK.FLY_ROUTE_ACCEPT_M)
        self.assertLess(DroneSDK.FLY_ROUTE_ACCEPT_M, DroneSDK.PASSED_M)

    def test_脚下的航点被跳过_远的不跳(self):
        飞过 = []
        sdk = bare(
            get_local_position=lambda: (0.0, 0.0, 2.0),
            world_to_local=lambda x, y, z: (x, y, z),
            face_yaw=lambda *a, **kw: True,
            fixed_altitude=lambda z: contextlib.nullcontext(),
            _goto_with_retry=lambda x, y, z, tag: 飞过.append((x, y)),
        )
        # 0.2 m 在脚下（<0.3）应跳过；3 m 那个要飞
        DroneSDK.fly_route(sdk, [(0.2, 0.0), (3.0, 0.0)], hold_s=0.0)
        self.assertEqual(飞过, [(3.0, 0.0)])

    def test_悬停漂移量级不会误触发(self):
        """实测漂移 0.06~0.27 m，都该被判成"在脚下"。"""
        for d in (0.06, 0.15, 0.27):
            self.assertLess(d, DroneSDK.ROUTE_SKIP_M, f'漂移 {d} 应判为重合')


class 收件箱语义(unittest.TestCase):
    """可靠事件通道"先回 ACK 再查处理函数"，没注册的事件会被确认后丢弃。"""

    def _sdk(self):
        sdk = bare(on_teammate_event=lambda name, cb: None)
        return sdk

    def test_重复注册不清空已收到的内容(self):
        sdk = self._sdk()
        DroneSDK.open_inbox(sdk, 'ev')
        sdk._inbox['ev'] = {'x': 1}
        sdk._inbox_seen.add('ev')
        DroneSDK.open_inbox(sdk, 'ev')              # 再注册一次
        self.assertEqual(sdk._inbox['ev'], {'x': 1})
        self.assertIn('ev', sdk._inbox_seen)

    def test_取走后复位_第二轮不会读到旧数据(self):
        """多轮流程里同一个事件会来好几次，不复位的话第二轮一进来就返回旧值。"""
        sdk = self._sdk()
        DroneSDK.open_inbox(sdk, 'ev')
        sdk._inbox['ev'] = {'n': 1}
        sdk._inbox_seen.add('ev')
        self.assertEqual(DroneSDK.wait_event(sdk, 'ev', 1.0), {'n': 1})
        self.assertNotIn('ev', sdk._inbox_seen)     # 已复位
        self.assertFalse(DroneSDK.event_ready(sdk, 'ev'))

    def test_required为False时超时返回None而不是抛(self):
        sdk = self._sdk()
        DroneSDK.open_inbox(sdk, 'ev')
        self.assertIsNone(DroneSDK.wait_event(sdk, 'ev', 0.05, required=False))

    def test_没注册就等会明确报错_而不是静默超时(self):
        sdk = self._sdk()
        with self.assertRaises(RuntimeError) as e:
            DroneSDK.wait_event(sdk, '没注册过', 0.05)
        self.assertIn('open_inbox', str(e.exception))

    def test_wait_any_event按列表顺序返回(self):
        """两个都到了时返回列表靠前的那个——综合任务靠这条保证
        EV_FIRE 优先于 EV_INSPECT_DONE。"""
        sdk = self._sdk()
        DroneSDK.open_inbox(sdk, 'a', 'b')
        sdk._inbox['a'] = {'from': 'a'}
        sdk._inbox['b'] = {'from': 'b'}
        sdk._inbox_seen.update({'a', 'b'})
        self.assertEqual(DroneSDK.wait_any_event(sdk, ['a', 'b'], 1.0)[0], 'a')


class 解散通知(unittest.TestCase):
    """2026-10-01 的真 bug：`state['sent'] = True` 写在 send 之前，发失败时
    sent 已置位，lead_formation 的 finally 兜底被自己堵死，僚机干等 600 秒。"""

    def test_发送成功才置位(self):
        sdk = bare(send_to_teammate=lambda ev: None)
        st = {'sent': False}
        self.assertTrue(DroneSDK._fire_disband(sdk, st, '测试'))
        self.assertTrue(st['sent'])

    def test_发送失败不置位_兜底才能接住(self):
        def 炸(ev):
            raise RuntimeError('队友没回 ACK')
        sdk = bare(send_to_teammate=炸)
        st = {'sent': False}
        self.assertFalse(DroneSDK._fire_disband(sdk, st, '测试'))
        self.assertFalse(st['sent'], 'sent 置位了的话 finally 的补发会被跳过')


class 间距默认值(unittest.TestCase):
    """spacing_m 不传时取实例上的值（run() 从 --spacing 存进去的）。"""

    def test_类上声明了spacing_m(self):
        # 只在 run() 里动态挂的话，编辑器认不出来、选手打 sdk.spacing_m 会标红
        self.assertTrue(hasattr(DroneSDK, 'spacing_m'))
        self.assertIsInstance(DroneSDK.spacing_m, float)

    def test_PHOTO_DIR可以被实例覆盖(self):
        sdk = bare()
        sdk.PHOTO_DIR = '/logs/别的目录'
        self.assertEqual(sdk.PHOTO_DIR, '/logs/别的目录')
        self.assertNotEqual(DroneSDK.PHOTO_DIR, '/logs/别的目录')   # 类属性没被改


class 几何(unittest.TestCase):
    def test_step_forward沿机头方向_返回的是离地高度不是局部z(self):
        """返回第三个值必须是 AGL：两机 odom 原点不在一处，把局部 z 当 AGL
        发给队友会偏。"""
        import math
        到达 = []
        sdk = bare(
            get_local_position=lambda: (1.0, 2.0, 5.0),     # 局部 z=5
            get_current_yaw=lambda: math.pi / 2,            # 正北
            goto_direct=lambda x, y, z: 到达.append((x, y, z)),
            local_to_world=lambda x, y, z: (x + 10, y + 20, z),
            get_agl=lambda: 2.0,                            # 离地只有 2
        )
        wx, wy, agl = DroneSDK.step_forward(sdk, 1.5)
        self.assertAlmostEqual(到达[0][0], 1.0, places=6)    # x 不变
        self.assertAlmostEqual(到达[0][1], 3.5, places=6)    # y 前移 1.5
        self.assertAlmostEqual(agl, 2.0, msg='返回的该是 AGL，不是局部 z=5')
        # 返回的 x/y 是世界系（local_to_world 的结果），不是局部系
        self.assertAlmostEqual(wx, 11.0, places=6)
        self.assertAlmostEqual(wy, 23.5, places=6)


@contextlib.contextmanager
def 不等舵机():
    """舵机没有位置反馈，代码里只能靠 sleep 等行程（每次 2 秒）。
    单元测试不关心真实耗时，停掉 sleep——不停的话光 shoot(3) 就要 12 秒。"""
    with mock.patch.object(cap.time, 'sleep', lambda *_: None):
        yield


class 发射与抓放(unittest.TestCase):
    def test_连发次数与标签(self):
        发了 = []
        sdk = bare(servos={1: object()},
                   set_servos=lambda d: 发了.append(sorted(d.items())),
                   play_sound_light=lambda e: 发了.append(('声光', e)))
        with 不等舵机():
            DroneSDK.shoot(sdk, shots=3, interval_s=0.0, sound='侦察机发射破窗弹')
        声光 = [x for x in 发了 if isinstance(x, tuple) and x[0] == '声光']
        self.assertEqual(len(声光), 1, '连发只播一次声光')
        舵机 = [x for x in 发了 if x not in 声光]
        self.assertEqual(len(舵机), 6, '每发两步（推出+复位），3 发共 6 次')

    def test_没配舵机不抛异常(self):
        sdk = bare(servos={})
        with 不等舵机():
            DroneSDK.shoot(sdk, shots=2, interval_s=0.0)     # 不该炸

    def test_抓紧和松开的PWM是真机实测值(self):
        self.assertEqual(DroneSDK.GRIP_CLOSE_PWM, 800)
        self.assertEqual(DroneSDK.GRIP_OPEN_PWM, 2000)


class 直飞限速(unittest.TestCase):
    def test_巡航值是实测拐点1点0(self):
        """8 m 航段实测：0.3->40.3s、1.0->14.6s、1.5->17.4s（反而更慢）。"""
        self.assertEqual(DroneSDK.DIRECT_CRUISE_MPS, 1.0)

    def test_用完恢复出厂默认(self):
        设过 = []
        sdk = bare(set_direct_speed=lambda v: 设过.append(v),
                   goto_direct=lambda x, y, z: None)
        DroneSDK._direct_leg(sdk, 1.0, 2.0, 3.0)
        self.assertEqual(设过, [1.0, 0.3], '飞完必须还原，否则落点精修会带着高限速')

    def test_飞行中抛异常也要恢复限速(self):
        设过 = []

        def 炸(x, y, z):
            raise RuntimeError('规划器急停')
        sdk = bare(set_direct_speed=lambda v: 设过.append(v), goto_direct=炸)
        with self.assertRaises(RuntimeError):
            DroneSDK._direct_leg(sdk, 1.0, 2.0, 3.0)
        self.assertEqual(设过, [1.0, 0.3], 'finally 没兜住')


if __name__ == '__main__':
    unittest.main(verbosity=2)

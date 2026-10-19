"""`set_servo()`的PWM -> 归一化换算测试：发出去的值经PX4输出侧
`interpolate(v,-1,1,MIN,MAX)`+四舍五入（mixer_module.cpp
`output_limit_calc_single`）之后，必须精确还原成选手给的PWM。
需要ROS2环境（`capabilities.py`会import rclpy），在contestant-sdk镜像里跑。
"""
import unittest

from contest_sdk.capabilities import (ROLE_GRIPPER, ROLE_LAUNCHER, SERVO_CONFIG,
                                      ServoSpec, servo_pwm_to_normalized)


def _px4_output(spec: ServoSpec, v: float) -> int:
    return round(spec.pwm_min + (v + 1.0) / 2.0 * (spec.pwm_max - spec.pwm_min))


class TestServo(unittest.TestCase):
    def test_nx02_config_matches_hardware(self):
        """NX02 的接线：MAIN7+MAIN9 共用 Actuator Set1 当爪子（**2026-10-18 起
        1000~1950**，下端 1000=抓紧、上端 1950=打开；上一版是 900~2100），MAIN5
        在 Set2 当发射器（800~2200，没变）。"""
        cfg = SERVO_CONFIG['NX02']
        self.assertEqual(cfg[1], ServoSpec(actuator_set=1, output='MAIN7+MAIN9',
                                           pwm_min=1000, pwm_max=1950, role=ROLE_GRIPPER,
                                           pwm_action=1000, pwm_idle=1950))
        self.assertEqual(cfg[2], ServoSpec(actuator_set=2, output='MAIN5',
                                           pwm_min=800, pwm_max=2200, role=ROLE_LAUNCHER,
                                           pwm_action=2200, pwm_idle=800))

    def test_nx01_launcher_travel_differs(self):
        """2026-10-18：**爪子两机相同（1000~1950）、发射机构两机不同**——NX01 的
        发射器是 900~2100，从那架飞控读回的真值。照搬 NX02 的 2200/800 会被
        servo_pwm_to_normalized() 判越界，这就是共享表被拆开的原因。"""
        nx01, nx02 = SERVO_CONFIG['NX01'], SERVO_CONFIG['NX02']
        self.assertEqual(nx01[1], nx02[1], '爪子两机相同（两边飞控都是 1000~1950）')
        self.assertEqual((nx01[2].pwm_min, nx01[2].pwm_max), (900, 2100))
        self.assertEqual((nx01[2].action_pwm, nx01[2].idle_pwm), (2100, 900))
        self.assertNotEqual(nx01[2], nx02[2], '发射器行程两机不同（10-18 起）')
        for pwm in (800, 2200):                 # NX02 发射器的两端，在 NX01 越界
            with self.assertRaises(ValueError):
                servo_pwm_to_normalized(nx01[2], pwm)

    def test_nx02_gripper_rejects_previous_travel(self):
        """回归（2026-10-18）：NX02 爪子行程从 900~2100 收到 1000~1950，照搬上一版
        的 900/2100 必须当场抛 ValueError，不能静默夹到端点。"""
        sp = SERVO_CONFIG['NX02'][1]
        for pwm in (900, 2100, 999, 1951):
            with self.assertRaises(ValueError):
                servo_pwm_to_normalized(sp, pwm)

    def test_gripper_direction_is_measured_not_derived(self):
        """2026-10-15 改装后实测：行程下端=抓紧、上端=打开，跟 10-09 那版
        （2200=抓紧）相反。方向只能来自 pwm_action/pwm_idle，不能按行程端点推，
        也不能照搬上一版的数值（10-18 起 NX02 是 1000/1950）。"""
        sp = SERVO_CONFIG['NX02'][1]
        self.assertEqual(sp.action_pwm, 1000)   # 抓紧
        self.assertEqual(sp.idle_pwm, 1950)     # 打开

    def test_action_and_idle_within_travel(self):
        """动作位/静止位必须落在该路的 PWM 行程里，否则发指令时才报越界。"""
        for ns, cfg in SERVO_CONFIG.items():
            for num, spec in cfg.items():
                for name, pwm in (('action', spec.action_pwm), ('idle', spec.idle_pwm)):
                    self.assertTrue(spec.pwm_min <= pwm <= spec.pwm_max,
                                    f'{ns} 舵机{num} 的 {name}={pwm} 不在 '
                                    f'{spec.pwm_min}~{spec.pwm_max} 内')

    def test_roles_are_known(self):
        """角色只能是这两个——写错了 grip()/shoot() 会静默地一个都不驱动。"""
        for ns, cfg in SERVO_CONFIG.items():
            for num, spec in cfg.items():
                self.assertIn(spec.role, (ROLE_GRIPPER, ROLE_LAUNCHER),
                              f'{ns} 的舵机{num} role 非法：{spec.role}')

    def test_gripper_and_launcher_use_different_sets(self):
        """爪子和发射器不能落在同一个 Actuator Set 上：一条
        MAV_CMD_DO_SET_ACTUATOR 的一个槽位会同时带动它们。"""
        for ns, cfg in SERVO_CONFIG.items():
            grip_sets = {s.actuator_set for s in cfg.values() if s.role == ROLE_GRIPPER}
            fire_sets = {s.actuator_set for s in cfg.values() if s.role == ROLE_LAUNCHER}
            self.assertFalse(grip_sets & fire_sets,
                             f'{ns} 的爪子和发射器共用 Set {grip_sets & fire_sets}')

    def test_every_pwm_roundtrips_exactly(self):
        specs = list(SERVO_CONFIG['NX02'].values()) + list(SERVO_CONFIG['NX01'].values())
        for spec in specs:
            for pwm in range(spec.pwm_min, spec.pwm_max + 1):
                v = servo_pwm_to_normalized(spec, pwm)
                self.assertTrue(-1.0 <= v <= 1.0)
                self.assertEqual(_px4_output(spec, v), pwm)

    def test_endpoints(self):
        spec = SERVO_CONFIG['NX02'][1]          # 爪子：min=抓紧、max=打开（10-15 起）
        self.assertEqual(servo_pwm_to_normalized(spec, spec.pwm_min), -1.0)
        self.assertEqual(servo_pwm_to_normalized(spec, spec.pwm_max), 1.0)
        self.assertEqual((spec.pwm_min, spec.pwm_max), (1000, 1950))

    def test_out_of_range_rejected(self):
        spec = SERVO_CONFIG['NX02'][1]
        for pwm in (spec.pwm_min - 1, spec.pwm_max + 1, 0, 800, 900, 2100, 2200):
            with self.assertRaises(ValueError):
                # 800 是 09-21 旧接线的抓紧值、2200 是 10-15 收窄行程前的打开位、
                # 900/2100 是 10-18 之前这一路的两端（也是 NX01 现在的两端），
                # 现在都越界——照搬上一版或另一架的常量会直接抛 ValueError，
                # 不会静默夹错。
                servo_pwm_to_normalized(spec, pwm)


if __name__ == '__main__':
    unittest.main()

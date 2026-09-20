#!/usr/bin/env python3
"""四电机相对偏差自动比对脚本——喂一份.ulg飞控日志，输出电机健康度体检报告。

背景：2026-09-02晚间连续炸机事故复盘（见docker_sim/DEBUG_JOURNAL.md同日期
条目、`fcu/2026-09-02晚间测试_连续炸机事故复盘.md`）定位到2号电机反复
推力异常，其中一次成功飞行（`202609022003_54.ulg`）里其实已经能看到早期
征兆，但当时是靠人工写脚本一段段翻数据才发现的。这个脚本把那套排查方法
固化下来，做成每次试飞后都能跑一遍的标准自检工具，不用等炸机了再回头
翻日志。

检测的四类信号（对应复盘文档里用过的方法）：
  1. 起转卡死——解锁后某个电机迟迟不跟其余电机一起爬油门，卡在怠速附近
  2. 骤降/骤升——飞行中某个电机输出在很短时间内跳变（经典的失控触发信号）
  3. 持续性相对偏差——按时间窗统计每个电机跟其余三个均值的偏差，超阈值
     的窗口会额外核对同一时段的摇杆输入，摇杆没动的偏差可信度更高
  4. 电池异常——电压骤降/电流骤增，以及不合理的"remaining"回升
  5. 姿态异常——roll/pitch偏离正常悬停基线的窗口

只做检测和报告，不判断"是不是一定要立刻停飞"这种决策——报告里列出的
只是"哪些证据、发生在什么时候"，人工再结合实际情况判断。

用法：
    python3 motor_health_check.py <flight.ulg> [--motors 4] [--window 5]
        [--dev-threshold 40] [--collapse-threshold 300]

依赖：pyulog（跟docker_sim其它ulg分析用的是同一个库，`pip install pyulog`
或`pip install --user pyulog`）。
"""
import argparse
import math
import sys
from bisect import bisect_left


def load_topic(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d.data
    return None


def quat_to_euler(q0, q1, q2, q3):
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1)))
    pitch = math.degrees(math.asin(sinp))
    return roll, pitch


def nearest_value(times, values, t):
    """给定按时间排好序的(times, values)，取离t最近的一个值。摇杆/电池这些
    topic的采样率跟actuator_outputs不一样，报告里核对某一时刻的状态时靠
    这个函数对齐，不强求同一帧。"""
    if not times:
        return None
    i = bisect_left(times, t)
    if i <= 0:
        return values[0]
    if i >= len(times):
        return values[-1]
    before, after = times[i - 1], times[i]
    return values[i - 1] if (t - before) <= (after - t) else values[i]


def fmt_t(t):
    return f"{t:7.2f}s"


def analyze(path, n_motors, window_s, dev_threshold, collapse_threshold,
            stuck_duration, stuck_ramp_threshold, stick_threshold):
    from pyulog import ULog

    ulog = ULog(path)
    t0 = ulog.start_timestamp

    def rel(t):
        return (t - t0) / 1e6

    outputs = load_topic(ulog, "actuator_outputs", 0)
    armed = load_topic(ulog, "actuator_armed", 0)
    attitude = load_topic(ulog, "vehicle_attitude", 0)
    manual = load_topic(ulog, "manual_control_setpoint", 0)
    battery = load_topic(ulog, "battery_status", 0)

    if outputs is None:
        print("!! 这份日志里没有actuator_outputs，无法做电机分析。")
        return 1

    out_t = [rel(t) for t in outputs["timestamp"]]
    out_cols = [outputs[f"output[{i}]"] for i in range(n_motors)]

    # armed区间：只在解锁状态下的数据才有意义，地面/上锁时的0值会干扰统计
    armed_intervals = []
    if armed is not None:
        at = [rel(t) for t in armed["timestamp"]]
        av = armed["armed"]
        start = None
        for t, v in zip(at, av):
            if v and start is None:
                start = t
            elif not v and start is not None:
                armed_intervals.append((start, t))
                start = None
        if start is not None:
            armed_intervals.append((start, out_t[-1]))

    def is_armed(t):
        if not armed_intervals:
            return True
        for a, b in armed_intervals:
            if a <= t <= b:
                return True
        return False

    # 只保留解锁期间、且至少有一路电机真的在转的样本
    rows = []
    for i, t in enumerate(out_t):
        if not is_armed(t):
            continue
        vals = [out_cols[m][i] for m in range(n_motors)]
        if max(vals) <= 0:
            continue
        rows.append((t, vals))

    if len(rows) < 10:
        print("!! 解锁期间的电机输出样本太少，跳过分析（可能是纯地面测试或数据不全）。")
        return 1

    all_vals = [v for _, vals in rows for v in vals]
    idle_floor = min(all_vals)
    sat_ceiling = max(all_vals)
    span = max(sat_ceiling - idle_floor, 1.0)
    idle_margin = span * 0.08
    print(f"电机输出观测范围: {idle_floor:.0f} ~ {sat_ceiling:.0f}"
          f"（怠速判定阈值 <{idle_floor + idle_margin:.0f}）")

    manual_t = [rel(t) for t in manual["timestamp"]] if manual else []
    manual_roll = manual.get("roll") if manual else None
    manual_pitch = manual.get("pitch") if manual else None

    def stick_active_near(t):
        if not manual_t:
            return None  # 没有摇杆数据（比如纯OFFBOARD自主飞行），无法判断
        r = nearest_value(manual_t, manual_roll, t)
        p = nearest_value(manual_t, manual_pitch, t)
        if r is None or p is None:
            return None
        return abs(r) > stick_threshold or abs(p) > stick_threshold

    findings = {m: [] for m in range(n_motors)}

    # ---- 1. 起转卡死检测：解锁后第一段时间里，是否有电机贴着怠速、明显
    # 跟不上其余电机——不要求"其余电机已经爬升多少"（起转阶段本身可能
    # 还在低速爬升，涨幅不大），只看"这一路是不是贴底 + 跟其余均值差距
    # 够大"，条件更宽松也更贴近实测数据（18_27_49那次前2.8秒其余电机
    # 本身也只在190-250附近，没有大幅爬升，但2号电机一直卡在109不动，
    # 跟其余均值差了近百个单位，这才是真正的信号）----
    arm_start = armed_intervals[0][0] if armed_intervals else rows[0][0]
    spool_rows = [(t, v) for t, v in rows if t - arm_start <= max(stuck_duration * 3, 6.0)]
    # 阈值用dev_threshold（相对偏差判定用的同一把尺子）而不是idle_margin——
    # idle_margin是给"是否贴底"这个独立判断用的，量级不一样，混用会导致
    # 阈值虚高、漏检（实测踩过这个坑：18_27_49那次2号电机卡在109、其余
    # 均值只比它高80-90个单位，如果拿idle_margin(151)当gap阈值就会漏判）。
    stuck_gap_threshold = dev_threshold
    for m in range(n_motors):
        stuck_start = None
        for t, vals in spool_rows:
            others_avg = sum(vals[k] for k in range(n_motors) if k != m) / (n_motors - 1)
            is_idle = vals[m] < idle_floor + idle_margin
            gap = others_avg - vals[m]
            if is_idle and gap > stuck_gap_threshold:
                if stuck_start is None:
                    stuck_start = t
            elif stuck_start is not None:
                dur = t - stuck_start
                if dur >= stuck_duration:
                    findings[m].append(
                        f"[起转卡死] {fmt_t(stuck_start)}~{fmt_t(t)}"
                        f"（持续{dur:.1f}s）：贴着怠速不动，同期其余电机均值比它高{gap:.0f}+个单位"
                    )
                stuck_start = None
        if stuck_start is not None and spool_rows:
            dur = spool_rows[-1][0] - stuck_start
            if dur >= stuck_duration:
                findings[m].append(
                    f"[起转卡死] {fmt_t(stuck_start)}~{fmt_t(spool_rows[-1][0])}"
                    f"（持续{dur:.1f}s，直到本段数据结束）：贴着怠速不动，跟不上其余电机"
                )

    # ---- 2. 骤降/骤升检测：短时间内跟自身近期基线的落差超过阈值。
    # 一旦进入"多个电机同时剧烈跳变"的全面失控阶段（姿态控制器在饱和边界
    # 疯狂找补，四个电机会互相牵连着一起跳），这些后续的骤升骤降是失控的
    # "结果"而不是"原因"，逐条列出只会淹没掉真正有诊断价值的第一次异常，
    # 所以检测到"全面失控"之后不再逐条报，只留一句汇总。----
    lookback = 0.4  # 秒，跟自己lookback秒之前的值比较，抓"短时间内跳变"
    collapse_events = []  # (t, motor, delta)
    for m in range(n_motors):
        j = 0
        for i, (t, vals) in enumerate(rows):
            while j < i and t - rows[j][0] > lookback:
                j += 1
            prev = rows[j][1][m]
            delta = vals[m] - prev
            if abs(delta) >= collapse_threshold:
                collapse_events.append((t, m, delta))
    collapse_events.sort(key=lambda e: e[0])

    chaos_onset = None
    chaos_window = 1.0
    for i, (t, m, delta) in enumerate(collapse_events):
        involved = {m}
        for t2, m2, _ in collapse_events[i:]:
            if t2 - t > chaos_window:
                break
            involved.add(m2)
        if len(involved) >= max(3, n_motors - 1):
            chaos_onset = t
            break

    for t, m, delta in collapse_events:
        if chaos_onset is not None and t >= chaos_onset:
            continue
        direction = "骤降" if delta < 0 else "骤升"
        findings[m].append(
            f"[{direction}] {fmt_t(t)}：{lookback:.1f}秒内落差{delta:+.0f}"
        )
    if chaos_onset is not None:
        n_after = sum(1 for t, m, d in collapse_events if t >= chaos_onset)
        print(f"\n注意：{fmt_t(chaos_onset)}起判定为全面失控阶段"
              f"（短时间内至少{max(3, n_motors-1)}个电机同时剧烈跳变）——"
              f"此后{n_after}条骤降/骤升不再逐条列出，多半是控制器在饱和边界反复找补的连锁反应，"
              "不代表这些电机本身独立出问题，诊断请优先看这个时间点之前的证据。")

    # ---- 3. 按时间窗统计相对偏差，核对同期摇杆输入 ----
    win_start = rows[0][0]
    win_rows = []
    for t, vals in rows:
        if t - win_start >= window_s:
            if win_rows:
                _report_window(win_start, win_rows, n_motors, dev_threshold,
                                stick_active_near, findings)
            win_start = t
            win_rows = []
        win_rows.append((t, vals))
    if win_rows:
        _report_window(win_start, win_rows, n_motors, dev_threshold,
                        stick_active_near, findings)

    # ---- 4. 电池异常 ----
    battery_findings = []
    if battery is not None:
        bt = [rel(t) for t in battery["timestamp"]]
        bv = battery["voltage_v"]
        brem = battery.get("remaining")
        j = 0
        for i in range(len(bt)):
            while j < i and bt[i] - bt[j] > 1.0:
                j += 1
            if bv[j] - bv[i] >= 3.0:
                battery_findings.append(
                    f"[电压骤降] {fmt_t(bt[i])}：1秒内从{bv[j]:.1f}V掉到{bv[i]:.1f}V"
                )
        if brem is not None:
            for i in range(1, len(bt)):
                if brem[i] - brem[i - 1] > 0.03:
                    battery_findings.append(
                        f"[remaining异常回升] {fmt_t(bt[i])}：从{brem[i-1]:.3f}升到{brem[i]:.3f}"
                        "（正常放电不应该变大，多半是遥测/连接毛刺）"
                    )

    # ---- 5. 姿态异常 ----
    attitude_findings = []
    if attitude is not None:
        at = [rel(t) for t in attitude["timestamp"]]
        rolls, pitches = [], []
        for q0, q1, q2, q3 in zip(attitude["q[0]"], attitude["q[1]"],
                                   attitude["q[2]"], attitude["q[3]"]):
            r, p = quat_to_euler(q0, q1, q2, q3)
            rolls.append(r)
            pitches.append(p)
        armed_att = [(t, r, p) for t, r, p in zip(at, rolls, pitches) if is_armed(t)]
        if armed_att:
            baseline = sorted(max(abs(r), abs(p)) for _, r, p in armed_att)
            baseline_p50 = baseline[len(baseline) // 2]
            att_threshold = max(15.0, baseline_p50 * 4)
            in_excursion = False
            exc_start = None
            exc_max = 0
            for t, r, p in armed_att:
                dev = max(abs(r), abs(p))
                if dev > att_threshold:
                    if not in_excursion:
                        in_excursion = True
                        exc_start = t
                        exc_max = dev
                    exc_max = max(exc_max, dev)
                elif in_excursion:
                    attitude_findings.append(
                        f"[姿态偏离] {fmt_t(exc_start)}~{fmt_t(t)}：最大倾角{exc_max:.1f}°"
                        f"（正常基线约{baseline_p50:.1f}°）"
                    )
                    in_excursion = False
            if in_excursion:
                attitude_findings.append(
                    f"[姿态偏离] {fmt_t(exc_start)}~结束：最大倾角{exc_max:.1f}°"
                    f"（正常基线约{baseline_p50:.1f}°，直到日志结束都没恢复）"
                )

    # ---- 汇总输出 ----
    print()
    print("=" * 60)
    print("电机健康度体检报告")
    print("=" * 60)
    any_flag = False
    for m in range(n_motors):
        if not findings[m]:
            continue
        any_flag = True
        print(f"\n>> {m + 1}号电机（output[{m}]）：{len(findings[m])}条异常")
        for line in findings[m]:
            print(f"   {line}")
    if not any_flag:
        print("\n四路电机输出全程未见明显异常。")

    if battery_findings:
        print(f"\n>> 电池：{len(battery_findings)}条异常")
        for line in battery_findings:
            print(f"   {line}")

    if attitude_findings:
        print(f"\n>> 姿态：{len(attitude_findings)}条异常")
        for line in attitude_findings:
            print(f"   {line}")

    print()
    print("=" * 60)
    # 结论看"谁最先出问题"，不是"谁异常条数最多"——一旦真正失控，四个
    # 电机会互相牵连着一起剧烈跳变，条数最多的往往是"被拖下水"的那个，
    # 不是根因（复盘文档里3号电机被打满补偿、但根因是2号先失效，就是
    # 这个道理）。这里只用"起转卡死"和"骤降"（失去推力方向）参与排名，
    # "骤升"更可能是补偿反应，不计入。
    # 直接从原始事件列表里取"起转卡死"起始时间 + 骤降事件时间，二者取并集
    first_bad = {}
    for t, m, delta in collapse_events:
        if delta < 0 and (chaos_onset is None or t < chaos_onset):
            first_bad.setdefault(m, t)
            first_bad[m] = min(first_bad[m], t)
    for m in range(n_motors):
        for t, vals in spool_rows:
            others_avg = sum(vals[k] for k in range(n_motors) if k != m) / (n_motors - 1)
            if vals[m] < idle_floor + idle_margin and others_avg - vals[m] > stuck_gap_threshold:
                first_bad[m] = min(first_bad.get(m, t), t)
                break

    if first_bad:
        worst = min(first_bad, key=first_bad.get)
        print(f"结论：{worst + 1}号电机（output[{worst}]）最先出现异常"
              f"（{fmt_t(first_bad[worst])}），建议优先排查这一路的电调/接线/电机本体。"
              "其它电机后续出现的骤升/骤降大概率是控制器为纠正它而产生的连锁反应。")
    else:
        print("结论：本次飞行未见电机层面的明显异常（起转卡死/骤降）。")
    print("（这份报告只做异常检测，不代表最终诊断，请结合实际情况人工复核。）")
    return 0


def _report_window(win_start, win_rows, n_motors, dev_threshold, stick_active_near, findings):
    win_end = win_rows[-1][0]
    means = []
    for m in range(n_motors):
        vals = [v[m] for _, v in win_rows]
        means.append(sum(vals) / len(vals))
    for m in range(n_motors):
        others = [means[k] for k in range(n_motors) if k != m]
        other_avg = sum(others) / len(others)
        dev = means[m] - other_avg
        if abs(dev) >= dev_threshold:
            mid_t = (win_start + win_end) / 2
            active = stick_active_near(mid_t)
            if active is None:
                conf = "（无摇杆数据，多半是纯自主飞行，无法用摇杆排除机动动作）"
            elif active:
                conf = "（同期有摇杆输入，不能完全排除是正常机动动作）"
            else:
                conf = "（同期摇杆无输入，可信度较高）"
            direction = "偏低" if dev < 0 else "偏高"
            findings[m].append(
                f"[相对偏差] {fmt_t(win_start)}~{fmt_t(win_end)}：比其余电机均值{direction}"
                f"{abs(dev):.0f}个单位 {conf}"
            )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulg", help="飞控日志文件路径(.ulg)")
    ap.add_argument("--motors", type=int, default=4, help="电机数量，默认4（四旋翼）")
    ap.add_argument("--window", type=float, default=5.0, help="相对偏差统计的时间窗(秒)，默认5")
    ap.add_argument("--dev-threshold", type=float, default=40.0,
                     help="判定'相对偏差'异常的阈值(PWM/DShot单位)，默认40")
    ap.add_argument("--collapse-threshold", type=float, default=300.0,
                     help="判定'骤降/骤升'的阈值(0.4秒内的落差)，默认300")
    ap.add_argument("--stuck-duration", type=float, default=1.5,
                     help="判定'起转卡死'需要持续的最短时间(秒)，默认1.5")
    ap.add_argument("--stuck-ramp-threshold", type=float, default=200.0,
                     help="判定其余电机'已经在爬升'的最小涨幅，默认200")
    ap.add_argument("--stick-threshold", type=float, default=0.1,
                     help="判定摇杆'有输入'的归一化幅值阈值，默认0.1")
    args = ap.parse_args()

    try:
        return analyze(args.ulg, args.motors, args.window, args.dev_threshold,
                        args.collapse_threshold, args.stuck_duration,
                        args.stuck_ramp_threshold, args.stick_threshold)
    except ImportError:
        print("!! 需要pyulog：pip install --user pyulog", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

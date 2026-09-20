#!/usr/bin/env python3
"""把 results/*.json 汇总成一份可直接交付的Markdown实验报告。

用法： python3 l3_report.py [--results results] [--out 真机侧L3实验报告.md]

报告结构固定：实验条件 -> 每个实验(H1~H6)的结果表 -> 与仿真侧参考值的对照
-> 判据核对 -> 结论与遗留。判据阈值集中写在下面的 CRITERIA 里，改判据只
改这一处。
"""
import argparse
import glob
import json
import os
from datetime import datetime

# 判据（依据见《实验方案_SE2在线标定真机侧L3.md》第6节）
CRITERIA = {
    'H1_sigma_max_m': 0.10,        # 静态水平噪声σ上限；超了要重新审视论文表1的0.05 m
    'H2_ratio_lo': 0.5,            # 实测σ_yaw / 式(21)预测，落在[0.5,2.0]算相符
    'H2_ratio_hi': 2.0,
    'H3_rmse_max_deg': 3.0,        # θ*收敛后RMSE上限
    'H3_truth_se_ratio': 0.5,      # 真值标准误必须小于RMSE的这个比例，否则测的是真值噪声
    'H5_improve_min': 1.5,         # 抗差开启后RMSE至少要降到原来的1/1.5
    'H6_segment_us_max': 2000.0,   # 单次"切段"更新耗时上限[µs]
}

# 仿真侧已完成的对照值（来自论文第4章，用于报告里的横向对比）
SIM_REF = {
    'H1_sigma_m': 0.05,
    'H2_sigma_yaw_pred_deg': 13.9,
    'H3_rmse_deg': '0.74 ± 0.13 (NX01) / 0.67 ± 0.20 (NX02)',
    'H4_batch_vs_sliding': '短时飞行下二者相同（均0.90°）',
    'H5_robust_gain': '同一份数据上RMSE降约3倍（1.83→0.59 / 2.47→0.79）',
    'H6_timing_us': 'x86 i9：未切段0.44 / 切段0.89（不抗差）；0.67 / 40.88（抗差）',
}


def load(results_dir):
    out = {}
    for p in sorted(glob.glob(os.path.join(results_dir, '*.json'))):
        name = os.path.basename(p)[:-5]
        try:
            out[name] = json.load(open(p))
        except Exception as e:
            print(f'跳过{p}: {e}')
    return out


def _fmt(v, n=3):
    if isinstance(v, float):
        return f'{v:.{n}f}'
    return str(v)


def build(res, out_path):
    L = []
    A = L.append
    A('# 局部-全局SE(2)在线标定方法 真机侧(L3)实验报告\n')
    A(f'生成时间：{datetime.now().strftime("%Y-%m-%d %H:%M")}　　'
      f'结果文件：{len(res)}个\n')
    A('本报告由 `l3_report.py` 从 `results/*.json` 自动汇总，对应实验方案文档'
      '《实验方案_SE2在线标定真机侧L3.md》。所有原始数据为rosbag，'
      '分析脚本为 `l3_analyze.py`。\n')

    checks = []

    # ---------------- H1 ----------------
    h1 = {k: v for k, v in res.items() if k.startswith('H1_')}
    A('\n## H1 静态观测噪声与野值率\n')
    if not h1:
        A('*（无数据）*\n')
    else:
        A('| 架次 | 话题 | 时长[s] | 实测发布率[Hz] | σx[m] | σy[m] | σz[m] | 水平σ[m] | 最大水平偏差[m] | 野值率(x/y) |')
        A('|---|---|---|---|---|---|---|---|---|---|')
        for k, v in sorted(h1.items()):
            A(f"| {v.get('tag')} | {v.get('topic')} | {_fmt(v.get('duration_s'),1)} | "
              f"{_fmt(v.get('rate_hz'),1)} | {_fmt(v['x']['std'],4)} | {_fmt(v['y']['std'],4)} | "
              f"{_fmt(v['z']['std'],4)} | {_fmt(v.get('sigma_xy'),4)} | "
              f"{_fmt(max(v['x']['max_abs_err'], v['y']['max_abs_err']),3)} | "
              f"{v['x']['outlier_ratio']:.1%} / {v['y']['outlier_ratio']:.1%} |")
            checks.append(('H1 水平σ ≤ %.2f m' % CRITERIA['H1_sigma_max_m'],
                           v.get('sigma_xy', 9) <= CRITERIA['H1_sigma_max_m'],
                           f"{v.get('tag')}: {_fmt(v.get('sigma_xy'),4)} m"))
        A(f"\n论文表1采用的σ为 {SIM_REF['H1_sigma_m']} m，仿真按该值注入噪声；"
          '本表给出真机实测值，若二者差异显著，论文表1的"依据"一列应改引本报告。\n')

    # ---------------- H2 ----------------
    h2 = {k: v for k, v in res.items() if k.startswith('H2_')}
    A('\n## H2 双标签测向精度与式(21)的基线标度律\n')
    if not h2:
        A('*（无数据）*\n')
    else:
        A('| 架次 | 基线L[m] | 样本数 | yaw均值[°] | 实测σ_yaw[°] | 式(21)预测[°] | 实测/预测 | σ_yaw×L[°·m] |')
        A('|---|---|---|---|---|---|---|---|')
        for k, v in sorted(h2.items()):
            r = v.get('ratio_meas_over_pred', 0)
            A(f"| {v.get('tag')} | {_fmt(v.get('baseline_used_m'))} | {v.get('n')} | "
              f"{_fmt(v.get('yaw_mean_deg'),2)} | {_fmt(v.get('yaw_std_deg'),2)} | "
              f"{_fmt(v.get('sigma_yaw_pred_deg'),2)} | {_fmt(r,2)} | "
              f"{_fmt(v.get('sigma_yaw_times_L'),3)} |")
            checks.append(('H2 实测/预测 ∈ [%.1f, %.1f]' % (CRITERIA['H2_ratio_lo'], CRITERIA['H2_ratio_hi']),
                           CRITERIA['H2_ratio_lo'] <= r <= CRITERIA['H2_ratio_hi'],
                           f"{v.get('tag')}: {_fmt(r,2)}"))
        vals = [v.get('sigma_yaw_times_L') for v in h2.values() if v.get('sigma_yaw_times_L')]
        if len(vals) >= 2:
            spread = (max(vals) - min(vals)) / (sum(vals) / len(vals))
            A(f"\n**基线标度律检验**：不同基线下 σ_yaw×L 的相对离散度为 {spread:.1%}"
              f"（理论上应当是常数 √2σ，与L无关）。这是比对单点值更强的验证。\n")
        A(f"论文式(21)在L=0.29 m、σ=0.05 m下的预测值为 {SIM_REF['H2_sigma_yaw_pred_deg']}°。\n")

    # ---------------- H3 ----------------
    h3 = {k: v for k, v in res.items() if k.startswith('H3_')}
    A('\n## H3 θ*在线标定精度（真机）\n')
    if not h3:
        A('*（无数据）*\n')
    else:
        A('| 架次 | 段数 | 累计位移[m] | 时长[s] | θ*末值[°] | 真值末值[°] | 误差均值[°] | 误差std[°] | RMSE[°] | 收敛段数 | 真值标准误[°] |')
        A('|---|---|---|---|---|---|---|---|---|---|---|')
        for k, v in sorted(h3.items()):
            A(f"| {v.get('tag')} | {v.get('n_segments_final')} | {_fmt(v.get('path_len_m_est'),0)} | "
              f"{_fmt(v.get('duration_s'),0)} | {_fmt(v.get('theta_final_deg'),2)} | "
              f"{_fmt(v.get('theta_true_final_deg'),2)} | {_fmt(v.get('err_mean_deg'),2)} | "
              f"{_fmt(v.get('err_std_deg'),2)} | {_fmt(v.get('err_rmse_deg'),2)} | "
              f"{v.get('n_segments_at_convergence')} | {_fmt(v.get('truth_standard_error_deg'),2)} |")
            rmse = v.get('err_rmse_deg', 99)
            checks.append(('H3 RMSE ≤ %.1f°' % CRITERIA['H3_rmse_max_deg'],
                           rmse <= CRITERIA['H3_rmse_max_deg'],
                           f"{v.get('tag')}: {_fmt(rmse,2)}°"))
            se = v.get('truth_standard_error_deg', 99)
            checks.append(('H3 真值标准误 < RMSE×%.1f' % CRITERIA['H3_truth_se_ratio'],
                           se < rmse * CRITERIA['H3_truth_se_ratio'],
                           f"{v.get('tag')}: 真值{_fmt(se,2)}° vs RMSE {_fmt(rmse,2)}°"))
        vals = [v.get('err_rmse_deg') for v in h3.values() if v.get('err_rmse_deg')]
        if len(vals) >= 2:
            m = sum(vals) / len(vals)
            sd = (sum((x - m) ** 2 for x in vals) / len(vals)) ** 0.5
            A(f"\n**{len(vals)}次重复汇总：RMSE = {m:.2f} ± {sd:.2f}°**\n")
        A(f"仿真侧对照（论文4.3.1节）：{SIM_REF['H3_rmse_deg']}\n")

    # ---------------- H4 ----------------
    h4 = {k: v for k, v in res.items() if k.startswith('H4_')}
    A('\n## H4 参数敏感性 / 批量对比 / 时间配对（离线，复用H3数据）\n')
    for k, v in sorted(h4.items()):
        A(f"\n**{v.get('tag')}**\n")
        if v.get('sweep'):
            A('| d_min[m] | W | 段数 | RMSE[°] | 偏差[°] |')
            A('|---|---|---|---|---|')
            for r in v['sweep']:
                A(f"| {r['d_min']} | {r['W']} | {r['n_seg']} | {_fmt(r['rmse'],2)} | {_fmt(r['bias'],2)} |")
        b = v.get('batch') or {}
        if b.get('batch') and b.get('sliding'):
            A(f"\n批量 RMSE {_fmt(b['batch']['rmse'],2)}° vs 滑窗(W=50) "
              f"{_fmt(b['sliding']['rmse'],2)}°　（仿真侧结论：{SIM_REF['H4_batch_vs_sliding']}）\n")
        if v.get('pairing'):
            pr = {kk: (vv or {}).get('rmse') for kk, vv in v['pairing'].items()}
            A(f"时间配对：最近一帧 {_fmt(pr.get('latest'),2)}° vs 时间戳插值 {_fmt(pr.get('interp'),2)}°\n")
    if not h4:
        A('*（无数据）*\n')

    # ---------------- H5 ----------------
    h5 = {k: v for k, v in res.items() if k.startswith('H5_')}
    A('\n## H5 抗差扩展（同一份数据上的开/关对照）\n')
    if not h5:
        A('*（无数据）*\n')
    else:
        A('| 架次 | 人工注入 | 关抗差RMSE[°] | 开抗差RMSE[°] | 改善倍数 | 残差中位数[m] | 残差MAD[m] | 最大残差[m] | 野值段占比 |')
        A('|---|---|---|---|---|---|---|---|---|')
        for k, v in sorted(h5.items()):
            p, r = v['modes'].get('plain', {}), v['modes'].get('robust', {})
            q = p.get('residual', {})
            gain = (p.get('rmse', 0) / r['rmse']) if r.get('rmse') else 0
            inj = (f"{v['inject_prob']:.0%}×{v['inject_mag']}m" if v.get('inject_prob') else '无')
            A(f"| {v.get('tag')} | {inj} | {_fmt(p.get('rmse'),2)} | {_fmt(r.get('rmse'),2)} | "
              f"{_fmt(gain,2)} | {_fmt(q.get('median'),3)} | {_fmt(q.get('mad'),3)} | "
              f"{_fmt(q.get('max'),3)} | {q.get('outlier_seg_ratio', 0):.1%} |")
            if v.get('inject_prob'):
                checks.append(('H5 注入野值下抗差改善 ≥ %.1f倍' % CRITERIA['H5_improve_min'],
                               gain >= CRITERIA['H5_improve_min'], f"{v.get('tag')}: {_fmt(gain,2)}倍"))
        A(f"\n仿真侧对照：{SIM_REF['H5_robust_gain']}。残差统计同时是现场判据——"
          '干净环境下野值段占比应接近0%、最大残差与噪声同量级；占比明显升高说明该开抗差。\n')

    # ---------------- H6 ----------------
    h6 = {k: v for k, v in res.items() if k.startswith('H6_')}
    A('\n## H6 机载计算开销\n')
    if not h6:
        A('*（无数据）*\n')
    else:
        A('| 平台 | 抗差 | 未切段中位数[µs] | 未切段p95[µs] | 切段中位数[µs] | 切段p95[µs] | 切段最大[µs] |')
        A('|---|---|---|---|---|---|---|')
        for k, v in sorted(h6.items()):
            for mode, label in (('plain', '关'), ('robust', '开')):
                m = v['modes'].get(mode, {})
                ns, cs = m.get('no_segment', {}), m.get('segment_cut', {})
                A(f"| {v.get('platform')} | {label} | {_fmt(ns.get('median_us'),2)} | "
                  f"{_fmt(ns.get('p95_us'),2)} | {_fmt(cs.get('median_us'),2)} | "
                  f"{_fmt(cs.get('p95_us'),2)} | {_fmt(cs.get('max_us'),2)} |")
                if cs.get('median_us'):
                    checks.append(('H6 切段耗时 ≤ %.0f µs' % CRITERIA['H6_segment_us_max'],
                                   cs['median_us'] <= CRITERIA['H6_segment_us_max'],
                                   f"{label}抗差: {_fmt(cs['median_us'],2)} µs"))
        A(f"\n仿真侧对照：{SIM_REF['H6_timing_us']}。\n")

    # ---------------- 判据 ----------------
    A('\n## 判据核对\n')
    A('| 判据 | 结果 | 实测 |')
    A('|---|---|---|')
    for name, ok, detail in checks:
        A(f"| {name} | {'✅通过' if ok else '❌未通过'} | {detail} |")
    n_fail = sum(1 for _, ok, _ in checks if not ok)
    A(f"\n共{len(checks)}条判据，未通过{n_fail}条。\n")

    A('\n## 结论与遗留\n')
    A('（按实际结果填写，建议覆盖以下几点）\n')
    A('1. 真机实测的σ与论文表1取值是否一致，是否需要改论文；\n')
    A('2. 式(21)的基线标度律在真机上是否成立；\n')
    A('3. θ*精度与仿真结果的差距及其解释（真实UWB发布率/时延/多径与仿真设置的差异）；\n')
    A('4. 抗差扩展在真实多径环境下是否必要（看残差统计与野值段占比）；\n')
    A('5. 机载计算开销相对x86的倍数，是否仍可忽略；\n')
    A('6. 本轮未覆盖的项（如仅一台真机，机间链式复合未验证）。\n')

    txt = '\n'.join(L)
    with open(out_path, 'w') as f:
        f.write(txt)
    print(f'报告已生成：{out_path}（{len(txt)}字符，{len(checks)}条判据，{n_fail}条未通过）')
    return txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default='results')
    ap.add_argument('--out', default='真机侧L3实验报告.md')
    a = ap.parse_args()
    res = load(a.results)
    if not res:
        print(f'!! {a.results} 下没有找到任何结果JSON')
        return 1
    build(res, a.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

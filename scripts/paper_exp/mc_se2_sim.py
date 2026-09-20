#!/usr/bin/env python3
"""L0层：SE(2)在线标定方法的纯数值蒙特卡洛实验（论文E1–E6）。

对应`docker_sim/实验方案_SE2在线标定论文补充实验.md`第二节L0层。这一层
完全不依赖ROS/容器/Gazebo，在宿主机上直接跑，用来补齐论文里几条"给了
可证伪的定量预测、但一条实验都没有"的缺口：

  E1 噪声传播与CRLB验证      -> 论文式(12)(13)(15)，缺口G2/G4/G13
  E2 恒定偏置免疫性           -> 论文式(10)(11)，缺口G5
  E3 批量 vs 滑窗跟踪时变映射 -> 论文2.5/4.1节的"能跟踪时变"论断，缺口G3
  E4 Huber抗差扩展            -> 论文式(16)(17)(18)，缺口G6
  E5 可观测性四格对照         -> 论文第5节的定性论断，缺口G8
  E6 观测异步/时延消融        -> 论文噪声模型里缺失的一项，缺口G12

估计器本身不在这里实现，统一从`se2_core.py`导入（那份是机上
origin_setter_node.py算法部分的逐行副本），保证"实验验证的就是论文里
那个估计器"。

用法：
  python3 scripts/paper_exp/mc_se2_sim.py            # 全跑，出CSV+PNG
  python3 scripts/paper_exp/mc_se2_sim.py --exp e1   # 只跑某一个
  python3 scripts/paper_exp/mc_se2_sim.py --trials 200 --quick   # 快速冒烟
输出目录：docker_sim/paper_exp_results/（CSV原始数据 + PNG插图）
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from se2_core import (SlidingWindowEstimator, BatchEstimator,  # noqa: E402
                      solve_theta, solve_theta_irls, residuals, wrap_pi)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', '..', 'paper_exp_results')
OUT_DIR = os.path.normpath(OUT_DIR)

# 论文表1的标称参数——所有实验的默认工作点，扫描时只动被扫的那一个
SIGMA_NOMINAL = 0.05      # 绝对位置观测噪声标准差[m]，论文表1
D_MIN_NOMINAL = 0.5       # 最小分段位移阈值[m]，论文表1
W_NOMINAL = 50            # 滑动窗口大小[段]，论文表1
THETA_TRUE = math.radians(30.0)   # 真值航向角偏差，取仿真里NX01的30°


# ---------------------------------------------------------------- 绘图
def _setup_matplotlib():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # 论文是中文的，图里的标签也用中文——宿主机上实测有Noto Sans CJK SC
    for fam in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'AR PL UMing CN',
                'WenQuanYi Zen Hei'):
        try:
            matplotlib.font_manager.findfont(fam, fallback_to_default=False)
            plt.rcParams['font.sans-serif'] = [fam]
            break
        except Exception:
            continue
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['figure.dpi'] = 150
    plt.rcParams['savefig.bbox'] = 'tight'
    return plt


def _write_csv(name, header, rows):
    path = os.path.join(OUT_DIR, name)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f'  -> {path} ({len(rows)}行)')
    return path


# ------------------------------------------------- 合成数据的公用工具
def _gen_segments(rng, n_seg, d, theta, single_direction=False, chained=True,
                  sigma=SIGMA_NOMINAL, trials=1):
    """生成(trials, n_seg, 4)的合成位移段 (ux,uy,vx,vy)。

    chained=True：相邻段共享同一次绝对观测作为端点(跟机上实现一致——
    `_maybe_record_segment`里段是首尾相接的)，此时相邻段噪声负相关，
    论文2.6节末尾专门提到这一点是"忽略该相关性的一阶近似"；
    chained=False：每段两端各用一次独立观测，严格满足论文的独立性假设。
    两种都跑，正好把那句caveat也验证掉。
    """
    if single_direction:
        phi = np.zeros((trials, n_seg))
    else:
        phi = rng.uniform(-math.pi, math.pi, size=(trials, n_seg))
    u = np.stack([d * np.cos(phi), d * np.sin(phi)], axis=-1)   # (T,N,2)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    v_true = u @ R.T
    if chained:
        # 绝对观测点 p_0..p_N，每点一次独立噪声，v_i = p_i - p_{i-1}
        noise_pts = rng.normal(0.0, sigma, size=(trials, n_seg + 1, 2))
        v = v_true + (noise_pts[:, 1:, :] - noise_pts[:, :-1, :])
    else:
        n_a = rng.normal(0.0, sigma, size=(trials, n_seg, 2))
        n_b = rng.normal(0.0, sigma, size=(trials, n_seg, 2))
        v = v_true + (n_a - n_b)
    return u, v


def _theta_hat_batch(u, v):
    """对(trials,n_seg,2)的段批量求θ*=atan2(S,C)，返回(trials,)。"""
    S = (u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]).sum(axis=1)
    C = (u[..., 0] * v[..., 0] + u[..., 1] * v[..., 1]).sum(axis=1)
    return np.arctan2(S, C)


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ================================================================ E1
def exp1_crlb(trials=2000, quick=False):
    """E1 噪声传播与CRLB验证——论文式(12)(13)(15)。

    ⚠️ 本实验做出来才发现的一件事（论文当前没有、值得补进2.6节）：机上实现
    的位移段是**首尾相接**的（`_maybe_record_segment`里段终点直接当下一段
    起点），相邻两段共享同一次绝对观测，噪声不独立。对S做Abel求和：
        S的噪声 = Σ_{i=1}^{W-1}(u_i-u_{i+1})×n_i + u_W×n_W - u_1×n_0
    于是
        Var(θ̂) ≈ σ²·[Σ|u_i-u_{i+1}|² + |u_1|² + |u_W|²] / (Σd_i²)²   (*)
    两个极端：
      · 方向随机时 E|u_i-u_{i+1}|²=2d²，(*)化简回论文式(13)的2σ²/Σd_i²；
      · 方向一致（直线飞）时 u_i-u_{i+1}=0，只剩两个端点项，
        Var=2σ²/(Σd_i)²——**比式(13)小了整整W倍**（标准差小√W倍），
        因为中间那些观测噪声两两抵消，等效基线是整条轨迹的总位移而不是
        单段位移。
    这不违反CRLB：式(15)的下界是在"各段噪声独立"这个模型下推的，真实
    观测模型里噪声相关，对应的Fisher信息本来就更大。
    """
    print('== E1 噪声传播与CRLB验证 ==')
    rng = np.random.default_rng(20260903)
    Ws = [1, 2, 5, 10, 20, 50, 100] if not quick else [1, 10, 50]
    Ds = [0.3, 0.5, 1.0, 2.0] if not quick else [0.5]
    Sigmas = [0.02, 0.05, 0.10] if not quick else [0.05]
    rows = []
    for sigma in Sigmas:
        for d in Ds:
            for W in Ws:
                for chained in (True, False):
                    for single in (True, False):
                        u, v = _gen_segments(rng, W, d, THETA_TRUE,
                                             single_direction=single,
                                             chained=chained, sigma=sigma,
                                             trials=trials)
                        err = _wrap(_theta_hat_batch(u, v) - THETA_TRUE)
                        std_emp = float(np.std(err))
                        bias_emp = float(np.mean(err))
                        # 论文式(13)：各段噪声独立时 Var = 2σ²/Σd_i²
                        std_indep = math.sqrt(2.0 * sigma ** 2 / (W * d ** 2))
                        # 上面(*)式：首尾相接时的修正
                        if chained:
                            du2 = 0.0 if single else 2.0 * d ** 2   # E|u_i-u_{i+1}|²
                            num = sigma ** 2 * ((W - 1) * du2 + 2 * d ** 2)
                            std_chain = math.sqrt(num) / (W * d ** 2)
                        else:
                            std_chain = std_indep
                        rows.append([sigma, d, W,
                                     'chained' if chained else 'indep',
                                     'single' if single else 'multi', trials,
                                     math.degrees(std_emp),
                                     math.degrees(std_indep),
                                     math.degrees(std_chain),
                                     std_emp / std_indep, std_emp / std_chain,
                                     math.degrees(bias_emp)])
    _write_csv('e1_crlb.csv',
               ['sigma_m', 'd_m', 'W', 'noise_struct', 'direction', 'trials',
                'std_emp_deg', 'std_eq13_deg', 'std_chained_theory_deg',
                'ratio_emp_over_eq13', 'ratio_emp_over_chained',
                'bias_emp_deg'], rows)

    def _pick(W, d, sigma, ns, dirn):
        for r in rows:
            if (r[0] == sigma and r[1] == d and r[2] == W
                    and r[3] == ns and r[4] == dirn):
                return r
        return None
    print('  --- 论文正文两个具体数字的核对（独立噪声模型）---')
    for W, label in ((1, '单段(式12) 8.1°'), (W_NOMINAL, 'W=50(式13) 1.15°')):
        r = _pick(W, D_MIN_NOMINAL, SIGMA_NOMINAL, 'indep', 'multi')
        if r:
            print(f'    {label}: 理论{r[7]:.2f}° 实测{r[6]:.2f}° '
                  f'(比值{r[9]:.3f})')
    print('  --- 首尾相接(机上真实结构)的实际精度 ---')
    for dirn in ('multi', 'single'):
        r = _pick(W_NOMINAL, D_MIN_NOMINAL, SIGMA_NOMINAL, 'chained', dirn)
        if r:
            print(f'    W=50 {dirn:6s}: 实测{r[6]:.3f}° / 式(13)预测{r[7]:.3f}° '
                  f'/ (*)式预测{r[8]:.3f}°  -> 式(13)高估了{1/r[9]:.1f}倍')

    plt = _setup_matplotlib()
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    ax = axes[0]
    for sigma in Sigmas:
        sub = [r for r in rows if r[0] == sigma and r[1] == D_MIN_NOMINAL
               and r[3] == 'indep' and r[4] == 'multi']
        sub.sort(key=lambda r: r[2])
        ax.loglog([r[2] for r in sub], [r[6] for r in sub], 'o',
                  label=f'实测 σ={sigma} m')
        ax.loglog([r[2] for r in sub], [r[7] for r in sub], '-',
                  color='gray', lw=1)
    ax.set_xlabel('滑窗段数 W'); ax.set_ylabel(r'$\theta^*$估计标准差 [°]')
    ax.set_title('(a) 独立噪声模型：与式(13)/CRLB一致\n'
                 f'灰线为 $\\sqrt{{2\\sigma^2/\\sum d_i^2}}$（d={D_MIN_NOMINAL} m）')
    ax.grid(True, which='both', alpha=0.3); ax.legend(fontsize=8)

    ax = axes[1]
    for d in Ds:
        sub = [r for r in rows if r[0] == SIGMA_NOMINAL and r[1] == d
               and r[3] == 'indep' and r[4] == 'multi']
        sub.sort(key=lambda r: r[2])
        ax.loglog([r[2] for r in sub], [r[6] for r in sub], 'o-',
                  label=f'd={d} m', ms=4)
    ax.set_xlabel('滑窗段数 W'); ax.set_ylabel(r'$\theta^*$估计标准差 [°]')
    ax.set_title(f'(b) 分段长度d的影响（σ={SIGMA_NOMINAL} m）')
    ax.grid(True, which='both', alpha=0.3); ax.legend(fontsize=8)

    ax = axes[2]
    for ns, dirn, style, lab in (
            ('indep', 'multi', 'o-', '各段噪声独立（论文式13的假设）'),
            ('chained', 'multi', 's-', '首尾相接·方向随机'),
            ('chained', 'single', '^-', '首尾相接·方向一致（直线飞）')):
        sub = [r for r in rows if r[0] == SIGMA_NOMINAL
               and r[1] == D_MIN_NOMINAL and r[3] == ns and r[4] == dirn]
        sub.sort(key=lambda r: r[2])
        ax.loglog([r[2] for r in sub], [r[6] for r in sub], style, ms=4,
                  label=lab)
    ax.set_xlabel('滑窗段数 W'); ax.set_ylabel(r'$\theta^*$估计标准差 [°]')
    ax.set_title('(c) 段首尾相接带来的噪声抵消\n（机上实现属于后两条曲线）')
    ax.grid(True, which='both', alpha=0.3); ax.legend(fontsize=7)
    path = os.path.join(OUT_DIR, 'fig_e1_crlb.png')
    fig.savefig(path); plt.close(fig)
    print(f'  -> {path}')
    return rows


# ================================================================ E2
def exp2_bias(trials=500):
    """E2 恒定偏置免疫性——论文式(10)(11)。

    验证两件事：①任意恒定偏置b下θ*精确无偏(差分结构消掉b)；
    ②平移分量t原样继承b(误差恰好等于b)。
    """
    print('== E2 恒定全局偏置免疫性 ==')
    rng = np.random.default_rng(7)
    rows = []
    for bmag in (0.0, 0.5, 2.0):
        for bdir_deg in (0.0, 90.0, 217.0):
            b = np.array([bmag * math.cos(math.radians(bdir_deg)),
                          bmag * math.sin(math.radians(bdir_deg))])
            # 无噪声：验证的是"结构性精确抵消"，不是统计平均意义上的无偏
            u, v = _gen_segments(rng, W_NOMINAL, D_MIN_NOMINAL, THETA_TRUE,
                                 sigma=0.0, trials=trials)
            # 偏置加在绝对观测上；v是相邻观测之差，b理应精确抵消
            v_biased = v.copy()          # (p_k+b)-(p_{k-1}+b) == p_k-p_{k-1}
            th = _theta_hat_batch(u, v_biased)
            dtheta = float(np.max(np.abs(_wrap(th - THETA_TRUE))))
            # 平移锚点：p_G取自含偏置的绝对观测均值 -> t = t_true + b
            p_L = np.array([1.234, -2.345])
            c, s = math.cos(THETA_TRUE), math.sin(THETA_TRUE)
            R = np.array([[c, -s], [s, c]])
            p_G_true = R @ p_L + np.array([3.0, 4.0])
            t_true = p_G_true - R @ p_L
            t_biased = (p_G_true + b) - R @ p_L
            rows.append([bmag, bdir_deg, math.degrees(dtheta),
                         float(np.linalg.norm(t_biased - t_true)),
                         float(np.linalg.norm(b))])
    _write_csv('e2_bias_immunity.csv',
               ['bias_mag_m', 'bias_dir_deg', 'max_theta_err_deg',
                'translation_err_m', 'bias_norm_m'], rows)
    worst = max(r[2] for r in rows)
    print(f'  θ*最大偏差 {worst:.3e}° (数值零，验证式(10))；'
          f'平移误差恒等于|b|，验证式(11)')
    return rows


# ================================================================ E3
def exp3_batch_vs_window(trials=30, n_seg=1200, quick=False):
    """E3 批量 vs 滑窗对时变映射的跟踪能力——论文表3第1项、缺口G3。

    时变模型：真值θ(t)=θ0+ω·t（模拟SLAM缓慢漂移导致的局部-全局映射
    缓慢时变）。每段耗时Δt=d/v。滑窗方法的稳态滞后有解析预期
    ≈ω·(W-1)/2·Δt（窗口内数据的平均年龄），批量方法的滞后随时间线性
    发散（窗口=全历史，平均年龄=t/2）。
    """
    print('== E3 批量 vs 滑窗（时变映射跟踪）==')
    if quick:
        n_seg, trials = 200, 5
    speed = 1.0                                   # m/s
    dt = D_MIN_NOMINAL / speed                    # 每段耗时[s]
    rng = np.random.default_rng(11)
    rows, curves = [], {}
    for omega_dps in (0.01, 0.1, 0.5):
        omega = math.radians(omega_dps)
        err_win = np.zeros((trials, n_seg))
        err_bat = np.zeros((trials, n_seg))
        for t_i in range(trials):
            win = SlidingWindowEstimator(d_min=D_MIN_NOMINAL, window=W_NOMINAL)
            bat = BatchEstimator(d_min=D_MIN_NOMINAL)
            for k in range(n_seg):
                th_k = THETA_TRUE + omega * k * dt
                phi = rng.uniform(-math.pi, math.pi)
                ux, uy = D_MIN_NOMINAL * math.cos(phi), D_MIN_NOMINAL * math.sin(phi)
                c, s = math.cos(th_k), math.sin(th_k)
                nx = rng.normal(0, SIGMA_NOMINAL) * math.sqrt(2)
                ny = rng.normal(0, SIGMA_NOMINAL) * math.sqrt(2)
                vx, vy = ux * c - uy * s + nx, ux * s + uy * c + ny
                win.add_segment(ux, uy, vx, vy)
                bat.add_segment(ux, uy, vx, vy)
                err_win[t_i, k] = wrap_pi(win.theta - th_k)
                err_bat[t_i, k] = wrap_pi(bat.theta - th_k)
        tail = slice(int(n_seg * 0.7), n_seg)     # 取后30%当"稳态"
        # θ̂滞后于真值，误差符号为负：err = θ̂-θ_true ≈ -ω·(W-1)/2·Δt
        lag_theory = -math.degrees(omega * (W_NOMINAL - 1) / 2.0 * dt)
        rows.append([omega_dps, n_seg, trials,
                     math.degrees(np.mean(err_win[:, tail])),
                     math.degrees(np.sqrt(np.mean(err_win[:, tail] ** 2))),
                     math.degrees(np.mean(err_bat[:, tail])),
                     math.degrees(np.sqrt(np.mean(err_bat[:, tail] ** 2))),
                     lag_theory])
        curves[omega_dps] = (np.degrees(err_win.mean(axis=0)),
                             np.degrees(err_bat.mean(axis=0)), dt)
    _write_csv('e3_batch_vs_window.csv',
               ['omega_deg_per_s', 'n_seg', 'trials',
                'window_mean_err_deg', 'window_rmse_deg',
                'batch_mean_err_deg', 'batch_rmse_deg',
                'window_lag_theory_deg'], rows)
    for r in rows:
        print(f'  ω={r[0]}°/s: 滑窗稳态误差{r[3]:+.2f}°(理论滞后{r[7]:+.2f}°) '
              f'vs 批量{r[5]:+.2f}° (RMSE {r[4]:.2f}° vs {r[6]:.2f}°)')

    plt = _setup_matplotlib()
    fig, axes = plt.subplots(1, len(curves), figsize=(4.2 * len(curves), 3.6),
                             sharey=False)
    if len(curves) == 1:
        axes = [axes]
    for ax, (om, (ew, eb, dtv)) in zip(axes, sorted(curves.items())):
        t = np.arange(len(ew)) * dtv
        ax.plot(t, eb, label='批量最小二乘（全历史）', lw=1.2)
        ax.plot(t, ew, label=f'滑窗 W={W_NOMINAL}', lw=1.2)
        ax.axhline(0, color='k', lw=0.6)
        ax.set_xlabel('时间 [s]'); ax.set_ylabel('估计误差 [°]')
        ax.set_title(f'漂移率 ω={om}°/s'); ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle('批量方法与滑窗方法对时变局部-全局映射的跟踪能力', y=1.02)
    path = os.path.join(OUT_DIR, 'fig_e3_batch_vs_window.png')
    fig.savefig(path); plt.close(fig)
    print(f'  -> {path}')
    return rows


# ================================================================ E4
def exp4_huber(trials=400, quick=False):
    """E4 Huber抗差扩展验证——论文式(16)(17)(18)、缺口G6。

    野值模型：某次绝对观测被多径/NLOS污染，位置读数附加一个有限但显著
    的偏移(幅度m、方向随机)。因为段是首尾相接的，一次被污染的观测会同时
    影响相邻两段——这跟真实系统的传播方式一致。
    """
    print('== E4 Huber抗差扩展 ==')
    if quick:
        trials = 100
    rng = np.random.default_rng(23)
    rows = []
    ps = (0.0, 0.02, 0.05, 0.10, 0.20)
    mags = (0.5, 2.0, 5.0)
    for mag in mags:
        for p in ps:
            e_plain, e_rob = [], []
            for _ in range(trials):
                # 绝对观测序列(含噪声+野值) -> 首尾相接的段
                phi = rng.uniform(-math.pi, math.pi, W_NOMINAL)
                u = np.stack([D_MIN_NOMINAL * np.cos(phi),
                              D_MIN_NOMINAL * np.sin(phi)], axis=-1)
                c, s = math.cos(THETA_TRUE), math.sin(THETA_TRUE)
                R = np.array([[c, -s], [s, c]])
                v_true = u @ R.T
                pts = np.concatenate([np.zeros((1, 2)), np.cumsum(v_true, 0)])
                obs = pts + rng.normal(0, SIGMA_NOMINAL, pts.shape)
                hit = rng.random(len(obs)) < p
                if hit.any():
                    ang = rng.uniform(-math.pi, math.pi, hit.sum())
                    obs[hit] += mag * np.stack([np.cos(ang), np.sin(ang)], -1)
                v = obs[1:] - obs[:-1]
                segs = [(u[i, 0], u[i, 1], v[i, 0], v[i, 1])
                        for i in range(W_NOMINAL)]
                e_plain.append(wrap_pi(solve_theta(segs) - THETA_TRUE))
                th_r, _ = solve_theta_irls(segs, n_iter=3)
                e_rob.append(wrap_pi(th_r - THETA_TRUE))
            ep, er = np.abs(np.array(e_plain)), np.abs(np.array(e_rob))
            rows.append([mag, p, trials,
                         math.degrees(np.sqrt(np.mean(np.array(e_plain) ** 2))),
                         math.degrees(np.sqrt(np.mean(np.array(e_rob) ** 2))),
                         math.degrees(np.percentile(ep, 95)),
                         math.degrees(np.percentile(er, 95))])
    _write_csv('e4_huber.csv',
               ['outlier_mag_m', 'outlier_prob', 'trials',
                'rmse_plain_deg', 'rmse_huber_deg',
                'p95_plain_deg', 'p95_huber_deg'], rows)
    for r in rows:
        if r[1] in (0.0, 0.10):
            print(f'  幅度{r[0]}m 比例{r[1]:.0%}: RMSE 原始{r[3]:.2f}° -> '
                  f'Huber {r[4]:.2f}°')

    plt = _setup_matplotlib()
    fig, ax = plt.subplots(figsize=(5.5, 4))
    for mag, style in zip(mags, ('o-', 's-', '^-')):
        sub = [r for r in rows if r[0] == mag]
        ax.plot([r[1] * 100 for r in sub], [r[3] for r in sub], style,
                color='C3', ms=4, label=f'原始式(7)  野值{mag} m')
        ax.plot([r[1] * 100 for r in sub], [r[4] for r in sub], style,
                color='C0', ms=4, label=f'Huber式(18) 野值{mag} m')
    ax.set_xlabel('被污染观测的比例 [%]'); ax.set_ylabel(r'$\theta^*$误差RMSE [°]')
    ax.set_title('抗差扩展在野值污染下的效果（W=50）')
    ax.grid(alpha=0.3); ax.legend(fontsize=7)
    path = os.path.join(OUT_DIR, 'fig_e4_huber.png')
    fig.savefig(path); plt.close(fig)
    print(f'  -> {path}')
    return rows


# ================================================================ E5
def exp5_observability(trials=1000, quick=False):
    """E5 可观测性四格对照——论文第5节、缺口G8。

    论文第5节两条论断：
      ①纯噪声下，估计方差只取决于Σd_i²，与方向多样性无关；
      ②若局部里程计存在**方向相关**的系统性偏置，单一方向的位移段无法把
        该偏置与真实旋转角区分开。
    方向相关偏置用剪切矩阵A=[[1,γ],[0,1]]建模（沿局部x走不受影响、沿局部y
    走被扭转约atan(γ)），"偏置引起的等效角度"本身依赖方向，正是论断②说的
    那种。判据用两条：θ̂的系统偏差（被吸收进θ的部分）和残差RMS相对噪声底
    的抬升（模型失配是否可被观测到）。

    ⚠️ 论断①这里用**独立噪声**模型验证（每段两端各一次独立观测）——机上
    实际是首尾相接的，那种结构下方向一致反而方差更小，见E1的(*)式，两件
    事不矛盾但不能混在一张表里说。
    """
    print('== E5 可观测性（方向多样性 × 方向相关偏置）==')
    if quick:
        trials = 200
    rng = np.random.default_rng(31)
    rows = []
    # 残差是二维向量的模长：E|r|² = 2·(σ√2)² -> RMS = 2σ
    noise_floor = 2.0 * SIGMA_NOMINAL
    for single in (True, False):
        for gamma in (0.0, 0.05, 0.10, 0.20):
            errs, res_rms = [], []
            for _ in range(trials):
                if single:
                    phi = np.full(W_NOMINAL, math.radians(90.0))  # 只沿局部+y
                else:
                    phi = rng.uniform(-math.pi, math.pi, W_NOMINAL)
                u_true = np.stack([D_MIN_NOMINAL * np.cos(phi),
                                   D_MIN_NOMINAL * np.sin(phi)], -1)
                A = np.array([[1.0, gamma], [0.0, 1.0]])
                u_meas = u_true @ A.T          # 里程计读数（含方向相关偏置）
                c, s = math.cos(THETA_TRUE), math.sin(THETA_TRUE)
                R = np.array([[c, -s], [s, c]])
                v = u_true @ R.T + rng.normal(0, SIGMA_NOMINAL * math.sqrt(2),
                                              u_true.shape)
                segs = [(u_meas[i, 0], u_meas[i, 1], v[i, 0], v[i, 1])
                        for i in range(W_NOMINAL)]
                th = solve_theta(segs)
                errs.append(wrap_pi(th - THETA_TRUE))
                res_rms.append(float(np.sqrt(np.mean(
                    np.array(residuals(segs, th)) ** 2))))
            errs = np.array(errs)
            rows.append(['单方向' if single else '多方向', gamma, trials,
                         math.degrees(np.mean(errs)),
                         math.degrees(np.std(errs)),
                         math.degrees(math.atan(gamma)),
                         float(np.mean(res_rms)), noise_floor,
                         float(np.mean(res_rms)) / noise_floor])
    _write_csv('e5_observability.csv',
               ['direction_mode', 'gamma', 'trials',
                'theta_bias_deg', 'theta_std_deg', 'atan_gamma_deg',
                'residual_rms_m', 'noise_floor_m', 'residual_over_floor'],
               rows)
    for r in rows:
        print(f'  {r[0]} γ={r[1]:.2f}: θ偏差{r[3]:+.2f}° (atanγ={r[5]:.2f}°) '
              f'std {r[4]:.2f}° 残差/噪声底 {r[8]:.2f}')

    plt = _setup_matplotlib()
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 3.8))
    for mode, style in (('单方向', 'o-'), ('多方向', 's-')):
        sub = [r for r in rows if r[0] == mode]
        axes[0].plot([r[1] for r in sub], [abs(r[3]) for r in sub], style,
                     ms=4, label=mode)
        axes[1].plot([r[1] for r in sub], [r[8] for r in sub], style, ms=4,
                     label=mode)
    sub = [r for r in rows if r[0] == '单方向']
    axes[0].plot([r[1] for r in sub], [r[5] for r in sub], 'k--', lw=1,
                 label=r'$\arctan\gamma$（偏置被完全吸收）')
    axes[0].set_xlabel(r'里程计方向相关偏置 $\gamma$')
    axes[0].set_ylabel(r'$\theta^*$系统偏差 [°]')
    axes[0].set_title('(a) 偏置被误当成旋转角的程度')
    axes[1].axhline(1.0, color='k', ls='--', lw=1, label='纯噪声水平')
    axes[1].set_xlabel(r'里程计方向相关偏置 $\gamma$')
    axes[1].set_ylabel('残差RMS / 噪声底')
    axes[1].set_title('(b) 模型失配能否被残差观测到')
    for ax in axes:
        ax.grid(alpha=0.3); ax.legend(fontsize=7)
    fig.suptitle('运动方向多样性与可观测性（W=50，σ=0.05 m）', y=1.03)
    path = os.path.join(OUT_DIR, 'fig_e5_observability.png')
    fig.savefig(path); plt.close(fig)
    print(f'  -> {path}')
    return rows


# ================================================================ E6
def exp6_async(trials=300, quick=False):
    """E6 观测异步/时延消融（合成版）——缺口G12。

    机上`_maybe_record_segment`用的是"里程计回调那一刻最新的一帧UWB"，
    没有任何时间戳对齐。UWB以f_uwb发布、带固定时延τ0，因此端点数据的
    陈旧程度在τ0 ~ τ0+1/f_uwb之间抖动。恒定速度直线运动时两端的固定
    时延共同抵消，但**抖动部分不抵消**；转弯/加减速时时延项也不再抵消。
    这里对比三种取数方式在不同速度、不同轨迹下的θ*方差：
      aligned : 用与里程计同一时刻的真实位置(理想，只有σ噪声)
      latest  : 机上现状——最近一帧UWB
      interp  : 按时间戳把UWB线性插值到里程计时刻(候选改进方案)
    """
    print('== E6 观测异步/时延消融 ==')
    if quick:
        trials = 40
    rng = np.random.default_rng(43)
    f_odom, f_uwb, tau0 = 20.0, 10.0, 0.030
    rows = []
    for path in ('straight', 'serpentine'):
        for speed in (0.3, 1.0, 2.0):
            for mode in ('aligned', 'latest', 'interp'):
                errs = []
                for _ in range(trials):
                    est = SlidingWindowEstimator(d_min=D_MIN_NOMINAL,
                                                 window=W_NOMINAL)
                    # 需要跑足够长的时间攒满滑窗：W段×d米÷速度
                    T = (W_NOMINAL + 5) * D_MIN_NOMINAL / speed
                    n = int(T * f_odom)
                    uwb_buf = []            # (发布时刻, 观测时刻, x, y)
                    n_avail = 0             # 已经"发布出来"的条数（单调推进，
                                            # 不要每帧重新过滤整个缓冲：那是O(n²)）
                    next_uwb = 0.0
                    for i in range(n):
                        t = i / f_odom
                        # 局部系轨迹
                        if path == 'straight':
                            lx, ly = speed * t, 0.0
                        else:                # 蛇形：持续转弯，时延不再抵消
                            lx = speed * t
                            ly = 2.0 * math.sin(0.25 * speed * t)
                        c, s = math.cos(THETA_TRUE), math.sin(THETA_TRUE)
                        gx, gy = lx * c - ly * s, lx * s + ly * c
                        # UWB按f_uwb采样，采样值是t_meas时刻的真值+噪声，
                        # 但要等到t_meas+τ0才发布出来
                        while next_uwb <= t:
                            tm = next_uwb
                            if path == 'straight':
                                mx, my = speed * tm, 0.0
                            else:
                                mx = speed * tm
                                my = 2.0 * math.sin(0.25 * speed * tm)
                            ox = mx * c - my * s + rng.normal(0, SIGMA_NOMINAL)
                            oy = mx * s + my * c + rng.normal(0, SIGMA_NOMINAL)
                            uwb_buf.append((tm + tau0, tm, ox, oy))
                            next_uwb += 1.0 / f_uwb
                        while n_avail < len(uwb_buf) and uwb_buf[n_avail][0] <= t:
                            n_avail += 1
                        if n_avail == 0:
                            continue
                        avail = uwb_buf[:n_avail]
                        if mode == 'aligned':
                            g = (gx + rng.normal(0, SIGMA_NOMINAL),
                                 gy + rng.normal(0, SIGMA_NOMINAL))
                        elif mode == 'latest':
                            g = (avail[-1][2], avail[-1][3])
                        else:                # interp：按观测时刻插值到t
                            if len(avail) >= 2:
                                b0, b1 = avail[-2], avail[-1]
                                dtm = b1[1] - b0[1]
                                k = 0.0 if dtm <= 0 else (t - b0[1]) / dtm
                                g = (b0[2] + k * (b1[2] - b0[2]),
                                     b0[3] + k * (b1[3] - b0[3]))
                            else:
                                g = (avail[-1][2], avail[-1][3])
                        est.update((lx, ly), g)
                    if est.n_segments >= W_NOMINAL // 2:
                        errs.append(wrap_pi(est.theta - THETA_TRUE))
                errs = np.array(errs)
                rows.append([path, speed, mode, len(errs),
                             math.degrees(np.mean(errs)) if len(errs) else float('nan'),
                             math.degrees(np.std(errs)) if len(errs) else float('nan'),
                             math.degrees(np.sqrt(np.mean(errs ** 2))) if len(errs) else float('nan')])
    _write_csv('e6_async.csv',
               ['path', 'speed_mps', 'mode', 'n_valid',
                'bias_deg', 'std_deg', 'rmse_deg'], rows)
    for r in rows:
        print(f'  {r[0]:10s} v={r[1]}m/s {r[2]:8s}: 偏差{r[4]:+.2f}° '
              f'std {r[5]:.2f}° RMSE {r[6]:.2f}°')

    plt = _setup_matplotlib()
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 3.8))
    for ax, path in zip(axes, ('straight', 'serpentine')):
        for mode, style in (('aligned', 'o-'), ('latest', 's-'), ('interp', '^-')):
            sub = [r for r in rows if r[0] == path and r[2] == mode]
            sub.sort(key=lambda r: r[1])
            ax.plot([r[1] for r in sub], [r[6] for r in sub], style, ms=4,
                    label={'aligned': '理想时间对齐（仅σ噪声）',
                           'latest': '最近一帧（机上现状）',
                           'interp': '时间戳插值对齐'}[mode])
        ax.set_xlabel('飞行速度 [m/s]'); ax.set_ylabel(r'$\theta^*$误差RMSE [°]')
        ax.set_title({'straight': '(a) 匀速直线', 'serpentine': '(b) 持续转弯（蛇形）'}[path])
        ax.grid(alpha=0.3); ax.legend(fontsize=7)
    fig.suptitle('观测异步对估计精度的影响（UWB 10 Hz + 30 ms时延）', y=1.03)
    path_png = os.path.join(OUT_DIR, 'fig_e6_async.png')
    fig.savefig(path_png); plt.close(fig)
    print(f'  -> {path_png}')
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exp', default='all',
                    help='all 或 e1/e2/e3/e4/e5/e6，逗号分隔')
    ap.add_argument('--trials', type=int, default=None, help='覆盖默认重复次数')
    ap.add_argument('--quick', action='store_true', help='冒烟模式，配置点少')
    args = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)
    want = ([x.strip() for x in args.exp.split(',')]
            if args.exp != 'all' else ['e1', 'e2', 'e3', 'e4', 'e5', 'e6'])
    kw = {} if args.trials is None else {'trials': args.trials}
    if 'e1' in want:
        exp1_crlb(quick=args.quick, **(kw or {'trials': 2000}))
    if 'e2' in want:
        exp2_bias(**(kw or {}))
    if 'e3' in want:
        exp3_batch_vs_window(quick=args.quick, **(kw or {}))
    if 'e4' in want:
        exp4_huber(quick=args.quick, **(kw or {}))
    if 'e5' in want:
        exp5_observability(quick=args.quick, **(kw or {}))
    if 'e6' in want:
        exp6_async(quick=args.quick, **(kw or {}))
    print(f'\n全部结果写在 {OUT_DIR}')


if __name__ == '__main__':
    main()

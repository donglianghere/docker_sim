#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞行时间线：把两机日志里真正要紧的那几十行捞出来，按时间并排。

    python3 scripts/timeline.py                      # 读 sim_leader / sim_follower 容器
    python3 scripts/timeline.py --events             # 只看跨机事件（排死锁用）
    python3 scripts/timeline.py --from a.log b.log   # 读保存下来的日志

**为什么要有这个**：排查时的动作几乎总是
`docker logs sim_leader | grep -E "航点|解散|声光|..."`，而选手不知道该 grep
什么。更要命的是跨机事件——可靠通道"先回 ACK 再查处理函数"，没注册的事件会
被确认后丢弃，这类问题**从单机日志根本看不出来**，必须把两机的事件时间线
并排对齐才看得见谁在等谁。2026-10-01 踩过两次：先是航线事件被静默丢弃
（僚机机头全程不变、退化成就地入列），改成"长机等 READY 再发"之后 READY
自己撞同一个坑，两边各等满 60 秒。

左边一列是长机，右边一列是僚机，中间是时刻。
"""
import argparse
import re
import subprocess
import sys
import unicodedata

# (正则, 类别)。类别用来上色/过滤，也用来决定哪些进 --events。
PATTERNS = [
    (r'起飞完成', '起降'), (r'降落完成，armed=False', '起降'),
    (r'已降落在自己起降点', '起降'), (r'⚠️ 落点', '起降'),
    (r'已确认落地', '起降'),
    (r'航点[^，]*，航向', '航线'), (r'已经在脚下', '航线'),
    (r'已到达\(', '航线'), (r'已到 \(', '航线'),
    (r'飞往.*（直线，不避障）', '航线'),
    (r'第 \d 轮', '阶段'), (r'本轮判定', '阶段'), (r'三轮全部完成', '阶段'),
    (r'=====', '阶段'),
    (r"事件'[^']+'已被\S+确认收到", '事件'), (r'收件箱已注册', '事件'),
    (r'等队友事件', '事件'), (r'通知队友进场', '事件'),
    (r'僚机已入位', '编队'), (r'僚机已过点', '编队'), (r'编队解散', '编队'),
    (r'僚机已过航点', '编队'), (r'起始站位', '编队'), (r'就地入列', '编队'),
    (r'编队跟随已出列', '编队'), (r'补发编队解散', '编队'),
    (r'声光反馈', '播报'),
    (r'拍照回传', '拍照'), (r'拍照失败', '拍照'),
    (r'发现\S*火情', '视觉'), (r'已对准', '视觉'), (r'仍没对准', '视觉'),
    (r'轮对准：', '视觉'),
    (r'抓取\S*完成', '机构'), (r'投放完成', '机构'), (r'释放\S*完成', '机构'),
    (r'发射\S*完成', '机构'),
    (r'限速 ->', '限速'),
    (r'Traceback|Error|错误|不可达|超时|⚠', '❗问题'),
]
COMPILED = [(re.compile(p), c) for p, c in PATTERNS]
TS = re.compile(r'^(\d{4}-\d\d-\d\dT)?(\d\d:\d\d:\d\d)')


def harvest(text, only=None):
    out = []
    for line in text.splitlines():
        m = TS.search(line)
        if not m:
            continue
        body = line[m.end():].lstrip('.0123456789Z ')
        for rx, cat in COMPILED:
            if rx.search(body):
                if only and cat not in only:
                    break
                out.append((m.group(2), cat, re.sub(r'^\[\S+\]\s*', '', body).strip()))
                break
    return out


def dwidth(t):
    """显示宽度。中文/全角字符占两列，len() 按一列算会把表格撑歪。"""
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in t)


def fit(t, w):
    """截断/补齐到显示宽度 w。"""
    if dwidth(t) > w:
        out = ''
        for c in t:
            if dwidth(out) + dwidth(c) > w - 1:
                break
            out += c
        t = out + '…'
    return t + ' ' * (w - dwidth(t))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--from', dest='files', nargs=2, metavar=('长机日志', '僚机日志'))
    ap.add_argument('--containers', nargs=2, default=['sim_leader', 'sim_follower'])
    ap.add_argument('--events', action='store_true', help='只看跨机事件和编队握手（排死锁）')
    ap.add_argument('--problems', action='store_true', help='只看异常')
    ap.add_argument('--width', type=int, default=58)
    args = ap.parse_args()

    only = None
    if args.events:
        only = {'事件', '编队'}
    elif args.problems:
        only = {'❗问题'}

    texts = []
    for i in (0, 1):
        if args.files:
            texts.append(open(args.files[i], encoding='utf-8', errors='replace').read())
        else:
            r = subprocess.run(['docker', 'logs', '-t', args.containers[i]],
                               capture_output=True, text=True)
            if r.returncode != 0:
                print(f'读不到容器 {args.containers[i]}（跑完加 --keep 才会留下），'
                      f'或者用 --from 指定日志文件', file=sys.stderr)
                return 2
            texts.append(r.stdout + r.stderr)

    rows = [(t, c, m, 0) for t, c, m in harvest(texts[0], only)] + \
           [(t, c, m, 1) for t, c, m in harvest(texts[1], only)]
    rows.sort(key=lambda r: r[0])
    w = args.width
    L, R = args.containers
    pad = (w - dwidth(L)) // 2
    print(f'{" " * pad}{L}{" " * (w - dwidth(L) - pad)} │   时刻   │ {R}')
    print('─' * w + '─┼──────────┼─' + '─' * w)
    probs = 0
    for t, cat, msg, side in rows:
        if cat == '❗问题':
            probs += 1
        txt = f'[{cat}] {msg}'
        if side == 0:
            print(f'{fit(txt, w)} │ {t} │')
        else:
            print(f'{" " * w} │ {t} │ {txt}')
    print('─' * w + '─┴──────────┴─' + '─' * w)
    print(f'共 {len(rows)} 条' + (f'，其中 ❗问题 {probs} 条' if probs else '，无异常'))
    return 0


if __name__ == '__main__':
    sys.exit(main())

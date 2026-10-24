#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""算监视器 `--route` 要的那串航线坐标，**坐标一律以 venue.py 为准**。

为什么要有这个脚本
------------------
监视器画"沿航线间距"时，手里这条线必须跟长机实际带队跑的那条逐点一致。
以前两个运行器（contestant_sim/run.sh、contestant_real/run_real.sh）里各
抄了一份坐标串，改了 venue.py 的航点忘了同步，间距就会读成**整条航线长**
（2026-10-15 实测 67 米、占 38% 样本，见 project_monitor_gap_route_artifacts）。

2026-10-23 核对发现那两份抄件**确实已经漂移**：
    run_real.sh 写 A=2.5,2.5  F=13.4,17   D=17.5,2.8
    venue.py 实为 A=2.7,2.7   F=10.0,17.5  D=17.2,2.7
所以改成每次由本脚本从 venue.py 现算，运行器不再存第二份坐标。

坐标从哪来、顺序从哪来
----------------------
* **坐标**：选手目录下的 `venue.py`，只认顶层 `ROUTE_x = (x, y)` 这类字面量。
  真机和仿真各有一份 venue.py，传哪个目录就用哪套场地，脚本本身不分真假。
* **顺序**：`rounds.py` 里的 `ROUTE_FULL`（编队整条航线）。顺序属于流程、
  不属于场地，所以不写死在这里——改了航线顺序这边也跟着变。

两种航线
--------
* `loop`（formation / mission）：`ROUTE_FULL` 整条，**末尾补回第一个点闭合**。
  闭合是硬要求：RoutePath 用首尾相接判断"绕回来了"，不闭合就会把一个小间距
  算成绕行整条航线的长度。
* `tail`（groundfire / highrise）：只有编队返航那一段 `G -> D`，同样闭合回
  `ROUTE_A`。这两个程序的编队段是从 G 点开始的，前面侦察段不算编队间距。

解析用 `ast` 不用 `import`
--------------------------
venue.py 是纯常量表、本可以直接 import，但 rounds.py 顶上有
`import contest_sdk`，而 contest_sdk 只装在容器里、地面站没有。两个文件都走
`ast.literal_eval` 就不用关心运行环境，也不会执行到任何副作用代码。

用法
----
    route_for_monitor.py <选手目录> <程序名> [--rounds <rounds.py 路径>]

输出一行形如 `2.7,2.7 2.7,22.2 17.0,22.5 ...`，直接喂给 `monitor.py --route`。
出错时往 stderr 写原因并以非 0 退出，运行器据此决定是否降级（不传 --route
只是没有间距曲线，不该让整个架次飞不了）。
"""
import argparse
import ast
import sys
from pathlib import Path

#: 没给 --rounds 时按这个相对路径找（docker_sim/src/contest_sdk/rounds.py）。
DEFAULT_ROUNDS = Path(__file__).resolve().parent.parent / 'src' / 'contest_sdk' / 'rounds.py'
#: 程序名以这些开头时用整条闭环航线，其余只用编队返航那一段。
LOOP_PREFIXES = ('formation', 'mission')


def load_consts(path: Path) -> dict:
    """取出一个 .py 里所有顶层 `NAME = <字面量>`，不执行任何代码。"""
    try:
        tree = ast.parse(path.read_text(encoding='utf-8'))
    except (OSError, SyntaxError) as exc:
        raise SystemExit(f'读不了 {path}：{exc}')
    out = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        tgt = node.targets[0]
        if not isinstance(tgt, ast.Name):
            continue
        try:
            out[tgt.id] = ast.literal_eval(node.value)
        except ValueError:
            pass        # 不是字面量（比如带条件表达式的 HIGH_FIRE），跳过
    return out


def load_name_list(path: Path, var: str) -> list:
    """取出 `var = [NAME_A, NAME_B, ...]` 里的那串名字。

    这些元素是变量引用不是字面量，literal_eval 吃不下，所以单独走一遍 AST。
    """
    try:
        tree = ast.parse(path.read_text(encoding='utf-8'))
    except (OSError, SyntaxError) as exc:
        raise SystemExit(f'读不了 {path}：{exc}')
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == var
                and isinstance(node.value, (ast.List, ast.Tuple))):
            names = [e.id for e in node.value.elts if isinstance(e, ast.Name)]
            if len(names) != len(node.value.elts):
                raise SystemExit(f'{path} 的 {var} 里有不是变量名的元素，这里只认 [ROUTE_A, ...] 这种写法')
            return names
    raise SystemExit(f'{path} 里找不到 {var}')


def as_xy(value, name: str) -> tuple:
    """把 venue 里取到的值收成 (x, y)，顺带挡住写错类型的情况。"""
    if (not isinstance(value, (tuple, list)) or len(value) < 2
            or not all(isinstance(v, (int, float)) for v in value[:2])):
        raise SystemExit(f'venue.py 的 {name} 不是 (x, y) 数值对：{value!r}')
    return float(value[0]), float(value[1])


def build(venue_dir: Path, program: str, rounds_py: Path) -> list:
    venue_py = venue_dir / 'venue.py'
    if not venue_py.is_file():
        raise SystemExit(f'找不到 {venue_py}')
    consts = load_consts(venue_py)

    stem = Path(program).stem
    if stem.startswith(LOOP_PREFIXES):
        names = load_name_list(rounds_py, 'ROUTE_FULL')
        closing = names[0]
    else:
        names = ['ROUTE_G', 'ROUTE_D']
        closing = 'ROUTE_A'

    missing = [n for n in names + [closing] if n not in consts]
    if missing:
        raise SystemExit(f'venue.py 里缺这些航点：{" ".join(missing)}')

    pts = [as_xy(consts[n], n) for n in names]
    tail = as_xy(consts[closing], closing)
    if pts[-1] != tail:          # 闭合；本来就回到起点就不重复补
        pts.append(tail)
    return pts


def main() -> None:
    ap = argparse.ArgumentParser(description='从 venue.py 算监视器的 --route 坐标串')
    ap.add_argument('venue_dir', help='选手目录（里面有 venue.py）')
    ap.add_argument('program', help='要跑的程序名，决定用整条航线还是返航段')
    ap.add_argument('--rounds', default=str(DEFAULT_ROUNDS),
                    help='rounds.py 路径，默认 src/contest_sdk/rounds.py')
    args = ap.parse_args()

    pts = build(Path(args.venue_dir), args.program, Path(args.rounds))
    print(' '.join(f'{x:g},{y:g}' for x, y in pts))


if __name__ == '__main__':
    sys.exit(main())

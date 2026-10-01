#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成选手版 SDK API 参考（.docx）。

    python3 scripts/gen_sdk_api_doc.py            # 写 API参考.docx + API参考.md
    python3 scripts/gen_sdk_api_doc.py --check    # 只检查分类有没有漏，不写文件

**只收录四个选手程序真正用过的 API**，不是全部 80 个。用过哪些是从
contestant_template/*_lite.py 里用 AST 扫出来的，程序改了重新跑一遍就同步——
不手工维护名单，手工名单迟早跟代码对不上。

签名和说明同样从 capabilities.py 读。分类表是人工定的（机器分不出该放哪类），
扫出来的 API 如果没进分类表会直接报错，而不是悄悄漏掉。
"""
import argparse
import ast
import glob
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src', 'contest_sdk', 'contest_sdk', 'capabilities.py')
PROGS = os.path.join(ROOT, 'contestant_template', '*_lite.py')
OUT_DOCX = os.path.join(ROOT, 'contestant_template', 'API参考.docx')
OUT_MD = os.path.join(ROOT, 'contestant_template', 'API参考.md')

CATEGORIES = [
    ('程序入口', ['run', 'spacing_m']),
    ('起飞 / 降落 / 返航', ['takeoff', 'return_home', 'own_pad']),
    ('航线飞行', ['fly_route', 'goto_world', 'hold_at', 'set_agl', 'step_forward']),
    ('编队', ['lead_formation', 'follow_formation']),
    ('跨机协同', ['open_inbox', 'wait_event', 'wait_any_event',
                  'send_to_teammate', 'yield_spot']),
    ('视觉：识别与对准', ['search_along', 'patrol', 'aim_at']),
    ('拍照', ['snapshot', 'PHOTO_DIR']),
    ('抓放与发射', ['grip', 'fetch_from', 'release_at', 'shoot']),
    ('声光播报', ['announce']),
    ('日志', ['progress']),
]

PITFALLS = {
    'open_inbox': '可靠事件通道是“先回 ACK 再查处理函数”，没注册的事件会被确认后丢弃。'
                  '任务一开始就把所有事件注册全，别等用到了再注册。',
    'aim_at': 'camera=\'down\' 会自动改走 precision_servo 闭环。前视那套几何把画面纵轴'
              '当成世界的高低，下视时纵轴其实是机体前后方向，照搬永远收敛不了。',
    'goto_world': 'direct=True 会关掉避障，只在算过余量的航段上开（tools/check_route.py）。'
                  '航线上的避障本身是考核点，不能为了快绕过去。',
    'snapshot': '不要给拍照配声光事件：声光事件是固定枚举，自造事件名会直接抛 ValueError '
                '把整个任务打断。',
    'return_home': '自带落点核对：落在起降点上才播 sound、才算数。任务流程里会降落好几次'
                   '（取器材、放器材、回家），“降落动作完成”本身说明不了任务结束。',
    'lead_formation': 'disband_at 要显式给。不给的话默认判据是“长机飞回自己起飞点上空”，'
                      '而起飞点不一定在航线上，编队可能在半路散掉。',
    'shoot': '连发时 sound 只播一次，播报点在第一发之前。',
    'PHOTO_DIR': 'snapshot() 的存图目录，用实例属性覆盖即可：sdk.PHOTO_DIR = \'/logs/xxx\'。',
    'spacing_m': 'run() 把命令行 --spacing 存成这个属性，任务函数直接读。',
}


def used_apis():
    """四个选手程序里 sdk.X / DroneSDK.X 用到的名字 -> 用在哪几个程序。"""
    out = {}
    for f in sorted(glob.glob(PROGS)):
        name = os.path.basename(f)
        for n in ast.walk(ast.parse(io.open(f, encoding='utf-8').read())):
            if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) \
                    and n.value.id in ('sdk', 'DroneSDK'):
                out.setdefault(n.attr, set()).add(name.replace('_lite.py', ''))
    return out


def collect():
    cls = next(n for n in ast.parse(io.open(SRC, encoding='utf-8').read()).body
               if isinstance(n, ast.ClassDef) and n.name == 'DroneSDK')
    api = {}
    for n in cls.body:
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and not n.name.startswith('_'):
            doc = ast.get_docstring(n) or ''
            para, args = [], []
            lines = doc.splitlines()
            i = 0
            while i < len(lines) and (lines[i].strip() or not para):
                if lines[i].strip():
                    para.append(lines[i].strip())
                i += 1
            summary = ''.join(para)
            if 'Args:' in doc:
                # 参数说明经常跨好几行。**按缩进判断**：缩进等于基准的那一行
                # 是新参数，更深的是上一条的续行，要接上去——否则一条说明会被
                # 拆成好几个条目，读起来是断的。
                blk = doc.split('Args:', 1)[1].splitlines()[1:]
                base = None
                for line in blk:
                    if not line.strip():
                        if args:
                            break
                        continue
                    ind = len(line) - len(line.lstrip())
                    if base is None:
                        base = ind
                    if ind <= base and line.strip().endswith(':') and ' ' not in line.strip():
                        break                      # 碰到 Returns:/Raises: 这类下一节
                    if ind > base and args:
                        args[-1] = args[-1].rstrip() + ' ' + line.strip()
                    else:
                        args.append(line.strip())
            sig = ast.unparse(n.args).replace('self, ', '').replace('self', '')
            api[n.name] = {'sig': f'{n.name}({sig})', 'summary': summary, 'args': args,
                           'static': any(isinstance(d, ast.Name) and d.id == 'staticmethod'
                                         for d in n.decorator_list)}
    for n in ast.walk(cls):      # 类属性（PHOTO_DIR 这种）
        if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name):
            api.setdefault(n.targets[0].id, {'sig': n.targets[0].id, 'summary': '',
                                             'args': [], 'static': False})
    for n in ast.walk(cls):      # 实例属性（spacing_m）
        if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store) \
                and isinstance(n.value, ast.Name) and n.value.id in ('self', 'sdk'):
            api.setdefault(n.attr, {'sig': n.attr, 'summary': '', 'args': [], 'static': False})
    return api


def _rich(par, text, base_size=None):
    """把 docstring 里的 Markdown 行内标记渲染成 Word 的格式。

    源码 docstring 用 **粗体** 和 `等宽` 标记重点，直接塞进 docx 就是一堆
    星号和反引号，在 Word 里很难看。这里按标记切段，分别设成粗体 / Consolas。
    """
    import re
    from docx.shared import Pt
    for seg in re.split(r'(\*\*[^*]+\*\*|`[^`]+`)', text):
        if not seg:
            continue
        if seg.startswith('**') and seg.endswith('**'):
            r = par.add_run(seg[2:-2]); r.bold = True
        elif seg.startswith('`') and seg.endswith('`'):
            r = par.add_run(seg[1:-1]); r.font.name = 'Consolas'
        else:
            r = par.add_run(seg)
        if base_size:
            r.font.size = Pt(base_size)


def _plain(text):
    """去掉 Markdown 标记，给表格单元格用（单元格里不做富文本）。"""
    import re
    return re.sub(r'\*\*([^*]+)\*\*', r'\1', text).replace('`', '')


def build(api, used):
    from docx import Document
    from docx.shared import Pt, RGBColor
    from docx.enum.text import WD_ALIGN_PARAGRAPH

    d = Document()
    for st, sz in (('Normal', 10.5),):
        d.styles[st].font.size = Pt(sz)
        d.styles[st].font.name = '微软雅黑'

    d.add_heading('contest_sdk API 参考（选手版）', 0)
    p = d.add_paragraph()
    p.add_run('只收录四个选手程序真正用过的 API。').bold = True
    p.add_run(f'共 {len(used)} 个，SDK 全部能力有 80 个，其余是底层/备用接口，'
              '写任务一般用不到。本文档由 scripts/gen_sdk_api_doc.py 自动生成，不要手改。')

    d.add_heading('怎么开始', level=1)
    d.add_paragraph('命令行解析、建 SDK、按角色分派、收尾，一句就够：', style='Intense Quote')
    c = d.add_paragraph()
    c.add_run("if __name__ == '__main__':\n    DroneSDK.run(leader=recon, follower=supply)"
              ).font.name = 'Consolas'

    d.add_heading('速查表', level=1)
    t = d.add_table(rows=1, cols=3)
    t.style = 'Light Grid Accent 1'
    for i, h in enumerate(('方法', '用在哪个程序', '做什么')):
        t.rows[0].cells[i].paragraphs[0].add_run(h).bold = True
    for cat, names in CATEGORIES:
        for m in names:
            if m not in used:
                continue
            r = t.add_row().cells
            r[0].paragraphs[0].add_run(api[m]['sig'].split('(')[0]).font.name = 'Consolas'
            r[1].text = '、'.join(sorted(used[m]))
            r[2].text = _plain(api[m]['summary'] or PITFALLS.get(m, ''))[:60]

    d.add_page_break()
    d.add_heading('逐个说明', level=1)
    for cat, names in CATEGORIES:
        shown = [m for m in names if m in used]
        if not shown:
            continue
        d.add_heading(cat, level=2)
        for m in shown:
            info = api[m]
            h = d.add_paragraph()
            run = h.add_run(info['sig'])
            run.bold = True
            run.font.name = 'Consolas'
            run.font.size = Pt(11)
            if info['static']:
                h.add_run('  （staticmethod）').italic = True
            if info['summary']:
                _rich(d.add_paragraph(), info['summary'])
            for a in info['args']:
                ap = d.add_paragraph(style='List Bullet')
                ap.paragraph_format.left_indent = Pt(24)
                _rich(ap, a)
            if m in PITFALLS:
                w = d.add_paragraph()
                w.add_run('⚠ ').font.color.rgb = RGBColor(0xC0, 0x00, 0x00)
                _rich(w, PITFALLS[m])
                for r in w.runs:
                    r.font.color.rgb = RGBColor(0xC0, 0x00, 0x00)
            u = d.add_paragraph()
            u.add_run('用在：' + '、'.join(sorted(used[m]))).italic = True
            u.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    return d


def build_md(api, used):
    """同样的内容输出一份 Markdown。docstring 本来就是 Markdown 风格的行内
    标记（**粗体**、`等宽`），这里原样保留，不像 docx 那样要转成格式。"""
    L = ['# contest_sdk API 参考（选手版）\n',
         f'> **只收录四个选手程序真正用过的 API**，共 {len(used)} 个。'
         f'SDK 全部能力有 80 个，其余是底层/备用接口，写任务一般用不到。\n'
         '> 本文档由 `scripts/gen_sdk_api_doc.py` 自动生成，**不要手改**；'
         '改了选手程序或 SDK 重新跑一遍即可。\n',
         '\n## 怎么开始\n',
         '命令行解析、建 SDK、按角色分派、收尾，一句就够：\n',
         "```python\nif __name__ == '__main__':\n"
         "    DroneSDK.run(leader=recon, follower=supply)\n```\n",
         '\n## 速查表\n',
         '| 方法 | 用在哪个程序 | 做什么 |', '|---|---|---|']
    for cat, names in CATEGORIES:
        for m in names:
            if m not in used:
                continue
            nm = api[m]['sig'].split('(')[0]
            desc = _plain(api[m]['summary'] or PITFALLS.get(m, ''))[:60]
            L.append(f'| [`{nm}`](#{nm.lower()}) | {"、".join(sorted(used[m]))} | {desc} |')
    L.append('\n## 逐个说明\n')
    for cat, names in CATEGORIES:
        shown = [m for m in names if m in used]
        if not shown:
            continue
        L.append(f'\n### {cat}\n')
        for m in shown:
            info = api[m]
            L.append(f'<a id="{m.lower()}"></a>')
            L.append(f'**`{info["sig"]}`**'
                     + ('  *(staticmethod)*' if info['static'] else '') + '\n')
            if info['summary']:
                L.append(info['summary'] + '\n')
            for a in info['args']:
                L.append(f'- {a}')
            if info['args']:
                L.append('')
            if m in PITFALLS:
                L.append(f'> ⚠️ {PITFALLS[m]}\n')
            L.append(f'*用在：{"、".join(sorted(used[m]))}*\n')
    return '\n'.join(L) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()

    api, used = collect(), used_apis()
    listed = {m for _, ms in CATEGORIES for m in ms}
    missing = sorted(set(used) - listed)
    unknown = sorted(set(used) - set(api))
    stale = sorted(listed - set(used))
    if missing or unknown:
        if missing:
            print(f'★NG  选手程序用了但没进分类表：{missing}', file=sys.stderr)
        if unknown:
            print(f'★NG  选手程序用了 SDK 里不存在的名字：{unknown}', file=sys.stderr)
        return 1
    if stale:
        print(f'注意  分类表里这些已经没有程序在用，已跳过：{stale}', file=sys.stderr)
    if args.check:
        print(f'OK   {len(used)} 个在用 API 全部有分类')
        return 0
    build(api, used).save(OUT_DOCX)
    io.open(OUT_MD, 'w', encoding='utf-8').write(build_md(api, used))
    cats = sum(1 for c, ms in CATEGORIES if any(m in used for m in ms))
    for f in (OUT_DOCX, OUT_MD):
        print(f'已生成 {os.path.relpath(f, ROOT)}（{len(used)} 个在用 API，{cats} 个分类）')
    return 0


if __name__ == '__main__':
    sys.exit(main())

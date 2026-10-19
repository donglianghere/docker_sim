#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""函数内"先用后赋"检查。
⚠️ 2026-10-13 修了一个漏报：原来用 `setdefault` 记行号，而 `ast.walk` 是
**广度优先、不是源码顺序**，记下的不是最小行号 —— 实测漏掉了 line_goto_node
里 `rate_hz` 第 534 行读、557 行赋的真 bug。必须取 min()。
用法: useb4assign.py <file.py> [...]"""
import ast, io, sys, builtins

BI = set(dir(builtins))
rc = 0
for P in sys.argv[1:]:
    t = ast.parse(io.open(P, encoding='utf-8').read())
    bad = []
    for fn in [n for n in ast.walk(t)
               if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        asg, ld = {}, {}
        a = fn.args
        args = {x.arg for x in a.args} | {x.arg for x in a.kwonlyargs} | {x.arg for x in a.posonlyargs}
        if a.vararg: args.add(a.vararg.arg)
        if a.kwarg: args.add(a.kwarg.arg)
        nested = {f.name for f in ast.walk(fn)
                  if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef)) and f is not fn}
        for n in ast.walk(fn):
            if isinstance(n, ast.Name):
                d = asg if isinstance(n.ctx, ast.Store) else ld
                d[n.id] = min(d.get(n.id, n.lineno), n.lineno)     # ← 取最小行号
            elif isinstance(n, ast.ExceptHandler) and n.name:
                asg[n.name] = min(asg.get(n.name, n.lineno), n.lineno)
        for k, l in ld.items():
            if k in args or k in BI or k in nested or k not in asg:
                continue
            if l < asg[k]:
                bad.append((fn.name, k, l, asg[k]))
    if bad:
        rc = 1
        print('⚠️ %s' % P)
        for f, n, l, a2 in sorted(bad, key=lambda x: x[2]):
            print('    %s(): %s 第 %d 行被读，最早赋值在第 %d 行' % (f, n, l, a2))
    else:
        print('OK %s' % P)
sys.exit(rc)

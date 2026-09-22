#!/usr/bin/env python3
"""Python 静态自检。抓三类 py_compile 抓不到的错：

1. 按函数作用域的未定义名字——名字只在"别的函数"里局部 import/赋值，在本
   函数里用就是运行时 NameError。只看"模块里有没有定义过"抓不到这个。
2. 模块级未定义名字（漏 import）。
3. 被 create_subscription/create_timer 引用但不存在的 self._on_* 回调。

实际踩过的：tf2_ros 漏 import（节点起不来）、PoseStamped 只在另一个方法里
局部 import（订阅那行运行时 NameError）、批量替换代码时把 _on_range 方法
整个删掉（节点启动即 AttributeError）。三次都是 py_compile 全过、一跑就崩。

    python3 scripts/check_python_static.py src/contest_mission/contest_mission/*.py
"""
import ast
import builtins
import sys

SAFE = set(dir(builtins)) | {'__file__', '__name__', '__doc__', 'self', 'cls'}


def bound_names(node):
    """一个作用域自己绑定的名字（不递归进嵌套函数/类的体）。"""
    out = set()

    def walk(n):
        for child in ast.iter_child_nodes(n):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                out.add(child.name)
                continue
            if isinstance(child, (ast.Import, ast.ImportFrom)):
                for a in child.names:
                    out.add((a.asname or a.name).split('.')[0])
            elif isinstance(child, ast.Name) and isinstance(child.ctx, (ast.Store, ast.Del)):
                out.add(child.id)
            elif isinstance(child, ast.arg):
                out.add(child.arg)
            elif isinstance(child, ast.ExceptHandler) and child.name:
                out.add(child.name)
            elif isinstance(child, (ast.Global, ast.Nonlocal)):
                out.update(child.names)
            walk(child)

    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
        args = node.args
        for a in args.posonlyargs + args.args + args.kwonlyargs:
            out.add(a.arg)
        for a in (args.vararg, args.kwarg):
            if a:
                out.add(a.arg)
    walk(node)
    return out


def used_names(node):
    """一个作用域里读取的名字（不递归进嵌套函数/类的体）。"""
    out = set()

    def walk(n):
        for child in ast.iter_child_nodes(n):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                continue
            if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load):
                out.add(child.id)
            walk(child)
    walk(node)
    return out


def scopes(tree):
    """产出 (作用域节点, 该作用域可见的名字)。方法只看得到模块级 + 自己，看不到类体。"""
    stack = [(tree, bound_names(tree) | SAFE)]
    while stack:
        node, visible = stack.pop()
        yield node, visible
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                stack.append((child, visible | bound_names(child)))
            elif isinstance(child, ast.ClassDef):
                # 类体本身的语句单独查；类体里定义的名字对方法**不可见**
                # （Python 作用域规则），所以方法只继承外层 visible、不继承类体。
                stack.append((child, visible | bound_names(child)))
                for m in ast.iter_child_nodes(child):
                    if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        stack.append((m, visible | bound_names(m)))


def check(path):
    tree = ast.parse(open(path, encoding='utf-8').read())
    problems = []
    seen = set()
    for node, visible in scopes(tree):
        key = id(node)
        if key in seen:
            continue
        seen.add(key)
        undef = sorted(used_names(node) - visible)
        if undef:
            where = getattr(node, 'name', '<module>')
            problems.append(f'作用域 {where} 未定义名字={undef}')

    defined, referenced = set(), set()
    for n in ast.walk(tree):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            defined.add(n.name)
        elif isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == 'self':
            (referenced if isinstance(n.ctx, ast.Load) else defined).add(n.attr)
    missing = sorted(a for a in referenced if a.startswith('_on_') and a not in defined)
    if missing:
        problems.append(f'缺失回调={missing}')
    return problems


def main():
    fails = 0
    for path in sys.argv[1:]:
        probs = check(path)
        name = path.split('/')[-1]
        print(('★NG  ' if probs else 'OK   ') + f'{name:34s}' + '; '.join(probs))
        fails += bool(probs)
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()

# 选手代码目录

**顶层四个 `*_lite.py` 就是选手程序。** 它们短到能一眼看完，飞行编排之外的
东西全在 SDK 里。

| 文件 | 行数 | 任务 |
|---|---|---|
| `formation_lite.py` | 38 | 编队飞行 |
| `groundfire_lite.py` | 64 | 地面火情：侦查 → 取物资 → 投弹 → 编队返航 |
| `highrise_lite.py` | 107 | 高层火情：巡检拍摄 → 协同灭火 → 编队返回 |
| `mission_lite.py` | 197 | 综合：三轮连贯，编队 + 两种火情（随机、两轮不重复） |

## 从哪开始看

1. `formation_lite.py` —— 38 行，最短的一个完整双机任务，看懂它就懂了骨架
2. [`../src/contest_sdk/API参考.md`](../src/contest_sdk/API参考.md) —— 80 个方法分 12 类，顶部有"常用的 20 个"
3. `mission_lite.py` —— 最全的一个，三轮编排、两种火情、跨机握手都在里面

想知道某个动作底层到底怎么做的，去 `normal/` 看对应的详细版。

## 怎么跑

**所有操作都在这个目录里完成，不用切目录。**

```bash
./run.sh formation_lite        # 参数就是程序文件名，.py 可省，能 Tab 补全
./run.sh mission_lite --keep   # 跑完保留容器，便于翻日志
./stop.sh                      # 一键清理所有仿真容器
./stop.sh --check              # 只看现在还剩什么，不动手
```

自己写的程序放在本目录就能同样跑：`./run.sh 我的程序`。
`run.sh` 按名字先在本目录找，找不到再去 `normal/`。

## 飞之前 / 飞之后

```bash
# 飞之前：航线安全。算每一段离所有障碍的余量，用 goto_direct 时尤其要跑
python3 ../scripts/check_route.py --route "3,3 3,22 17,22 17,16"

# 飞之后：时间线
python3 ../scripts/timeline.py --events     # 两机事件并排，排编队死锁用
python3 ../scripts/timeline.py --problems   # 只看异常
```

静态自检不用手动跑——`run.sh` 每次起飞前会自动查一遍程序所在目录，不通过就不飞。

```bash
```

## 目录

```
.
├── run.sh / stop.sh        跑 / 清理，都在这儿
├── *_lite.py               四个选手程序
├── normal/                 详细版：同样四个任务，实现摊开写，每个决定都注明了为什么
├── archive/                旧场景 fire_drill_room 的示例，坐标系不同，跑不了
└── contestant_network.sh   共用网络配置，仓库外的 start_*.sh 靠它，别挪别改名
```

`normal/` 和 `*_lite.py` **行为完全相同**，用的也是同一组跨机事件名，所以可以
混搭（lite 长机配 normal 僚机也能飞）。

# 选手代码目录

你要动的只有 `我的任务.py`。其余文件分三类：**公共工具箱**、**四个考核程序**、
**四个简化版**。

## 从哪开始看

| 想做什么 | 看哪个 |
|---|---|
| 写自己的任务 | `我的任务.py`（固定文件名，不要改名） |
| 查 SDK 有哪些能力 | [`../src/contest_sdk/API参考.md`](../src/contest_sdk/API参考.md)，顶部有"常用的 20 个" |
| 看一个完整任务怎么写 | `formation_lite.py`（38 行）是最短的，`mission_lite.py` 是最全的 |
| 看某个动作的底层细节 | 对应的老版（`formation.py` 等），里面写满了踩坑记录 |

## 文件清单

**公共工具箱**（被下面的程序 import，别删）

- `utils.py` —— `land_or_confirm` / `transfer_to` / `descend_onto` / `fire_launcher`

**四个考核程序**（样题场景 `sample_room`，写得详细，每个决定都注明了为什么）

| 文件 | 任务 |
|---|---|
| `formation.py` | 编队飞行 |
| `groundfire.py` | 地面火情：侦查 → 取物资 → 投弹 → 编队返航 |
| `highrise.py` | 高层火情：巡检拍摄 → 协同灭火 → 编队返回 |
| `mission.py` | 综合：三轮连贯，编队 + 两种火情（火情随机、两轮不重复） |

**四个简化版**（`*_lite.py`，行为跟上面**完全相同**，实现搬进了 SDK）

| 文件 | 行数 | 对应老版 |
|---|---|---|
| `formation_lite.py` | 38 | 841 |
| `groundfire_lite.py` | 64 | 284 |
| `highrise_lite.py` | 107 | 747 |
| `mission_lite.py` | 197 | 617 |

两版用的是**同一组跨机事件名**，所以可以混搭（lite 长机配老版僚机也能飞）。

**启动脚本**

- `运行仿真.sh` / `运行仿真.bat` / `运行真机.sh` —— 跑 `我的任务.py` 用
- `contestant_network.sh` —— 网络配置，容器启动时要用，别动

## 怎么跑

跑自己的 `我的任务.py`：

```bash
bash 运行仿真.sh
```

跑上面八个程序里的任何一个，用仓库根的一键脚本（参数就是文件名）：

```bash
cd ../一键仿真
./run.sh formation_lite        # .py 可省，能 Tab 补全
./run.sh mission
./stop.sh                      # 一键清理
```

## 飞之前先跑这两样

```bash
# 航线安全：算每一段离所有障碍的余量。打算用 goto_direct 时尤其要跑
python3 ../scripts/check_route.py --route "3,3 3,22 17,22 17,16"

# 静态自检：抓"调了不存在的函数"这类 py_compile 抓不到、一飞就崩的错
python3 ../scripts/check_python_static.py 我的任务.py
```

飞完了看时间线：

```bash
python3 ../scripts/timeline.py --events     # 两机事件并排，排编队死锁用
python3 ../scripts/timeline.py --problems   # 只看异常
```

## archive/

旧场景 `fire_drill_room` 的示例，**坐标系跟现在的样题场景不一样，不能直接跑**。
留着是因为里面有些写法仍有参考价值。见 [`archive/README.md`](archive/README.md)。

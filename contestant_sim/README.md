# 选手代码目录

**顶层四个 `*_lite.py` 就是选手程序。** 它们短到能一眼看完，飞行编排之外的
东西全在 SDK 里。

| 文件 | 行数 | 任务 |
|---|---|---|
| `formation_lite.py` | 38 | 编队飞行 |
| `groundfire_lite.py` | 64 | 地面火情：侦查 → 取物资 → 投弹 → 编队返航 |
| `highrise_lite.py` | 107 | 高层火情：巡检拍摄 → 协同灭火 → 编队返回 |
| `mission_lite.py` | 197 | 综合：三轮连贯，编队 + 两种火情（随机、两轮不重复） |

## 参数不用记：让编辑器提示

### 怎么开起来

**1. 装 VS Code，再装 Python 扩展**（扩展市场搜 `Python`，微软出的那个，
会连带装上 Pylance——真正提供补全的是它）。

**2. 用 VS Code 打开 `docker_sim` 仓库目录，或者直接打开本目录**，两个都行，
配置文件两边都放了。

**3. 打开任意一个 `*_lite.py`，右下角状态栏确认解释器选的是 python3**
（点一下可以切换；选哪个 python3 都行，这里只做静态分析、不真的运行）。

### 效果

```python
def recon(sdk: DroneSDK):
    sdk.          # ← 打到这个点，自动弹出全部方法列表
```

- **悬停在方法名上** → 浮出完整签名、一句话说明、每个参数的意思、踩过的坑
- **打完左括号** → 参数提示条跟着出来，当前打到第几个参数会高亮
- **参数名写错** → 当场红波浪线，比如 `sdk.takeoff(bogus=1)` 报
  `No parameter named "bogus"`
- **传错类型** → 同样当场标出来

### 两个前提，缺一个就没有提示

**① 函数参数必须标注类型。** 这是最容易漏的一条：

```python
def recon(sdk):              # ✗ 编辑器不知道 sdk 是什么，打 sdk. 什么都没有
def recon(sdk: DroneSDK):    # ✓
```

**② 工作区根目录要有配置文件。** 配置只在**工作区根目录**生效，所以仓库根和
本目录各放了一份 `pyrightconfig.json` + `.vscode/settings.json`。开到别的
目录（比如只开 `API参考/`）就会报 `"DroneSDK" is unknown import symbol`，
那说明开错目录了，不是代码有问题。

### 为什么宿主机没装 ROS 也能用

`contest_sdk` 装在 docker 镜像里，宿主机上确实没有。但**静态分析只读源码、
不执行代码**，配置里的 `extraPaths` 指向仓库里的 SDK 源码
（`src/contest_sdk`），所以不需要宿主机能真正 import 它。

### 命令行也能查

```bash
pip install pyright        # 第一次装
cd contestant_sim && pyright        # 查一遍自己的程序
```

`run.sh` 起飞前也会顺带跑一次（装了才跑，**只提示不拦飞行**）。

## 从哪开始看

1. `formation_lite.py` —— 38 行，最短的一个完整双机任务，看懂它就懂了骨架
2. `API参考/<程序名>.md`（或 `.docx`）—— **每个程序一份**，只列它用到的 API，按程序里出现的先后排、标了行号
3. `mission_lite.py` —— 最全的一个，三轮编排、两种火情、跨机握手都在里面

想知道某个动作底层到底怎么做的，去 `API参考/<程序名>.md` 查那个 API 的签名和说明。

## 怎么跑

**所有操作都在这个目录里完成，不用切目录。**

```bash
./run.sh formation_lite        # 参数就是程序文件名，.py 可省，能 Tab 补全
./run.sh mission_lite --keep   # 跑完保留容器，便于翻日志
./stop.sh                      # 一键清理所有仿真容器
./stop.sh --check              # 只看现在还剩什么，不动手

./run_test.sh t4               # 仿真里跑单机测试（只起一架），见下
./run_test.sh                  # 不给就列出可选的测试

./stop_all.sh                  # 关掉**一切**仿真相关栈：stop.sh 全部 + 声光栈
                               # + 选手调试容器（后两个只在它们处于仿真域 21
                               # 时才关；挂在真机域 20 的是别人在用，不动）
./stop_all.sh --check          # 只看会动什么
```

> 网页栈（`gcs-gcs-1` / `gcs-backend-1`）`stop_all.sh` **不关** —— `run.sh`
> 本来也不起它，它是仿真/真机共用的观察面。要关用 `../shell/stop_gcs.sh`。

自己写的程序放在本目录就能同样跑：`./run.sh 我的程序`。
`run.sh` 只在本目录按名字找。

## 单机测试（`单机测试/`，用 `run_test.sh` 跑）

五个单机专项测试，和 `../contestant_real/单机测试/` 下的**逐字节相同** ——
坐标各取自己目录的 `venue.py`。先在仿真里把逻辑跑通，再上真机。

```bash
./run_test.sh t1              # 物资抓取（默认 NX02，带夹爪）
./run_test.sh t3 --n          # 高层火情，用 N 点看 1# 楼
./run_test.sh t4 --nx02       # 避障，指定用 NX02
./run_test.sh t1 --restart    # 强制重起仿真（默认复用已在跑的）
```

**为什么不能用 `run.sh` 跑它们**：`run.sh` 无条件起两架（`sim_leader` +
`sim_follower` 跑同一个程序），而这五个测试都是
`DroneSDK.run(leader=test, follower=test)` —— 两架会同时飞向同一个点
（t1 两架都去物资点，t2/t3 都去同一个火情位）。不是安全问题，是测试本身
没意义。所以单机测试有自己的运行器。

跟真机 `run_test.sh` 的区别：仿真栈归本脚本管（默认复用，`--restart` 才重起），
就绪判据等 PX4 `Ready for takeoff` + flight-stack 节点，而且不需要
`vision_real.sh` 那一步 —— 仿真的检测节点随 flight-stack 一起起。

## 飞之前 / 飞之后

```bash
# 飞之前：航线安全。算每一段离所有障碍的余量，用 goto_direct 时尤其要跑
python3 tools/check_route.py --route "3,3 3,22 17,22 17,16"

# 飞之后：时间线
python3 tools/timeline.py --events     # 两机事件并排，排编队死锁用
python3 tools/timeline.py --problems   # 只看异常
```

静态自检不用手动跑——`run.sh` 每次起飞前会自动查一遍程序所在目录，不通过就不飞。

```bash
```

## 目录

```
.
├── run.sh / stop.sh   跑 / 清理
├── *_lite.py          四个选手程序，在这儿改
├── tools/             飞前查航线、飞后看时间线
├── API参考/           每个程序一份（.md + .docx），自动生成，不要手改
└── README.md          本文件
```

这个目录里**只有选手要用的东西**。仿真设施（docker-compose、监视、裁判、
SDK 源码）都在上一层仓库里，不用关心。

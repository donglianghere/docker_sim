# normal —— 详细版

跟上一层四个 `*_lite.py` **行为完全相同**，区别只在于：lite 版把编排之外的
实现搬进了 SDK，这里是摊开写的。

| 文件 | 行数 | 对应的 lite |
|---|---|---|
| `formation.py` | 841 | `formation_lite.py`（38） |
| `groundfire.py` | 284 | `groundfire_lite.py`（64） |
| `highrise.py` | 747 | `highrise_lite.py`（107） |
| `mission.py` | 617 | `mission_lite.py`（197） |
| `utils.py` | —— | 公共工具箱，只有这几个程序用 |

**什么时候该看这里**：想知道某个动作底层到底在做什么、以及为什么要这么做。
这些文件里写满了实测记录——哪个参数是怎么定出来的、哪种写法试过但不行、
某个判据为什么不能再紧一点。lite 版为了短，这些都压进 SDK 的 docstring 了。

两版用的是同一组跨机事件名，可以混搭。跑法一样：

```bash
cd ../../一键仿真
./run.sh mission          # normal 版
./run.sh mission_lite     # lite 版
```

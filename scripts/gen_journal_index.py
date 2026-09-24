#!/usr/bin/env python3
"""给 DEBUG_JOURNAL.md 生成主题索引（2026-09-24新增）。

为什么需要：日志到2026-09已经736条/36243行/158万字，而且**条目长度是均匀的**
（中位1819字符，最长的10条只占6%），也就是说"太长"不是某几条啰嗦，是条目数量
本身。原文一个字都不该删——CLAUDE.md要求所有提问经验都记在这里，而且价值恰恰
在细节（某个函数忘了rstrip、udpsink往不可达地址会阻塞连累同管线的appsink），
删成一句话结论，下次遇到就复现不了推理过程；再加上里面有多条是**订正前一条**的
（「纠错：192.168.2.103就是本机」推翻了09-19那条），删掉任一半另一半就变误导。

所以这里只加一层索引，不动原文：解析出每条的日期/标题/行号，按关键词打主题标签，
按主题重新组织成一份目录。日志增长后重跑一次即可（幂等，直接覆盖索引文件）。

用法：
    cd docker_sim && python3 scripts/gen_journal_index.py
"""
import os
import re
import time
from collections import Counter

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "DEBUG_JOURNAL.md")
OUT = os.path.join(REPO, "DEBUG_JOURNAL_INDEX.md")

# 主题 -> 关键词。关键词取项目里实际在用的写法（含缩写和中英文混写），
# 命中在**标题**里权重更高(×5)，正文命中按次数累计但封顶，避免一条长文
# 因为反复提到某个词就被硬塞进不相干的主题。
TOPICS = {
    # ⚠️ 这里刻意**不放** "docker"/"容器"/"build" 这几个词：第一版放了，结果
    # 736条里有338条(46%)被打上"容器与镜像"标签——这个项目每条记录几乎都会
    # 提一嘴docker，通用词在这里没有任何区分度，等于没分类。只留真正专属的。
    "容器与镜像": ["dockerfile", "镜像", "entrypoint", "force-recreate", "colcon",
                   "docker compose", "recreate", "构建缓存", "分层"],
    "仿真世界(Gazebo)": ["gazebo", "sim-world", "sdf", "world", "仿真场景", "模型加载",
                         "RTF", "渲染", "iris"],
    "规划器": ["ego_planner", "ego-planner", "mighty", "planner", "规划", "轨迹",
               "A*", "bspline", "replan", "避障", "膨胀", "目标点", "goal",
               "占据栅格", "occupancy", "grid_map", "航点"],
    "控制器": ["px4ctrl", "so3ctrl", "pt4ctrl", "控制器", "推力", "悬停", "PID",
               "thrust", "增益", "偏航", "yaw", "升力", "过冲", "跟踪误差",
               "控制律"],
    "定位与SLAM": ["dlio", "point_lio", "point-lio", "fast_lio", "slam", "里程计",
                   "odom", "uwb", "定位", "标定", "外参", "漂移", "点云", "建图",
                   "global_mapper", "关键帧", "imu", "发散"],
    "飞控与PX4": ["px4", "mavros", "飞控", "ekf2", "解锁", "arm", "offboard",
                  "ulg", "失控保护", "降落"],
    "视觉与相机": ["相机", "camera", "yolo", "apriltag", "二维码", "qr", "视觉",
                   "图像", "mjpeg", "推流", "tensorrt", "engine"],
    "地面站GCS": ["地面站", "gcs", "网页", "前端", "rosbridge", "面板", "浏览器",
                  "index.html"],
    "网络与DDS": ["dds", "cyclonedds", "fastdds", "网段", "广播", "心跳", "组播",
                  "ROS_DOMAIN", "网卡", "丢包"],
    "真机与硬件": ["真机", "jetson", "orin", "nx01", "nx02", "雷达", "mid360",
                   "舵机", "电池", "jetpack", "串口"],
    "选手SDK与任务": ["选手", "contestant", "contest_sdk", "sdk", "任务脚本",
                      "声光", "编队", "火情", "灭火", "比赛", "大赛"],
    "流程与同步": ["同步", "rsync", "git", "提交", "备份", "归档", "分叉", "快照",
                   "文档", "记录"],
    # 下面两类是第一版漏掉的——当时98条落进"其它"，翻出来一看全是有明确
    # 主题的：RViz显示调整、tmux监控窗格、QoS不匹配/rclcpp陷阱这类ROS2机制
    # 问题。不是"杂项"，是关键词没覆盖。
    "可视化与调试工具": ["rviz", "tmux", "可视化", "窗格", "显示", "点云颜色",
                         "rosbag", "bag", "终端", "日志打印", "探针"],
    "ROS2机制": ["qos", "rclcpp", "rclpy", "话题", "tf", "回调", "executor",
                 "消息类型", "时间戳", "sim time", "命名空间", "节点"],
}
MAX_TAGS = 3


def parse_entries(text):
    """切成条目并记住每条在原文里的起始行号（1-based）。"""
    entries = []
    line_no = 1
    buf, buf_start = None, None
    for line in text.split("\n"):
        if line.startswith("## "):
            if buf is not None:
                entries.append((buf_start, "\n".join(buf)))
            buf, buf_start = [line], line_no
        elif buf is not None:
            buf.append(line)
        line_no += 1
    if buf is not None:
        entries.append((buf_start, "\n".join(buf)))
    return entries


def tag(entry):
    title = entry.split("\n", 1)[0]
    low_t, low_b = title.lower(), entry.lower()
    scores = {}
    for topic, keys in TOPICS.items():
        s = 0
        for k in keys:
            kl = k.lower()
            if kl in low_t:
                s += 10          # 标题命中 = 这条就是讲它的，强信号
            s += min(low_b.count(kl), 5)   # 正文命中封顶，防长文刷分
        if s:
            scores[topic] = s
    top = sorted(scores.items(), key=lambda kv: -kv[1])[:MAX_TAGS]
    # 阈值10 ≈ "标题里出现过，或正文反复出现"。低于这个的是顺带提一嘴，
    # 不该让它把条目塞进不相干的主题里。
    return [t for t, s in top if s >= 10] or ["其它"]


def main():
    text = open(SRC, encoding="utf-8").read()
    entries = parse_entries(text)
    date_re = re.compile(r"20\d\d-\d\d-\d\d")
    rows, by_topic, by_month = [], {}, Counter()
    cur_date = "????-??-??"
    for start, e in entries:
        title = e.split("\n", 1)[0][3:].strip()
        m = date_re.search(title)
        if m:
            cur_date = m.group(0)
        tags = tag(e)
        rows.append((cur_date, title, start, tags))
        by_month[cur_date[:7]] += 1
        for t in tags:
            by_topic.setdefault(t, []).append((cur_date, title, start))

    out = []
    out.append("# DEBUG_JOURNAL 主题索引\n")
    out.append(f"> 自动生成，**不要手改**。重新生成：`cd docker_sim && python3 scripts/gen_journal_index.py`\n")
    out.append(f"> 源文件 `DEBUG_JOURNAL.md`：{len(entries)} 条 / {text.count(chr(10)):,} 行 / "
               f"{len(text):,} 字符　生成时间：{time.strftime('%Y-%m-%d %H:%M')}\n")
    out.append("\n**怎么用**：每行末尾的 `L1234` 是原文行号，`sed -n '1234,+60p' DEBUG_JOURNAL.md` "
               "直接看全文。一条记录最多挂 3 个主题，所以会在多处出现。\n")

    out.append("\n## 分布\n")
    out.append("| 主题 | 条数 | | 月份 | 条数 |")
    out.append("|---|---:|---|---|---:|")
    tl = sorted(by_topic.items(), key=lambda kv: -len(kv[1]))
    ml = sorted(by_month.items())
    for i in range(max(len(tl), len(ml))):
        a = f"{tl[i][0]} | {len(tl[i][1])}" if i < len(tl) else " | "
        b = f"{ml[i][0]} | {ml[i][1]}" if i < len(ml) else " | "
        out.append(f"| {a} |  | {b} |")

    for topic, items in tl:
        out.append(f"\n## {topic}（{len(items)} 条）\n")
        for d, t, ln in items:
            t = t if len(t) <= 78 else t[:77] + "…"
            out.append(f"- `{d}` {t}　`L{ln}`")

    open(OUT, "w", encoding="utf-8").write("\n".join(out) + "\n")
    print(f"已生成 {OUT}：{len(entries)} 条 → {len(tl)} 个主题")
    for t, items in tl:
        print(f"    {len(items):>4}  {t}")


if __name__ == "__main__":
    main()

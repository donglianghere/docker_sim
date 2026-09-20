#!/usr/bin/env python3
"""px4ctrl悬停振荡专项排查：把一段rosbag2录制转成CSV，供宿主机离线做FFT/
相关性分析——不在容器里直接画图/算频谱，只导出数值，分析代码留在宿主机
（跟ulog那边的分析习惯一致，方便两份数据交叉核对）。

背景：2026-09-10排查px4ctrl真机悬停振荡时，发现两个从ulog反推数据的坑：
①ulog不记录用的是哪个板外控制器（px4ctrl/pt4ctrl/so3ctrl三选一），必须
  用offboard_control_mode.position/attitude两个bool字段才能可靠区分；
②PX4固件日志系统对estimator_aid_src_*这类话题统一按固定500ms/2Hz抽样
  记录（见logged_topics.cpp的kEKFVerboseIntervalMilliseconds），事后
  反推不出真实的EKF2外部视觉输入频率。
这个脚本直接从rosbag2里读原始消息（真实到达时刻+真实内容），不依赖PX4
日志的抽样限制，专门用来补上这两个坑——同时把px4ctrl自己的调试话题
（debugPx4ctrl：des_v/fb_a/des_a/des_q/err_axisang等，px4ctrl级联环
每一层的输入输出）也导出来，能直接定位振荡是在位置环/速度环/姿态环
哪一层产生的，不用像ulog那样只能间接对比setpoint跟actual。

用法（必须在能识别quadrotor_msgs/mavros_msgs等自定义消息类型的容器里跑，
比如flight-stack-hw-1，跟record_rosbag.sh要求source同一批workspace一样
的道理——本机（宿主机）大概率没装这些自定义msg包，直接在宿主机跑会在
deserialize自定义类型时失败）：
  docker cp scripts/analyze_px4ctrl_bag.py docker_sim-flight-stack-hw-1:/tmp/
  docker exec docker_sim-flight-stack-hw-1 bash -c '
    source /opt/ros/humble/setup.bash
    source /opt/mavros_ws/install/setup.bash 2>/dev/null || true
    for d in /opt/*/install/setup.bash; do source "$d" 2>/dev/null || true; done
    python3 /tmp/analyze_px4ctrl_bag.py /logs/incidents/<时间戳>_px4ctrl悬停振荡专项测试/bag_xxx /tmp/px4ctrl_csv
  '
  docker cp docker_sim-flight-stack-hw-1:/tmp/px4ctrl_csv ./px4ctrl_csv_真机拉回来的
输出：<out_dir>/<话题名（/换成__）>.csv，每行 t_sec(相对bag起始),<字段...>。
"""
import sys
import os
import csv

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# 每个话题只挑分析真正用得上的数值字段，不是整条消息全转CSV（省事、
# CSV也小很多）。字段名跟消息定义里的路径一致，嵌套字段用get_nested取。
FIELD_MAP = {
    'dlio/odom_node/odom': [
        ('px', 'pose.pose.position.x'), ('py', 'pose.pose.position.y'), ('pz', 'pose.pose.position.z'),
        ('vx', 'twist.twist.linear.x'), ('vy', 'twist.twist.linear.y'), ('vz', 'twist.twist.linear.z'),
        ('stamp', 'header.stamp'),
    ],
    'mavros/vision_pose/pose_cov': [
        ('px', 'pose.pose.position.x'), ('py', 'pose.pose.position.y'), ('pz', 'pose.pose.position.z'),
        ('stamp', 'header.stamp'),
    ],
    'mavros/imu/data': [
        ('qx', 'orientation.x'), ('qy', 'orientation.y'), ('qz', 'orientation.z'), ('qw', 'orientation.w'),
        ('gx', 'angular_velocity.x'), ('gy', 'angular_velocity.y'), ('gz', 'angular_velocity.z'),
    ],
    'mavros/setpoint_raw/target_attitude': [
        ('qx', 'orientation.x'), ('qy', 'orientation.y'), ('qz', 'orientation.z'), ('qw', 'orientation.w'),
        ('thrust', 'thrust'), ('type_mask', 'type_mask'),
    ],
    'debugPx4ctrl': [
        ('des_v_x', 'des_v_x'), ('des_v_y', 'des_v_y'), ('des_v_z', 'des_v_z'),
        ('fb_a_x', 'fb_a_x'), ('fb_a_y', 'fb_a_y'), ('fb_a_z', 'fb_a_z'),
        ('des_a_x', 'des_a_x'), ('des_a_y', 'des_a_y'), ('des_a_z', 'des_a_z'),
        ('des_q_x', 'des_q_x'), ('des_q_y', 'des_q_y'), ('des_q_z', 'des_q_z'), ('des_q_w', 'des_q_w'),
        ('des_thr', 'des_thr'), ('hover_percentage', 'hover_percentage'),
        ('thr_scale_compensate', 'thr_scale_compensate'), ('voltage', 'voltage'),
        ('err_axisang_x', 'err_axisang_x'), ('err_axisang_y', 'err_axisang_y'),
        ('err_axisang_z', 'err_axisang_z'), ('err_axisang_ang', 'err_axisang_ang'),
        ('fb_rate_x', 'fb_rate_x'), ('fb_rate_y', 'fb_rate_y'), ('fb_rate_z', 'fb_rate_z'),
    ],
    'mavros/local_position/pose': [
        ('px', 'pose.position.x'), ('py', 'pose.position.y'), ('pz', 'pose.position.z'),
    ],
    'mavros/state': [
        ('armed', 'armed'), ('mode', 'mode'), ('connected', 'connected'),
    ],
}


def get_nested(obj, path):
    for part in path.split('.'):
        obj = getattr(obj, part)
    if hasattr(obj, 'sec') and hasattr(obj, 'nanosec'):
        return obj.sec + obj.nanosec / 1e9
    return obj


def match_suffix(topic_name, key):
    return topic_name.endswith('/' + key) or topic_name == key


def main():
    if len(sys.argv) < 3:
        print(f"用法: {sys.argv[0]} <bag_dir> <out_dir>", file=sys.stderr)
        sys.exit(1)
    bag_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    msg_classes = {}
    for name, type_str in type_map.items():
        try:
            msg_classes[name] = get_message(type_str)
        except Exception as e:
            print(f"跳过 {name} ({type_str})：{e}", file=sys.stderr)

    writers = {}
    files = {}
    t_first = None
    counts = {}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_classes:
            continue
        matched_key = None
        for key in FIELD_MAP:
            if match_suffix(topic, key):
                matched_key = key
                break
        if matched_key is None:
            continue

        if t_first is None:
            t_first = t_ns
        t_sec = (t_ns - t_first) / 1e9

        msg = deserialize_message(data, msg_classes[topic])
        fields = FIELD_MAP[matched_key]
        row = [t_sec]
        for _, path in fields:
            try:
                row.append(get_nested(msg, path))
            except Exception:
                row.append('')

        if topic not in writers:
            safe_name = topic.strip('/').replace('/', '__') + '.csv'
            fpath = os.path.join(out_dir, safe_name)
            f = open(fpath, 'w', newline='')
            files[topic] = f
            w = csv.writer(f)
            w.writerow(['t_sec'] + [name for name, _ in fields])
            writers[topic] = w
            counts[topic] = 0

        writers[topic].writerow(row)
        counts[topic] += 1

    for f in files.values():
        f.close()

    print("== 导出完成 ==")
    for topic, n in counts.items():
        print(f"  {topic}: {n} 条")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""量仿真雷达对 (7,2) 那根 r=0.25 圆柱的实际打点情况。

背景：2026-09-28 编队飞行时长机直穿这根圆柱坠机，规划器日志全程
no_collision_segment（地图里根本没这个障碍）。怀疑是 Gazebo ray 插件的
**水平角分辨率**太粗：<horizontal><samples>100</samples> 覆盖 360°，
每条射线 3.6°，7 米外相邻射线间隔 0.44 m，而圆柱直径才 0.5 m。

逐帧统计（按到圆柱的距离分档）：
  n_cyl   圆柱盒子里的点数（z 0.2~5 m）
  n_band  其中落在"跟飞机同高 ±0.1 m"的点数——只有这些点才会在飞机所在
          高度层形成障碍格（grid_map 竖直只膨胀 ±0.1 m）
"""
import sys, time, csv
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

NS = sys.argv[1]; DUR = float(sys.argv[2]); OUT = sys.argv[3]
PAD = (float(sys.argv[4]), float(sys.argv[5]))     # 本机起降点世界坐标
CYL_W = (7.0, 2.0)                                  # 圆柱世界坐标
CYL = (CYL_W[0] - PAD[0], CYL_W[1] - PAD[1])        # 换到本机局部系
HALF = 0.6            # 盒子半宽：圆柱 r=0.25，放宽到 0.6 把边缘点也算上
ZLO, ZHI = 0.2, 5.0

rclpy.init(); n = Node('cyl_diag')
st = {'pos': None}
f = open(OUT, 'w', newline=''); w = csv.writer(f)
w.writerow(['t', 'd_cyl', 'x', 'y', 'z', 'n_pts', 'n_cyl', 'n_band', 'cyl_zmin', 'cyl_zmax'])

def on_odom(m):
    p = m.pose.pose.position
    st['pos'] = (p.x, p.y, p.z)

def on_cloud(m):
    if st['pos'] is None: return
    arr = point_cloud2.read_points_numpy(m, field_names=('x','y','z'), skip_nans=True)
    if arr.shape[0] == 0: return
    x, y, z = st['pos']
    sel = ((np.abs(arr[:,0]-CYL[0]) < HALF) & (np.abs(arr[:,1]-CYL[1]) < HALF)
           & (arr[:,2] > ZLO) & (arr[:,2] < ZHI))
    zs = arr[sel, 2]
    n_band = int((np.abs(zs - z) <= 0.10).sum()) if zs.size else 0
    w.writerow(['%.2f' % time.time(), '%.2f' % float(np.hypot(x-CYL[0], y-CYL[1])),
                '%.2f' % x, '%.2f' % y, '%.2f' % z, int(arr.shape[0]), int(zs.size), n_band,
                '%.2f' % (zs.min() if zs.size else 0.0),
                '%.2f' % (zs.max() if zs.size else 0.0)])

n.create_subscription(Odometry, f'/{NS}/dlio/odom_node/odom', on_odom, 10)
n.create_subscription(PointCloud2, f'/{NS}/dlio/odom_node/pointcloud/deskewed', on_cloud, 10)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
f.close(); print('done'); rclpy.shutdown()

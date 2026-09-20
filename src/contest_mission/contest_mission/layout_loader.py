#!/usr/bin/env python3
"""读取fire_drill_room_layout.yaml的小工具——不是ROS节点。

阶段4的`pillar_detector_node.py`已经有一份读这份yaml的私有函数
（`_load_layout_geometry()`，只取它需要的房间/道具尺寸），这次阶段7.3
`scenario_reset_node.py`需要读更完整的内容（每根立柱/障碍物/物资点的
坐标+尺寸，用来做随机化重置），没有回头改阶段4那份已经验证过的代码
（避免不必要地牵动已经跑通的模块），单独提供这一份读取更完整的版本。
"""
from ament_index_python.packages import get_package_share_directory
import yaml


def load_layout() -> dict:
    share_dir = get_package_share_directory('contest_mission')
    with open(f'{share_dir}/config/fire_drill_room_layout.yaml') as f:
        return yaml.safe_load(f)

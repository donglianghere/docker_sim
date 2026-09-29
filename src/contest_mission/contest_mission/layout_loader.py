#!/usr/bin/env python3
"""读取fire_drill_room_layout.yaml的小工具——不是ROS节点。

阶段4的`pillar_detector_node.py`已经有一份读这份yaml的私有函数
（`_load_layout_geometry()`，只取它需要的房间/道具尺寸），这次阶段7.3
`scenario_reset_node.py`需要读更完整的内容（每根立柱/障碍物/物资点的
坐标+尺寸，用来做随机化重置），没有回头改阶段4那份已经验证过的代码
（避免不必要地牵动已经跑通的模块），单独提供这一份读取更完整的版本。
"""
import os

from ament_index_python.packages import get_package_share_directory
import yaml

# WORLD_ENV -> 布局文件。2026-09-29 新增样题版场景之后，同一个容器里可能跑
# 两套坐标系完全不同的场景（fire_drill_room 原点在房间中心、坐标有正有负；
# sample_room 原点在房间西南角、坐标全正），读错一份不会报错、只会让所有
# 道具坐标整体偏十几米，非常难查——所以按 WORLD_ENV 选，选不到就明确报错，
# 不要"默认回退到 fire_drill_room"那种会静默跑偏的做法。
_LAYOUT_BY_WORLD = {
    'fire_drill_room': 'fire_drill_room_layout.yaml',
    'sample_room': 'sample_room_layout.yaml',
}


def layout_filename(world_env: str = None) -> str:
    """当前 WORLD_ENV 对应的布局文件名。"""
    w = world_env or os.environ.get('WORLD_ENV', 'fire_drill_room')
    if w not in _LAYOUT_BY_WORLD:
        raise KeyError(
            f"WORLD_ENV='{w}' 没有对应的布局文件。已知：{sorted(_LAYOUT_BY_WORLD)}。"
            f"新增场景要同时在这里登记，否则读到的会是别的场景的坐标。")
    return _LAYOUT_BY_WORLD[w]


def load_layout(world_env: str = None) -> dict:
    share_dir = get_package_share_directory('contest_mission')
    with open(f'{share_dir}/config/{layout_filename(world_env)}') as f:
        return yaml.safe_load(f)

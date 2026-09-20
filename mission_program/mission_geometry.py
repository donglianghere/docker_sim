"""这次任务专属的纯几何计算函数（对应清单D1节"立柱确定性编号排序函数"
"长距离分段插值函数"）——纯函数，跟ROS无关，不需要收进`contest_sdk`
（2.1节第8条已经说明：这次任务专属的逻辑，不是复用现有`contest_
mission`文件，没有跨容器import的风险，写在全流程程序自己的helper
文件里就够）。
"""
import math
from typing import List, Sequence, Tuple


def sort_pillars_deterministic(
    pillar_xy_list: Sequence[Tuple[float, float]]
) -> List[Tuple[float, float]]:
    """按"先比X坐标、X相同再比Y坐标"字典序排序（方案4.3.2节）。

    真机场景只给3个坐标点、不给编号——这个排序结果的下标就是程序自己
    认定的编号（0/1/2），不依赖外部给的标签（比如仿真配置文件里原有的
    "1#/2#/3#"字样，那是仿真侧自己起的名字，跟这里排序出来的编号不需要
    对上，两者是两套独立的编号规则，只需要保证同一份坐标输入排序结果
    稳定可复现）。
    """
    return sorted(pillar_xy_list, key=lambda xy: (xy[0], xy[1]))


def select_nearest_pillar_index(
    current_xy: Tuple[float, float],
    sorted_pillar_xy_list: Sequence[Tuple[float, float]],
    visited_indices: Sequence[int] = (),
    tie_epsilon_m: float = 1e-6,
) -> int:
    """从还没绕飞过的候选立柱里选"离当前位置最近"的那一个，距离相等
    （差值小于`tie_epsilon_m`）时固定选编号小的那个（方案4.3.2节给的
    平局兜底规则，1#/3#到地面火情点距离精确相等√52≈7.211米就是这条
    规则要处理的具体场景）。

    Args:
        current_xy: 当前位置局部坐标。
        sorted_pillar_xy_list: `sort_pillars_deterministic()`排好序的
            立柱坐标列表，下标即编号。
        visited_indices: 已经绕飞过的立柱编号，这次选择要跳过。
        tie_epsilon_m: 判定"距离相等"的容差，默认1微米量级，只是防
            浮点误差，不是真的允许出现明显的距离差异也算平局。

    Returns:
        选中的立柱编号（`sorted_pillar_xy_list`的下标）。

    Raises:
        ValueError: 所有候选立柱都已经绕飞过（没有可选的了）。
    """
    best_index = None
    best_distance = None
    for index, xy in enumerate(sorted_pillar_xy_list):
        if index in visited_indices:
            continue
        distance = math.hypot(xy[0] - current_xy[0], xy[1] - current_xy[1])
        if best_distance is None or distance < best_distance - tie_epsilon_m:
            best_index = index
            best_distance = distance
        # 距离在tie_epsilon_m容差内算平局——不覆盖，保留编号更小的那个
        # （前面已经按sorted_pillar_xy_list从小到大的下标顺序遍历，先
        # 遇到的编号天然更小，遇到平局时"不覆盖"就等价于"取编号小的"）。
    if best_index is None:
        raise ValueError(
            f"没有可选的立柱了（候选{len(sorted_pillar_xy_list)}根，"
            f"已绕飞{sorted(visited_indices)}）")
    return best_index


def interpolate_waypoints(
    start_xyz: Tuple[float, float, float],
    end_xyz: Tuple[float, float, float],
    max_step_m: float,
) -> List[Tuple[float, float, float]]:
    """按最大步长把一段长距离目标拆成多个子航点（方案4.2节，应对
    `ego_planner`长距离目标容易卡在`GEN_NEW_TRAJ`重规划死循环这个坑，
    不是根治，是应对措施）。

    返回值**不包含`start_xyz`本身**（调用方当前已经在这个点，不需要
    "飞往自己所在的位置"这一步），但包含`end_xyz`（最后一个子航点精确
    等于终点，不会因为除不尽而有误差）。

    Args:
        start_xyz: 起点（局部坐标，只用来算距离，不出现在返回值里）。
        end_xyz: 终点（局部坐标）。
        max_step_m: 每段最大步长（米），必须大于0。

    Returns:
        子航点列表，长度至少1（起点终点距离为0时也会返回`[end_xyz]`
        这一个点，不返回空列表——调用方不需要额外判断"这段要不要飞"）。
    """
    if max_step_m <= 0:
        raise ValueError(f"max_step_m必须大于0，收到{max_step_m}")

    dx = end_xyz[0] - start_xyz[0]
    dy = end_xyz[1] - start_xyz[1]
    dz = end_xyz[2] - start_xyz[2]
    total_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    if total_distance <= max_step_m:
        return [end_xyz]

    num_segments = math.ceil(total_distance / max_step_m)
    return [
        (
            start_xyz[0] + dx * i / num_segments,
            start_xyz[1] + dy * i / num_segments,
            start_xyz[2] + dz * i / num_segments,
        )
        for i in range(1, num_segments + 1)
    ]

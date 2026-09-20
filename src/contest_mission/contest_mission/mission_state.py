#!/usr/bin/env python3
"""任务状态话题的约定——只是一份契约定义，不是ROS节点。

对应《2026大赛任务系统开发执行方案.md》阶段7.1。用`std_msgs/String`+
约定字符串枚举，不新建`.msg`接口包（`contest_mission`是纯`ament_python`
包，生成自定义`.msg`需要额外一个`ament_cmake`接口包，为一个枚举字符串
不值得付这个编译成本，方案原文本身也是这么建议的）。

**这次会话没有实现"谁来发布这个话题"**——按用户要求，阶段7.4（把
阶段2~6拼成完整任务状态机、真正会持续publish这个话题的节点）推迟到
阶段8 SDK做出来之后再写（用SDK写会比现在用原始ROS2接口写更干净，
不用写两遍）。这个模块现在的作用：
1. 给未来发布方（阶段8 SDK内部、或阶段10的参考选手方案）提供统一的
   话题名/合法状态值，不用各自重新约定一遍、容易约定出岔子。
2. 给阶段9.4（GCS前端订阅这个话题展示任务状态）提前对齐话题名/取值，
   GCS那边可以先按这份契约把订阅逻辑写好，不用等阶段7.4真正有节点在
   发布之后才能开始。

话题是**per-drone命名空间下的相对话题名**（`/NX01/mission_state`、
`/NX02/mission_state`各自独立），跟这个项目其余话题的命名习惯一致——
双机任务里两架飞机的任务阶段不一定同步（比如侦察机已经在"aiming"，
行动机可能还在"enroute"），没有理由做成一个跨机共享的话题。
"""
from std_msgs.msg import String

MISSION_STATE_TOPIC = 'mission_state'

# 跟方案原文阶段7.1给的枚举完全一致，没有额外增删——发布方如果需要
# 更细粒度的状态，应该在message.data里追加冒号分隔的子状态（比如
# "searching:orbit_pillar_2"），而不是往这个枚举里加新的顶层值，
# 保持消费方（GCS/裁判等）只需要认识这6个值就够用。
VALID_MISSION_STATES = ('idle', 'searching', 'loading', 'enroute', 'aiming', 'done')


def is_valid_mission_state(state: str) -> bool:
    """允许"顶层值"或"顶层值:子状态"两种形式，只校验顶层值。"""
    top_level = state.split(':', 1)[0]
    return top_level in VALID_MISSION_STATES


def make_mission_state_msg(state: str) -> String:
    if not is_valid_mission_state(state):
        raise ValueError(
            f"'{state}'不是合法的任务状态，顶层值必须是{VALID_MISSION_STATES}之一"
        )
    msg = String()
    msg.data = state
    return msg

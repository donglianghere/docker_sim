"""docker_sim集成专用launch文件：origin_setter + frame_align_bridge两个节点，
跟PLANNER/CONTROLLER两个开关都无关——起飞点锁定+跨机map系对齐这两件事在哪种
规划器/控制器组合下都需要，所以entrypoint里应当无条件启动这两个节点，不像
px4ctrl_bridge/ego_planner_bridge那样区分组合。

odom_topic用默认值('dlio/odom_node/odom')就已经能同时覆盖
LOCALIZATION_SOURCE=uwb_slam和=gt两种模式——两者按README的既有约定发布的是
同一个最终话题名，这里不需要按LOCALIZATION_SOURCE分支处理。

2026-09-14新增：origin_setter_node的`rotation_estimation_enabled`参数**要**
按LOCALIZATION_SOURCE分支——上面这段"跟LOCALIZATION_SOURCE无关"说的是"要不要
起这两个节点"这件事，不是"节点内部所有行为都不感知这个开关"。origin_setter
自己的SE(2)在线旋转估计原本设计成"不感知LOCALIZATION_SOURCE"（θ*会自然收敛
到接近0），但实测踩过坑：容器刚重启、UWB/里程计还没稳定的窗口里，一次瞬态
噪声就能被当成真实位移，算出一个完全是噪声的θ*，把`world_to_local()`全部
污染（2026-09-14实测复现：NX01/NX02分别算出164.9°/70.5°，飞机飞到错误位置
贴着障碍物卡死，详见docker_sim/DEBUG_JOURNAL.md同日期条目）。用户确认的
架构事实：`gt`/`uwb_imu`（含`single_uwb_imu`）模式下局部系与全局系之间架构
上就是纯平移，不存在真实旋转需要估计——这套SE(2)标定真正要解决的场景只有
`uwb_slam`/`single_uwb_slam`（DLIO SLAM局部系yaw相对全局系有未知漂移）。
所以这里按LOCALIZATION_SOURCE算一个布尔值传给origin_setter_node：只有
`uwb_slam`/`single_uwb_slam`两个值下才打开，其余模式强制关闭（θ*永远
保持0.0，等价于纯平移假设），不是把阈值/样本数调高这种治标不治本的做法。

frame_align_bridge_node（2026-08-12新增）：把origin_setter锁定出来的
`world -> {ns}/map`TF换算成`/frame_align/{i}/{j}`发布给mighty/ego_planner_swarm
的跨机对齐闭环用，详见该节点文件头注释。num_agents/namespace_prefix需要
跟uwb_sim那边的同名参数保持一致，否则两边算出的机名列表对不上。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    num_agents = LaunchConfiguration('num_agents')
    namespace_prefix = LaunchConfiguration('namespace_prefix')
    localization_source = LaunchConfiguration('localization_source')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('NAMESPACE', default_value='NX01'),
        description='NX01/NX02——必须跟MAVROS/mighty/DLIO/uwb_sim用同一个值',
    )
    declare_num_agents = DeclareLaunchArgument(
        'num_agents',
        default_value=EnvironmentVariable('NUM_AGENTS', default_value='2'),
        description='机队规模，必须跟uwb_sim的num_agents一致',
    )
    declare_namespace_prefix = DeclareLaunchArgument(
        'namespace_prefix',
        default_value='NX',
        description='机名前缀，必须跟uwb_sim的namespace_prefix一致',
    )
    declare_localization_source = DeclareLaunchArgument(
        'localization_source',
        default_value=EnvironmentVariable('LOCALIZATION_SOURCE', default_value='gt'),
        description=(
            '决定origin_setter_node的两个开关：SE(2)旋转估计——'
            'uwb_slam/single_uwb_slam打开，其余(gt/uwb_imu/single_uwb_imu等)'
            '强制θ*=0，见文件头2026-09-14说明；以及锁定前的高度一致性检查——'
            'uwb_imu/single_uwb_imu打开，见2026-09-24说明'),
    )
    # PythonExpression对多个候选值做or判断，字符串拼接比链式布尔substitution
    # 可读——跟同目录其它launch文件(pt4ctrl_docker_sim.launch.py等)用
    # PythonExpression判断planner=='ego_planner'是同一个established模式。
    rotation_estimation_enabled = PythonExpression([
        "'", localization_source, "' in ('uwb_slam', 'single_uwb_slam')"])

    # 2026-09-24新增：锁定原点前要不要检查"odom的z跟测距雷达对得上"（见
    # origin_setter_node.py同日期说明）。只有uwb_imu系的定位源下odom的z才
    # 来自测距雷达，也只有这种情况下雷达还没上线时飞控高度会自由漂移、
    # 把漂移量锁进TF的z偏移里；gt/uwb_slam的z不是这么来的，开了反而会拿
    # 两个不同含义的高度互相比、永远对不上而锁不上原点。
    height_check_enabled = PythonExpression([
        "'", localization_source, "' in ('uwb_imu', 'single_uwb_imu')"])

    # 2026-09-03：θ*的Huber抗差开关（论文2.7节，见origin_setter_node.py文件头
    # 说明）从环境变量ROTATION_ROBUST读，默认false=行为跟以前完全一致。做
    # 论文的抗差验证实验(E15)时在docker-compose里把它设成true即可，不用改
    # 代码、不用重新build镜像——这正是为什么要走环境变量而不是写死参数。
    origin_setter_node = Node(
        package='uwb_origin_bridge',
        executable='origin_setter_node',
        name='origin_setter',
        namespace=namespace,
        output='screen',
        parameters=[{
            'rotation_estimation_enabled': ParameterValue(
                rotation_estimation_enabled, value_type=bool),
            'height_check_enabled': ParameterValue(
                height_check_enabled, value_type=bool),
            'rotation_robust_enabled': ParameterValue(
                EnvironmentVariable('ROTATION_ROBUST', default_value='false'),
                value_type=bool),
            'rotation_seg_min_disp_m': ParameterValue(
                EnvironmentVariable('ROTATION_SEG_MIN_DISP_M',
                                    default_value='0.5'), value_type=float),
            'rotation_window_size': ParameterValue(
                EnvironmentVariable('ROTATION_WINDOW_SIZE',
                                    default_value='50'), value_type=int),
        }],
    )

    frame_align_bridge_node = Node(
        package='uwb_origin_bridge',
        executable='frame_align_bridge_node',
        name='frame_align_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'num_agents': ParameterValue(num_agents, value_type=int),
            'namespace_prefix': namespace_prefix,
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_num_agents,
        declare_namespace_prefix,
        declare_localization_source,
        origin_setter_node,
        frame_align_bridge_node,
    ])

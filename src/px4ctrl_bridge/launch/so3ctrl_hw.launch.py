"""真机部署专用launch文件：so3ctrl_node + goal_to_poscmd + px4_param_relax
+ takeoff_gate 四个节点打包一起启动，2026-09-06基于so3ctrl_docker_sim.
launch.py改的，跟那份文件唯一的实质差异是 auto_takeoff_land.no_RC 从
True改成False（外置成PX4CTRL_NO_RC环境变量）——真机上必须用真实遥控器
数据（RC failsafe要真的生效），不能再假装"RC永远处于hover挡+command挡+
摇杆居中"这种仿真专用的免遥控器模式。跟px4ctrl_hw.launch.py是同一个改法，
补齐的是px4ctrl_hw.launch.py2026-08-14就有、but so3ctrl一直缺失的hw版本
（entrypoint.sh里DEPLOY_TARGET=hw+CONTROLLER=so3ctrl之前会直接fail
fast退出，这份文件补上之后需要同步放开entrypoint.sh里的两处限制）。

no_RC=False之后的现实影响（首次真机测试前必须确认，跟px4ctrl_hw.launch.py
文件头描述完全一致，因为so3ctrl复用的就是px4ctrl的飞行状态机PX4CtrlFSM）：
  - so3ctrl_node会真的订阅mavros/rc/in并解析RC通道（so3ctrl跟px4ctrl共用
    input.cpp同一套解析逻辑），飞控RC_MAP_*参数必须已经按真实遥控器接线
    配好（ch0-3对应Roll/Pitch/Yaw/Throttle，channels[4]/[5]/[7]对应
    mode/gear/reboot_cmd），见docker_sim/真机部署操作清单.md"3.3 QGC直连
    飞控预配置"一节——这一步必须先用QGC通过USB直连飞控完成，不是这个
    launch文件能替你做的事。
  - auto_takeoff_land.enable/enable_auto_arm 保持True不变——no_RC只影响
    "RC数据从哪来"，不影响"要不要响应takeoff_gate触发的自动起飞"这个
    独立的开关。首次真机测试建议先只验证RC failsafe确实生效（人为断开
    遥控器信号，观察so3ctrl是否正确回退MANUAL_CTRL），再验证takeoff_gate
    自动起飞，不要一次性两个都测。

2026-09-06同批同步：so3ctrl的PX4CtrlFSM.cpp已经补齐px4ctrl 2026-08-27
那两处真机安全修复（AUTO_TAKEOFF卡死退出条件、CMD_CTRL->AUTO_LAND直接
触发），这份launch文件把so3ctrl真正接到DEPLOY_TARGET=hw路径，两者配合
才算完整补齐——只改FSM代码没有hw launch文件的话，so3ctrl依然没有任何
路径能在真机上跑起来。

其余部分（mass/hover_percentage的环境变量覆盖、姿态角限幅/手动模式限速/
低电压阈值/精确推力模型标定系数/级联PID增益的环境变量外置、PLANNER决定
cmd话题remap目标、goal_to_poscmd/px4_param_relax/takeoff_gate三个公共
节点）逐字照抄px4ctrl_hw.launch.py（只把package/executable/name从
'px4ctrl'/'px4ctrl_node'/'px4ctrl'换成'so3ctrl'/'so3ctrl_node'/
'so3ctrl'，环境变量名沿用同一套PX4CTRL_*前缀，不单独为so3ctrl起
SO3CTRL_*前缀——同一时刻只会有一个控制器在跑，不存在两边同时读取
互相冲突的场景，沿用同一套环境变量名减少选手/操作员要记忆的变量数量），
没有改动，改动了多份文件要同步维护成本更高，各自的注释保持独立完整，
不需要来回对照。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    so3ctrl_share = get_package_share_directory('so3ctrl')
    base_param_file = os.path.join(so3ctrl_share, 'config', 'ctrl_param_fpv.yaml')

    namespace = LaunchConfiguration('namespace')
    planner = LaunchConfiguration('planner')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('NAMESPACE', default_value='NX01'),
        description='NX01/NX02——必须跟MAVROS(namespace=<ns>/mavros)、mighty、DLIO用同一个值',
    )
    declare_planner = DeclareLaunchArgument(
        'planner',
        default_value=EnvironmentVariable('PLANNER', default_value='ego_planner'),
        description='mighty/ego_planner——决定so3ctrl的cmd话题接goal_to_poscmd还是position_cmd',
    )
    is_ego_planner = PythonExpression(["'", planner, "' == 'ego_planner'"])

    so3ctrl_base_kwargs = dict(
        package='so3ctrl',
        executable='so3ctrl_node',
        name='so3ctrl',
        namespace=namespace,
        output='screen',
        parameters=[
            base_param_file,
            {
                # mass/hover_percentage覆盖成跟mighty/ros2_px4_stack/px4ctrl
                # 同一份数字，而不是ctrl_param_fpv.yaml里那个1.2kg/0.30的
                # 通用默认值。真机上VEHICLE_MASS_KG/VEHICLE_HOVER_THRUST这两个
                # 环境变量必须换成真实机体称重+桨盘推力测算的数字（vehicle_
                # profile.yaml里的仿真iris数值不能照搬）。
                'mass': ParameterValue(
                    EnvironmentVariable('VEHICLE_MASS_KG', default_value='1.2'), value_type=float),
                'thrust_model.hover_percentage': ParameterValue(
                    EnvironmentVariable('VEHICLE_HOVER_THRUST', default_value='0.30'), value_type=float),
                # 跟px4ctrl_hw.launch.py同一批外置参数（2026-08-27那次给
                # px4ctrl加的，so3ctrl这份文件是新建的，直接一步到位带上，
                # 不需要像px4ctrl那样分两次改）——姿态角限幅/手动模式限速/
                # 低电压阈值/精确推力模型标定系数(K1/K2/K3)/级联PID增益
                # (Kp/Kv/KAng三组，Kvi*/Kvd*两组yaml标注"No use now"没有
                # 外置)。默认值原样保留yaml里的数字，compose没设这些环境
                # 变量时行为完全不变。K1/K2/K3是accurate_thrust_model:=true
                # 时才用得上的精确推力模型标定系数，真机要用必须先标定出真实
                # 数值再通过环境变量传入，不能沿用这里的占位默认值。
                'max_angle': ParameterValue(
                    EnvironmentVariable('PX4CTRL_MAX_ANGLE_DEG', default_value='30.0'), value_type=float),
                'max_manual_vel': ParameterValue(
                    EnvironmentVariable('PX4CTRL_MAX_MANUAL_VEL', default_value='1.0'), value_type=float),
                'low_voltage': ParameterValue(
                    EnvironmentVariable('PX4CTRL_LOW_VOLTAGE', default_value='13.2'), value_type=float),
                'thrust_model.K1': ParameterValue(
                    EnvironmentVariable('PX4CTRL_THRUST_K1', default_value='0.7583'), value_type=float),
                'thrust_model.K2': ParameterValue(
                    EnvironmentVariable('PX4CTRL_THRUST_K2', default_value='1.6942'), value_type=float),
                'thrust_model.K3': ParameterValue(
                    EnvironmentVariable('PX4CTRL_THRUST_K3', default_value='0.6786'), value_type=float),
                'gain.Kp0': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KP0', default_value='1.5'), value_type=float),
                'gain.Kp1': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KP1', default_value='1.5'), value_type=float),
                'gain.Kp2': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KP2', default_value='1.5'), value_type=float),
                'gain.Kv0': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KV0', default_value='1.5'), value_type=float),
                'gain.Kv1': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KV1', default_value='1.5'), value_type=float),
                'gain.Kv2': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KV2', default_value='1.5'), value_type=float),
                'gain.KAngR': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KANGR', default_value='20.0'), value_type=float),
                'gain.KAngP': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KANGP', default_value='20.0'), value_type=float),
                'gain.KAngY': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KANGY', default_value='20.0'), value_type=float),
                # 2026-09-07新增：yaw锁定开关，静态参数(启动前用环境变量定死，
                # 不支持飞行中动态切换)，见docker_sim/DEBUG_JOURNAL.md
                # 同日期条目完整设计讨论。开启后CMD_CTRL态忽略规划器算出来的
                # 行进方向朝向，改成锁定在起飞瞬间的实际朝向——so3ctrl的
                # 位置/加速度控制律本身不依赖当前yaw，锁定朝向不影响位置/
                # 轨迹跟踪，只是前视相机不再跟随飞行方向。默认false(行为
                # 不变)，首次真机测试前建议先在仿真里验证过。
                'yaw_lock_enabled': ParameterValue(
                    EnvironmentVariable('PX4CTRL_YAW_LOCK_ENABLED', default_value='false'), value_type=bool),
                # 真机部署与仿真的实质差异：no_RC默认False，真实遥控器数据
                # 接管RC failsafe。⚠️ 默认值必须保持false——no_RC=true会
                # 跳过真实RC failsafe，只应该在明确知道自己在做什么的
                # dry-run/台架测试场景下临时打开（PX4CTRL_NO_RC=true）。
                'auto_takeoff_land.no_RC': ParameterValue(
                    EnvironmentVariable('PX4CTRL_NO_RC', default_value='false'), value_type=bool),
                'auto_takeoff_land.enable': True,
                'auto_takeoff_land.enable_auto_arm': True,
            },
        ],
    )
    # odom这条remap两种planner都一样（DLIO/gt_odom_bridge发的都是这同一个
    # 相对话题名），cmd这条按planner二选一——同一个so3ctrl_node定义不了
    # "remappings列表按条件变化"，用两份Node+IfCondition/UnlessCondition
    # 互斥代替，同一时刻只有一份会真正启动，不会两边都抢so3ctrl这个节点名。
    so3ctrl_node_mighty = Node(
        **so3ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 'cmd'不remap——goal_to_poscmd在同一个namespace下发布到相对名
            # 'cmd'，天然对上。
        ],
        condition=UnlessCondition(is_ego_planner),
    )
    so3ctrl_node_ego_planner = Node(
        **so3ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            ('cmd', 'position_cmd'),
        ],
        condition=IfCondition(is_ego_planner),
    )

    goal_to_poscmd_node = Node(
        package='px4ctrl_bridge',
        executable='goal_to_poscmd',
        name='goal_to_poscmd',
        namespace=namespace,
        output='screen',
        # ego_planner模式下不需要这条Goal->PositionCommand转换桥（traj_server
        # 原生发PositionCommand），起了也没有mighty发Goal消息喂给它，纯粹
        # 空等，不启动更干净。
        condition=UnlessCondition(is_ego_planner),
    )

    # 一次性节点：放宽/收紧PX4失控保护参数——这个节点自己读DEPLOY_TARGET
    # 环境变量决定具体放宽哪些参数，不是这个launch文件传参决定的。
    px4_param_relax_node = Node(
        package='px4ctrl_bridge',
        executable='px4_param_relax',
        name='px4_param_relax',
        namespace=namespace,
        output='screen',
    )

    # 一次性节点：等/tmp/takeoff_go出现再触发起飞，配合
    # docker_sim/scripts/launch_control.sh的操作习惯。
    takeoff_gate_node = Node(
        package='px4ctrl_bridge',
        executable='takeoff_gate',
        name='takeoff_gate',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        declare_planner,
        so3ctrl_node_mighty,
        so3ctrl_node_ego_planner,
        goal_to_poscmd_node,
        px4_param_relax_node,
        takeoff_gate_node,
    ])

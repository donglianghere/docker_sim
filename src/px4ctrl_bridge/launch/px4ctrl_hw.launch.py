"""真机部署专用launch文件：px4ctrl_node + goal_to_poscmd + px4_param_relax
+ takeoff_gate 四个节点打包一起启动，2026-08-14基于px4ctrl_docker_sim.
launch.py改的，跟那份文件唯一的实质差异是 auto_takeoff_land.no_RC 从
True改成False——真机上必须用真实遥控器数据（RC failsafe要真的生效），不能
再假装"RC永远处于hover挡+command挡+摇杆居中"这种仿真专用的免遥控器模式。
DEPLOY_TARGET=hw时flight-stack-entrypoint.sh用这份文件而不是
px4ctrl_docker_sim.launch.py（见entrypoint.sh里CONTROLLER dispatch那段的
完整说明）。

no_RC=False之后的现实影响（首次真机测试前必须确认）：
  - px4ctrl_node会真的订阅mavros/rc/in并解析RC通道（vendor/px4ctrl_ros2/
    px4ctrl/src/input.cpp:22-98），飞控RC_MAP_*参数必须已经按真实遥控器
    接线配好（ch0-3对应Roll/Pitch/Yaw/Throttle，channels[4]/[5]/[7]对应
    mode/gear/reboot_cmd），见docker_sim/真机部署操作清单.md"3.3 QGC直连
    飞控预配置"一节——这一步必须先用QGC通过USB直连飞控完成，不是这个
    launch文件能替你做的事。
  - auto_takeoff_land.enable/enable_auto_arm 保持True不变——no_RC只影响
    "RC数据从哪来"，不影响"要不要响应takeoff_gate触发的自动起飞"这个
    独立的开关，两者是正交的（PX4CtrlFSM的RC failsafe逻辑本来就设计成
    "不管当前是不是在自动起降流程里，摇杆/开关状态异常时都能随时接管"，
    见DEBUG_JOURNAL.md 2026-08-10 Q2记录）。首次真机测试建议先只验证RC
    failsafe确实生效（人为断开遥控器信号，观察px4ctrl是否正确回退
    MANUAL_CTRL），再验证takeoff_gate自动起飞，不要一次性两个都测。

其余部分（mass/hover_percentage的环境变量覆盖、PLANNER决定cmd话题remap
目标、goal_to_poscmd/px4_param_relax/takeoff_gate三个公共节点）逐字照抄
px4ctrl_docker_sim.launch.py，没有改动，改动了两份文件要同步维护成本更高，
两份文件的注释各自独立完整，不需要来回对照。
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
    px4ctrl_share = get_package_share_directory('px4ctrl')
    base_param_file = os.path.join(px4ctrl_share, 'config', 'ctrl_param_fpv.yaml')

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
        description='mighty/ego_planner——决定px4ctrl的cmd话题接goal_to_poscmd还是position_cmd',
    )
    is_ego_planner = PythonExpression(["'", planner, "' == 'ego_planner'"])

    px4ctrl_base_kwargs = dict(
        package='px4ctrl',
        executable='px4ctrl_node',
        name='px4ctrl',
        namespace=namespace,
        output='screen',
        parameters=[
            base_param_file,
            {
                # mass/hover_percentage覆盖成跟mighty/ros2_px4_stack同一份数字，
                # 而不是ctrl_param_fpv.yaml里那个1.2kg/0.30的通用默认值。真机上
                # VEHICLE_MASS_KG/VEHICLE_HOVER_THRUST这两个环境变量必须换成真实
                # 机体称重+桨盘推力测算的数字（vehicle_profile.yaml里的仿真iris
                # 数值不能照搬），见entrypoint.sh里这两个变量的解析逻辑。
                'mass': ParameterValue(
                    EnvironmentVariable('VEHICLE_MASS_KG', default_value='1.2'), value_type=float),
                'thrust_model.hover_percentage': ParameterValue(
                    EnvironmentVariable('VEHICLE_HOVER_THRUST', default_value='0.30'), value_type=float),
                # 2026-08-27新增：姿态角限幅/手动模式限速/低电压阈值/精确推力
                # 模型标定系数(K1/K2/K3)/级联PID增益，原来全部写死在
                # ctrl_param_fpv.yaml里，同样外置成环境变量，跟mass/
                # hover_percentage同一个模式——默认值原样保留yaml里的数字，
                # compose没设这些环境变量时行为完全不变。K1/K2/K3是
                # accurate_thrust_model:=true时才用得上的精确推力模型标定
                # 系数(yaml注释写着"Needs precise calibration!")，真机要用
                # 精确推力模型必须先标定出真实数值再通过这几个环境变量传入，
                # 不能沿用这里的占位默认值。gain下面的Kvi*/Kvd*两组
                # (积分/微分项)yaml注释标注"No use now"(代码里没有实际用到)，
                # 没有外置，只外置了确认在用的Kp/Kv/KAng三组增益。
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
                # 行进方向朝向，改成锁定在起飞瞬间的实际朝向——四旋翼平动
                # 加速度只由roll/pitch倾角决定跟yaw无关，不影响位置/轨迹
                # 跟踪，只是前视相机不再跟随飞行方向。默认false(行为不变)，
                # 首次真机测试前建议先在仿真里验证过。
                'yaw_lock_enabled': ParameterValue(
                    EnvironmentVariable('PX4CTRL_YAW_LOCK_ENABLED', default_value='false'), value_type=bool),
                # 真机部署与仿真默认的实质差异：no_RC默认False，真实遥控器数据
                # 接管RC failsafe，见文件头docstring完整说明。2026-08-18改成
                # 环境变量PX4CTRL_NO_RC外部可覆盖——之前排查"没接/没配好RC导致
                # px4ctrl卡在Waiting for RC、takeoff_land话题完全没反应"这个
                # 问题时，只能手动kill掉px4ctrl_node再用ros2 run重新起、临时
                # 传参跳过等RC，不方便也容易忘了改回来；外部化之后
                # PX4CTRL_NO_RC=true可以直接在docker-compose.hw.yml/环境变量
                # 里配，不用碰代码，也不用手动kill+relaunch。⚠️ 默认值必须
                # 保持false——no_RC=true会跳过真实RC failsafe，只应该在明确
                # 知道自己在做什么的dry-run/台架测试场景下临时打开。
                'auto_takeoff_land.no_RC': ParameterValue(
                    EnvironmentVariable('PX4CTRL_NO_RC', default_value='false'), value_type=bool),
                'auto_takeoff_land.enable': True,
                'auto_takeoff_land.enable_auto_arm': True,
            },
        ],
    )
    # odom这条remap两种planner都一样（DLIO/gt_odom_bridge发的都是这同一个
    # 相对话题名），cmd这条按planner二选一——同一个px4ctrl_node定义不了
    # "remappings列表按条件变化"，用两份Node+IfCondition/UnlessCondition
    # 互斥代替，同一时刻只有一份会真正启动，不会两边都抢px4ctrl这个节点名。
    px4ctrl_node_mighty = Node(
        **px4ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 'cmd'不remap——goal_to_poscmd在同一个namespace下发布到相对名
            # 'cmd'，天然对上。
        ],
        condition=UnlessCondition(is_ego_planner),
    )
    px4ctrl_node_ego_planner = Node(
        **px4ctrl_base_kwargs,
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
    # 环境变量决定具体放宽哪些参数（真机模式明显收紧，见
    # px4ctrl_bridge/px4_param_relax.py文件头docstring），不是这个launch
    # 文件传参决定的。放在px4ctrl_node之后起没有强制先后依赖关系（它只是走
    # mavros/param/set服务，跟px4ctrl本身是否已经起来无关），但把它跟px4ctrl
    # 放在同一个launch文件里能保证"只要px4ctrl这条链路被启动，参数调整就
    # 一定会跑一次"，不需要在entrypoint.sh里单独再记一行。
    px4_param_relax_node = Node(
        package='px4ctrl_bridge',
        executable='px4_param_relax',
        name='px4_param_relax',
        namespace=namespace,
        output='screen',
    )

    # 一次性节点：等/tmp/takeoff_go出现再触发起飞，配合
    # docker_sim/scripts/launch_control.sh的操作习惯（真机部署这套操作习惯
    # 要不要保留、还是换成GCS侧的service call，是本机地面站设计那块的
    # 独立决定，这里先保留跟仿真一致的触发方式）。
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
        px4ctrl_node_mighty,
        px4ctrl_node_ego_planner,
        goal_to_poscmd_node,
        px4_param_relax_node,
        takeoff_gate_node,
    ])

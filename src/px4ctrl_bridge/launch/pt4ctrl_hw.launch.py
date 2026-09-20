"""真机部署专用launch文件：pt4ctrl_node + goal_to_poscmd + px4_param_relax
+ takeoff_gate 四个节点打包一起启动，2026-08-29基于pt4ctrl_docker_sim.
launch.py改的，跟那份文件唯一的实质差异是 auto_takeoff_land.no_RC 从
True改成读PX4CTRL_NO_RC环境变量（默认false）——真机上必须用真实遥控器数据
（RC failsafe要真的生效），不能再假装"RC永远处于hover挡+command挡+摇杆
居中"这种仿真专用的免遥控器模式。这跟px4ctrl_hw.launch.py相对
px4ctrl_docker_sim.launch.py的差异是同一个模式，逐段对照着改的。

⚠️ pt4ctrl本身"2026-08-13新增、全新实现、未实测"（见entrypoint里CONTROLLER
dispatch那段的原话和DEBUG_JOURNAL.md）。这份launch文件只是把它的真机启动
路径补齐，**不代表pt4ctrl这条控制链路已经过真机验证**。首次使用前必须先
台架/不装桨走一遍全流程。

no_RC=False之后的现实影响（首次真机测试前必须确认），跟px4ctrl_hw.launch.py
完全一样：
  - pt4ctrl_node会真的订阅mavros/rc/in并解析RC通道，飞控RC_MAP_*参数必须
    已经按真实遥控器接线配好（ch0-3对应Roll/Pitch/Yaw/Throttle，
    channels[4]/[5]/[7]对应mode/gear/reboot_cmd），见docker_sim/真机部署
    操作清单.md"3.3 QGC直连飞控预配置"一节——这一步必须先用QGC通过USB直连
    飞控完成，不是这个launch文件能替你做的事。
  - auto_takeoff_land.enable/enable_auto_arm 保持True不变——no_RC只影响
    "RC数据从哪来"，不影响"要不要响应takeoff_gate触发的自动起飞"这个独立
    的开关，两者是正交的。首次真机测试建议先只验证RC failsafe确实生效
    （人为断开遥控器信号，观察pt4ctrl是否正确回退MANUAL_CTRL），再验证
    takeoff_gate自动起飞，不要一次性两个都测。

⚠️ 哪些PX4CTRL_*环境变量对pt4ctrl【不生效】（很容易踩的坑）：

pt4ctrl没有控制律——它砍掉了px4ctrl的控制律、改发轨迹setpoint让PX4自己的
位置控制环接管。所以pt4ctrl的config/ctrl_param_fpv.yaml比px4ctrl的同名文件
精简很多，**根本不存在**这些key（见那份yaml的文件头说明）：

    gain.* / rotor_drag.* / thrust_model.* / mass / gra / max_angle /
    low_voltage / use_bodyrate_ctrl

因此 docker-compose.hw.yml 里这几个环境变量设了对pt4ctrl**没有任何效果**：

    VEHICLE_MASS_KG          VEHICLE_HOVER_THRUST
    PX4CTRL_MAX_ANGLE_DEG    PX4CTRL_LOW_VOLTAGE
    PX4CTRL_THRUST_K1/K2/K3  PX4CTRL_GAIN_*

ROS2允许传多余的parameters（未被declare_parameter读取的条目静默忽略），
硬传不会报错，但会让人误以为这些数字对pt4ctrl也有意义——所以这里跟
pt4ctrl_docker_sim.launch.py一样，一个都不传。

**确实生效**的只有两个（pt4ctrl的yaml里真实存在的字段）：

    PX4CTRL_NO_RC          → auto_takeoff_land.no_RC
    PX4CTRL_MAX_MANUAL_VEL → max_manual_vel

其余部分（PLANNER决定cmd话题remap目标、goal_to_poscmd/px4_param_relax/
takeoff_gate三个公共节点）逐字照抄pt4ctrl_docker_sim.launch.py，没有改动。
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
    pt4ctrl_share = get_package_share_directory('pt4ctrl')
    base_param_file = os.path.join(pt4ctrl_share, 'config', 'ctrl_param_fpv.yaml')

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
        description='mighty/ego_planner——决定pt4ctrl的cmd话题接goal_to_poscmd还是position_cmd',
    )
    is_ego_planner = PythonExpression(["'", planner, "' == 'ego_planner'"])

    pt4ctrl_base_kwargs = dict(
        package='pt4ctrl',
        executable='pt4ctrl_node',
        name='pt4ctrl',
        namespace=namespace,
        output='screen',
        parameters=[
            base_param_file,
            {
                # 真机部署与仿真默认的实质差异：no_RC默认false，真实遥控器数据
                # 接管RC failsafe，见文件头docstring完整说明。外置成环境变量
                # PX4CTRL_NO_RC，跟px4ctrl_hw.launch.py共用同一个变量名（同一
                # 台机器上同一时刻只会起其中一个控制器，不会冲突）。
                # ⚠️ 默认值必须保持false——no_RC=true会跳过真实RC failsafe，
                # 只应该在明确知道自己在做什么的dry-run/台架测试场景下临时打开。
                'auto_takeoff_land.no_RC': ParameterValue(
                    EnvironmentVariable('PX4CTRL_NO_RC', default_value='false'), value_type=bool),
                'auto_takeoff_land.enable': True,
                'auto_takeoff_land.enable_auto_arm': True,
                # pt4ctrl的yaml里确实存在的另一个可调字段（默认1.0，跟
                # px4ctrl_hw.launch.py用同一个环境变量名和同一个默认值，
                # 不设时行为跟仿真版完全一致）。
                'max_manual_vel': ParameterValue(
                    EnvironmentVariable('PX4CTRL_MAX_MANUAL_VEL', default_value='1.0'),
                    value_type=float),
                # 2026-09-07新增：yaw锁定开关，静态参数(启动前用环境变量定死，
                # 不支持飞行中动态切换)，见docker_sim/DEBUG_JOURNAL.md
                # 同日期条目完整设计讨论。开启后CMD_CTRL态忽略规划器算出来的
                # 行进方向朝向，改成锁定在起飞瞬间的实际朝向——pt4ctrl不算
                # 姿态、把yaw原样转发给PX4自己的位置控制环，锁定朝向不影响
                # 位置/轨迹跟踪，只是前视相机不再跟随飞行方向。默认false
                # (行为不变)，且pt4ctrl本身仍是未实测状态，务必先仿真+台架
                # 验证过。
                'yaw_lock_enabled': ParameterValue(
                    EnvironmentVariable('PX4CTRL_YAW_LOCK_ENABLED', default_value='false'), value_type=bool),
            },
        ],
    )
    pt4ctrl_node_mighty = Node(
        **pt4ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 'cmd'不remap——goal_to_poscmd在同一个namespace下发布到相对名
            # 'cmd'，天然对上。
        ],
        condition=UnlessCondition(is_ego_planner),
    )
    pt4ctrl_node_ego_planner = Node(
        **pt4ctrl_base_kwargs,
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
        condition=UnlessCondition(is_ego_planner),
    )

    # px4_param_relax/takeoff_gate两个节点跟具体控制器完全无关（前者只走
    # mavros/param/set服务，后者只发quadrotor_msgs/TakeoffLand），
    # px4ctrl_bridge包里已有的实现直接复用，不需要为pt4ctrl另写一份。
    # px4_param_relax自己读DEPLOY_TARGET环境变量决定放宽/收紧哪些参数
    # （真机模式明显收紧，见px4ctrl_bridge/px4_param_relax.py文件头），
    # 不是这个launch文件传参决定的。
    px4_param_relax_node = Node(
        package='px4ctrl_bridge',
        executable='px4_param_relax',
        name='px4_param_relax',
        namespace=namespace,
        output='screen',
    )

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
        pt4ctrl_node_mighty,
        pt4ctrl_node_ego_planner,
        goal_to_poscmd_node,
        px4_param_relax_node,
        takeoff_gate_node,
    ])

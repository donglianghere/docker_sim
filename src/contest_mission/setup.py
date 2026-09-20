from setuptools import setup

package_name = 'contest_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/fire_drill_room_layout.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='2026大赛任务系统：仿真侧感知/任务状态机/裁判/场景重置节点',
    license='TODO',
    entry_points={
        'console_scripts': [
            'qr_apriltag_detect_node = contest_mission.qr_apriltag_detect_node:main',
            'position_cmd_relay_node = contest_mission.position_cmd_relay_node:main',
            'pillar_detector_node = contest_mission.pillar_detector_node:main',
            'precision_servo_node = contest_mission.precision_servo_node:main',
            'fire_pillar_aim_node = contest_mission.fire_pillar_aim_node:main',
            'formation_follower_node = contest_mission.formation_follower_node:main',
            'actuator_action_node = contest_mission.actuator_action_node:main',
            'mission_judge_node = contest_mission.mission_judge_node:main',
            'scenario_reset_node = contest_mission.scenario_reset_node:main',
        ],
    },
)

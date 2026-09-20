from setuptools import find_packages, setup
from glob import glob

package_name = 'ego_planner_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robots',
    maintainer_email='robots@todo.todo',
    description=(
        'docker_sim项目专用：把ego-planner-swarm的规划器核心接到'
        'DLIO/Gazebo点云里程计和px4ctrl(ROS2版)的launch文件'
    ),
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_goal_bridge_node = ego_planner_bridge.rviz_goal_bridge_node:main',
            'poscmd_to_goal_node = ego_planner_bridge.poscmd_to_goal_node:main',
        ],
    },
)

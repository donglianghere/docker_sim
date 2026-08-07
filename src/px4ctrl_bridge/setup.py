from setuptools import find_packages, setup
from glob import glob

package_name = 'px4ctrl_bridge'

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
        'docker_sim项目专用：把mighty规划器接到px4ctrl(ROS2版)所需的'
        '三个辅助节点（Goal->PositionCommand桥接、PX4参数放宽、起飞口令闸门）'
    ),
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_to_poscmd = px4ctrl_bridge.goal_to_poscmd:main',
            'px4_param_relax = px4ctrl_bridge.px4_param_relax:main',
            'takeoff_gate = px4ctrl_bridge.takeoff_gate:main',
        ],
    },
)

from setuptools import setup

package_name = 'nlink_uwb_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nlink_uwb_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='nlink_parser2真实UWB驱动到uwb_origin_bridge标准接口(uwb/pose_abs)的桥接',
    license='TODO',
    entry_points={
        'console_scripts': [
            'nlink_pose_bridge_node = nlink_uwb_bridge.nlink_pose_bridge_node:main',
        ],
    },
)

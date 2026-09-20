from setuptools import setup

package_name = 'uwb_origin_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uwb_origin_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='UWB(或Gazebo真值替身)起飞点一键锁定：把绝对位置锁存成局部里程计到任务坐标系的偏移',
    license='TODO',
    entry_points={
        'console_scripts': [
            'origin_setter_node = uwb_origin_bridge.origin_setter_node:main',
            'uwb_imu_fusion_node = uwb_origin_bridge.uwb_imu_fusion_node:main',
            'frame_align_bridge_node = uwb_origin_bridge.frame_align_bridge_node:main',
        ],
    },
)

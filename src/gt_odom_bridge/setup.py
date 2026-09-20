from setuptools import setup

package_name = 'gt_odom_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Republish Gazebo ground-truth pose as a DLIO-shaped odom topic for isolated planning/control testing',
    license='TODO',
    entry_points={
        'console_scripts': [
            'gt_odom_bridge_node = gt_odom_bridge.gt_odom_bridge_node:main',
            'name_label_node = gt_odom_bridge.name_label_node:main',
            'gt_cloud_bridge_node = gt_odom_bridge.gt_cloud_bridge_node:main',
            'local_position_readback_node = gt_odom_bridge.local_position_readback_node:main',
        ],
    },
)

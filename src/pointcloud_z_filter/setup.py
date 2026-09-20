from setuptools import setup

package_name = 'pointcloud_z_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pointcloud_z_filter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='按Z轴高度区间过滤PointCloud2，只给地面站/RViz显示用，不改动原始话题',
    license='TODO',
    entry_points={
        'console_scripts': [
            'z_filter_node = pointcloud_z_filter.z_filter_node:main',
        ],
    },
)

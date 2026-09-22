"""contest_sdk是纯Python包，不是ROS2功能包（不需要colcon build/ament），
选手容器里用`pip install .`（或者阶段C定下来的镜像COPY方式）安装即可。
这里用最朴素的setuptools写法，不引入colcon/ament相关的样板文件。
"""
from setuptools import setup, find_packages

setup(
    name='contest_sdk',
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        'rclpy',
        'pyserial',
    ],
    # 声光反馈常驻程序，等价于`python3 -m contest_sdk.sound_light_server`
    entry_points={
        'console_scripts': [
            'contest-sound-light-server = contest_sdk.sound_light_server:main',
        ],
    },
)

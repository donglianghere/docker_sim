"""contest_sdk是纯Python包，不是ROS2功能包（不需要colcon build/ament），
选手容器里用`pip install .`（或者阶段C定下来的镜像COPY方式）安装即可。
这里用最朴素的setuptools写法，不引入colcon/ament相关的样板文件。
"""
from setuptools import setup, find_packages

setup(
    name='contest_sdk',
    version='0.0.1',
    packages=find_packages(),
    # rounds.py 是**顶层**模块（选手程序 `from rounds import ...` 直接吃它），
    # 它 import 的 venue 由选手目录提供——运行器把那个目录挂成 /workspace 并
    # 排在 PYTHONPATH 最前，所以仿真/真机各自绑到自己那份场地表。
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

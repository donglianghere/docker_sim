from setuptools import setup

package_name = 'vision_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/vision_stack.launch.py',
            'launch/apriltag_stack.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/tags.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='机载视觉感知：YOLO目标识别（TensorRT手写推理）+ QR识别 + AprilTag精准降落桥接 + 后续视频回传',
    license='TODO',
    entry_points={
        'console_scripts': [
            'yolo_detector_node = vision_stack.yolo_detector_node:main',
            'qr_detect_node = vision_stack.qr_detect_node:main',
            'image_publisher_node = vision_stack.image_publisher_node:main',
            'video_stream_node = vision_stack.video_stream_node:main',
        ],
    },
)

from setuptools import setup

package_name = 'smart_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'cv_bridge',
        'ultralytics',
        'pillow'
    ],
    zip_safe=True,
    author='Haicoi',
    author_email='thanvanhai1021988@gmail.com',
    description='Multi-Camera Smart Camera ROS2 package with RTSP and YOLOv8',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_camera_launch_rtsp = smart_camera.multi_camera_launch_rtsp:main',
            'yolo_multi_subscriber = smart_camera.yolo_multi_subscriber:main',
            'camera_manager_gui = smart_camera.camera_manager_gui:main'
        ],
    },
)

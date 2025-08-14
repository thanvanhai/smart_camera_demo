from setuptools import find_packages, setup

package_name = 'smart_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haicoi',
    maintainer_email='haicoi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'camera_node = smart_camera.camera_node:main',
            'yolo_node = smart_camera.yolo_node:main',
            'multi_camera_launch = smart_camera.multi_camera_launch:main',
        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_package'

def package_files(directory, pattern="*"):
    return [f for f in glob(os.path.join(directory, pattern)) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        package_files('launch')),
        (os.path.join('share', package_name, 'config'),
        package_files('config', "*.yaml")),
        (os.path.join('share', package_name, 'rviz'),
        package_files('rviz', "*.rviz")),

    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='sharmamanu2727@gmail.com',
    description='Learning ROS',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'minimal_param_node = my_package.python_parameters_node:main',
            'spawn_and_start = turtle_pong.spawn_and_start:main',
            'paddle_teleop = turtle_pong.paddle_teleop:main',
            'ball_node = turtle_pong.ball_node:main',
        ],
    },
)
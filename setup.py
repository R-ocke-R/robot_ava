import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/talker.launch.py',
            'launch/listener.launch.py',
            'launch/python_parameters_launch.py'
        ])
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='sharmamanu2727@gmail.com',
    description='Learning ROS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'minimal_param_node = my_package.python_parameters_node:main',
        ],
    },
)

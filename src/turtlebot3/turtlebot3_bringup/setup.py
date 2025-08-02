from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_bringup'

setup(
    name=package_name,
    version='2.1.4',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROBOTIS',
    author_email='willson@robotis.com',
    maintainer='ROBOTIS',
    maintainer_email='willson@robotis.com',
    description='ROS 2 launch scripts for starting the TurtleBot3',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_ros_create_script = turtlebot3_bringup.script.create_udev_rules:main',
            'web_bridge_pi = turtlebot3_bringup.launch.web_bridge_pi:main',
        ],
    },
) 
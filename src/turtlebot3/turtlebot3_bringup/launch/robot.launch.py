#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import launch.conditions


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    
    # Xử lý model LiDAR, bao gồm lds02rr
    LDS_MODEL = os.environ.get('LDS_MODEL', 'lds02rr')
    
    namespace = LaunchConfiguration('namespace', default='')
    
    # Thay thế usb_port bằng tham số I2C
    i2c_device = LaunchConfiguration('i2c_device', default='/dev/i2c-1')
    i2c_address = LaunchConfiguration('i2c_address', default='8')  # 0x08 trong hệ thập phân

    if ROS_DISTRO == 'humble':
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                TURTLEBOT3_MODEL + '.yaml'))
    else:
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                TURTLEBOT3_MODEL + '.yaml'))

    # Chọn gói LiDAR phù hợp
    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/hlds_laser.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('lds02rr_lidar'), 'launch'))
        LDS_LAUNCH_FILE = '/lds02rr.launch.py'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_slam = LaunchConfiguration('enable_slam', default='true')
    enable_explorer = LaunchConfiguration('enable_explorer', default='true')  # Mặc định enable explorer_bringup

    # Lấy đường dẫn đến file pwm.py trong cùng thư mục
    pwm_script = os.path.join(os.path.dirname(__file__), 'pwm.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        # Thay thế tham số usb_port bằng tham số I2C
        DeclareLaunchArgument(
            'i2c_device',
            default_value=i2c_device,
            description='I2C device for Xiao BLE (e.g. /dev/i2c-1)'),
            
        DeclareLaunchArgument(
            'i2c_address',
            default_value=i2c_address,
            description='I2C address (decimal) for Xiao BLE'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),

        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Enable SLAM Cartographer after robot setup (true/false)'),

        DeclareLaunchArgument(
            'enable_explorer',
            default_value='true',
            description='Enable explorer_bringup integration (true/false)'),

        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyAMA0',  # Có thể cần điều chỉnh tùy thuộc vào lds02rr
                              'frame_id': 'base_scan',
                              'namespace': namespace}.items(),
        ),

        # Cập nhật node turtlebot3_ros để sử dụng XiaoBLEI2CWrapper
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[
                tb3_param_dir,
                {'namespace': namespace},
                {'i2c_device': i2c_device},
                {'i2c_address': i2c_address},
                {'hardware_interface': 'xiao_ble_i2c'}  # Chỉ định sử dụng XiaoBLEI2CWrapper thay vì Dynamixel
            ],
            # Xóa usb_port argument và thay bằng tham số phù hợp nếu cần
            output='screen'),

        # Thêm việc chạy pwm.py
        # ExecuteProcess(
        #     cmd=['python3', pwm_script],
        #     output='screen'
        # ),

        # Thêm Web Bridge để streaming dữ liệu qua WebSocket
        ExecuteProcess(
            cmd=['python3', os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch',
                'web_bridge_pi.py'
            )],
            output='screen'
        ),


        # # Delay 5 giây rồi khởi động SLAM Cartographer tự động (nếu enable_slam=true)
        # TimerAction(
        #     period=8.0,  # Đợi 5 giây cho robot setup xong (sau khi IMU calibration hoàn tất)
        #     actions=[
        #         # SLAM Cartographer (không RViz để tiết kiệm tài nguyên Pi Zero 2W)
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([
        #                 get_package_share_directory('turtlebot3_cartographer'), 
        #                 '/launch/cartographer.launch.py'
        #             ]),
        #             launch_arguments={
        #                 'use_sim_time': 'false',
        #                 'use_rviz': 'false'  # Tắt RViz để tiết kiệm tài nguyên Pi Zero 2W
        #             }.items(),
        #         ),

        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([
        #                 get_package_share_directory('turtlebot3_navigation2'), 
        #                 '/launch/navigation2.launch.py'
        #             ]),
        #             launch_arguments={
        #                 'use_sim_time': 'false',
        #                 'use_rviz': 'false'  # Tắt RViz để tiết kiệm tài nguyên Pi Zero 2W
        #             }.items(),
        #         ),

        #         ExecuteProcess(
        #             cmd=['ros2', 'launch', 'explorer_bringup', 'explorer.launch.py'],
        #             output='screen',
        #             condition=launch.conditions.IfCondition(enable_explorer)
        #         ),

        #         ExecuteProcess(
        #             cmd=['ros2', 'run', 'explorer_bringup', 'manager'],
        #             output='screen',
        #             condition=launch.conditions.IfCondition(enable_explorer)
        #         ),


        #     ],
        #     condition=launch.conditions.IfCondition(enable_slam)
        # ),
    ])

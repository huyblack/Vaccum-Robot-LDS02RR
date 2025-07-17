#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Custom SLAM Toolbox config for LDS-02 LiDAR
    slam_config = os.path.join(
        turtlebot3_bringup_dir,
        'config',
        'slam_toolbox_lds02.yaml'
    )
    
    # Get URDF file
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file = os.path.join(
        turtlebot3_description_dir,
        'urdf',
        f'turtlebot3_{TURTLEBOT3_MODEL}.urdf'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': False
        }]
    )
    
    # LDS02RR LiDAR
    lds02rr_node = Node(
        package='lds02rr_lidar',
        executable='lds02rr_node',
        name='lds02rr_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyAMA0',
            'frame_id': 'base_scan',
        }]
    )
    
    # TurtleBot3 node
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        name='turtlebot3_node',
        output='screen'
    )
    
    # SLAM toolbox with custom config
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            slam_toolbox_dir, '/launch/online_async_launch.py'
        ]),
        launch_arguments={'params_file': slam_config}.items()
    )
    
    return LaunchDescription([
        robot_state_publisher,
        lds02rr_node,
        turtlebot3_node,
        slam_launch,
    ]) 
#!/usr/bin/env python3

"""
Launch file cho hệ thống BCD Coverage
Chạy thuật toán Boustrophedon Cellular Decomposition cho robot hút bụi
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
import os


def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start rviz2 automatically if this flag is true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="true",
            description="Start robot_state_publisher automatically if this flag is true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Use simulation (Gazebo) if this flag is true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("turtlebot3_gazebo"), "worlds", "empty_world.world"]
            ),
            description="Full path to the world model file to load",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "model",
            default_value="burger",
            description="model type [burger, waffle, waffle_pi]",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if this flag is true",
        )
    )

    # Initialize Arguments
    use_rviz = LaunchConfiguration("use_rviz")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_sim = LaunchConfiguration("use_sim")
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("turtlebot3_description"), "urdf", "turtlebot3.urdf.xacro"]
            ),
            " ",
            "model:=",
            model,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_robot_state_pub),
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_factory.so",
            "-s",
            "libgazebo_ros_init.so",
            world,
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    # Spawn robot
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "turtlebot3",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    # SLAM
    slam_toolbox = Node(
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "config", "mapper_params_online_async.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )

    # Navigation2
    nav2_bringup_cmd = Node(
        package="nav2_bringup",
        executable="navigation_launch.py",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("turtlebot3_navigation2"), "param", "burger.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_sim),
    )

    # BCD Coverage Node
    bcd_coverage_node = Node(
        package="explorer_wanderer",
        executable="bcd_coverage",
        name="bcd_coverage",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # BCD Controller Node
    bcd_controller_node = Node(
        package="explorer_wanderer",
        executable="bcd_controller",
        name="bcd_controller",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # RVIZ2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_navigation2"), "rviz", "turtlebot3_navigation.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Wait for Gazebo to start
    wait_for_gazebo = TimerAction(
        period=5.0,
        actions=[
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gazebo,
                    on_exit=[slam_toolbox, nav2_bringup_cmd, bcd_coverage_node, bcd_controller_node],
                ),
                condition=IfCondition(use_sim),
            )
        ],
    )

    nodes = [
        robot_state_pub_node,
        gazebo,
        spawn_entity_cmd,
        wait_for_gazebo,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes) 
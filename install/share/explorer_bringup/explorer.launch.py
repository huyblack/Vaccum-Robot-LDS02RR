import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def launch_setup(context, *args, **kwargs):

    # map_name = LaunchConfiguration('map_name', default='map10')
    nav2_file_dir = get_package_share_directory('turtlebot3_navigation2')
 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    x_pose = LaunchConfiguration('x_pose', default='2.0')
    y_pose = LaunchConfiguration('y_pose', default='3.0')

    param_file_name = TURTLEBOT3_MODEL + '.yaml'

    # Loại bỏ Cartographer vì đã có trong robot.launch.py
    # cartographer_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
    #     ),
    #         launch_arguments={
    #             'use_sim_time': use_sim_time,
    #         }.items()
    # )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_file_dir, 'param', param_file_name)}.items(),
    )

    wanderer_cmd = Node(
        package='explorer_wanderer',
        executable='wanderer_server',
        name='wanderer_server',
        output='screen',
    )

    discoverer_cmd = Node(
        package='explorer_wanderer',
        executable='discoverer_server',
        name='discoverer_server',
        output='screen',
    )

    # watchtower_cmd = Node(
    #     package='explorer_map_utils',
    #     executable='watchtower',
    #     name='watchtower',
    #     output='screen',
    #     parameters=[{'map_name': map_name}],
    # )

    return [
        # cartographer_cmd,  # Loại bỏ Cartographer
        nav2_cmd,
        wanderer_cmd,
        # discoverer_cmd,
        # watchtower_cmd,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
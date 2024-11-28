from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def get_path(package, dir, file):
    return os.path.join(
        get_package_share_directory(package),
        dir,
        file
    )

def generate_launch_description():
    rviz2_config = get_path("ackermann_ekf", "config", "conf.rviz")

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_world_map',
            output='screen',
            arguments=[
                '0', '0', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                '0', '0', '0', '1', # Rotation: 0
                'world',
                'map'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            output='screen',
            arguments=[
                '0', '0', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                '0', '0', '0', '1', # Rotation: 0
                'map',
                'odom'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_odom_base_link',
            output='screen',
            arguments=[
                '0', '0', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                '0', '0', '0', '1', # Rotation: 0
                'odom',
                'meow_link'
            ]
        ),
        Node(
            package='ackermann_ekf',
            executable='ackermann_ekf',
            name='ackermann_ekf_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['--display-config', rviz2_config],
            name='rviz2',
            output='screen',
        )
    ])
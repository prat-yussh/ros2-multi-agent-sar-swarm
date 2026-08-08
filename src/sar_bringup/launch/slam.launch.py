from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'robot_1/odom',
                'base_frame': 'robot_1/base_link',
                'map_frame': 'map',
                'max_laser_range': 30.0,
                'resolution': 0.05
            }],
            remappings=[('/scan', '/robot_1/scan')]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])

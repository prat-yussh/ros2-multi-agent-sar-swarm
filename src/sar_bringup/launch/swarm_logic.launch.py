from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sar_logic', executable='robot_controller', name='brain_1', parameters=[{'robot_name': 'robot_1'}], output='screen'),
        Node(package='sar_logic', executable='robot_controller', name='brain_2', parameters=[{'robot_name': 'robot_2'}], output='screen'),
        Node(package='sar_logic', executable='robot_controller', name='brain_3', parameters=[{'robot_name': 'robot_3'}], output='screen')
    ])

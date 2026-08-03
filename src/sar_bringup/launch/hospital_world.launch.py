import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_sar_gazebo = get_package_share_directory('sar_gazebo')
    pkg_sar_bringup = get_package_share_directory('sar_bringup')
    
    world_file = os.path.join(pkg_sar_gazebo, 'worlds', 'disaster_hospital.sdf')
    spawn_launch = os.path.join(pkg_sar_bringup, 'launch', 'spawn_robot.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_file} -r'}.items()
    )
    
    launch_actions = [gazebo]

    # THE FIX: Squeezed together to perfectly fit through the 2-meter doorway!
    robots = [
        {'name': 'robot_1', 'x': '0.0', 'y': '12.0', 'yaw': '-1.57', 'color': '0 0.2 0.8 1'},
        {'name': 'robot_2', 'x': '0.7', 'y': '12.0', 'yaw': '-1.57', 'color': '0 0.8 0.2 1'},
        {'name': 'robot_3', 'x': '-0.7','y': '12.0', 'yaw': '-1.57', 'color': '0.8 0.8 0 1'}
    ]

    for bot in robots:
        launch_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={'robot_name': bot['name'], 'x': bot['x'], 'y': bot['y'], 'yaw': bot['yaw'], 'robot_color': bot['color']}.items()
        ))

    return LaunchDescription(launch_actions)

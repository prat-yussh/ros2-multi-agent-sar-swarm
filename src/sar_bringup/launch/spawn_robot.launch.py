import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sar_gazebo = get_package_share_directory('sar_gazebo')
    xacro_file = os.path.join(pkg_sar_gazebo, 'urdf', 'rescue_robot.xacro')

    name = LaunchConfiguration('robot_name')
    color = LaunchConfiguration('robot_color')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    
    robot_desc = Command(['xacro ', xacro_file, ' robot_name:=', name, " color_rgba:='", color, "'"])

    # THE FIX: Wrapped robot_desc in ParameterValue(..., value_type=str)
    rsp = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher', 
        namespace=name, 
        parameters=[{'robot_description': ParameterValue(robot_desc, value_type=str)}]
    )
    
    spawn = Node(package='ros_gz_sim', executable='create', arguments=['-name', name, '-string', robot_desc, '-x', x, '-y', y, '-z', '0.2', '-Y', yaw], output='screen')
    
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name=['bridge_', name], arguments=[
        ['/', name, '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        ['/', name, '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        ['/', name, '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
    ], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name'), 
        DeclareLaunchArgument('robot_color'), 
        DeclareLaunchArgument('x'), 
        DeclareLaunchArgument('y'), 
        DeclareLaunchArgument('yaw'), 
        rsp, spawn, bridge
    ])

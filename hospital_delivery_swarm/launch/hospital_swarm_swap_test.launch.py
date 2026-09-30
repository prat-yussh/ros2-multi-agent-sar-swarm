#!/usr/bin/env python3
"""
hospital_swarm.launch.py  (v4 - home pads aligned under each robot's own doorway)

CHANGE FROM v3: home positions moved from fixed hallway spots (-10/0/10) to sit
directly below each robot's own ward doorway (same x as the ward). This removes
the need for any lateral hallway travel before turning into the room, which was
causing large in-place rotations that could destabilize (see hospital_navigator.py
v6 for the matching hysteresis fix).

Missions (edit ROBOTS below to change assignments):
  robot1: home (-8, 0) -> Pharmacy       (-8, 5.25)
  robot2: home ( 0, 0) -> Patient Room 1 ( 0, 5.25)
  robot3: home ( 8, 0) -> Patient Room 2 ( 8, 5.25)

Set LOOP_MISSIONS = True below to make every robot repeat its delivery run
indefinitely instead of stopping after one trip.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


PACKAGE_NAME = 'hospital_swarm'

# (robot_name, home_x, home_y, ward_x, ward_y, ward_name)
ROBOTS = [
    ('robot1',  0.0, 0.0,  0.0, 5.25, 'Patient Room 1'),
    ('robot2', -8.0, 0.0, -8.0, 5.25, 'Pharmacy'),
    ('robot3',  8.0, 0.0,  8.0, 5.25, 'Patient Room 2'),
]

LOOP_MISSIONS = False

FIRST_SPAWN_DELAY = 15.0
SPAWN_STAGGER = 6.0
BRIDGE_AFTER_SPAWN_DELAY = 4.0


def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    world_path = os.path.join(pkg_share, 'worlds', 'hospital_ward.sdf')
    template_path = os.path.join(pkg_share, 'models', 'diff_drive_robot', 'model.sdf.template')
    generator_script = os.path.join(pkg_share, 'scripts', 'generate_robot_sdf.py')

    generated_dir = os.path.expanduser('~/.hospital_swarm_generated')
    os.makedirs(generated_dir, exist_ok=True)

    actions = []

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )
    actions.append(gz_sim)

    spawn_delay = FIRST_SPAWN_DELAY
    for (name, hx, hy, wx, wy, ward_name) in ROBOTS:
        robot_sdf_path = os.path.join(generated_dir, f'{name}.sdf')
        subprocess.run(
            ['python3', generator_script, template_path, name, robot_sdf_path],
            check=True
        )

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name,
                '-file', robot_sdf_path,
                '-x', str(hx), '-y', str(hy), '-z', '0.1',
                '-Y', '0',
            ],
            output='screen'
        )

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            output='screen'
        )

        navigator_args = [
            '--robot_name', name,
            '--home_x', str(hx), '--home_y', str(hy),
            '--ward_x', str(wx), '--ward_y', str(wy),
            '--ward_name', ward_name,
        ]
        if LOOP_MISSIONS:
            navigator_args.append('--loop')

        navigator_node = Node(
            package=PACKAGE_NAME,
            executable='hospital_navigator.py',
            name=f'{name}_navigator',
            arguments=navigator_args,
            output='screen'
        )

        actions.append(TimerAction(period=spawn_delay, actions=[spawn_node]))
        actions.append(TimerAction(
            period=spawn_delay + BRIDGE_AFTER_SPAWN_DELAY,
            actions=[bridge_node, navigator_node]
        ))

        spawn_delay += SPAWN_STAGGER

    return LaunchDescription(actions)

"""Mapping tour: drive through every room while the OCGM is built and shown live.

Needs the simulator running (ros2 launch robot_sim sim.launch.py).

    ros2 launch robot_sim tour.launch.py output_dir:=/path/to/ocgm_dir
    ros2 launch robot_sim tour.launch.py mode:=teleport     # kinematic, no physics

Starts
  * live_ocgm.py     - writes data.csv, builds the map, opens the live map
                       window, saves ocgm_<id>.csv every few seconds
  * path_follower.py - drives a route that covers every room (starts 6 s later so the first
                       scan is taken at the spawn point)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    output_dir = LaunchConfiguration('output_dir')

    args = [
        DeclareLaunchArgument('output_dir', default_value=os.getcwd(),
                              description='Where data.csv and ocgm_<id>.csv are written'),
        DeclareLaunchArgument('mode', default_value='drive',
                              description="'drive' (wheels, physics) or 'teleport'"),
        DeclareLaunchArgument('speed', default_value='0.5'),
        DeclareLaunchArgument('coverage', default_value='true',
                              description='Auto stops that see every room (false = named route)'),
    ]

    live_map = Node(
        package='robot_sim',
        executable='live_ocgm.py',
        output='screen',
        parameters=[{
            'output': PathJoinSubstitution([output_dir, 'data.csv']),
            'output_dir': output_dir,
        }],
    )

    follower = Node(
        package='robot_sim',
        executable='path_follower.py',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'speed': LaunchConfiguration('speed'),
            'coverage': LaunchConfiguration('coverage'),
        }],
    )

    return LaunchDescription(args + [live_map, TimerAction(period=6.0, actions=[follower])])

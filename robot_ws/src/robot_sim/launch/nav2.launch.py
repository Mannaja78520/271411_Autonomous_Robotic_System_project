"""Nav2 autonomous navigation.

Two modes:

  slam:=True   (default)  Nav2 builds the map online with slam_toolbox.
  slam:=False  map:=<file.yaml>  Nav2 localises with AMCL on a saved map.

    ros2 launch robot_sim nav2.launch.py
    ros2 launch robot_sim nav2.launch.py slam:=False map:=$HOME/smart_building_map.yaml

Then send goals with the "2D Goal Pose" tool in RViz, or from the command line
with the nav2_simple_commander helper in scripts/nav_to_room.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_path, 'maps', 'smart_building_map.yaml')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': 'True',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        bringup,
    ])

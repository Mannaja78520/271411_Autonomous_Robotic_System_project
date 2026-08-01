"""Online asynchronous SLAM (slam_toolbox) from the 2D LiDAR.

    ros2 launch robot_sim slam.launch.py

Publishes the map on /map and the map->odom transform.  Save the result with:

    ros2 run nav2_map_server map_saver_cli -f ~/smart_building_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('robot_sim'),
                               'config', 'slam_params.yaml')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        slam,
    ])

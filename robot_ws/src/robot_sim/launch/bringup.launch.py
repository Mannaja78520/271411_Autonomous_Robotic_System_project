"""One-shot bring-up for the demo: Gazebo + robot + RViz, and optionally SLAM,
Nav2 and the keyboard teleop terminal.

    # manual driving with RViz showing the LiDAR
    ros2 launch robot_sim bringup.launch.py

    # manual driving + live SLAM map
    ros2 launch robot_sim bringup.launch.py slam:=true

    # full autonomy: SLAM + Nav2, send goals from RViz
    ros2 launch robot_sim bringup.launch.py nav2:=true

Arguments: rviz, teleop, slam, nav2 (all true/false).  nav2:=true implies slam.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _include(pkg_path, name, condition=None, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', name)),
        launch_arguments=kwargs.items(),
        condition=condition,
    )


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_sim')

    args = [
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav2', default_value='false'),
    ]

    # Nav2 brings up its own slam_toolbox instance (bringup_launch with
    # slam:=True), so the standalone SLAM node must NOT also be started when
    # nav2 is requested -- two mapping nodes would fight over map->odom.
    want_slam = PythonExpression(["'true' if '",
                                  LaunchConfiguration('slam'),
                                  "' == 'true' and '",
                                  LaunchConfiguration('nav2'),
                                  "' != 'true' else 'false'"])

    sim = _include(pkg_path, 'sim.launch.py', rviz='false')
    rviz = _include(pkg_path, 'rviz.launch.py',
                    condition=IfCondition(LaunchConfiguration('rviz')))

    # Give Gazebo time to publish /clock and the first scan before the mapping
    # and navigation stacks start looking for transforms.
    slam = TimerAction(period=8.0, actions=[
        _include(pkg_path, 'slam.launch.py', condition=IfCondition(want_slam))])
    nav2 = TimerAction(period=12.0, actions=[
        _include(pkg_path, 'nav2.launch.py', slam='True',
                 condition=IfCondition(LaunchConfiguration('nav2')))])
    teleop = TimerAction(period=6.0, actions=[
        _include(pkg_path, 'keyboard_control.launch.py',
                 condition=IfCondition(LaunchConfiguration('teleop')))])

    return LaunchDescription(args + [sim, rviz, slam, nav2, teleop])

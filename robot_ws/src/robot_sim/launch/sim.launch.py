"""Bring up Gazebo with the smart_building world and spawn the robot.

    ros2 launch robot_sim sim.launch.py
    ros2 launch robot_sim sim.launch.py rviz:=true
    ros2 launch robot_sim sim.launch.py x:=0.0 y:=0.0 yaw:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GAZEBO_PREFIX = '/usr/share/gazebo-11'


def gazebo_environment():
    """Set the paths that /usr/share/gazebo/setup.sh would normally export.

    Without them Gazebo cannot find its material scripts and falls back to
    querying the online model database, which stalls world loading on a
    machine with no internet access.
    """
    return [
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            f'{GAZEBO_PREFIX}:{os.environ.get("GAZEBO_RESOURCE_PATH", "")}'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            f'{GAZEBO_PREFIX}/models:{os.environ.get("GAZEBO_MODEL_PATH", "")}'),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:'
            f'{os.environ.get("GAZEBO_PLUGIN_PATH", "")}'),
        SetEnvironmentVariable('OGRE_RESOURCE_PATH',
                               '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0'),
        # Everything in this world is built from primitives, so never go online.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
    ]


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_sim')
    world_file = os.path.join(pkg_path, 'worlds', 'smart_building.world')
    rviz_config = os.path.join(pkg_path, 'config', 'main.rviz')

    args = [
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also start RViz2'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo client window'),
        # Spawn point: main corridor, west end, facing east (+x)
        DeclareLaunchArgument('x', default_value='-13.5'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'scout_bot',
            # Loading the 162-model world can take longer than the default 30 s.
            '-timeout', '180',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + gazebo_environment() +
                             [rsp, gazebo, spawn_entity, rviz])

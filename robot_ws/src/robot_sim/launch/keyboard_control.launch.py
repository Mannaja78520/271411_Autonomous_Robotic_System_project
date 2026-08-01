"""Start the keyboard teleop node in its own terminal.

The node reads raw keystrokes with termios, so it needs a real TTY.  When
launched from `ros2 launch` the process is given a pipe instead, therefore the
node is started inside a separate terminal emulator (xterm, or gnome-terminal
as a fallback).  If neither is installed, run it directly instead:

    ros2 run robot_sim keyboard_control.py
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    params = os.path.join(get_package_share_directory('robot_sim'),
                          'config', 'speed_control.yaml')
    command = ('ros2 run robot_sim keyboard_control.py '
               f'--ros-args --params-file {params}')

    if shutil.which('xterm'):
        proc = ExecuteProcess(
            cmd=['xterm', '-title', 'keyboard_control', '-geometry', '90x35',
                 '-e', command],
            output='screen')
    elif shutil.which('gnome-terminal'):
        proc = ExecuteProcess(
            cmd=['gnome-terminal', '--title=keyboard_control', '--',
                 'bash', '-lc', command],
            output='screen')
    else:
        return LaunchDescription([
            LogInfo(msg='No terminal emulator found (install xterm). '
                        'Run instead:  ros2 run robot_sim keyboard_control.py'),
        ])

    return LaunchDescription([proc])

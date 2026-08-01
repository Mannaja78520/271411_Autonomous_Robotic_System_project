#!/usr/bin/env python3
"""Send the robot to a named room using Nav2.

Requires Nav2 to be running (ros2 launch robot_sim nav2.launch.py).

    ros2 run robot_sim nav_to_room.py --list
    ros2 run robot_sim nav_to_room.py meeting_room
    ros2 run robot_sim nav_to_room.py office cafeteria lab      # tour

The room table lives in config/locations.yaml so it can be edited without
touching the code.  Poses are expressed in the `map` frame; with SLAM running
from the spawn point, `map` and `odom` start aligned, so the world coordinates
in the YAML file work directly.
"""

import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def load_locations():
    path = os.path.join(get_package_share_directory('robot_sim'),
                        'config', 'locations.yaml')
    with open(path) as fh:
        return yaml.safe_load(fh)['locations']


def make_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(float(yaw_deg))
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    locations = load_locations()
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]

    if not args or args[0] in ('--list', '-l', '-h', '--help'):
        print('Known rooms:')
        for name, v in locations.items():
            print(f'  {name:<16} x={v["x"]:>7.2f}  y={v["y"]:>7.2f}  yaw={v["yaw"]:>6.1f} deg')
        print('\nUsage: ros2 run robot_sim nav_to_room.py <room> [<room> ...]')
        return

    unknown = [a for a in args if a not in locations]
    if unknown:
        print(f'Unknown room(s): {", ".join(unknown)}')
        print(f'Known: {", ".join(locations)}')
        sys.exit(1)

    rclpy.init()
    navigator = BasicNavigator()

    # waitUntilNav2Active() blocks on AMCL's lifecycle state, and AMCL does not
    # exist when Nav2 runs with slam:=True (slam_toolbox is not a lifecycle
    # node either), so that call would hang forever in SLAM mode.  Detect which
    # localiser is running and only use the helper when AMCL is really there;
    # goToPose() waits for the NavigateToPose action server on its own.
    services = []
    for _ in range(20):
        services = [name for name, _ in navigator.get_service_names_and_types()]
        if any(s.startswith('/amcl/') or s.startswith('/slam_toolbox/')
               for s in services):
            break
        time.sleep(0.5)

    if any(s.startswith('/amcl/') for s in services):
        print('Waiting for Nav2 to become active (AMCL on a saved map) ...',
              flush=True)
        navigator.waitUntilNav2Active()
    else:
        print('Waiting for Nav2 to become active (online SLAM) ...', flush=True)
        for _ in range(120):
            topics = dict(navigator.get_topic_names_and_types())
            if '/map' in topics and navigator.count_publishers('/map') > 0:
                break
            time.sleep(0.5)
        else:
            print('No /map after 60 s - is SLAM running?')
            sys.exit(1)
        time.sleep(2.0)   # let the first map->odom transform settle

    for room in args:
        v = locations[room]
        goal = make_pose(navigator, v['x'], v['y'], v['yaw'])
        print(f'--> {room} ({v["x"]:.2f}, {v["y"]:.2f})', flush=True)
        navigator.goToPose(goal)

        last_report = -1
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback is None:
                continue
            elapsed = feedback.navigation_time.sec
            if elapsed >= last_report + 5:      # one line every 5 s, not per callback
                last_report = elapsed
                print(f'    {feedback.distance_remaining:.2f} m remaining '
                      f'({elapsed} s)', flush=True)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'    arrived at {room}', flush=True)
        elif result == TaskResult.CANCELED:
            print(f'    goal to {room} was canceled')
            break
        else:
            print(f'    failed to reach {room}')
            break

    # Do not call navigator.lifecycleShutdown(): that would tear down the Nav2
    # stack the user launched separately, so the next goal could not be sent.
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

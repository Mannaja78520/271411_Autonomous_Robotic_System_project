#!/usr/bin/env python3
"""Record 2D LiDAR scans together with the sensor pose into data.csv.

ROS 2 / Gazebo counterpart of `save_laser_show_pointcloud.py` from the
CoppeliaSim OCGM example.  Every time the sensor has moved or turned enough, one
row is appended to the CSV:

    col 1        x of the sensor base in the world frame (m)
    col 2        y of the sensor base in the world frame (m)
    col 3        heading of the sensor base w.r.t. the world x axis (rad)
    col 4+2(i-1) range measured by beam i (m), i = 1..N
    col 5+2(i-1) angle of beam i w.r.t. the sensor front (rad)

Beams with no return (nothing inside range_max) are written as range_max; the
OCGM builder treats them as "free up to range_max, no obstacle".

The pose comes from the Gazebo ground-truth plugin (/ground_truth, frame
`world`) composed with the fixed base_link -> laser_frame offset from TF, so
the map is not smeared by wheel-odometry drift.

    ros2 run robot_sim lidar_logger.py --ros-args -p output:=/path/data.csv

Subscribes: /scan (sensor_msgs/LaserScan), /ground_truth (nav_msgs/Odometry)
"""

import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SCAN_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LidarLogger(Node):
    def __init__(self):
        super().__init__('lidar_logger')

        self.declare_parameter('output', os.path.join(os.getcwd(), 'data.csv'))
        self.declare_parameter('min_move', 0.25)        # m between saved scans
        self.declare_parameter('min_turn', 0.35)        # rad between saved scans
        self.declare_parameter('min_period', 0.5)       # s between saved scans
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('base_frame', 'base_link')

        self.output = self.get_parameter('output').value
        self.min_move = self.get_parameter('min_move').value
        self.min_turn = self.get_parameter('min_turn').value
        self.min_period = self.get_parameter('min_period').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.base_pose = None       # (x, y, yaw) of base_link, ground truth
        self.offset = None          # (dx, dy, dyaw) base_link -> laser_frame
        self.last_saved = None      # sensor pose of the last saved row
        self.last_time = None
        self.rows = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        os.makedirs(os.path.dirname(os.path.abspath(self.output)), exist_ok=True)
        self.file = open(self.output, 'w')

        self.create_subscription(Odometry, '/ground_truth', self.on_pose, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SCAN_QOS)

        self.get_logger().info(f'Logging LiDAR scans to {self.output}')

    def on_pose(self, msg: Odometry):
        p = msg.pose.pose
        self.base_pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def lookup_offset(self):
        """Fixed mounting of the laser on the base (published by robot_state_publisher)."""
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.laser_frame, Time())
        except Exception:  # noqa: BLE001 - TF not up yet, try on the next scan
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_from_quaternion(t.transform.rotation))

    def sensor_pose(self):
        bx, by, byaw = self.base_pose
        dx, dy, dyaw = self.offset
        c, s = math.cos(byaw), math.sin(byaw)
        return (bx + c * dx - s * dy, by + s * dx + c * dy, wrap(byaw + dyaw))

    def should_save(self, pose, now):
        if self.last_saved is None:
            return True
        if now - self.last_time < self.min_period:
            return False
        moved = math.hypot(pose[0] - self.last_saved[0], pose[1] - self.last_saved[1])
        turned = abs(wrap(pose[2] - self.last_saved[2]))
        return moved >= self.min_move or turned >= self.min_turn

    def on_scan(self, msg: LaserScan):
        if self.base_pose is None:
            return
        if self.offset is None:
            self.offset = self.lookup_offset()
            if self.offset is None:
                return

        pose = self.sensor_pose()
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.should_save(pose, now):
            return

        ranges = [r if math.isfinite(r) and r <= msg.range_max else msg.range_max
                  for r in msg.ranges]
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(ranges))]

        values = [f'{pose[0]:.4f}', f'{pose[1]:.4f}', f'{pose[2]:.5f}']
        for r, angle in zip(ranges, angles):
            values.append(f'{r:.4f}')
            values.append(f'{angle:.5f}')
        self.file.write(','.join(values) + '\n')
        self.file.flush()
        self.on_saved(pose, ranges, angles, msg.range_min, msg.range_max)

        self.last_saved = pose
        self.last_time = now
        self.rows += 1
        if self.rows % 20 == 0:
            self.get_logger().info(
                f'{self.rows} scans saved, sensor at '
                f'({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f} deg)')

    def on_saved(self, pose, ranges, angles, range_min, range_max):
        """Called for every row written to the CSV; subclasses hook in here."""

    def close(self):
        self.file.close()
        self.get_logger().info(f'Saved {self.rows} scans to {self.output}')


def main():
    rclpy.init()
    node = LidarLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

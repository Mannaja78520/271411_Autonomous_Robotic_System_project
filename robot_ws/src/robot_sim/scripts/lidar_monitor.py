#!/usr/bin/env python3
"""Print a live text readout of the 2D LiDAR.

Useful during the demo video to show that the beams drawn in Gazebo really do
produce range data: it prints the closest obstacle in eight 45-degree sectors
around the robot, plus an ASCII bar for each sector.

Subscribes: /scan (sensor_msgs/LaserScan)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

SCAN_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

# (label, sector centre in degrees).  Every sector is 45 deg wide, and the
# comparison wraps around +-180 so the rear sector is not split in two.
SECTORS = [
    ('FRONT', 0.0),
    ('FRONT-LEFT', 45.0),
    ('LEFT', 90.0),
    ('REAR-LEFT', 135.0),
    ('REAR', 180.0),
    ('REAR-RIGHT', -135.0),
    ('RIGHT', -90.0),
    ('FRONT-RIGHT', -45.0),
]
SECTOR_HALF_WIDTH = 22.5


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor_node')
        self.declare_parameter('period', 0.5)
        self.period = self.get_parameter('period').value
        self.latest = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, SCAN_QOS)
        self.create_timer(self.period, self.report)

    def on_scan(self, msg):
        self.latest = msg

    def report(self):
        msg = self.latest
        if msg is None:
            self.get_logger().warn('waiting for /scan ...')
            return

        mins = {name: msg.range_max for name, _ in SECTORS}
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue
            bearing = math.degrees(wrap(msg.angle_min + i * msg.angle_increment))
            for name, centre in SECTORS:
                offset = abs(math.degrees(wrap(math.radians(bearing - centre))))
                if offset <= SECTOR_HALF_WIDTH:
                    mins[name] = min(mins[name], r)

        lines = [f'--- LiDAR  ({len(msg.ranges)} beams, '
                 f'{math.degrees(msg.angle_max - msg.angle_min):.0f} deg FOV, '
                 f'max {msg.range_max:.1f} m) ---']
        for name, _ in SECTORS:
            d = mins[name]
            bars = int(min(d, 10.0) * 3)
            lines.append(f'{name:<12}{d:6.2f} m  {"#" * bars}')
        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = LidarMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

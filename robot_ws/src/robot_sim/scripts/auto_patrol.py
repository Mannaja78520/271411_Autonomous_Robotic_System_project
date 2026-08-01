#!/usr/bin/env python3
"""Autonomous patrol demo: drive a list of waypoints using /odom feedback
while the 2D LiDAR provides reactive obstacle avoidance.

This is deliberately Nav2-free so the robot can be shown driving itself in the
video without a map or a localisation stack.  Control law:

  * heading controller turns the robot toward the active waypoint,
  * a forward speed profile that scales down with heading error and with the
    distance reported by the front sector of the laser scan,
  * "front" is not an angular cone but the strip the chassis would actually
    sweep if it kept going straight, so a wide doorway ahead does not read as
    an obstacle just because the robot is passing close to a wall,
  * if that strip is shorter than `stop_distance`, the robot rotates in place
    toward the goal (or toward the freer side if the goal side is closed).
    The turn direction is latched while it stays blocked, otherwise it dithers,
  * if it is still blocked after `escape_after` seconds - typically because it
    clipped the edge of a doorway and no rotation can clear it - it reverses
    away (provided the rear is clear) and tries the approach again,
  * a stall watchdog treats "no closer to the waypoint for `stall_timeout`
    seconds" as blocked as well, which catches the case where the robot creeps
    forward at a few mm/s just outside `stop_distance`,
  * heading errors above `turn_in_place` are corrected by rotating on the spot,
    because a skid-steer base arcing through a doorway drifts past it.

Subscribes: /odom (nav_msgs/Odometry), /scan (sensor_msgs/LaserScan)
Publishes : /cmd_vel (geometry_msgs/Twist)
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

SCAN_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

# Default tour: corridor -> office -> meeting room -> cafeteria -> lab
# The pairs at y = +-1.2 sit in the door mouth: they force the robot to line up
# with the opening before it commits, instead of arcing into the door frame.
# Keep every waypoint at least `stop_distance` + `goal_tolerance` (0.8 m) clear
# of furniture, otherwise the robot stops short and can never report arrival.
DEFAULT_ROUTE = [
    -11.0, 0.0,     # corridor, in front of the reception door
    0.0, 0.0,       # corridor centre
    0.0, 1.2,       # office doorway
    0.0, 2.9,       # into the open-plan office
    0.0, 1.2,       # back out through the doorway
    0.0, 0.0,
    9.0, 0.0,       # corridor, in front of the meeting room
    9.0, 1.2,       # meeting-room doorway
    9.0, 3.4,       # into the meeting room
    9.0, 1.2,
    9.0, 0.0,
    0.0, 0.0,
    0.0, -1.2,      # cafeteria doorway
    0.0, -2.7,      # into the cafeteria
    0.0, -1.2,
    0.0, 0.0,
    -11.0, 0.0,     # back to the start
]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AutoPatrol(Node):
    def __init__(self):
        super().__init__('auto_patrol_node')

        self.declare_parameter('waypoints', DEFAULT_ROUTE)
        self.declare_parameter('loop', True)
        self.declare_parameter('max_linear', 0.55)      # m/s
        self.declare_parameter('max_angular', 1.0)      # rad/s
        self.declare_parameter('goal_tolerance', 0.25)  # m
        self.declare_parameter('stop_distance', 0.55)   # m
        self.declare_parameter('slow_distance', 1.80)   # m
        self.declare_parameter('escape_after', 4.0)     # s blocked before reversing
        self.declare_parameter('escape_time', 2.5)      # s spent reversing
        self.declare_parameter('stall_timeout', 6.0)    # s without progress = stuck
        self.declare_parameter('turn_in_place', 1.0)    # rad of heading error
        self.declare_parameter('settle_time', 0.8)      # s of braking at each waypoint
        self.declare_parameter('half_width', 0.35)      # m, half the swept corridor

        flat = self.get_parameter('waypoints').get_parameter_value().double_array_value
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.escape_after = self.get_parameter('escape_after').value
        self.escape_time = self.get_parameter('escape_time').value
        self.stall_timeout = self.get_parameter('stall_timeout').value
        self.turn_in_place = self.get_parameter('turn_in_place').value
        self.settle_time = self.get_parameter('settle_time').value
        self.half_width = self.get_parameter('half_width').value

        self.pose = None          # (x, y, yaw)
        self.sectors = None       # (front, left, right, rear) minimum ranges
        self.index = 0
        self.finished = False
        self.blocked_since = None  # monotonic time the robot first got stuck
        self.turn_sign = 1.0       # latched escape rotation direction
        self.best_distance = float('inf')  # closest approach to the active goal
        self.progress_time = time.monotonic()
        self.settle_until = 0.0    # brake window entered after each waypoint

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SCAN_QOS)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Auto patrol started with {len(self.waypoints)} waypoints.')

    # ------------------------------------------------------------------ input
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_scan(self, msg: LaserScan):
        def sector_min(centre_deg, half_width_deg):
            """Closest valid return inside a sector, wrapping across +-180."""
            centre = math.radians(centre_deg)
            half = math.radians(half_width_deg)
            best = msg.range_max
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r < msg.range_min:
                    continue
                bearing = msg.angle_min + i * msg.angle_increment
                if abs(wrap(bearing - centre)) <= half:
                    best = min(best, r)
            return best

        def corridor_min(half_width, backwards=False):
            """Closest return inside the straight strip the robot would sweep.

            A fixed angular cone flags the robot as blocked whenever it passes
            near a wall, even with a wide doorway dead ahead.  Projecting each
            beam into robot coordinates and keeping only those inside a strip
            as wide as the chassis answers the question that actually matters:
            is anything in the way if I keep driving straight?
            """
            best = msg.range_max
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r < msg.range_min:
                    continue
                bearing = msg.angle_min + i * msg.angle_increment
                forward = r * math.cos(bearing)
                lateral = r * math.sin(bearing)
                if backwards:
                    forward = -forward
                if forward > 0.0 and abs(lateral) <= half_width:
                    best = min(best, forward)
            return best

        self.sectors = (
            corridor_min(self.half_width),                # front
            sector_min(60.0, 35.0),                       # left
            sector_min(-60.0, 35.0),                      # right
            corridor_min(self.half_width, backwards=True),  # rear
        )

    # ---------------------------------------------------------------- control
    def escape_command(self, cmd, now, left, right, rear, heading_error):
        """Fill `cmd` with a recovery motion for a blocked or stalled robot."""
        if self.blocked_since is None:
            self.blocked_since = now
            # Turn toward the goal when that side has room, otherwise toward
            # whichever side is more open.  Latched until the robot is free
            # again, so it commits instead of dithering left-right.
            goal_side = left if heading_error > 0 else right
            if goal_side > self.stop_distance:
                self.turn_sign = math.copysign(1.0, heading_error)
            else:
                self.turn_sign = 1.0 if left > right else -1.0

        blocked_for = now - self.blocked_since

        if blocked_for > self.escape_after and rear > 0.6:
            # Rotating alone cannot clear it (typically a doorway edge):
            # reverse out and let the heading controller line up again.
            cmd.linear.x = -0.30
            cmd.angular.z = 0.8 * self.turn_sign
            if blocked_for > self.escape_after + self.escape_time:
                self.blocked_since = None
                self.best_distance = float('inf')
                self.progress_time = now
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular * self.turn_sign

    def control_loop(self):
        if self.pose is None or self.finished:
            return

        if time.monotonic() < self.settle_until:
            self.cmd_pub.publish(Twist())
            return

        x, y, yaw = self.pose
        gx, gy = self.waypoints[self.index]
        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.index + 1}/{len(self.waypoints)} '
                f'({gx:.1f}, {gy:.1f})')
            self.index += 1
            self.best_distance = float('inf')
            self.progress_time = time.monotonic()
            # Brake before chasing the next waypoint: a skid-steer base that
            # starts turning while still rolling slides off the door centre.
            self.settle_until = self.progress_time + self.settle_time
            if self.index >= len(self.waypoints):
                if not self.loop:
                    self.finished = True
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info('Patrol complete.')
                    return
                self.index = 0
            return

        heading_error = wrap(math.atan2(dy, dx) - yaw)
        front, left, right, rear = (self.sectors if self.sectors
                                    else (99.0, 99.0, 99.0, 99.0))

        now = time.monotonic()
        cmd = Twist()

        # Stall watchdog, measured against the *goal*: creeping forward at a few
        # mm/s just outside `stop_distance`, or wedging against a door frame,
        # both leave the distance to the waypoint unchanged.
        if distance < self.best_distance - 0.10:
            self.best_distance = distance
            self.progress_time = now
        stalled = (now - self.progress_time) > self.stall_timeout

        if front < self.stop_distance:
            # Something physically in the way.
            self.escape_command(cmd, now, left, right, rear, heading_error)
        elif abs(heading_error) > self.turn_in_place:
            # Big heading change: rotate on the spot instead of arcing, so the
            # skid-steer base does not drift past a doorway while it turns.
            # Deliberate turning is progress, so the watchdog is held off.
            self.blocked_since = None
            self.progress_time = now
            cmd.angular.z = math.copysign(self.max_angular, heading_error)
        elif stalled:
            # Path is nominally clear but the robot is getting nowhere.
            self.escape_command(cmd, now, left, right, rear, heading_error)
        else:
            self.blocked_since = None
            # Slow down for tight headings and for approaching obstacles.
            heading_gain = max(0.0, math.cos(heading_error))
            clearance_gain = min(1.0, (front - self.stop_distance) /
                                 max(self.slow_distance - self.stop_distance, 1e-3))
            approach_gain = min(1.0, distance / 1.0)

            cmd.linear.x = self.max_linear * heading_gain * clearance_gain * approach_gain
            cmd.angular.z = max(-self.max_angular,
                                min(self.max_angular, 1.6 * heading_error))

            # Gentle wall-following bias while squeezing through doorways.
            if left < 0.6:
                cmd.angular.z -= 0.4
            if right < 0.6:
                cmd.angular.z += 0.4

        self.cmd_pub.publish(cmd)

    def stop(self):
        # On SIGTERM rclpy tears the context down before this runs, and
        # publishing into a dead context raises RCLError.
        if rclpy.ok():
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = AutoPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

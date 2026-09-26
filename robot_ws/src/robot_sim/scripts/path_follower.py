#!/usr/bin/env python3
"""Move the robot through every room along a planned, collision-free path.

Counterpart of the CoppeliaSim OCGM example, where the sensor follows a path
drawn in the scene.  Instead of relying on a navigation stack (which can wedge
the skid-steer base in a doorway and ruin a long mapping run), the path is
planned offline and the robot tracks it:

1. The world file is rasterised into a 5 cm obstacle grid (same code as
   tools/check_world.py) and inflated by `clearance`.
2. A* runs between consecutive named locations from config/locations.yaml.
   The step cost grows near obstacles, so the path keeps to the middle of
   corridors and doorways.
3. Pure pursuit tracks the path.  In `mode:=drive` (default) the robot really
   drives: /cmd_vel goes to the diff-drive plugin, the wheels turn, and the
   Gazebo ground-truth pose closes the loop.  If the base stops making
   progress for `stuck_timeout` seconds (wedged on a door frame), it is lifted
   back onto the path through /set_entity_state and carries on.
   In `mode:=teleport` a kinematic unicycle is integrated instead and its pose
   written into Gazebo every step - no physics, never gets stuck.

By default (`coverage:=true`) the stops are not the named rooms but a set
chosen automatically so that, between them, the LiDAR sees every corner of
the building; `coverage:=false` drives the named `route` instead.

The LiDAR keeps scanning the whole time, so running lidar_logger.py alongside
records a complete tour of the building.

    ros2 run robot_sim path_follower.py
    ros2 run robot_sim path_follower.py --ros-args -p coverage:=false \
        -p route:="['reception', 'office', 'home']" -p speed:=0.3

Needs: Gazebo running the smart_building world with the robot spawned.
"""

import heapq
from collections import deque
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt

PKG = get_package_share_directory('robot_sim')
sys.path.insert(0, os.path.join(PKG, 'tools'))
import check_world as cw  # noqa: E402  (rasteriser shared with the world checker)

DEFAULT_ROUTE = [
    'corridor_west', 'reception', 'lounge', 'server_room', 'lounge',
    'office', 'office_north', 'meeting_room', 'meeting_north', 'corridor_east',
    'lab_north', 'lab', 'lab_test_area', 'cafeteria',
    'warehouse_aisle1', 'warehouse_aisle2', 'warehouse_aisle3', 'warehouse', 'home',
]


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ------------------------------------------------------------------ planning
class Planner:
    def __init__(self, world_file, clearance):
        grid = cw.rasterise(cw.collect_shapes(world_file))      # [iy, ix]
        self.res = cw.RESOLUTION
        self.extent = cw.EXTENT
        self.blocked = cw.inflate(grid, clearance)
        # Distance to the nearest real obstacle, in metres.
        self.dist = distance_transform_edt(~grid) * self.res

    def to_cell(self, x, y):
        return (int((y + self.extent) / self.res), int((x + self.extent) / self.res))

    def to_world(self, cell):
        iy, ix = cell
        return ((ix + 0.5) * self.res - self.extent, (iy + 0.5) * self.res - self.extent)

    def plan(self, start, goal):
        """A* on the inflated grid, 8-connected, obstacle-averse step cost."""
        s, g = self.to_cell(*start), self.to_cell(*goal)
        n = self.blocked.shape[0]
        if self.blocked[g]:
            # Named goals are placed for a 0.38 m radius; snap to the nearest
            # cell that honours the (possibly larger) clearance.
            free = np.argwhere(~self.blocked)
            g = tuple(free[np.argmin(((free - np.array(g)) ** 2).sum(axis=1))])
        steps = [(dy, dx, math.hypot(dy, dx)) for dy in (-1, 0, 1)
                 for dx in (-1, 0, 1) if dy or dx]
        best = {s: 0.0}
        parent = {s: None}
        heap = [(0.0, s)]
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == g:
                break
            cost = best[cur]
            for dy, dx, length in steps:
                nxt = (cur[0] + dy, cur[1] + dx)
                if not (0 <= nxt[0] < n and 0 <= nxt[1] < n) or self.blocked[nxt]:
                    continue
                # Up to 3x more expensive right at the inflation boundary.
                penalty = 1.0 + 2.0 * max(0.0, 1.0 - self.dist[nxt] / 1.2)
                new = cost + length * penalty
                if new < best.get(nxt, math.inf):
                    best[nxt] = new
                    parent[nxt] = cur
                    h = math.hypot(g[0] - nxt[0], g[1] - nxt[1])
                    heapq.heappush(heap, (new + h, nxt))
        if g not in parent:
            raise RuntimeError(f'no path from {start} to {goal}')
        cells = []
        cur = g
        while cur is not None:
            cells.append(cur)
            cur = parent[cur]
        return [self.to_world(c) for c in reversed(cells)]

    # ---------------------------------------------------------- coverage
    def coverage_viewpoints(self, start, sensor_range=6.0, spacing=0.6,
                            target=0.985, rays=240):
        """Pick a small set of stops from which the LiDAR sees the whole building.

        Works on a 10 cm copy of the obstacle grid.  Every reachable cell on a
        `spacing` lattice is a candidate stop; for each candidate, `rays` beams
        are cast out to `sensor_range` and the cells they cross before the first
        obstacle (plus that obstacle) are what the robot would observe there.
        Greedy set cover then keeps adding the candidate that reveals the most
        still-unseen cells until `target` of everything visible from anywhere
        is covered.  Returns the stops in world coordinates, unordered.
        """
        k = 2                                                  # 5 cm -> 10 cm
        n = self.blocked.shape[0] // k
        raw = self.dist <= 0.0                                 # real obstacles
        coarse = raw[:n * k, :n * k].reshape(n, k, n, k).any(axis=(1, 3))
        cres = self.res * k

        reach, _ = cw.flood(self.blocked, start)               # 5 cm, inflated
        step = max(1, int(round(spacing / self.res)))
        cand = [(iy, ix) for iy in range(0, reach.shape[0], step)
                for ix in range(0, reach.shape[1], step) if reach[iy, ix]]

        angles = np.linspace(-math.pi, math.pi, rays, endpoint=False)
        t = np.arange(0.0, sensor_range, cres / 2.0)
        dx = np.cos(angles)[:, None] * t[None, :]
        dy = np.sin(angles)[:, None] * t[None, :]

        seen_sets = []
        for iy, ix in cand:
            x, y = self.to_world((iy, ix))
            cx = np.floor((x + dx + self.extent) / cres).astype(np.int64)
            cy = np.floor((y + dy + self.extent) / cres).astype(np.int64)
            np.clip(cx, 0, n - 1, out=cx)
            np.clip(cy, 0, n - 1, out=cy)
            hit = coarse[cy, cx]
            first = np.where(hit.any(axis=1), hit.argmax(axis=1), hit.shape[1] - 1)
            keep = np.arange(t.size)[None, :] <= first[:, None]
            seen_sets.append(np.unique(cy[keep] * n + cx[keep]))

        universe = np.unique(np.concatenate(seen_sets))
        covered = np.zeros(n * n, dtype=bool)
        chosen = []
        total = universe.size
        while covered[universe].sum() < target * total:
            gains = [np.count_nonzero(~covered[s]) for s in seen_sets]
            best = int(np.argmax(gains))
            if gains[best] == 0:
                break
            chosen.append(self.to_world(cand[best]))
            covered[seen_sets[best]] = True
        return chosen, covered[universe].mean()

    def geodesic(self, points):
        """Pairwise travel distances (m) between points through free space."""
        k = 2
        n = self.blocked.shape[0] // k
        free = ~self.blocked[:n * k, :n * k].reshape(n, k, n, k).all(axis=(1, 3))
        cres = self.res * k
        cells = [(int((y + self.extent) / cres), int((x + self.extent) / cres))
                 for x, y in points]
        out = np.full((len(points), len(points)), np.inf)
        for i, (sy, sx) in enumerate(cells):
            dist = np.full((n, n), np.inf)
            dist[sy, sx] = 0.0
            queue = deque([(sy, sx)])
            while queue:
                y, x = queue.popleft()
                d = dist[y, x] + 1.0
                for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                    if 0 <= ny < n and 0 <= nx < n and free[ny, nx] and dist[ny, nx] > d:
                        dist[ny, nx] = d
                        queue.append((ny, nx))
            out[i] = [dist[cy, cx] * cres for cy, cx in cells]
        return out


def order_tour(dist, start=0, end=None):
    """Nearest-neighbour tour over a distance matrix, improved with 2-opt."""
    n = len(dist)
    todo = set(range(n)) - {start} - ({end} if end is not None else set())
    tour = [start]
    while todo:
        nxt = min(todo, key=lambda j: dist[tour[-1], j])
        tour.append(nxt)
        todo.remove(nxt)
    if end is not None:
        tour.append(end)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(tour) - 2):
            for j in range(i + 1, len(tour) - 1):
                a, b, c, d = tour[i - 1], tour[i], tour[j], tour[j + 1]
                if dist[a, c] + dist[b, d] < dist[a, b] + dist[c, d] - 1e-6:
                    tour[i:j + 1] = reversed(tour[i:j + 1])
                    improved = True
    return tour


def resample(path, spacing):
    """Evenly spaced points along a polyline."""
    out = [path[0]]
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        d = spacing - carry
        while d <= seg:
            t = d / seg
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            d += spacing
        carry = seg - (d - spacing)
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out


# ------------------------------------------------------------------- follower
class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.declare_parameter('world', os.path.join(PKG, 'worlds', 'smart_building.world'))
        self.declare_parameter('route', DEFAULT_ROUTE)
        # coverage:=true ignores `route` and plans stops that let the LiDAR see
        # every part of the building (see Planner.coverage_viewpoints).
        self.declare_parameter('coverage', True)
        self.declare_parameter('sensor_range', 6.0)   # m trusted for coverage
        self.declare_parameter('entity', 'scout_bot')
        self.declare_parameter('speed', 0.4)          # m/s
        self.declare_parameter('turn_rate', 0.9)      # rad/s
        self.declare_parameter('clearance', 0.45)     # m kept from any obstacle
        self.declare_parameter('lookahead', 0.5)      # m, pure pursuit
        self.declare_parameter('rate', 25.0)          # Hz control / pose updates
        self.declare_parameter('mode', 'drive')       # 'drive' or 'teleport'
        self.declare_parameter('stuck_timeout', 6.0)  # s without progress (drive)

        self.entity = self.get_parameter('entity').value
        self.speed = self.get_parameter('speed').value
        self.turn_rate = self.get_parameter('turn_rate').value
        self.lookahead = self.get_parameter('lookahead').value
        self.dt = 1.0 / self.get_parameter('rate').value
        self.mode = self.get_parameter('mode').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        if self.mode not in ('drive', 'teleport'):
            raise RuntimeError(f"mode must be 'drive' or 'teleport', not {self.mode!r}")

        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        for cli in (self.get_cli, self.set_cli):
            if not cli.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f'{cli.srv_name} not available - is the '
                                   'gazebo_ros_state plugin in the world?')

        self.x, self.y, self.yaw, self.z = self.read_pose()
        self.path = self.build_path()
        self.index = 0
        self.travelled = 0.0
        self.done = False
        self.best_index = 0
        self.progress_time = self.now()
        self.rescues = 0
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth', self.on_ground_truth, 10)
        self.get_logger().info(
            f'[{self.mode}] path through {self.stops} stops, '
            f'{self.path_length:.1f} m, about {self.path_length / self.speed / 60:.1f} min')
        self.create_timer(self.dt, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_ground_truth(self, msg):
        if self.mode != 'drive':
            return
        p = msg.pose.pose
        self.x, self.y = p.position.x, p.position.y
        self.yaw = yaw_from_quaternion(p.orientation)

    def read_pose(self):
        req = GetEntityState.Request()
        req.name = self.entity
        req.reference_frame = 'world'
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        if res is None or not res.success:
            raise RuntimeError(f'entity {self.entity} not found in Gazebo')
        p = res.state.pose
        return p.position.x, p.position.y, yaw_from_quaternion(p.orientation), p.position.z

    def build_path(self):
        planner = Planner(self.get_parameter('world').value,
                          self.get_parameter('clearance').value)
        start = (self.x, self.y)

        if self.get_parameter('coverage').value:
            stops, covered = planner.coverage_viewpoints(
                start, sensor_range=self.get_parameter('sensor_range').value)
            points = [start] + stops + [start]
            order = order_tour(planner.geodesic(points), 0, len(points) - 1)
            goals = [(f'stop {k}', points[i]) for k, i in enumerate(order[1:-1], 1)]
            goals.append(('start', start))
            self.get_logger().info(
                f'Coverage plan: {len(stops)} stops see {covered * 100:.1f} % of the '
                'area visible from anywhere in the building')
        else:
            with open(os.path.join(PKG, 'config', 'locations.yaml')) as fh:
                locations = yaml.safe_load(fh)['locations']
            route = self.get_parameter('route').value
            unknown = [r for r in route if r not in locations]
            if unknown:
                raise RuntimeError(f'unknown location(s): {unknown}')
            goals = [(name, (float(locations[name]['x']), float(locations[name]['y'])))
                     for name in route]

        path = [start]
        for name, goal in goals:
            leg = planner.plan(path[-1], goal)
            path += leg[1:]
        self.stops = len(goals)
        path = resample(path, 0.1)
        self.path_length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                               for a, b in zip(path, path[1:]))
        return path

    def publish_pose(self):
        state = EntityState()
        state.name = self.entity
        state.reference_frame = 'world'
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation.z = math.sin(self.yaw / 2.0)
        state.pose.orientation.w = math.cos(self.yaw / 2.0)
        req = SetEntityState.Request()
        req.state = state
        self.set_cli.call_async(req)

    def step(self):
        if self.done:
            return
        # Advance the tracked index past points already within reach.
        while (self.index < len(self.path) - 1 and
               math.hypot(self.path[self.index][0] - self.x,
                          self.path[self.index][1] - self.y) < self.lookahead):
            self.index += 1

        tx, ty = self.path[self.index]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if self.index == len(self.path) - 1 and dist < (0.05 if self.mode == 'teleport' else 0.2):
            self.done = True
            if self.mode == 'drive':
                self.cmd_pub.publish(Twist())
            self.get_logger().info(f'Tour complete, {self.travelled:.1f} m driven'
                                   f'{f", {self.rescues} rescue(s)" if self.rescues else ""}.')
            return

        heading_error = wrap(math.atan2(dy, dx) - self.yaw)
        # Rotate on the spot for sharp turns, like the real base would.
        forward_gain = max(0.0, math.cos(heading_error)) ** 4
        if self.mode == 'drive':
            self.drive(heading_error, forward_gain, dist)
        else:
            self.teleport_step(heading_error, forward_gain, dist)

    def drive(self, heading_error, forward_gain, dist):
        now = self.now()
        if self.index > self.best_index:
            self.best_index = self.index
            self.progress_time = now
        elif now - self.progress_time > self.stuck_timeout:
            self.rescue()
            return

        cmd = Twist()
        cmd.linear.x = self.speed * forward_gain * min(1.0, dist / 0.3)
        cmd.angular.z = max(-self.turn_rate, min(self.turn_rate, 2.0 * heading_error))
        self.cmd_pub.publish(cmd)
        self.travelled += cmd.linear.x * self.dt

    def rescue(self):
        """Put the robot back on the path a little ahead of where it got stuck."""
        self.rescues += 1
        i = min(self.index + 3, len(self.path) - 1)
        j = min(i + 1, len(self.path) - 1)
        self.x, self.y = self.path[i]
        if j != i:
            self.yaw = math.atan2(self.path[j][1] - self.y, self.path[j][0] - self.x)
        self.cmd_pub.publish(Twist())
        self.publish_pose()
        self.index = self.best_index = i
        self.progress_time = self.now()
        self.get_logger().warn(f'No progress for {self.stuck_timeout:.0f} s, '
                               f'placed back on the path at ({self.x:.2f}, {self.y:.2f})')

    def teleport_step(self, heading_error, forward_gain, dist):
        turn = max(-self.turn_rate * self.dt, min(self.turn_rate * self.dt, heading_error))
        self.yaw = wrap(self.yaw + turn)
        forward = min(self.speed * self.dt * forward_gain, dist)
        self.x += forward * math.cos(self.yaw)
        self.y += forward * math.sin(self.yaw)
        self.travelled += forward
        self.publish_pose()


def main():
    rclpy.init()
    node = PathFollower()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mode == 'drive' and rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

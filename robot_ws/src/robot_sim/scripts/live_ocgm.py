#!/usr/bin/env python3
"""Build the occupancy grid map live while the robot drives, and show it.

Does everything lidar_logger.py does (the same data.csv, row for row) and on
top of that feeds every saved scan straight into a log-odds occupancy grid.  A
matplotlib window shows the map growing, the robot footprint and heading, its
trail and the current scan.  The map is written to disk every `save_every`
seconds and once more on exit, so a run that is stopped early still leaves a
usable map behind:

    <output_dir>/ocgm_<student_id>.csv   P(occupied) in (0, 1), row 0 = north
    <output_dir>/ocgm_<student_id>_live.png

The grid layout and the inverse sensor model are the same as in
myworld_ocgm_<student_id>.py, so the live map and the one rebuilt offline from
data.csv agree except for a few dozen cells on cell borders (data.csv stores
ranges rounded to 0.1 mm, which moves an end point across a border now and then).

    ros2 run robot_sim live_ocgm.py --ros-args -p output_dir:=/path/to/dir
"""

import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from lidar_logger import LidarLogger  # noqa: E402

# Must match myworld_ocgm_660610840.py
RES = 0.05
X_MIN, X_MAX = -16.0, 16.0
Y_MIN, Y_MAX = -16.0, 16.0
L_OCC = np.log(0.85 / 0.15)
L_FREE = np.log(0.30 / 0.70)
L_CLAMP = 6.0

# Robot footprint in base_link (chassis 0.50 x 0.42 m), closed polygon.
FOOTPRINT = np.array([[0.25, 0.21], [-0.25, 0.21], [-0.25, -0.21],
                      [0.25, -0.21], [0.25, 0.21]])


class OccupancyGrid:
    def __init__(self):
        self.rows = int(round((Y_MAX - Y_MIN) / RES))
        self.cols = int(round((X_MAX - X_MIN) / RES))
        self.log_odds = np.zeros((self.rows, self.cols))

    def cells(self, x, y):
        col = np.floor((x - X_MIN) / RES).astype(np.int64)
        row = np.floor((Y_MAX - y) / RES).astype(np.int64)
        inside = (row >= 0) & (row < self.rows) & (col >= 0) & (col < self.cols)
        return (row * self.cols + col)[inside]

    def update(self, sx, sy, sth, ranges, angles, range_min, range_max):
        r = np.asarray(ranges)
        a = sth + np.asarray(angles)
        valid = r > range_min
        r, a = r[valid], a[valid]
        hit = r < range_max - 1e-3
        cos_a, sin_a = np.cos(a), np.sin(a)

        t = np.arange(0.0, range_max, RES / 2.0)
        before = t[None, :] < (r[:, None] - RES)
        fx = sx + t[None, :] * cos_a[:, None]
        fy = sy + t[None, :] * sin_a[:, None]
        free = np.unique(self.cells(fx[before], fy[before]))
        occ = np.unique(self.cells(sx + r[hit] * cos_a[hit], sy + r[hit] * sin_a[hit]))
        free = np.setdiff1d(free, occ, assume_unique=True)

        flat = self.log_odds.reshape(-1)
        flat[free] += L_FREE
        flat[occ] += L_OCC
        np.clip(self.log_odds, -L_CLAMP, L_CLAMP, out=self.log_odds)
        return r[hit] * cos_a[hit] + sx, r[hit] * sin_a[hit] + sy

    def probability(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))


class LiveOcgm(LidarLogger):
    def __init__(self):
        super().__init__()
        self.declare_parameter('output_dir', os.path.dirname(os.path.abspath(self.output)))
        self.declare_parameter('student_id', '660610840')
        self.declare_parameter('save_every', 15.0)     # s between map snapshots
        self.declare_parameter('show', True)

        out_dir = self.get_parameter('output_dir').value
        sid = self.get_parameter('student_id').value
        self.csv_path = os.path.join(out_dir, f'ocgm_{sid}.csv')
        self.png_path = os.path.join(out_dir, f'ocgm_{sid}_live.png')
        self.save_every = self.get_parameter('save_every').value
        self.show = self.get_parameter('show').value and bool(os.environ.get('DISPLAY'))

        self.grid = OccupancyGrid()
        self.trail = []
        self.scan_xy = (np.empty(0), np.empty(0))
        self.dirty = False
        self.last_snapshot = time.monotonic()
        self.setup_plot()

    # ----------------------------------------------------------------- mapping
    def on_saved(self, pose, ranges, angles, range_min, range_max):
        self.scan_xy = self.grid.update(*pose, ranges, angles, range_min, range_max)
        self.trail.append(pose[:2])
        self.dirty = True

    def snapshot(self):
        prob = self.grid.probability()
        np.savetxt(self.csv_path, prob, delimiter=',', fmt='%.4f')
        self.fig.savefig(self.png_path, dpi=110)
        self.get_logger().info(f'map snapshot ({self.rows} scans) -> {self.csv_path}')

    # ---------------------------------------------------------------- display
    def setup_plot(self):
        import matplotlib
        if not self.show:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        self.plt = plt
        if self.show:
            plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8.5, 9))
        try:
            self.fig.canvas.manager.set_window_title('Live occupancy grid map')
        except AttributeError:
            pass
        ax = self.ax
        self.image = ax.imshow(self.grid.probability(), cmap='gray_r', vmin=0.0, vmax=1.0,
                               extent=[X_MIN, X_MAX, Y_MIN, Y_MAX], origin='upper',
                               interpolation='nearest')
        self.trail_line, = ax.plot([], [], 'r-', lw=1.0, label='path')
        self.scan_dots = ax.scatter([], [], s=1.5, c='tab:blue', label='current scan')
        self.body, = ax.plot([], [], '-', color='tab:orange', lw=2.0)
        self.heading, = ax.plot([], [], '-', color='tab:green', lw=2.0)
        ax.set_xlim(X_MIN, X_MAX)
        ax.set_ylim(Y_MIN, Y_MAX)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Live OCGM - waiting for scans')
        ax.legend(loc='lower right', fontsize=8)
        self.fig.colorbar(self.image, ax=ax, orientation='horizontal', fraction=0.04,
                          pad=0.07, label='P(occupied)')
        self.fig.tight_layout()
        if self.show:
            plt.show(block=False)

    def redraw(self):
        self.image.set_data(self.grid.probability())
        if self.trail:
            tx, ty = zip(*self.trail)
            self.trail_line.set_data(tx, ty)
        self.scan_dots.set_offsets(np.column_stack(self.scan_xy))
        if self.base_pose is not None:
            bx, by, byaw = self.base_pose
            c, s = math.cos(byaw), math.sin(byaw)
            fx = bx + FOOTPRINT[:, 0] * c - FOOTPRINT[:, 1] * s
            fy = by + FOOTPRINT[:, 0] * s + FOOTPRINT[:, 1] * c
            self.body.set_data(fx, fy)
            self.heading.set_data([bx, bx + 0.6 * c], [by, by + 0.6 * s])
        known = np.abs(self.grid.log_odds) > 1e-9
        self.ax.set_title(f'Live OCGM - {self.rows} scans, '
                          f'{known.mean() * 100:.1f} % of the area observed')
        if self.show:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def window_open(self):
        return not self.show or self.plt.fignum_exists(self.fig.number)

    def close(self):
        self.redraw()
        self.snapshot()
        super().close()


def main():
    rclpy.init()
    node = LiveOcgm()
    last_draw = 0.0
    try:
        while rclpy.ok() and node.window_open():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            # Robot marker moves every frame; the map image only changes on
            # new scans, but redrawing both at ~5 Hz is cheap enough.
            if now - last_draw > 0.2:
                node.redraw()
                last_draw = now
                if node.show:
                    node.plt.pause(0.001)
            if node.dirty and now - node.last_snapshot > node.save_every:
                node.snapshot()
                node.last_snapshot = now
                node.dirty = False
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

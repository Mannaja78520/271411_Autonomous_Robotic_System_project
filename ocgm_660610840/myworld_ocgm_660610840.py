#!/usr/bin/env python3
"""Occupancy Grid Map (OCGM) from logged 2D LiDAR scans - student 660610840.

Input  : data.csv recorded in Gazebo by `ros2 run robot_sim lidar_logger.py`
         (same format as the CoppeliaSim example save_laser_show_pointcloud.py)

             col 1        x of the sensor base (m)
             col 2        y of the sensor base (m)
             col 3        heading of the sensor base w.r.t. the world x axis (rad)
             col 4+2(i-1) range of beam i (m),                i = 1..684
             col 5+2(i-1) angle of beam i w.r.t. sensor front (rad)

         topview.png from `ros2 run robot_sim capture_topview.py` (optional),
         a to-scale top view of the Gazebo world covering x, y in [-16, 16] m.

Output : ocgm_660610840.csv  2D array of occupancy probabilities in (0, 1)
                             0 -> certainly free, 1 -> certainly occupied,
                             0.5 -> never observed.
                             Row 0 is the NORTH edge (largest y) and column 0
                             the WEST edge (smallest x), i.e. the CSV reads
                             like the picture.  Cell (r, c) covers
                             x in [X_MIN + c*RES, X_MIN + (c+1)*RES) and
                             y in [Y_MAX - (r+1)*RES, Y_MAX - r*RES).
         ocgm_660610840.png  plot of the map next to the Gazebo top view.

Method : classic log-odds occupancy grid (Thrun, Probabilistic Robotics ch. 9).
         For every scan and every beam, the cells the beam passes through get
         the "free" update and the cell holding the end point gets the
         "occupied" update.  Beams with no return (range == max range) only
         clear cells.  Each cell is updated at most once per scan so a cell
         crossed by many neighbouring beams does not get over-counted.

Usage  : python3 myworld_ocgm_660610840.py [data.csv] [topview.png]
"""

import os
import sys

import matplotlib.pyplot as plt
import numpy as np

STUDENT_ID = '660610840'
HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ settings
# Grid: the world is 30 m x 30 m (walls at +-15 m); one extra metre each side.
# 5 cm cells: small enough to resolve chair and table legs (~5 cm) and the
# 2.2 m doorways cleanly, large compared with the 1 cm range noise of the
# simulated LiDAR, and the whole map is still only 640 x 640 cells.
RES = 0.05
X_MIN, X_MAX = -16.0, 16.0
Y_MIN, Y_MAX = -16.0, 16.0

# Inverse sensor model in log-odds form.
L_OCC = np.log(0.85 / 0.15)    # end point seen:   p(occ) = 0.85
L_FREE = np.log(0.30 / 0.70)   # beam passed:      p(occ) = 0.30
L_CLAMP = 6.0                  # keeps p inside (0.0025, 0.9975), never 0 or 1

MAX_RANGE = 16.0               # simulated LiDAR range_max (m)
MIN_RANGE = 0.15               # simulated LiDAR range_min (m)


# --------------------------------------------------------------------- load
def load_data(path):
    """Same variables as load_laser_show_pointcloud.py."""
    data = np.loadtxt(path, delimiter=',', ndmin=2)
    X = data[:, 0:1]                  # column vector, sensor x
    Y = data[:, 1:2]                  # column vector, sensor y
    TH = data[:, 2:3]                 # column vector, sensor heading
    senser_length = data[:, 3::2]     # column i = range of beam i
    senser_angle = data[:, 4::2]      # column i = angle of beam i
    return X, Y, TH, senser_length, senser_angle


def show_pointcloud(X, Y, TH, senser_length, senser_angle, ax):
    """All end points in world coordinates, as in the CoppeliaSim example."""
    hit = (senser_length < MAX_RANGE - 1e-3) & (senser_length > MIN_RANGE)
    ang = TH + senser_angle
    px = X + senser_length * np.cos(ang)
    py = Y + senser_length * np.sin(ang)
    ax.scatter(px[hit], py[hit], s=0.2, c='k')
    ax.plot(X[:, 0], Y[:, 0], 'r-', lw=1, label='sensor path')


# ---------------------------------------------------------------------- map
def world_to_cell(x, y, shape):
    """World (x, y) -> (row, col) with row 0 at the north edge."""
    col = np.floor((x - X_MIN) / RES).astype(np.int64)
    row = np.floor((Y_MAX - y) / RES).astype(np.int64)
    inside = (row >= 0) & (row < shape[0]) & (col >= 0) & (col < shape[1])
    return row, col, inside


def build_ocgm(X, Y, TH, senser_length, senser_angle):
    rows = int(round((Y_MAX - Y_MIN) / RES))
    cols = int(round((X_MAX - X_MIN) / RES))
    log_odds = np.zeros((rows, cols))

    # Sample every beam at half-cell spacing so no crossed cell is skipped.
    step = RES / 2.0
    t = np.arange(0.0, MAX_RANGE, step)                     # (S,)

    for k in range(len(X)):
        r = senser_length[k].copy()
        valid = r > MIN_RANGE
        r, a = r[valid], TH[k, 0] + senser_angle[k][valid]
        hit = r < MAX_RANGE - 1e-3

        # Free space: samples strictly before the end point.
        cos_a, sin_a = np.cos(a), np.sin(a)
        before = t[None, :] < (r[:, None] - RES)            # (B, S)
        fx = X[k, 0] + t[None, :] * cos_a[:, None]
        fy = Y[k, 0] + t[None, :] * sin_a[:, None]
        fr, fc, fin = world_to_cell(fx[before], fy[before], log_odds.shape)
        free = np.unique(fr[fin] * cols + fc[fin])

        # Occupied: the cell of each end point that is a real return.
        ox = X[k, 0] + r[hit] * cos_a[hit]
        oy = Y[k, 0] + r[hit] * sin_a[hit]
        orow, ocol, oin = world_to_cell(ox, oy, log_odds.shape)
        occ = np.unique(orow[oin] * cols + ocol[oin])

        # A cell that is an end point in this scan is not also cleared by it.
        free = np.setdiff1d(free, occ, assume_unique=True)

        flat = log_odds.reshape(-1)
        flat[free] += L_FREE
        flat[occ] += L_OCC
        np.clip(log_odds, -L_CLAMP, L_CLAMP, out=log_odds)

    return 1.0 - 1.0 / (1.0 + np.exp(log_odds))


# --------------------------------------------------------------------- plot
def plot(prob, X, Y, TH, senser_length, senser_angle, topview_path, out_png):
    extent = [X_MIN, X_MAX, Y_MIN, Y_MAX]
    has_top = topview_path is not None and os.path.exists(topview_path)
    ncols = 3 if has_top else 2
    fig, axes = plt.subplots(1, ncols, figsize=(7 * ncols, 8.2))

    ax = axes[0]
    if has_top:
        # Captured with the same +-16 m window, so it shares the metric axes.
        ax.imshow(plt.imread(topview_path), extent=extent, origin='upper')
        ax.set_title('Gazebo top view')
        ax = axes[1]

    im = ax.imshow(prob, cmap='gray_r', vmin=0.0, vmax=1.0, extent=extent,
                   origin='upper', interpolation='nearest')
    ax.plot(X[:, 0], Y[:, 0], 'r-', lw=0.8, label='sensor path')
    ax.plot(X[0, 0], Y[0, 0], 'go', ms=6, label='start')
    ax.set_title(f'Occupancy grid map ({RES * 100:.0f} cm cells)')
    ax.legend(loc='lower right', fontsize=8)
    # Horizontal bar under the map keeps every panel the same size and scale.
    cax = ax.inset_axes([0.0, -0.16, 1.0, 0.03])
    fig.colorbar(im, cax=cax, orientation='horizontal', label='P(occupied)')

    ax = axes[-1]
    show_pointcloud(X, Y, TH, senser_length, senser_angle, ax)
    ax.set_title(f'LiDAR point cloud ({len(X)} scans)')
    ax.legend(loc='lower right', fontsize=8)

    for ax in axes:
        ax.set_xlim(X_MIN, X_MAX)
        ax.set_ylim(Y_MIN, Y_MAX)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.grid(alpha=0.2)

    fig.suptitle(f'myworld OCGM - {STUDENT_ID}')
    fig.subplots_adjust(left=0.04, right=0.99, top=0.9, bottom=0.17, wspace=0.18)
    fig.savefig(out_png, dpi=130)
    print(f'plot saved to {out_png}')
    return fig


def main():
    data_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'data.csv')
    topview_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, 'topview.png')

    X, Y, TH, senser_length, senser_angle = load_data(data_path)
    print(f'{len(X)} scans x {senser_length.shape[1]} beams loaded from {data_path}')

    prob = build_ocgm(X, Y, TH, senser_length, senser_angle)

    csv_path = os.path.join(HERE, f'ocgm_{STUDENT_ID}.csv')
    np.savetxt(csv_path, prob, delimiter=',', fmt='%.4f')
    known = np.abs(prob - 0.5) > 1e-6
    print(f'map {prob.shape[0]} x {prob.shape[1]} cells saved to {csv_path}')
    print(f'  observed cells: {known.mean() * 100:.1f} %, '
          f'occupied (p > 0.65): {(prob > 0.65).sum()}, free (p < 0.35): {(prob < 0.35).sum()}')
    print(f'  probability range: [{prob.min():.4f}, {prob.max():.4f}]')

    plot(prob, X, Y, TH, senser_length, senser_angle, topview_path,
         os.path.join(HERE, f'ocgm_{STUDENT_ID}.png'))
    if os.environ.get('MPLBACKEND', '').lower() != 'agg':
        plt.show()


if __name__ == '__main__':
    main()

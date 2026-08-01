#!/usr/bin/env python3
"""Static sanity checks on the generated world.

Two independent checks, both run straight on the `.world` XML so they validate
what Gazebo will actually load:

1. **Overlap check** - every collision shape is turned into a world-frame
   axis-aligned bounding box; any pair of boxes from *different* models that
   interpenetrate by more than a tolerance is reported.  Catches furniture
   placed inside other furniture or inside a wall.

2. **Reachability check** - the collision shapes are rasterised into a 5 cm
   occupancy grid at the robot's height, the grid is inflated by the robot
   radius, and a flood fill is run from the spawn point.  Every named goal in
   `config/locations.yaml` must be reachable, otherwise the doorway or the
   aisle leading to it is too narrow.

Usage:  python3 check_world.py [world_file] [locations.yaml]
"""

import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import yaml

RESOLUTION = 0.05          # m per cell
EXTENT = 16.0              # half size of the rasterised area, m
ROBOT_RADIUS = 0.38        # m, matches robot_radius in nav2_params.yaml
ROBOT_TOP = 0.45           # m, obstacles below this height block the robot
FLOOR_CLEARANCE = 0.02     # m, ignore anything lying flat on the floor
OVERLAP_TOL = 0.03         # m of interpenetration allowed before reporting
SPAWN = (-13.5, 0.0)

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


# ---------------------------------------------------------------- geometry
def parse_pose(text):
    if text is None:
        return (0.0,) * 6
    v = [float(t) for t in text.split()]
    v += [0.0] * (6 - len(v))
    return tuple(v[:6])


def rot_z(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def shape_of(geometry):
    """Return (kind, sx, sy, sz) with sx/sy/sz the full extents in link frame."""
    box = geometry.find('box')
    if box is not None:
        sx, sy, sz = [float(t) for t in box.find('size').text.split()]
        return 'box', sx, sy, sz
    cyl = geometry.find('cylinder')
    if cyl is not None:
        r = float(cyl.find('radius').text)
        l = float(cyl.find('length').text)
        return 'cylinder', 2 * r, 2 * r, l
    sph = geometry.find('sphere')
    if sph is not None:
        r = float(sph.find('radius').text)
        return 'sphere', 2 * r, 2 * r, 2 * r
    plane = geometry.find('plane')
    if plane is not None:
        return 'plane', 0.0, 0.0, 0.0
    return None, 0.0, 0.0, 0.0


def collect_shapes(world_file):
    """Yield one dict per collision shape, in world coordinates."""
    root = ET.parse(world_file).getroot()
    world = root.find('world')
    shapes = []

    for model in world.findall('model'):
        mname = model.get('name')
        mx, my, mz, _, _, myaw = parse_pose(
            model.find('pose').text if model.find('pose') is not None else None)

        for link in model.findall('link'):
            lx, ly, lz, lroll, lpitch, lyaw = parse_pose(
                link.find('pose').text if link.find('pose') is not None else None)

            for collision in link.findall('collision'):
                kind, sx, sy, sz = shape_of(collision.find('geometry'))
                if kind is None or kind == 'plane':
                    continue

                # link -> model frame (links only ever use yaw, or a roll/pitch
                # of +-90 deg on wheel-style cylinders)
                if abs(lroll) > 1e-6 or abs(lpitch) > 1e-6:
                    # conservative: swap the axes the rotation would swap
                    if abs(abs(lroll) - math.pi / 2) < 1e-3:
                        sy, sz = sz, sy
                    elif abs(abs(lpitch) - math.pi / 2) < 1e-3:
                        sx, sz = sz, sx
                    kind = 'box'

                yaw = myaw + lyaw
                wx, wy = rot_z(lx, ly, myaw)
                wx += mx
                wy += my
                wz = mz + lz

                shapes.append({
                    'model': mname, 'kind': kind, 'yaw': yaw,
                    'x': wx, 'y': wy, 'z': wz,
                    'sx': sx, 'sy': sy, 'sz': sz,
                })
    return shapes


def aabb(s):
    """World-frame axis-aligned bounding box of one shape."""
    if s['kind'] == 'sphere':
        hx = hy = s['sx'] / 2.0
    elif s['kind'] == 'cylinder':
        hx = hy = s['sx'] / 2.0
    else:
        c, sn = abs(math.cos(s['yaw'])), abs(math.sin(s['yaw']))
        hx = s['sx'] / 2.0 * c + s['sy'] / 2.0 * sn
        hy = s['sx'] / 2.0 * sn + s['sy'] / 2.0 * c
    hz = s['sz'] / 2.0
    return (s['x'] - hx, s['x'] + hx,
            s['y'] - hy, s['y'] + hy,
            s['z'] - hz, s['z'] + hz)


# ------------------------------------------------------------ check 1: overlap
IGNORED = ('floor', 'ground_plane')
# Structural walls are *meant* to interpenetrate where they join at corners
# and T-junctions, so wall-to-wall pairs are not reported.
STRUCTURE = ('wall_', 'corridor_wall', 'divider_', 'server_wall')


def is_structure(name):
    return name.startswith(STRUCTURE)


def check_overlaps(shapes, ignore_models=IGNORED):
    boxes = [(s, aabb(s)) for s in shapes if s['model'] not in ignore_models]
    findings = []

    for i in range(len(boxes)):
        si, bi = boxes[i]
        for j in range(i + 1, len(boxes)):
            sj, bj = boxes[j]
            if si['model'] == sj['model']:
                continue
            if is_structure(si['model']) and is_structure(sj['model']):
                continue
            ox = min(bi[1], bj[1]) - max(bi[0], bj[0])
            oy = min(bi[3], bj[3]) - max(bi[2], bj[2])
            oz = min(bi[5], bj[5]) - max(bi[4], bj[4])
            if ox > OVERLAP_TOL and oy > OVERLAP_TOL and oz > OVERLAP_TOL:
                findings.append((min(ox, oy, oz), si['model'], sj['model'],
                                 round(si['x'], 2), round(si['y'], 2)))

    # one line per model pair, worst penetration first
    seen = {}
    for depth, a, b, x, y in findings:
        key = tuple(sorted((a, b)))
        if key not in seen or depth > seen[key][0]:
            seen[key] = (depth, x, y)
    return sorted(((d, k[0], k[1], x, y) for k, (d, x, y) in seen.items()),
                  reverse=True)


# -------------------------------------------------------- check 2: reachability
def rasterise(shapes):
    n = int(2 * EXTENT / RESOLUTION)
    grid = np.zeros((n, n), dtype=bool)

    def to_cell(v):
        return int((v + EXTENT) / RESOLUTION)

    for s in shapes:
        if s['model'] in ('floor', 'ground_plane'):
            continue
        x0, x1, y0, y1, z0, z1 = aabb(s)
        if z1 <= FLOOR_CLEARANCE or z0 >= ROBOT_TOP:
            continue  # too low to matter, or high enough to drive under

        ix0, ix1 = max(to_cell(x0), 0), min(to_cell(x1) + 1, n)
        iy0, iy1 = max(to_cell(y0), 0), min(to_cell(y1) + 1, n)
        if ix0 >= ix1 or iy0 >= iy1:
            continue

        if s['kind'] in ('cylinder', 'sphere'):
            r = s['sx'] / 2.0
            ys, xs = np.mgrid[iy0:iy1, ix0:ix1]
            px = (xs + 0.5) * RESOLUTION - EXTENT
            py = (ys + 0.5) * RESOLUTION - EXTENT
            mask = (px - s['x']) ** 2 + (py - s['y']) ** 2 <= r * r
        else:
            ys, xs = np.mgrid[iy0:iy1, ix0:ix1]
            px = (xs + 0.5) * RESOLUTION - EXTENT - s['x']
            py = (ys + 0.5) * RESOLUTION - EXTENT - s['y']
            c, sn = math.cos(-s['yaw']), math.sin(-s['yaw'])
            lx = px * c - py * sn
            ly = px * sn + py * c
            mask = (np.abs(lx) <= s['sx'] / 2.0) & (np.abs(ly) <= s['sy'] / 2.0)

        grid[iy0:iy1, ix0:ix1] |= mask

    return grid


def inflate(grid, radius_m):
    r = int(round(radius_m / RESOLUTION))
    out = grid.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            out |= np.roll(np.roll(grid, dy, axis=0), dx, axis=1)
    return out


def flood(grid, start):
    n = grid.shape[0]
    sx = int((start[0] + EXTENT) / RESOLUTION)
    sy = int((start[1] + EXTENT) / RESOLUTION)
    seen = np.zeros_like(grid)
    if grid[sy, sx]:
        return seen, False
    q = deque([(sy, sx)])
    seen[sy, sx] = True
    while q:
        y, x = q.popleft()
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < n and 0 <= nx < n and not seen[ny, nx] and not grid[ny, nx]:
                seen[ny, nx] = True
                q.append((ny, nx))
    return seen, True


def check_reachability(shapes, locations):
    grid = rasterise(shapes)
    inflated = inflate(grid, ROBOT_RADIUS)
    reach, ok = flood(inflated, SPAWN)
    results = []
    if not ok:
        return [('SPAWN', SPAWN[0], SPAWN[1], 'spawn point is inside an obstacle')]

    for name, v in locations.items():
        x, y = float(v['x']), float(v['y'])
        ix = int((x + EXTENT) / RESOLUTION)
        iy = int((y + EXTENT) / RESOLUTION)
        if inflated[iy, ix]:
            results.append((name, x, y, 'goal is blocked (obstacle within robot radius)'))
        elif not reach[iy, ix]:
            results.append((name, x, y, 'goal is free but not reachable from the spawn point'))
    return results


# ------------------------------------------------------------------------ main
def main():
    world_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        PKG, 'worlds', 'smart_building.world')
    loc_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        PKG, 'config', 'locations.yaml')

    shapes = collect_shapes(world_file)
    models = {s['model'] for s in shapes}
    print(f'{world_file}: {len(models)} models, {len(shapes)} collision shapes\n')

    print('--- overlap check ---')
    overlaps = check_overlaps(shapes)
    if not overlaps:
        print('  no interpenetrating models\n')
    else:
        for depth, a, b, x, y in overlaps:
            print(f'  {depth:5.2f} m  {a}  <->  {b}   near ({x}, {y})')
        print()

    print('--- reachability check ---')
    with open(loc_file) as fh:
        locations = yaml.safe_load(fh)['locations']
    problems = check_reachability(shapes, locations)
    if not problems:
        print(f'  all {len(locations)} named goals reachable from '
              f'{SPAWN} with a {ROBOT_RADIUS} m robot radius\n')
    else:
        for name, x, y, why in problems:
            print(f'  {name:<16} ({x:6.2f}, {y:6.2f})  {why}')
        print()

    return 1 if (overlaps or problems) else 0


if __name__ == '__main__':
    sys.exit(main())

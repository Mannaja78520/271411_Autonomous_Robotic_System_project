# Occupancy Grid Map in Gazebo + ROS 2 — 660610840

ROS 2 Humble / Gazebo Classic 11 version of the CoppeliaSim OCGM assignment.

| CoppeliaSim assignment | This submission |
|------------------------|-----------------|
| `myworld_xxx.ttt` scene | `myworld_660610840.world` (30 m × 30 m smart building, 7 rooms) + robot in `robot_ws/src/robot_sim` |
| `save_laser_show_pointcloud.py` | `robot_sim/scripts/lidar_logger.py` → `data.csv` |
| `load_laser_show_pointcloud.py` + OCGM code | `myworld_ocgm_660610840.py` |
| map file | `ocgm_660610840.csv` |
| top-view image | `topview.png` (from `robot_sim/scripts/capture_topview.py`) |

## data.csv format

One row per saved scan (a new row whenever the sensor moved ≥ 0.25 m or turned
≥ 0.35 rad, at most every 0.5 s):

* col 1, 2 — x, y of the sensor base in the world frame (m)
* col 3 — heading of the sensor base w.r.t. the world x axis (rad)
* col 4 + 2(i−1) — range of beam i (m), i = 1…684
* col 5 + 2(i−1) — angle of beam i w.r.t. the sensor front (rad)

The pose is the Gazebo ground truth (`libgazebo_ros_p3d`) of `base_link`
composed with the fixed `base_link → laser_frame` transform, i.e. the exact
sensor pose, the same thing the CoppeliaSim example reads with
`sim.getObjectPosition()`. Beams without a return are written as the maximum
range (16 m).

## ocgm_660610840.csv format

640 × 640 cells of 5 cm covering x, y ∈ [−16, 16] m. Each value is P(occupied)
in (0, 1): ≈0 free, ≈1 occupied, 0.5 never observed. Row 0 is the north edge
(y = +16 m), column 0 the west edge (x = −16 m), so the CSV reads like the
picture.

## Reproduce

```bash
cd ~/271411/robot_ws && make build

# terminal 1: simulator
make sim
# terminal 2: to-scale top view of the world (once)
make topview
# terminal 3: autonomous mapping tour. The robot drives every room on its
# wheels while a window shows the occupancy grid growing; data.csv and
# ocgm_660610840.csv are written as it goes. Ctrl+C (or close the window)
# after "Tour complete" to write the final map.
make tour

# alternative: drive yourself with the live map open
make livemap     # terminal 3
make teleop      # terminal 4

# rebuild the map offline from data.csv and plot it next to the top view
make ocgm
```

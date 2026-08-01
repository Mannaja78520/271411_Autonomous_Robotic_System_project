# robot_sim

Four-wheel differential (skid-steer) mobile robot with a 2D LiDAR, simulated in
Gazebo Classic 11 inside a 30 m x 30 m furnished indoor world.

For the full run-through (install, launch, demo script for the video) see
[`../../../README.md`](../../../README.md) at the top of the `271411` folder.

## Package layout

| Path | Contents |
|------|----------|
| `description/` | Robot model in xacro: `robot_core` (chassis + 4 wheels), `gazebo_control` (skid-steer plugin), `lidar` (2D laser), `inertial_macros` |
| `worlds/smart_building.world` | The generated 30 x 30 m, 7-room world |
| `tools/generate_world.py` | Script that writes the world file — edit this, not the `.world` |
| `launch/` | `sim`, `rsp`, `rviz`, `slam`, `nav2`, `keyboard_control`, `bringup` |
| `scripts/` | `keyboard_control.py`, `auto_patrol.py`, `lidar_monitor.py`, `nav_to_room.py` |
| `config/` | RViz layout, SLAM params, Nav2 params, teleop speeds, room coordinates |
| `maps/` | Destination for maps saved from SLAM |

## Robot

* Chassis 0.50 x 0.42 x 0.18 m, mass ~24 kg.
* Four independently driven wheels, radius 0.09 m, track 0.43 m, wheel base 0.34 m.
* Driven by `libgazebo_ros_diff_drive` with `num_wheel_pairs = 2`, so the two
  left wheels and the two right wheels form one differential pair each — a
  4-wheel skid-steer base commanded through `/cmd_vel`.
* Wheel friction is anisotropic and **anchored to the wheel** with
  `<fdir1>0 0 1</fdir1>` (the axle): `mu1 = 0.2` sideways scrub, `mu2 = 1.0`
  rolling. Both parts matter — see below.

### Why `fdir1` is not optional

Without `<fdir1>`, ODE chooses the first friction direction itself and it comes
out fixed in the **world**, not on the wheel. `mu1`/`mu2` then stop meaning
"rolling"/"lateral" and instead act along fixed world axes, so the base can only
generate a yaw couple when it happens to be aligned a certain way. Measured on
this robot with `mu1 = 1.0, mu2 = 0.4` and no `fdir1`, driving along the
corridor:

| Command | Result |
|---------|--------|
| `linear.x = 0.4` for 3 s | 112 cm travelled — fine |
| `angular.z = 0.5` for 4 s | **0.0°** of yaw (should be 114°) |
| `angular.z = 1.5` for 4 s | **0.0°** of yaw |

With `fdir1` set to the axle and the coefficients swapped to their correct
meanings:

| Command | Result |
|---------|--------|
| `linear.x = 0.4` for 3 s | 117 cm |
| `angular.z = 0.5` for 4 s | 102° of yaw (89 % of ideal) |
| `linear.x = 0.15, angular.z = 0.5` | 60 cm and 102° together |

The remaining ~11 % loss is normal skid-steer scrub: all four wheels have to
slide sideways for the body to yaw.

## LiDAR

`description/lidar.xacro`, mounted on the mast at 0.425 m above the floor.

| Property | Value |
|----------|-------|
| Beams | 720 |
| Field of view | 360 deg |
| Range | 0.15 – 16.0 m |
| Update rate | 10 Hz |
| Noise | Gaussian, sigma = 0.01 m |
| Topic | `/scan` (`sensor_msgs/LaserScan`), frame `laser_frame` |

`<visualize>true</visualize>` is set, so the blue laser fan is drawn inside the
Gazebo client — that is what shows the beams on camera.

## Regenerating the world

```bash
python3 src/robot_sim/tools/generate_world.py
colcon build --symlink-install --packages-select robot_sim
```

# 271411 — Mobile Robot Simulation (4-Wheel Differential + 2D LiDAR)

A ROS 2 Humble + Gazebo Classic 11 simulation of a **four-wheel differential
(skid-steer) mobile robot** carrying a **2D LiDAR**, driving around a
**30 m × 30 m indoor building with seven rooms**, furniture and people.

Everything is controlled from Python: manual keyboard teleoperation, an
autonomous waypoint patrol, a LiDAR readout, SLAM mapping and full Nav2
navigation.

The robot description and the keyboard controller are derived from the two
earlier projects in this folder:

| Source | What was reused |
|--------|-----------------|
| `271201-Lab_ws/src/articubot_one` | Robot xacro structure, differential-drive plugin setup, LiDAR sensor block, launch layout |
| `swerve_drive_differential_v1/robot_ws/src/differential_drive_v1` | `keyboard_control.py` (identical key map), speed-parameter file, launch-in-a-terminal pattern, Nav2 bring-up pattern |

---

## 1. Folder layout

```
271411/
├── README.md                     <- this file
├── robot_ws/                     <- THE WORKSPACE TO RUN
│   ├── Makefile                  <- shortcuts for every command below
│   └── src/robot_sim/
│       ├── description/          <- robot model (xacro)
│       ├── worlds/               <- smart_building.world (30 x 30 m)
│       ├── tools/generate_world.py
│       ├── launch/               <- sim, rsp, rviz, slam, nav2, teleop, bringup
│       ├── scripts/              <- keyboard_control, auto_patrol, lidar_monitor, nav_to_room
│       ├── config/               <- rviz, slam, nav2, speeds, room coordinates
│       └── maps/                 <- saved SLAM maps go here
├── 271201-Lab_ws/                <- previous lab (prototype source)
└── swerve_drive_differential_v1/ <- real-robot workspace (keyboard control source)
```

---

## 2. Prerequisites

Already installed on this machine:

* Ubuntu + **ROS 2 Humble** (`/opt/ros/humble`)
* **Gazebo Classic 11** + `gazebo_ros_pkgs`
* `slam_toolbox`, `nav2_bringup`, `rviz2`, `xacro`

Optional, only for the pop-up teleop terminal:

```bash
sudo apt install xterm        # gnome-terminal is used automatically if xterm is missing
```

No internet is needed at run time: the world is built entirely from primitive
shapes, and the launch files stop Gazebo from contacting the online model
database.

---

## 3. Build

```bash
cd ~/271411/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -Wno-dev
source install/setup.bash
```

or simply:

```bash
cd ~/271411/robot_ws && make build
```

**Source `install/setup.bash` in every new terminal** before running anything
below (the `make` targets do it for you).

---

## 4. Quick start (for the video)

One command starts Gazebo, the robot, RViz and a keyboard-control terminal:

```bash
cd ~/271411/robot_ws
make gui
```

Equivalent to:

```bash
ros2 launch robot_sim bringup.launch.py rviz:=true teleop:=true
```

Add mapping or navigation:

```bash
ros2 launch robot_sim bringup.launch.py slam:=true    # + live SLAM map
ros2 launch robot_sim bringup.launch.py nav2:=true    # + SLAM + Nav2 (goals from RViz)
```

---

## 5. Running each part separately

Use one terminal per block. Terminal 1 always runs the simulator.

### 5.1 Simulator only

```bash
make sim
# or
ros2 launch robot_sim sim.launch.py                 # Gazebo GUI, robot spawned in the corridor
ros2 launch robot_sim sim.launch.py rviz:=true      # also open RViz
ros2 launch robot_sim sim.launch.py gui:=false      # headless (no Gazebo window)
ros2 launch robot_sim sim.launch.py x:=0 y:=-8 yaw:=1.57   # spawn somewhere else
```

The robot spawns in the main corridor at **(-13.5, 0)** facing east.

### 5.2 Manual control — keyboard (Python)

```bash
make teleop
# or
ros2 run robot_sim keyboard_control.py
```

| Key | Action |
|-----|--------|
| `w` / `s` | forward / backward (step 0.15 m/s) |
| `a` / `d` | slide left / right — swerve platform only, ignored by this differential base |
| `q` / `e` | turn left / right (step 0.30 rad/s) |
| `W`/`S`, `A`/`D`, `Q`/`E` | increase / decrease the corresponding step size |
| `space` | brake (all speeds to zero) |
| `m` / `r` | switch between PWM (`manual`) and m/s (`mps`) mode |
| `p` | quit |

The node publishes `/cmd_vel`, `/keyboard_input` and `/movement_mode`, exactly
like the one on the real robot. Speeds are read from
`config/speed_control.yaml`.

> The controller reads raw keystrokes, so it needs a real terminal. Run it with
> `ros2 run` in its own terminal, or use `ros2 launch robot_sim
> keyboard_control.launch.py`, which opens one for you.

### 5.3 Autonomous control — waypoint patrol (Python, no Nav2)

```bash
make patrol
# or
ros2 run robot_sim auto_patrol.py
```

Drives a fixed tour (corridor → office → meeting room → cafeteria → back) using
`/odom` for the heading controller and `/scan` for reactive obstacle avoidance:

* it slows down as the front sector closes in,
* rotates in place when something is within 0.65 m, keeping the same turn
  direction until it is clear so it cannot dither,
* and if rotating does not help within 4 s — the usual case when it clips a
  doorway edge — it reverses out (rear sector permitting) and lines up again.

Custom route and limits:

```bash
ros2 run robot_sim auto_patrol.py --ros-args \
  -p waypoints:="[0.0, 0.0, 9.0, 0.0, 9.0, 5.0]" \
  -p max_linear:=0.5 -p loop:=false
```

### 5.4 LiDAR readout

```bash
make lidar
# or
ros2 run robot_sim lidar_monitor.py
```

Prints the closest obstacle in eight 45° sectors, ten times a second — handy
for narrating the LiDAR section of the video.

Raw data:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

### 5.5 SLAM — build a map of the building

Terminal 1: `make sim`  Terminal 2: `make slam`  Terminal 3: `make teleop` (or `make patrol`)

```bash
ros2 launch robot_sim slam.launch.py
```

Watch the map grow in RViz (`make rviz`). When the building is covered:

```bash
make savemap
# writes src/robot_sim/maps/smart_building_map.{pgm,yaml}
```

### 5.6 Nav2 — autonomous navigation

With online SLAM (no saved map needed):

```bash
# Terminal 1
make sim
# Terminal 2
make nav2
# Terminal 3
make rviz
```

In RViz press **2D Goal Pose** and click anywhere in the building — Nav2 plans
a path and drives the robot there.

> **With online SLAM you can only navigate inside the part of the building that
> has already been scanned.** Right after start-up the map is only a few metres
> across, so a goal at the far end of the corridor is rejected with
> `failed to generate a valid path`. Drive around with `make teleop` or
> `make patrol` for a minute first, or click goals that get progressively
> further away. Once the map covers the building, goals anywhere in it work.
>
> Note also that `map` and `odom` only start aligned: as SLAM closes loops it
> shifts `map->odom`, so the room coordinates in `locations.yaml` (which are
> world coordinates) drift by a few tens of centimetres. Nav2 still arrives —
> the `distance remaining` readout just jumps around while the map is corrected.

Or send goals by room name from Python:

```bash
make rooms                                    # list the known rooms
ros2 run robot_sim nav_to_room.py meeting_room
ros2 run robot_sim nav_to_room.py office cafeteria lab      # multi-stop tour
```

Room coordinates live in `config/locations.yaml`.

With a previously saved map instead of live SLAM:

```bash
ros2 launch robot_sim nav2.launch.py slam:=False \
  map:=$HOME/271411/robot_ws/src/robot_sim/maps/smart_building_map.yaml
```

### 5.7 Everything at once

```bash
ros2 launch robot_sim bringup.launch.py nav2:=true
```

SLAM starts 8 s after Gazebo and Nav2 at 12 s, so the transforms are ready
before the stacks come up.

---

## 6. Command reference

| Command | What it does |
|---------|--------------|
| `make build` | build the workspace |
| `make world` | regenerate the `.world` file from `tools/generate_world.py` |
| `make check` | validate the world: no interpenetration, all goals reachable |
| `make sim` | Gazebo + robot |
| `make gui` | Gazebo + robot + RViz + teleop terminal |
| `make rviz` | RViz with the prepared layout |
| `make teleop` | keyboard control |
| `make patrol` | autonomous waypoint patrol |
| `make lidar` | LiDAR sector printout |
| `make slam` | slam_toolbox mapping |
| `make nav2` | Nav2 stack |
| `make rooms` | list Nav2 room goals |
| `make savemap` | save the current map |
| `make kill` | kill leftover Gazebo processes |
| `make clean` | delete `build/ install/ log/` |

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | command in |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR out |
| `/odom` | `nav_msgs/Odometry` | wheel odometry out |
| `/joint_states`, `/tf` | — | robot state |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM out |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 goal in |

---

## 7. The robot

| Item | Value |
|------|-------|
| Drive | 4 wheels, skid-steer / differential (`libgazebo_ros_diff_drive`, 2 wheel pairs) |
| Chassis | 0.50 × 0.42 × 0.18 m, ≈24 kg total |
| Wheels | radius 0.09 m, width 0.05 m, track 0.43 m, wheel base 0.34 m |
| Top speed | ~1.5 m/s (teleop limit), 0.7 m/s under Nav2 |
| Sensor | 2D LiDAR, 720 beams, 360°, 0.15–16 m, 10 Hz, Gaussian noise σ = 0.01 m |
| Sensor height | 0.425 m above the floor, on the front mast |
| Frames | `base_link`, `base_footprint`, `chassis`, `mast`, `laser_frame`, 4 × wheels |

The LiDAR has `<visualize>true</visualize>`, so **the blue laser fan is drawn
in the Gazebo window** — that is the beam display required by the assignment.
The same scan appears in RViz as red points.

---

## 8. The world — "Smart Building", 30 m × 30 m

Flat floor, 2.6 m brick outer walls, a 3.5 m central corridor running east–west,
and **seven enclosed rooms** all opening onto it through 2.2 m doorways.

```
        +y (north)
 +---------------------------------------------------+  y = +15
 | SERVER |            |                    | MEETING|
 |  ROOM  | RECEPTION  |   OPEN-PLAN        |  ROOM  |
 |--------+ & LOUNGE   |     OFFICE         |        |
 |        |            |                    |        |
 +===D====+=====D======+=========D==========+===D====+  y = +1.75
 |                MAIN CORRIDOR                      |   <- robot spawns here
 +===D====+=====D======+=========D==========+===D====+  y = -1.75
 |                     |                    |        |
 |     WAREHOUSE       |     CAFETERIA      |ROBOTICS|
 |     / STORAGE       |                    |  LAB   |
 +---------------------------------------------------+  y = -15
x = -15                                             x = +15
```

| Room | Contents |
|------|----------|
| **Reception & Lounge** | reception counter with receptionist, three sofas, coffee table, two visitors, potted plants, wall sign |
| **Server room** | six server racks with blinking front panels, UPS cabinet |
| **Open-plan office** | nine desks with monitors and chairs, three staff, filing cabinets, plants, bin |
| **Meeting room** | 4.2 m conference table, eight chairs, whiteboard, wall screen, sideboard, two people |
| **Warehouse / storage** | four 7 m pallet racks, crate stacks, pallets, forklift, two workers in hi-vis, tool cabinet |
| **Cafeteria** | nine round tables with chairs (three rows), 6 m serving counter, two vending machines, three diners, plants |
| **Robotics lab** | three workbenches, two robot arms behind a safety fence, cone obstacle course, crates, tool cabinet, whiteboard, two engineers |
| **Corridor** | plants, bench, lockers, bin, a person walking |

162 static models / 555 collision shapes in total — all of them are seen by the
LiDAR, so the scan silhouette clearly shows chairs, table legs and people.

To change the layout, edit `src/robot_sim/tools/generate_world.py` and run
`make world && make build`.

### Validating the world

```bash
cd ~/271411/robot_ws && make check
```

`tools/check_world.py` reads the generated `.world` and reports

* any two models whose collision shapes interpenetrate (furniture inside
  furniture, or inside a wall), and
* any goal in `config/locations.yaml` that a 0.30 m-radius robot cannot reach
  from the spawn point — it rasterises every collision shape into a 5 cm grid,
  inflates it by the robot radius and flood-fills from the spawn point.

Both checks must come back clean before recording:

```
--- overlap check ---
  no interpenetrating models

--- reachability check ---
  all 13 named goals reachable from (-13.5, 0.0) with a 0.3 m robot radius
```

---

## 9. Suggested 5-minute video script

| Time | Show | Say / do |
|------|------|----------|
| 0:00–0:30 | Gazebo zoomed out on the whole building | 30 × 30 m, seven rooms, corridor layout |
| 0:30–1:30 | Fly the Gazebo camera room by room | name each room and its furniture/people |
| 1:30–2:00 | Zoom onto the robot | four driven wheels, differential/skid-steer, LiDAR on the mast |
| 2:00–3:00 | `make teleop`, drive along the corridor | show the **blue laser fan** sweeping the walls; `w/s/q/e/space` |
| 3:00–3:40 | Drive into a room, split screen with RViz | red scan points matching chairs, tables and people; run `make lidar` to show the numbers |
| 3:40–4:20 | `make patrol` | robot drives itself between rooms, slows down and turns away from obstacles |
| 4:20–5:00 | `make slam` / `make nav2` + RViz | map building live, then a 2D Goal Pose click and autonomous navigation |

Screen recording:

```bash
ffmpeg -f x11grab -framerate 25 -i :1 -c:v libx264 -preset ultrafast demo.mp4
```

---

## 10. Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Service /spawn_entity unavailable` | An old Gazebo is still running: `make kill`, then relaunch. |
| Gazebo hangs on "Getting models from http://models.gazebosim.org" | Launch through `sim.launch.py` / `make sim` — it sets `GAZEBO_MODEL_DATABASE_URI=""`. Do not start `gzserver` by hand. |
| `Unable to find shader lib` in the Gazebo log | `source /usr/share/gazebo/setup.sh` before launching, or use the launch files, which set the paths themselves. |
| RViz or Gazebo dies with `libpthread.so.0: undefined symbol` | You are in a **snap** terminal (e.g. the terminal inside snap-installed VS Code). Run from a normal system terminal. |
| No teleop window appears | `sudo apt install xterm`, or run `ros2 run robot_sim keyboard_control.py` directly. |
| Robot does not move | Check `/cmd_vel` is being published (`ros2 topic echo /cmd_vel`) and that the simulation is not paused in Gazebo. |
| Robot drives straight but will not turn on the spot | Wheel friction needs `<fdir1>` in `robot_core.xacro`. Without it ODE anchors the friction directions to the world instead of the wheel, so yaw only works at certain headings. Already fixed here — do not remove that tag. |
| Nav2 refuses goals (`failed to generate a valid path`) | The goal is outside the area SLAM has mapped so far. Drive around first to extend the map, or pick a closer goal. |
| Nav2 goals ignored, `nav_to_room.py` never starts | It waits for the localiser that is actually running. If nothing happens, check `/map` has a publisher (`ros2 topic info /map`). |
| Nodes/topics not visible from a new terminal after several hard kills | Stale FastDDS locks: kill everything, then `rm -f /dev/shm/fastrtps_*` and start again. |
| Simulation runs slowly | `ros2 launch robot_sim sim.launch.py gui:=false` and watch in RViz instead; the Gazebo GUI is the expensive part. |

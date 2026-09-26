# Convenience targets for the robot_sim workspace.
# Every target assumes ROS 2 Humble is installed; `source install/setup.bash`
# is done inside each recipe, so a plain `make sim` works from a fresh shell.

SHELL := /bin/bash
WS    := $(shell pwd)
SETUP := source /opt/ros/humble/setup.bash && source $(WS)/install/setup.bash
OCGM  := $(abspath $(WS)/../ocgm_660610840)

.PHONY: help build clean world check sim gui slam nav2 rviz teleop patrol lidar rooms savemap kill record topview ocgm tour livemap

help:
	@echo "make build     - colcon build the workspace"
	@echo "make world     - regenerate worlds/smart_building.world from tools/generate_world.py"
	@echo "make check     - validate the world (overlaps + reachable goals)"
	@echo "make sim       - Gazebo + robot (no RViz)"
	@echo "make gui       - Gazebo + robot + RViz + keyboard teleop terminal"
	@echo "make slam      - SLAM only (needs 'make sim' running)"
	@echo "make nav2      - Nav2 with online SLAM (needs 'make sim' running)"
	@echo "make rviz      - RViz only"
	@echo "make teleop    - keyboard teleop in this terminal"
	@echo "make patrol    - autonomous waypoint patrol demo"
	@echo "make lidar     - print live LiDAR sector distances"
	@echo "make rooms     - list the named Nav2 goals"
	@echo "make savemap   - save the current SLAM map into src/robot_sim/maps"
	@echo "make record    - log /scan + ground-truth pose into ../ocgm_660610840/data.csv"
	@echo "make tour      - drive every room + live OCGM window, saves data.csv/ocgm csv (needs 'make sim')"
	@echo "make livemap   - live OCGM window + data.csv while you drive yourself (make teleop)"
	@echo "make topview   - save a to-scale top view of the world (needs 'make sim')"
	@echo "make ocgm      - build the occupancy grid map from data.csv and plot it"
	@echo "make clean     - remove build/ install/ log/"
	@echo "make kill      - kill leftover gazebo processes"

build:
	source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -Wno-dev

clean:
	rm -rf build install log

world:
	python3 src/robot_sim/tools/generate_world.py

check:
	python3 src/robot_sim/tools/check_world.py

sim:
	$(SETUP) && ros2 launch robot_sim sim.launch.py

gui:
	$(SETUP) && ros2 launch robot_sim bringup.launch.py rviz:=true teleop:=true

slam:
	$(SETUP) && ros2 launch robot_sim slam.launch.py

nav2:
	$(SETUP) && ros2 launch robot_sim nav2.launch.py

rviz:
	$(SETUP) && ros2 launch robot_sim rviz.launch.py

teleop:
	$(SETUP) && ros2 run robot_sim keyboard_control.py \
		--ros-args --params-file $(WS)/install/robot_sim/share/robot_sim/config/speed_control.yaml

patrol:
	$(SETUP) && ros2 run robot_sim auto_patrol.py

lidar:
	$(SETUP) && ros2 run robot_sim lidar_monitor.py

rooms:
	$(SETUP) && ros2 run robot_sim nav_to_room.py --list

savemap:
	$(SETUP) && ros2 run nav2_map_server map_saver_cli \
		-f $(WS)/src/robot_sim/maps/smart_building_map

kill:
	-pkill -9 -f gzserver
	-pkill -9 -f gzclient

record:
	$(SETUP) && ros2 run robot_sim lidar_logger.py --ros-args -p output:=$(OCGM)/data.csv

tour:
	$(SETUP) && ros2 launch robot_sim tour.launch.py output_dir:=$(OCGM)

livemap:
	$(SETUP) && ros2 run robot_sim live_ocgm.py --ros-args -p output:=$(OCGM)/data.csv

topview:
	$(SETUP) && ros2 run robot_sim capture_topview.py --ros-args -p output:=$(OCGM)/topview.png

ocgm:
	cd $(OCGM) && python3 myworld_ocgm_660610840.py

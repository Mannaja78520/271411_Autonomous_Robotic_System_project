# Saved maps

Run the SLAM demo, then save the finished map here:

    ros2 run nav2_map_server map_saver_cli -f ~/271411/robot_ws/src/robot_sim/maps/smart_building_map

After saving, rebuild the workspace so the map is installed, then launch Nav2
in localisation mode:

    ros2 launch robot_sim nav2.launch.py slam:=False

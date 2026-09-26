cmake_minimum_required(VERSION 3.8)
project(robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch worlds config maps tools
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/keyboard_control.py
    scripts/auto_patrol.py
    scripts/lidar_monitor.py
    scripts/nav_to_room.py
    scripts/lidar_logger.py
    scripts/capture_topview.py
    scripts/path_follower.py
    scripts/live_ocgm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

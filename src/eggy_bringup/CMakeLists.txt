cmake_minimum_required(VERSION 3.0.2)
project(eggy_bringup)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  move_base_msgs
  tf
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS actionlib roscpp rospy nav_msgs sensor_msgs std_msgs geometry_msgs move_base_msgs tf tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include
)

add_executable(eggy_odom_fuser_node src/eggy_odom_fuser_node.cpp)
target_link_libraries(eggy_odom_fuser_node ${catkin_LIBRARIES})

add_executable(meter_rknn_detect_node src/meter_rknn_detect_node.cpp)
target_link_libraries(meter_rknn_detect_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} rknnrt)

install(TARGETS eggy_odom_fuser_node meter_rknn_detect_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/auto_map_explore.py
  scripts/kimi_inspection_bridge.py
  scripts/kimi_inspection_server.py
  scripts/inspection_servo_route_runner.py
  scripts/meter_visual_servo.py
  scripts/eggy_nav_mode_status.py
  scripts/eggy_network_status.py
  scripts/eggy_shell_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(eggy_bringup)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs std_msgs geometry_msgs tf2_ros
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

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

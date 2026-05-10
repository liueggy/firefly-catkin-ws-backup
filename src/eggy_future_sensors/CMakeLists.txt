cmake_minimum_required(VERSION 3.0.2)
project(eggy_future_sensors)
find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs)
catkin_package()
install(DIRECTORY launch config README.assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

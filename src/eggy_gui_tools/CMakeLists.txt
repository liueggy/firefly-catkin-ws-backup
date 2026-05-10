cmake_minimum_required(VERSION 3.0.2)
project(eggy_gui_tools)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(eggy_external_imu)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)
catkin_package()
catkin_install_python(PROGRAMS scripts/external_yb_imu_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

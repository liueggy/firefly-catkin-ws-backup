#include <exception>

#include <ros/ros.h>

#include "eggy_base_driver/base_driver.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "stm32_base_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    eggy_base_driver::BaseDriver driver(nh, pnh);
    ros::spin();
  } catch (const std::exception& exc) {
    ROS_FATAL_STREAM("stm32_base_driver_node failed: " << exc.what());
    return 1;
  }
  return 0;
}

#ifndef EGGY_BASE_DRIVER_BASE_DRIVER_HPP
#define EGGY_BASE_DRIVER_BASE_DRIVER_HPP

#include <array>
#include <mutex>
#include <thread>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <tf2_ros/transform_broadcaster.h>

#include "eggy_base_driver/serial_port.hpp"
#include "eggy_base_driver/stm32_protocol.hpp"

namespace eggy_base_driver {

class BaseDriver {
 public:
  BaseDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~BaseDriver();

 private:
  void LoadParams(ros::NodeHandle& pnh);
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void ControlTimerCallback(const ros::TimerEvent& event);
  void DiagnosticTimerCallback(const ros::TimerEvent& event);
  void ReadThread();
  void ExtractFrames();
  void HandleStatus(const StatusFrame& status, const ros::Time& stamp);
  void PublishOdometry(const StatusFrame& status, const ros::Time& stamp);
  void PublishStopFrames();

  std::string port_;
  int baudrate_ = 115200;
  double control_rate_ = 20.0;
  double status_timeout_ = 1.0;
  double cmd_vel_timeout_ = 0.5;
  double max_linear_x_ = 0.5;
  double max_linear_y_ = 0.5;
  double max_angular_z_ = 1.0;
  double acc_lsb_per_g_ = 16384.0;
  double standard_gravity_ = 9.80665;
  double gyro_lsb_per_deg_s_ = 16.4;
  bool publish_tf_ = true;
  std::string base_frame_id_ = "base_link";
  std::string odom_frame_id_ = "odom";
  std::string imu_frame_id_ = "imu_link";

  SerialPort serial_;
  std::mutex serial_mutex_;
  std::thread read_thread_;
  bool running_ = false;
  std::vector<uint8_t> read_buffer_;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher imu_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher voltage_pub_;
  ros::Publisher flag_stop_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Timer control_timer_;
  ros::Timer diagnostic_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  geometry_msgs::Twist latest_cmd_;
  ros::Time latest_cmd_time_;
  ros::Time last_status_time_;
  ros::Time last_odom_time_;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double last_voltage_ = 0.0;
  uint8_t last_flag_stop_ = 0;
  uint64_t valid_frames_ = 0;
  uint64_t bad_frames_ = 0;
};

}  // namespace eggy_base_driver

#endif  // EGGY_BASE_DRIVER_BASE_DRIVER_HPP

#include "eggy_base_driver/base_driver.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>

namespace eggy_base_driver {

BaseDriver::BaseDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  LoadParams(pnh);
  if (!serial_.Open(port_, baudrate_)) {
    ROS_FATAL_STREAM("Failed to open serial port " << port_ << " at " << baudrate_);
    throw std::runtime_error("failed to open serial port");
  }

  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 20);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 20);
  voltage_pub_ = nh.advertise<std_msgs::Float32>("battery/voltage", 10);
  flag_stop_pub_ = nh.advertise<std_msgs::UInt8>("base/flag_stop", 10);
  diagnostics_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &BaseDriver::CmdVelCallback, this);

  latest_cmd_time_ = ros::Time(0);
  running_ = true;
  read_thread_ = std::thread(&BaseDriver::ReadThread, this);
  control_timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &BaseDriver::ControlTimerCallback, this);
  diagnostic_timer_ = nh.createTimer(ros::Duration(1.0), &BaseDriver::DiagnosticTimerCallback, this);
  ROS_INFO_STREAM("C++ STM32 base driver started: port=" << port_ << " baudrate=" << baudrate_);
}

BaseDriver::~BaseDriver() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  PublishStopFrames();
  serial_.Close();
}

void BaseDriver::LoadParams(ros::NodeHandle& pnh) {
  pnh.param<std::string>("port", port_, "/dev/ttyACM0");
  pnh.param<int>("baudrate", baudrate_, 115200);
  pnh.param<double>("control_rate", control_rate_, 20.0);
  pnh.param<double>("status_timeout", status_timeout_, 1.0);
  pnh.param<double>("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
  pnh.param<double>("max_linear_x", max_linear_x_, 0.5);
  pnh.param<double>("max_linear_y", max_linear_y_, 0.5);
  pnh.param<double>("max_angular_z", max_angular_z_, 1.0);
  pnh.param<double>("acc_lsb_per_g", acc_lsb_per_g_, 16384.0);
  pnh.param<double>("standard_gravity", standard_gravity_, 9.80665);
  pnh.param<double>("gyro_lsb_per_deg_s", gyro_lsb_per_deg_s_, 16.4);
  pnh.param<bool>("publish_tf", publish_tf_, true);
  pnh.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  pnh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  pnh.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");
}

void BaseDriver::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  latest_cmd_ = *msg;
  latest_cmd_time_ = ros::Time::now();
}

void BaseDriver::ControlTimerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  if ((now - latest_cmd_time_).toSec() <= cmd_vel_timeout_) {
    vx = latest_cmd_.linear.x;
    vy = latest_cmd_.linear.y;
    wz = latest_cmd_.angular.z;
  }

  const auto frame = Stm32Protocol::BuildControlFrame(vx, vy, wz, max_linear_x_, max_linear_y_, max_angular_z_);
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_.Write(frame.data(), frame.size())) {
    ROS_ERROR_THROTTLE(2.0, "Failed to write control frame to STM32");
  }
}

void BaseDriver::ReadThread() {
  uint8_t buffer[256];
  while (running_ && ros::ok()) {
    const ssize_t n = serial_.Read(buffer, sizeof(buffer));
    if (n > 0) {
      read_buffer_.insert(read_buffer_.end(), buffer, buffer + n);
      ExtractFrames();
    } else if (n < 0) {
      ROS_ERROR_THROTTLE(2.0, "Serial read error");
      ros::Duration(0.1).sleep();
    } else {
      ros::Duration(0.005).sleep();
    }
  }
}

void BaseDriver::ExtractFrames() {
  while (read_buffer_.size() >= kStatusFrameSize) {
    const auto it = std::find(read_buffer_.begin(), read_buffer_.end(), kFrameHeader);
    if (it == read_buffer_.end()) {
      read_buffer_.clear();
      return;
    }
    if (it != read_buffer_.begin()) {
      read_buffer_.erase(read_buffer_.begin(), it);
    }
    if (read_buffer_.size() < kStatusFrameSize) {
      return;
    }

    std::array<uint8_t, kStatusFrameSize> raw{};
    std::copy_n(read_buffer_.begin(), kStatusFrameSize, raw.begin());

    StatusFrame status;
    if (!Stm32Protocol::ParseStatusFrame(raw, &status)) {
      ++bad_frames_;
      // Resync conservatively: drop only the current header byte.
      // The STM32 stream may contain partial frames or embedded 0x7B bytes;
      // dropping a full frame on parse failure can keep the parser misaligned forever.
      read_buffer_.erase(read_buffer_.begin());
      continue;
    }

    read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + kStatusFrameSize);
    ++valid_frames_;
    HandleStatus(status, ros::Time::now());
  }
}

void BaseDriver::HandleStatus(const StatusFrame& status, const ros::Time& stamp) {
  last_status_time_ = stamp;
  last_voltage_ = status.voltage;
  last_flag_stop_ = status.flag_stop;

  sensor_msgs::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = imu_frame_id_;
  imu.orientation_covariance[0] = -1.0;
  imu.linear_acceleration.x = status.acc_x_raw / acc_lsb_per_g_ * standard_gravity_;
  imu.linear_acceleration.y = status.acc_y_raw / acc_lsb_per_g_ * standard_gravity_;
  imu.linear_acceleration.z = status.acc_z_raw / acc_lsb_per_g_ * standard_gravity_;
  const double gyro_scale = M_PI / 180.0 / gyro_lsb_per_deg_s_;
  imu.angular_velocity.x = status.gyro_x_raw * gyro_scale;
  imu.angular_velocity.y = status.gyro_y_raw * gyro_scale;
  imu.angular_velocity.z = status.gyro_z_raw * gyro_scale;
  imu_pub_.publish(imu);

  PublishOdometry(status, stamp);

  std_msgs::Float32 voltage_msg;
  voltage_msg.data = static_cast<float>(status.voltage);
  voltage_pub_.publish(voltage_msg);

  std_msgs::UInt8 flag_msg;
  flag_msg.data = status.flag_stop;
  flag_stop_pub_.publish(flag_msg);
}

void BaseDriver::PublishOdometry(const StatusFrame& status, const ros::Time& stamp) {
  double dt = 0.0;
  if (!last_odom_time_.isZero()) {
    dt = (stamp - last_odom_time_).toSec();
  }
  last_odom_time_ = stamp;

  if (dt > 0.0 && dt < 1.0) {
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    x_ += (status.vx * cos_yaw - status.vy * sin_yaw) * dt;
    y_ += (status.vx * sin_yaw + status.vy * cos_yaw) * dt;
    yaw_ += status.wz * dt;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
  }

  const double qz = std::sin(yaw_ * 0.5);
  const double qw = std::cos(yaw_ * 0.5);

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;
  odom.twist.twist.linear.x = status.vx;
  odom.twist.twist.linear.y = status.vy;
  odom.twist.twist.angular.z = status.wz;
  odom_pub_.publish(odom);

  if (publish_tf_) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;
    tf_broadcaster_.sendTransform(transform);
  }
}

void BaseDriver::DiagnosticTimerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  const double age = last_status_time_.isZero() ? 999.0 : (now - last_status_time_).toSec();

  diagnostic_msgs::DiagnosticStatus status;
  status.name = "eggy_base_driver/stm32_serial_cpp";
  status.hardware_id = port_;
  if (age > status_timeout_) {
    status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = "No recent STM32 status frame";
  } else if (last_flag_stop_ != 0) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "STM32 reports Flag_Stop != 0";
  } else {
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "OK";
  }

  auto add_value = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  add_value("port", port_);
  add_value("voltage_v", std::to_string(last_voltage_));
  add_value("flag_stop", std::to_string(last_flag_stop_));
  add_value("valid_frames", std::to_string(valid_frames_));
  add_value("bad_frames", std::to_string(bad_frames_));
  add_value("last_status_age_s", std::to_string(age));

  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = now;
  array.status.push_back(status);
  diagnostics_pub_.publish(array);
}

void BaseDriver::PublishStopFrames() {
  const auto stop = Stm32Protocol::BuildControlFrame(0.0, 0.0, 0.0, max_linear_x_, max_linear_y_, max_angular_z_);
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_.IsOpen()) {
    return;
  }
  for (int i = 0; i < 5; ++i) {
    serial_.Write(stop.data(), stop.size());
    ros::Duration(0.02).sleep();
  }
}

}  // namespace eggy_base_driver

#include <cmath>
#include <mutex>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

#include "eggy_bringup/odom_fusion_math.h"

class EggyOdomFuser {
 public:
  EggyOdomFuser() : nh_(), pnh_("~") {
    pnh_.param<std::string>("odom_frame_id", odom_frame_, "odom");
    pnh_.param<std::string>("base_frame_id", base_frame_, "base_link");
    pnh_.param<std::string>("wheel_odom_topic", wheel_topic_, "/wheel_odom");
    pnh_.param<std::string>("imu_topic", imu_topic_, "/stm32/imu/data_raw");
    pnh_.param<bool>("publish_tf", publish_tf_, true);
    pnh_.param<double>("rate", rate_hz_, 30.0);
    pnh_.param<int>("bias_samples", bias_samples_, 50);
    pnh_.param<double>("wheel_yaw_correction_rate",
                       wheel_yaw_correction_rate_, 0.35);
    pnh_.param<double>("stationary_linear_threshold",
                       stationary_linear_threshold_, 0.015);
    pnh_.param<double>("stationary_angular_threshold",
                       stationary_angular_threshold_, 0.025);
    pnh_.param<double>("bias_learning_rate", bias_learning_rate_, 0.002);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 20);
    wheel_sub_ = nh_.subscribe(wheel_topic_, 50, &EggyOdomFuser::wheelCb, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 200, &EggyOdomFuser::imuCb, this);

    ROS_INFO("MPU6050 C++ odom fuser: wheel=%s imu=%s rate=%.1fHz tf=%s bias_samples=%d yaw_correction=%.2f/s",
             wheel_topic_.c_str(), imu_topic_.c_str(), rate_hz_, publish_tf_ ? "true" : "false",
             bias_samples_, wheel_yaw_correction_rate_);
  }

  void spin() {
    ros::Rate rate(rate_hz_);
    bool warned = false;
    while (ros::ok()) {
      ros::spinOnce();

      nav_msgs::Odometry wheel;
      double yaw = 0.0;
      bool ready = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_wheel_ && has_yaw_) {
          wheel = latest_wheel_;
          yaw = base_yaw_;
          ready = true;
        }
      }

      if (!ready) {
        if (!warned) {
          ROS_WARN("waiting for wheel odom and MPU6050 gyro calib...");
          warned = true;
        }
        rate.sleep();
        continue;
      }

      const ros::Time now = ros::Time::now();
      nav_msgs::Odometry out;
      out.header.stamp = now;
      out.header.frame_id = odom_frame_;
      out.child_frame_id = base_frame_;
      out.pose.pose.position = wheel.pose.pose.position;
      out.pose.pose.orientation = quatFromYaw(yaw);
      out.pose.covariance = wheel.pose.covariance;
      out.twist.twist.linear = wheel.twist.twist.linear;
      out.twist.twist.angular.z = wheel.twist.twist.angular.z;
      out.twist.covariance = wheel.twist.covariance;
      odom_pub_.publish(out);

      if (publish_tf_) {
        geometry_msgs::TransformStamped tr;
        tr.header.stamp = now;
        tr.header.frame_id = odom_frame_;
        tr.child_frame_id = base_frame_;
        tr.transform.translation.x = out.pose.pose.position.x;
        tr.transform.translation.y = out.pose.pose.position.y;
        tr.transform.translation.z = out.pose.pose.position.z;
        tr.transform.rotation = out.pose.pose.orientation;
        tf_broadcaster_.sendTransform(tr);
      }

      rate.sleep();
    }
  }

 private:
  static double yawFromQuat(const geometry_msgs::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::Quaternion quatFromYaw(double yaw) {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  void wheelCb(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_wheel_ = *msg;
    has_wheel_ = true;
  }

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
    const double gz = msg->angular_velocity.z;
    const double now = (msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp).toSec();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_bias_) {
      bias_sum_ += gz;
      bias_count_ += 1;
      if (bias_count_ >= bias_samples_) {
        gyro_bias_ = bias_sum_ / static_cast<double>(bias_count_);
        base_yaw_ = has_wheel_ ? yawFromQuat(latest_wheel_.pose.pose.orientation) : 0.0;
        last_stamp_ = now;
        has_bias_ = true;
        has_yaw_ = true;
        ROS_INFO("MPU6050 C++ fuser ready: gyro_bias=%.6f rad/s, init_yaw=%.2f deg",
                 gyro_bias_, base_yaw_ * 180.0 / M_PI);
      }
      return;
    }

    const double dt = now - last_stamp_;
    last_stamp_ = now;
    if (dt <= 0.0 || dt > 0.2) {
      return;
    }
    const bool stationary =
        has_wheel_ &&
        std::hypot(latest_wheel_.twist.twist.linear.x,
                   latest_wheel_.twist.twist.linear.y) <=
            stationary_linear_threshold_ &&
        std::abs(latest_wheel_.twist.twist.angular.z) <=
            stationary_angular_threshold_;
    if (stationary) {
      gyro_bias_ = eggy_bringup::LearnStationaryGyroBias(
          gyro_bias_, gz, bias_learning_rate_);
    }
    const double integrated = eggy_bringup::NormalizeAngle(
        base_yaw_ + (gz - gyro_bias_) * dt);
    base_yaw_ = has_wheel_
                    ? eggy_bringup::CorrectYawTowardWheel(
                          integrated,
                          yawFromQuat(latest_wheel_.pose.pose.orientation),
                          wheel_yaw_correction_rate_, dt)
                    : integrated;
    has_yaw_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher odom_pub_;
  ros::Subscriber wheel_sub_;
  ros::Subscriber imu_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::mutex mutex_;
  nav_msgs::Odometry latest_wheel_;
  bool has_wheel_{false};
  bool has_bias_{false};
  bool has_yaw_{false};
  double base_yaw_{0.0};
  double gyro_bias_{0.0};
  double bias_sum_{0.0};
  int bias_count_{0};
  int bias_samples_{50};
  double last_stamp_{0.0};
  double wheel_yaw_correction_rate_{0.35};
  double stationary_linear_threshold_{0.015};
  double stationary_angular_threshold_{0.025};
  double bias_learning_rate_{0.002};

  std::string odom_frame_;
  std::string base_frame_;
  std::string wheel_topic_;
  std::string imu_topic_;
  bool publish_tf_{true};
  double rate_hz_{30.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "eggy_external_imu_odom_fuser");
  EggyOdomFuser fuser;
  fuser.spin();
  return 0;
}

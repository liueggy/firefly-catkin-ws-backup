#pragma once

#include <algorithm>
#include <cmath>

namespace eggy_bringup {

inline double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double CorrectYawTowardWheel(double integrated_yaw, double wheel_yaw,
                                    double correction_rate, double dt) {
  const double gain =
      (std::max)(0.0, (std::min)(correction_rate * dt, 1.0));
  return NormalizeAngle(integrated_yaw +
                        gain * NormalizeAngle(wheel_yaw - integrated_yaw));
}

inline double LearnStationaryGyroBias(double current_bias, double gyro_z,
                                      double learning_rate) {
  const double gain = (std::max)(0.0, (std::min)(learning_rate, 1.0));
  return current_bias + gain * (gyro_z - current_bias);
}

}  // namespace eggy_bringup

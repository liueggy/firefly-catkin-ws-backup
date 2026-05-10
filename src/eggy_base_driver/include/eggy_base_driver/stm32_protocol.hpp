#ifndef EGGY_BASE_DRIVER_STM32_PROTOCOL_HPP
#define EGGY_BASE_DRIVER_STM32_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include <vector>

namespace eggy_base_driver {

constexpr uint8_t kFrameHeader = 0x7B;
constexpr uint8_t kFrameTail = 0x7D;
constexpr std::size_t kControlFrameSize = 11;
constexpr std::size_t kStatusFrameSize = 24;

struct StatusFrame {
  uint8_t flag_stop = 0;
  double vx = 0.0;       // m/s
  double vy = 0.0;       // m/s
  double wz = 0.0;       // rad/s
  int16_t acc_x_raw = 0;
  int16_t acc_y_raw = 0;
  int16_t acc_z_raw = 0;
  int16_t gyro_x_raw = 0;
  int16_t gyro_y_raw = 0;
  int16_t gyro_z_raw = 0;
  double voltage = 0.0;  // V
};

class Stm32Protocol {
 public:
  static std::array<uint8_t, kControlFrameSize> BuildControlFrame(double vx, double vy, double wz,
                                                                  double max_vx, double max_vy,
                                                                  double max_wz);
  static bool ParseStatusFrame(const std::array<uint8_t, kStatusFrameSize>& frame, StatusFrame* status);
  static uint8_t XorChecksum(const uint8_t* data, std::size_t length);

 private:
  static int16_t ReadInt16BE(const uint8_t* data);
  static void AppendInt16BE(std::array<uint8_t, kControlFrameSize>* frame, std::size_t offset, int16_t value);
  static double Clamp(double value, double low, double high);
};

}  // namespace eggy_base_driver

#endif  // EGGY_BASE_DRIVER_STM32_PROTOCOL_HPP

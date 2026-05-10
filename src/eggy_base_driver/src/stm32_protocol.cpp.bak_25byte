#include "eggy_base_driver/stm32_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace eggy_base_driver {

std::array<uint8_t, kControlFrameSize> Stm32Protocol::BuildControlFrame(
    double vx, double vy, double wz, double max_vx, double max_vy, double max_wz) {
  const int16_t x = static_cast<int16_t>(std::lround(Clamp(vx, -max_vx, max_vx) * 1000.0));
  const int16_t y = static_cast<int16_t>(std::lround(Clamp(vy, -max_vy, max_vy) * 1000.0));
  const int16_t z = static_cast<int16_t>(std::lround(Clamp(wz, -max_wz, max_wz) * 1000.0));

  std::array<uint8_t, kControlFrameSize> frame{};
  frame[0] = kFrameHeader;
  frame[1] = 0x00;
  frame[2] = 0x00;
  AppendInt16BE(&frame, 3, x);
  AppendInt16BE(&frame, 5, y);
  AppendInt16BE(&frame, 7, z);
  frame[9] = XorChecksum(frame.data(), 9);
  frame[10] = kFrameTail;
  return frame;
}

bool Stm32Protocol::ParseStatusFrame(const std::array<uint8_t, kStatusFrameSize>& frame,
                                     StatusFrame* status) {
  if (status == nullptr) {
    return false;
  }
  if (frame[0] != kFrameHeader || frame[23] != kFrameTail) {
    return false;
  }
  if (XorChecksum(frame.data(), 22) != frame[22]) {
    return false;
  }

  status->flag_stop = frame[1];
  status->vx = ReadInt16BE(&frame[2]) / 1000.0;
  status->vy = ReadInt16BE(&frame[4]) / 1000.0;
  status->wz = ReadInt16BE(&frame[6]) / 1000.0;
  status->acc_x_raw = ReadInt16BE(&frame[8]);
  status->acc_y_raw = ReadInt16BE(&frame[10]);
  status->acc_z_raw = ReadInt16BE(&frame[12]);
  status->gyro_x_raw = ReadInt16BE(&frame[14]);
  status->gyro_y_raw = ReadInt16BE(&frame[16]);
  status->gyro_z_raw = ReadInt16BE(&frame[18]);
  status->voltage = ReadInt16BE(&frame[20]) / 1000.0;
  return true;
}

uint8_t Stm32Protocol::XorChecksum(const uint8_t* data, std::size_t length) {
  uint8_t checksum = 0;
  for (std::size_t i = 0; i < length; ++i) {
    checksum ^= data[i];
  }
  return checksum;
}

int16_t Stm32Protocol::ReadInt16BE(const uint8_t* data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
}

void Stm32Protocol::AppendInt16BE(std::array<uint8_t, kControlFrameSize>* frame, std::size_t offset,
                                  int16_t value) {
  const auto raw = static_cast<uint16_t>(value);
  (*frame)[offset] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  (*frame)[offset + 1] = static_cast<uint8_t>(raw & 0xFF);
}

double Stm32Protocol::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

}  // namespace eggy_base_driver

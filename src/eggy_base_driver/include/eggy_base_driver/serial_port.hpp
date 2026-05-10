#ifndef EGGY_BASE_DRIVER_SERIAL_PORT_HPP
#define EGGY_BASE_DRIVER_SERIAL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>
#include <termios.h>

namespace eggy_base_driver {

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();

  bool Open(const std::string& port, int baudrate);
  void Close();
  bool IsOpen() const;
  ssize_t Read(uint8_t* buffer, std::size_t size);
  bool Write(const uint8_t* data, std::size_t size);
  const std::string& port() const { return port_; }

 private:
  static speed_t ToSpeed(int baudrate);
  int fd_ = -1;
  std::string port_;
};

}  // namespace eggy_base_driver

#endif  // EGGY_BASE_DRIVER_SERIAL_PORT_HPP

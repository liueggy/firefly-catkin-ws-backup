#include "eggy_base_driver/serial_port.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace eggy_base_driver {

SerialPort::~SerialPort() { Close(); }

bool SerialPort::Open(const std::string& port, int baudrate) {
  Close();
  port_ = port;
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    Close();
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, ToSpeed(baudrate));
  cfsetospeed(&tty, ToSpeed(baudrate));

  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    Close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::Close() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::IsOpen() const { return fd_ >= 0; }

ssize_t SerialPort::Read(uint8_t* buffer, std::size_t size) {
  if (fd_ < 0) {
    return -1;
  }
  const ssize_t n = read(fd_, buffer, size);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return 0;
  }
  return n;
}

bool SerialPort::Write(const uint8_t* data, std::size_t size) {
  if (fd_ < 0) {
    return false;
  }
  std::size_t written = 0;
  while (written < size) {
    const ssize_t n = write(fd_, data + written, size - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(1000);
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

speed_t SerialPort::ToSpeed(int baudrate) {
  switch (baudrate) {
    case 9600:
      return B9600;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
  }
}

}  // namespace eggy_base_driver

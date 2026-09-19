#include "serial_port.hpp"

#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {
speed_t baud_const(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}
}  // namespace

bool SerialPort::open(const std::string& dev, int baud) {
    close();
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "[serial] cannot open %s\n", dev.c_str());
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

    cfsetospeed(&tty, baud_const(baud));
    cfsetispeed(&tty, baud_const(baud));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8-bit
    tty.c_cflag |= (CLOCAL | CREAD);              // ignore modem ctrl, enable RX
    tty.c_cflag &= ~(PARENB | PARODD);            // no parity
    tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                      // no HW flow control

    cfmakeraw(&tty);                              // raw mode
    tty.c_cc[VMIN]  = 0;                          // non-blocking read
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

int SerialPort::read(uint8_t* buf, int max) {
    if (fd_ < 0) return 0;
    const ssize_t n = ::read(fd_, buf, max);
    return n > 0 ? (int)n : 0;
}

bool SerialPort::write(const uint8_t* buf, int len) {
    if (fd_ < 0) return false;
    return ::write(fd_, buf, len) == len;
}

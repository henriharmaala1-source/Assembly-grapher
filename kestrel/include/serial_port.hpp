#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Minimal blocking-free POSIX serial port (termios). Raw 8N1. Linux/Pi only.
class SerialPort {
public:
    ~SerialPort() { close(); }

    bool open(const std::string& dev, int baud);   // e.g. "/dev/ttyAMA0", 115200
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Non-blocking read of whatever is available (up to buf size). Returns count.
    int  read(uint8_t* buf, int max);
    bool write(const uint8_t* buf, int len);

private:
    int fd_ = -1;
};

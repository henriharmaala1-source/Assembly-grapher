// SerialPort for builds with no termios -- the Windows desktop build of
// nav-sim, which talks to ArduPilot SITL over tcp:/udp: (byte_link.hpp) and
// has no flight controller on a COM port to open. CMake picks this file in
// place of serial_port.cpp; it compiles anywhere, so it is checked on Linux
// too (test_mavlink_backend links the real one).
#include "serial_port.hpp"

#include <cstdio>

bool SerialPort::open(const std::string& dev, int) {
    std::fprintf(stderr, "[serial] %s: this build has no serial ports -- use tcp:HOST:PORT, "
                         "udpout:HOST:PORT or udp:PORT\n", dev.c_str());
    return false;
}
void SerialPort::close() { fd_ = -1; }
int SerialPort::read(uint8_t*, int) { return 0; }
bool SerialPort::write(const uint8_t*, int) { return false; }

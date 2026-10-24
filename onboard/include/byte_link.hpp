#pragma once

#include <cstdint>
#include <string>

#include "serial_port.hpp"

// ByteLink -- the byte stream a MAVLink backend talks over, chosen by the
// shape of the port string:
//
//   /dev/ttyAMA0, COM3 ...   a serial device (the Pi on the flight controller)
//   tcp:HOST:PORT            TCP client      (ArduPilot SITL serves SERIAL0 on
//                                             tcp:127.0.0.1:5760)
//   udpout:HOST:PORT         UDP, sending to HOST:PORT and reading its replies
//   udp:PORT                 UDP, listening on PORT; replies go to whoever
//                             sent last (MAVProxy-style "udpin")
//
// So the SAME backend, with the same tests, flies the real aircraft over a
// UART and the simulated one over a socket: a SITL run exercises the code that
// flies, not a parallel implementation of it.
//
// Non-blocking throughout: read() returns what is there (0 if nothing), and
// write() never waits. A TCP peer that goes away closes the link -- isOpen()
// turns false and the backend's link-up logic sees silence, as it would on a
// cut wire.
class ByteLink {
public:
    ~ByteLink() { close(); }

    bool open(const std::string& spec, int baud);
    void close();
    bool isOpen() const;

    int  read(uint8_t* buf, int max);
    bool write(const uint8_t* buf, int len);

    // "serial", "tcp", "udpout", "udp" -- for log lines.
    const char* kind() const;

private:
    enum class Kind { NONE, SERIAL, TCP, UDP_OUT, UDP_IN };
    Kind        kind_ = Kind::NONE;
    SerialPort  serial_;
    long long   sock_ = -1;               // socket handle (SOCKET on Windows)
    // UDP: where datagrams go. Fixed for udpout; the last sender for udp:PORT.
    unsigned char peer_[32] = {};
    int         peerLen_ = 0;
};

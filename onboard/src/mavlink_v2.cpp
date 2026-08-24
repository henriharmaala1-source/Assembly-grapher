#include "mavlink_v2.hpp"

namespace mav {

// CRC_EXTRA and full payload length, lifted from pymavlink's own tables for the
// common dialect. These two columns are the entire compatibility contract with
// the autopilot; test_mavlink.cpp pins every row against real pymavlink output.
namespace {
struct Row { uint32_t id; uint8_t crc; int len; };
constexpr Row kMsgs[] = {
    { MSG_HEARTBEAT,                      50,   9 },
    { MSG_SYS_STATUS,                    124,  31 },
    { MSG_GPS_RAW_INT,                    24,  52 },
    { MSG_ATTITUDE,                       39,  28 },
    { MSG_GLOBAL_POSITION_INT,           104,  28 },
    { MSG_RC_CHANNELS,                   118,  42 },
    { MSG_RC_CHANNELS_OVERRIDE,          124,  38 },
    { MSG_COMMAND_LONG,                  152,  33 },
    { MSG_COMMAND_ACK,                   143,  10 },
    { MSG_SET_ATTITUDE_TARGET,            49,  39 },
    { MSG_SET_POSITION_TARGET_LOCAL_NED, 143,  53 },
    { MSG_VISION_POSITION_ESTIMATE,      158, 117 },
    { MSG_VISION_SPEED_ESTIMATE,         208,  57 },
    { MSG_EKF_STATUS_REPORT,              71,  22 },
    { MSG_STATUSTEXT,                     83,  54 },
};
const Row* find(uint32_t id) {
    for (const Row& r : kMsgs) if (r.id == id) return &r;
    return nullptr;
}
}  // namespace

bool crcExtraFor(uint32_t msgid, uint8_t& out) {
    if (const Row* r = find(msgid)) { out = r->crc; return true; }
    return false;
}

int maxPayloadFor(uint32_t msgid) {
    const Row* r = find(msgid);
    return r ? r->len : 0;
}

uint16_t crcAccumulate(uint8_t byte, uint16_t crc) {
    uint8_t tmp = byte ^ uint8_t(crc & 0xFF);
    tmp ^= uint8_t(tmp << 4);
    return uint16_t((crc >> 8) ^ (uint16_t(tmp) << 8) ^ (uint16_t(tmp) << 3)
                                ^ (uint16_t(tmp) >> 4));
}

int Codec::frame(uint32_t msgid, const Payload& p, uint8_t* out) {
    uint8_t extra;
    if (!crcExtraFor(msgid, extra)) return 0;

    // v2 TRUNCATION: strip trailing zeros, but never below one byte. Doing this
    // is not optional politeness -- ArduPilot sizes its own decode against the
    // received length, and a frame padded to full length is still accepted, but
    // the CRC is computed over the TRUNCATED payload. Encode it one way and
    // check it the other and every frame is silently dropped.
    int n = p.n;
    while (n > 1 && p.b[n - 1] == 0) --n;

    int k = 0;
    out[k++] = 0xFD;
    out[k++] = uint8_t(n);
    out[k++] = 0;                 // incompat_flags: 0 = unsigned
    out[k++] = 0;                 // compat_flags
    out[k++] = seq_++;
    out[k++] = sysid_;
    out[k++] = compid_;
    out[k++] = uint8_t(msgid);
    out[k++] = uint8_t(msgid >> 8);
    out[k++] = uint8_t(msgid >> 16);
    for (int i = 0; i < n; ++i) out[k++] = p.b[i];

    uint16_t crc = 0xFFFF;
    for (int i = 1; i < k; ++i) crc = crcAccumulate(out[i], crc);   // from len onward
    crc = crcAccumulate(extra, crc);
    out[k++] = uint8_t(crc & 0xFF);
    out[k++] = uint8_t(crc >> 8);
    return k;
}

bool Codec::feed(uint8_t byte, Msg& out) {
    switch (st_) {
    case St::STX:
        if (byte == 0xFD) { st_ = St::LEN; crc_ = 0xFFFF; }
        return false;
    case St::LEN:
        rxLen_ = byte; crc_ = crcAccumulate(byte, crc_); st_ = St::INCOMPAT; return false;
    case St::INCOMPAT:
        rxIncompat_ = byte; crc_ = crcAccumulate(byte, crc_); st_ = St::COMPAT; return false;
    case St::COMPAT:
        crc_ = crcAccumulate(byte, crc_); st_ = St::SEQ; return false;
    case St::SEQ:
        crc_ = crcAccumulate(byte, crc_); st_ = St::SYSID; return false;
    case St::SYSID:
        rxSys_ = byte; crc_ = crcAccumulate(byte, crc_); st_ = St::COMPID; return false;
    case St::COMPID:
        rxComp_ = byte; crc_ = crcAccumulate(byte, crc_);
        st_ = St::MSGID; idx_ = 0; rxId_ = 0; return false;
    case St::MSGID:
        rxId_ |= uint32_t(byte) << (8 * idx_);
        crc_ = crcAccumulate(byte, crc_);
        if (++idx_ == 3) {
            idx_ = 0;
            st_ = rxLen_ ? St::PAYLOAD : St::CRC;
        }
        return false;
    case St::PAYLOAD:
        rxPay_[idx_++] = byte;
        crc_ = crcAccumulate(byte, crc_);
        if (idx_ >= rxLen_) { st_ = St::CRC; idx_ = 0; }
        return false;
    case St::CRC:
        if (idx_ == 0) { rxCrc_ = byte; ++idx_; return false; }
        rxCrc_ |= uint16_t(byte) << 8;
        st_ = St::STX;
        {
            uint8_t extra;
            // Unknown message -- not an error, just not ours. An autopilot emits
            // dozens we never asked for and counting them as CRC failures would
            // make the only health signal on this link useless.
            if (!crcExtraFor(rxId_, extra)) return false;
            // Signed frames carry 13 trailing bytes we did not consume; rather
            // than half-parse one, refuse. Enabling signing is a deliberate act
            // on the FC and should fail visibly here, not corrupt the stream.
            if (rxIncompat_ & 0x01) return false;
            uint16_t want = crcAccumulate(extra, crc_);
            if (want != rxCrc_) { ++crcErr_; return false; }
            ++framesRx_;
            out.id = rxId_; out.sysid = rxSys_; out.compid = rxComp_;
            const int full = maxPayloadFor(rxId_);
            out.len = full;
            // ZERO-EXTEND. The sender stripped trailing zeros; every reader below
            // indexes by the message's documented offsets, so the tail must be
            // materialised as the zeros it stood for.
            for (int i = 0; i < full; ++i) out.pay[i] = (i < rxLen_) ? rxPay_[i] : 0;
            return true;
        }
    }
    return false;
}

}  // namespace mav

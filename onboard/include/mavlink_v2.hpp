#pragma once

#include <cstdint>
#include <cstring>

// ---------------------------------------------------------------------------
// MAVLink v2 framing and the handful of messages this project actually uses.
//
// WRITTEN OUT RATHER THAN GENERATED, on purpose. The official route is to
// generate ~400 headers from common.xml and add mavgen to the build. We use
// fourteen messages. Vendoring a code generator and a megabyte of headers to
// get fourteen structs would make this the largest dependency in a tree whose
// whole discipline is not having any -- the tracker is OpenCV-free for the same
// reason, and the Pi has to build this.
//
// THE RISK OF HAND-ROLLING IS SILENT REJECTION, and it is handled by testing
// rather than by care. A wrong CRC_EXTRA or a field one byte out of order does
// not error: the autopilot drops the frame and the aircraft simply does not
// respond, which on a bench looks identical to a wiring fault. So every message
// below is pinned by a GOLDEN FRAME in test/test_mavlink.cpp -- the exact bytes
// pymavlink emits for the same arguments, byte-for-byte. The canonical
// implementation is the oracle; this file only has to agree with it.
//
// THREE THINGS THAT BITE, all of them v2-specific:
//
//   PAYLOAD TRUNCATION  v2 drops trailing ZERO bytes from the wire payload, so
//                       the same message is a different length depending on its
//                       VALUES. RC_CHANNELS_OVERRIDE with channels 9-18 unset is
//                       18 bytes, not 38. Encoders that skip this still parse;
//                       decoders that do not ZERO-EXTEND read garbage.
//   CRC_EXTRA           the CRC covers the header, the truncated payload, and
//                       then one extra byte derived from the message's field
//                       signature. It exists precisely to make a version
//                       mismatch fail loudly instead of silently.
//   FIELD ORDER         the wire order is NOT the XML order. Fields are sorted
//                       by descending type size, with the extension fields kept
//                       at the end unsorted. Every layout here came from
//                       pymavlink's own ordered field list, not from reading the
//                       XML top to bottom.
//
// Frame: 0xFD | len | incompat | compat | seq | sysid | compid | msgid[3] |
//        payload[len] | crc[2].  incompat = 0 means unsigned, which is what we
// send; signed frames (incompat bit 0) are rejected by the parser rather than
// mis-parsed.
// ---------------------------------------------------------------------------

namespace mav {

enum : uint32_t {
    MSG_HEARTBEAT                     = 0,
    MSG_SYS_STATUS                    = 1,
    MSG_GPS_RAW_INT                   = 24,
    MSG_ATTITUDE                      = 30,
    MSG_GLOBAL_POSITION_INT           = 33,
    MSG_RC_CHANNELS                   = 65,
    MSG_RC_CHANNELS_OVERRIDE          = 70,
    MSG_COMMAND_LONG                  = 76,
    MSG_COMMAND_ACK                   = 77,
    MSG_SET_ATTITUDE_TARGET           = 82,
    MSG_SET_POSITION_TARGET_LOCAL_NED = 84,
    MSG_VISION_POSITION_ESTIMATE      = 102,
    MSG_VISION_SPEED_ESTIMATE         = 103,
    MSG_EKF_STATUS_REPORT             = 193,
    MSG_OBSTACLE_DISTANCE             = 330,
    MSG_STATUSTEXT                    = 253,
};

// MAV_CMD values used with COMMAND_LONG.
enum : uint16_t {
    CMD_DO_SET_MODE          = 176,
    CMD_COMPONENT_ARM_DISARM = 400,
    CMD_SET_MESSAGE_INTERVAL = 511,
};

// ArduCopter custom_mode values (COPTER_MODE). ACRO is 1 -- the manual mode an
// FPV pilot flies -- and GUIDED / GUIDED_NOGPS are where a companion computer
// gets to steer. Those two coexisting on one AUX switch is the entire reason
// this backend is worth having.
enum : uint32_t {
    COPTER_STABILIZE = 0,  COPTER_ACRO   = 1,  COPTER_ALT_HOLD = 2,
    COPTER_AUTO      = 3,  COPTER_GUIDED = 4,  COPTER_LOITER   = 5,
    COPTER_RTL       = 6,  COPTER_LAND   = 9,  COPTER_POSHOLD  = 16,
    COPTER_BRAKE     = 17, COPTER_GUIDED_NOGPS = 20, COPTER_SMART_RTL = 21,
};

enum : uint8_t {
    MAV_TYPE_ONBOARD_CONTROLLER = 18,
    MAV_AUTOPILOT_INVALID       = 8,
    MAV_STATE_ACTIVE            = 4,
    MODE_FLAG_CUSTOM_MODE_ENABLED = 1,   // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    MODE_FLAG_SAFETY_ARMED        = 0x80,
};

// MAV_FRAME. BODY_NED (8) is the one that matters: velocity setpoints expressed
// in the aircraft's own frame, which is exactly what the trajectory planner
// produces and saves the companion from having to know its own heading.
enum : uint8_t { FRAME_LOCAL_NED = 1, FRAME_BODY_NED = 8, FRAME_BODY_OFFSET_NED = 9 };

// CRC_EXTRA for the messages above; 0 (and false) for anything else, which the
// parser treats as "not for us" rather than guessing.
bool crcExtraFor(uint32_t msgid, uint8_t& out);

// Max payload length per known message, needed for ZERO-EXTENSION on receive:
// a truncated frame must be read as though the missing tail were zeros.
int maxPayloadFor(uint32_t msgid);

// Little-endian payload writer. Fixed 255-byte buffer, no allocation -- this
// runs on the FC link thread next to a real-time deadline.
struct Payload {
    uint8_t b[255]{};
    int     n = 0;

    void u8 (uint8_t  v) { b[n++] = v; }
    void i8 (int8_t   v) { u8(uint8_t(v)); }
    void u16(uint16_t v) { b[n++] = uint8_t(v); b[n++] = uint8_t(v >> 8); }
    void i16(int16_t  v) { u16(uint16_t(v)); }
    void u32(uint32_t v) { for (int i = 0; i < 4; ++i) b[n++] = uint8_t(v >> (8 * i)); }
    void i32(int32_t  v) { u32(uint32_t(v)); }
    void u64(uint64_t v) { for (int i = 0; i < 8; ++i) b[n++] = uint8_t(v >> (8 * i)); }
    void f32(float    v) { uint32_t t; std::memcpy(&t, &v, 4); u32(t); }
    void zeros(int k)    { for (int i = 0; i < k; ++i) b[n++] = 0; }
};

// A decoded message. `pay` is ZERO-EXTENDED to the message's full length, so
// readers never have to think about truncation -- the single most common source
// of "it works until the value happens to be zero" bugs in hand-written MAVLink.
struct Msg {
    uint32_t id     = 0xFFFFFFFF;
    uint8_t  sysid  = 0, compid = 0;
    uint8_t  pay[255]{};
    int      len    = 0;   // zero-extended length, not the wire length

    uint8_t  u8 (int o) const { return pay[o]; }
    int8_t   i8 (int o) const { return int8_t(pay[o]); }
    uint16_t u16(int o) const { return uint16_t(pay[o] | (pay[o+1] << 8)); }
    int16_t  i16(int o) const { return int16_t(u16(o)); }
    uint32_t u32(int o) const {
        return uint32_t(pay[o]) | (uint32_t(pay[o+1]) << 8)
             | (uint32_t(pay[o+2]) << 16) | (uint32_t(pay[o+3]) << 24);
    }
    int32_t  i32(int o) const { return int32_t(u32(o)); }
    uint64_t u64(int o) const {
        uint64_t v = 0;
        for (int i = 7; i >= 0; --i) v = (v << 8) | pay[o + i];
        return v;
    }
    float    f32(int o) const { uint32_t t = u32(o); float f; std::memcpy(&f, &t, 4); return f; }
};

// CRC-16/MCRF4XX, the X.25 variant MAVLink uses. Init 0xFFFF.
uint16_t crcAccumulate(uint8_t byte, uint16_t crc);

class Codec {
public:
    Codec(uint8_t sysid, uint8_t compid) : sysid_(sysid), compid_(compid) {}

    void setIds(uint8_t sysid, uint8_t compid) { sysid_ = sysid; compid_ = compid; }
    uint8_t sysid() const { return sysid_; }

    // Frame a message into `out` (needs 12 + 255 bytes). Returns the byte count,
    // or 0 if the message id has no CRC_EXTRA here. Applies v2 truncation and
    // advances the sequence counter.
    int frame(uint32_t msgid, const Payload& p, uint8_t* out);

    // Byte-at-a-time receive. Returns true when `out` holds a complete, CRC-valid
    // message of a KNOWN id. Unknown ids and signed frames are consumed and
    // discarded -- an autopilot emits plenty of both and neither is an error.
    bool feed(uint8_t byte, Msg& out);

    long crcErrors() const { return crcErr_; }
    long framesRx()  const { return framesRx_; }

private:
    uint8_t sysid_, compid_;
    uint8_t seq_ = 0;

    enum class St { STX, LEN, INCOMPAT, COMPAT, SEQ, SYSID, COMPID, MSGID, PAYLOAD, CRC };
    St       st_ = St::STX;
    uint8_t  rxLen_ = 0, rxIncompat_ = 0, rxSys_ = 0, rxComp_ = 0;
    uint32_t rxId_  = 0;
    int      idx_ = 0;
    uint8_t  rxPay_[255]{};
    uint16_t crc_ = 0, rxCrc_ = 0;
    long     crcErr_ = 0, framesRx_ = 0;
};

}  // namespace mav

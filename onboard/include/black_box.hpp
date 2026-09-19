#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "world_model.hpp"

// Crash-survivable flight-data black box.
//
// WHY a "black box" and not a full recorder/replay tool: the first hardware
// trials are exactly when we have no idea what will go wrong, and the failure
// modes that matter most (brownout, lock-up, crash-on-impact) are the ones a
// buffered-in-RAM recorder loses. So this is built to the opposite priority:
// keep whatever made it to disk, no matter how the process dies.
//
//   * APPEND-ONLY, FIXED-SIZE records — a power cut mid-write corrupts at most
//     the final record; every record before it stays readable.
//   * PER-RECORD CRC32 — a torn final record is detected and skipped by the
//     decoder, never silently misparsed as real flight data.
//   * PERIODIC fsync — data actually reaches the storage medium on a bounded
//     interval, not just the OS page cache (which a hard reset would drop).
//   * RECORD SYNC WORD — the decoder can resynchronise to the next record if a
//     file is truncated or spliced, rather than giving up at the first bad byte.
//
// It is deliberately NOT a replay engine. Decoding is a separate offline tool
// (blackbox_decode) that emits CSV; anything richer (replay through the SITL
// harness) is future work and does not belong in the flight-critical path.
//
// Overhead: one fixed ~120-byte record per fly-loop tick into a stdio buffer;
// the fsync happens on a timer (default 1 s), off the per-tick cost. Logging is
// best-effort — a write error disables the box, it never throws or blocks the
// control loop.

namespace bb {

inline constexpr uint16_t kVersion    = 1;
inline constexpr uint16_t kRecordSync = 0xEB90;   // starts every record
inline constexpr int      kOpModeLen  = 8;        // fixed, NUL-padded
inline constexpr int      kPhaseLen   = 8;        // fixed, NUL-padded

// File header, written once at open on a fresh file:
//   'K''B''B''1'  version:u16le  recordSize:u16le
inline constexpr char kMagic[4] = {'K', 'B', 'B', '1'};
inline constexpr int  kHeaderSize = 4 + 2 + 2;

// Fixed record layout (little-endian), sync..payload covered by the trailing
// CRC32. Sizes must match the packer in black_box.cpp exactly (static_assert'd
// there against kRecordSize).
//   sync:u16  tMonoS:f64  frameId:u32  behavior:u8  flags:u8
//   opMode:char[8]  phase:char[8]
//   ctlRoll ctlPitch ctlYaw ctlThrottle : f32 x4
//   estPe estPn estPu : f32 x3   estVe estVn estVu : f32 x3   estEphM : f32
//   vehBattV vehAltM vehRollDeg vehPitchDeg vehYawDeg : f32 x5
//   vehLat vehLon : f64 x2   vehSats:u8  vehFix:u8   corridorOpen : f32
//   crc32 : u32
inline constexpr int kRecordSize =
    2 + 8 + 4 + 1 + 1 + kOpModeLen + kPhaseLen +
    16 + 12 + 12 + 4 + 20 + 16 + 1 + 1 + 4 + 4;

// flags bitfield
enum Flag : uint8_t {
    F_CONTROL_ACTIVE = 1 << 0,
    F_MISSION_ACTIVE = 1 << 1,
    F_MISSION_GO     = 1 << 2,
    F_EST_VALID      = 1 << 3,
    F_EST_GPS_DENIED = 1 << 4,
    F_CORRIDOR_VALID = 1 << 5,
    F_VEH_ARMED      = 1 << 6,
    F_VEH_LINK       = 1 << 7,
};

// The decoded form of one record — what the decoder reconstructs and the test
// asserts on. Mirrors the packed fields above.
struct Record {
    double   tMonoS = 0.0;
    uint32_t frameId = 0;
    uint8_t  behavior = 0;
    uint8_t  flags = 0;
    char     opMode[kOpModeLen + 1] = {};   // +1 for a decoder-added NUL
    char     phase[kPhaseLen + 1]   = {};
    float    ctlRoll = 0, ctlPitch = 0, ctlYaw = 0, ctlThrottle = 0;
    float    estPe = 0, estPn = 0, estPu = 0;
    float    estVe = 0, estVn = 0, estVu = 0;
    float    estEphM = 0;
    float    vehBattV = 0, vehAltM = 0, vehRollDeg = 0, vehPitchDeg = 0, vehYawDeg = 0;
    double   vehLat = 0, vehLon = 0;
    uint8_t  vehSats = 0, vehFix = 0;
    float    corridorOpen = 0;
};

uint32_t crc32(const uint8_t* data, int len);   // standard poly 0xEDB88320

// Pack a WorldState into a kRecordSize-byte record (incl. sync + CRC). Exposed
// so the decoder round-trip test can pack/unpack without touching the file I/O.
void packRecord(const WorldState& s, uint8_t out[kRecordSize]);

// Parse one record from `in` (must be kRecordSize bytes). Returns false if the
// sync word or CRC is wrong (a torn/garbage record).
bool unpackRecord(const uint8_t in[kRecordSize], Record& out);

}  // namespace bb

class BlackBox {
public:
    ~BlackBox();

    // Opens `path` for append. Writes the file header only if the file is new
    // (a pre-existing box is continued, not clobbered — the decoder handles a
    // multi-session file). Returns false (and stays disabled) on any error.
    bool open(const std::string& path, double fsyncIntervalS = 1.0);
    bool isOpen() const { return fp_ != nullptr; }

    // Append one record for this tick. Best-effort: on a write failure the box
    // disables itself and returns without throwing. fsync fires on the timer.
    void log(const WorldState& s);

    void close();

    uint32_t recordCount() const { return nRecords_; }

private:
    std::FILE* fp_            = nullptr;
    double     fsyncIntervalS_ = 1.0;
    double     lastSyncS_     = 0.0;
    uint32_t   nRecords_      = 0;
};

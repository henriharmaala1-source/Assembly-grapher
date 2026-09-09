#include "black_box.hpp"

#include <unistd.h>   // fsync, fileno

#include <cassert>
#include <cstring>

namespace {

// ---- little-endian pack helpers (offset-advancing) ------------------------
inline void putU8 (uint8_t* p, int& o, uint8_t v)  { p[o++] = v; }
inline void putU16(uint8_t* p, int& o, uint16_t v) { p[o++] = v & 0xFF; p[o++] = (v >> 8) & 0xFF; }
inline void putU32(uint8_t* p, int& o, uint32_t v) {
    p[o++] = v & 0xFF; p[o++] = (v >> 8) & 0xFF; p[o++] = (v >> 16) & 0xFF; p[o++] = (v >> 24) & 0xFF;
}
inline void putF32(uint8_t* p, int& o, float v)  { uint32_t u; std::memcpy(&u, &v, 4); putU32(p, o, u); }
inline void putF64(uint8_t* p, int& o, double v) {
    uint64_t u; std::memcpy(&u, &v, 8);
    for (int i = 0; i < 8; ++i) p[o++] = (u >> (8 * i)) & 0xFF;
}
inline void putStr(uint8_t* p, int& o, const std::string& s, int fixed) {
    for (int i = 0; i < fixed; ++i) p[o++] = (i < (int)s.size()) ? (uint8_t)s[i] : 0;
}

// ---- little-endian unpack helpers -----------------------------------------
inline uint8_t  getU8 (const uint8_t* p, int& o) { return p[o++]; }
inline uint16_t getU16(const uint8_t* p, int& o) { uint16_t v = p[o] | (p[o+1] << 8); o += 2; return v; }
inline uint32_t getU32(const uint8_t* p, int& o) {
    uint32_t v = (uint32_t)p[o] | ((uint32_t)p[o+1] << 8) | ((uint32_t)p[o+2] << 16) | ((uint32_t)p[o+3] << 24);
    o += 4; return v;
}
inline float  getF32(const uint8_t* p, int& o) { uint32_t u = getU32(p, o); float v; std::memcpy(&v, &u, 4); return v; }
inline double getF64(const uint8_t* p, int& o) {
    uint64_t u = 0; for (int i = 0; i < 8; ++i) u |= (uint64_t)p[o++] << (8 * i);
    double v; std::memcpy(&v, &u, 8); return v;
}

}  // namespace

namespace bb {

uint32_t crc32(const uint8_t* data, int len) {
    uint32_t c = 0xFFFFFFFFu;
    for (int i = 0; i < len; ++i) {
        c ^= data[i];
        for (int k = 0; k < 8; ++k)
            c = (c & 1) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
    }
    return c ^ 0xFFFFFFFFu;
}

void packRecord(const WorldState& s, uint8_t out[kRecordSize]) {
    uint8_t flags = 0;
    if (s.controlActive) flags |= F_CONTROL_ACTIVE;
    if (s.missionActive) flags |= F_MISSION_ACTIVE;
    if (s.missionGo)     flags |= F_MISSION_GO;
    if (s.estValid)      flags |= F_EST_VALID;
    if (s.estGpsDenied)  flags |= F_EST_GPS_DENIED;
    if (s.corridorValid) flags |= F_CORRIDOR_VALID;
    if (s.vehArmed)      flags |= F_VEH_ARMED;
    if (s.vehLink)       flags |= F_VEH_LINK;

    int o = 0;
    putU16(out, o, kRecordSync);
    putF64(out, o, s.tickMonoS);
    putU32(out, o, (uint32_t)s.frameId);
    putU8 (out, o, (uint8_t)s.behavior);
    putU8 (out, o, flags);
    putStr(out, o, s.opMode,       kOpModeLen);
    putStr(out, o, s.missionPhase, kPhaseLen);
    putF32(out, o, s.control.roll);
    putF32(out, o, s.control.pitch);
    putF32(out, o, s.control.yaw);
    putF32(out, o, s.control.throttle);
    putF32(out, o, s.estPe); putF32(out, o, s.estPn); putF32(out, o, s.estPu);
    putF32(out, o, s.estVe); putF32(out, o, s.estVn); putF32(out, o, s.estVu);
    putF32(out, o, s.estEphM);
    putF32(out, o, s.vehBattV);
    putF32(out, o, s.vehAltM);
    putF32(out, o, s.vehRollDeg);
    putF32(out, o, s.vehPitchDeg);
    putF32(out, o, s.vehYawDeg);
    putF64(out, o, s.vehLat);
    putF64(out, o, s.vehLon);
    putU8 (out, o, (uint8_t)s.vehSats);
    putU8 (out, o, (uint8_t)s.vehFix);
    putF32(out, o, s.corridorOpen);

    // CRC over everything written so far (sync..payload), then the CRC itself.
    const uint32_t c = crc32(out, o);
    putU32(out, o, c);
    assert(o == kRecordSize);   // packer and kRecordSize must agree exactly
}

bool unpackRecord(const uint8_t in[kRecordSize], Record& r) {
    int o = 0;
    if (getU16(in, o) != kRecordSync) return false;
    // CRC covers all but the trailing 4 bytes.
    const uint32_t want = crc32(in, kRecordSize - 4);
    int co = kRecordSize - 4;
    const uint32_t got = getU32(in, co);
    if (want != got) return false;

    r.tMonoS  = getF64(in, o);
    r.frameId = getU32(in, o);
    r.behavior = getU8(in, o);
    r.flags    = getU8(in, o);
    std::memcpy(r.opMode, in + o, kOpModeLen); r.opMode[kOpModeLen] = 0; o += kOpModeLen;
    std::memcpy(r.phase,  in + o, kPhaseLen);  r.phase[kPhaseLen]  = 0; o += kPhaseLen;
    r.ctlRoll = getF32(in, o); r.ctlPitch = getF32(in, o);
    r.ctlYaw  = getF32(in, o); r.ctlThrottle = getF32(in, o);
    r.estPe = getF32(in, o); r.estPn = getF32(in, o); r.estPu = getF32(in, o);
    r.estVe = getF32(in, o); r.estVn = getF32(in, o); r.estVu = getF32(in, o);
    r.estEphM = getF32(in, o);
    r.vehBattV = getF32(in, o); r.vehAltM = getF32(in, o);
    r.vehRollDeg = getF32(in, o); r.vehPitchDeg = getF32(in, o); r.vehYawDeg = getF32(in, o);
    r.vehLat = getF64(in, o); r.vehLon = getF64(in, o);
    r.vehSats = getU8(in, o); r.vehFix = getU8(in, o);
    r.corridorOpen = getF32(in, o);
    return true;
}

}  // namespace bb

// --------------------------------------------------------------------------

BlackBox::~BlackBox() { close(); }

bool BlackBox::open(const std::string& path, double fsyncIntervalS) {
    if (fp_) return false;
    fsyncIntervalS_ = fsyncIntervalS;

    // Is there an existing, non-empty file? If so, continue it (append); else
    // write a fresh header. "ab" creates-or-appends; we probe size separately.
    bool fresh = true;
    if (std::FILE* probe = std::fopen(path.c_str(), "rb")) {
        std::fseek(probe, 0, SEEK_END);
        if (std::ftell(probe) > 0) fresh = false;
        std::fclose(probe);
    }

    fp_ = std::fopen(path.c_str(), "ab");
    if (!fp_) return false;

    if (fresh) {
        uint8_t hdr[bb::kHeaderSize];
        int o = 0;
        for (char c : bb::kMagic) hdr[o++] = (uint8_t)c;
        hdr[o++] = bb::kVersion & 0xFF;  hdr[o++] = (bb::kVersion >> 8) & 0xFF;
        hdr[o++] = bb::kRecordSize & 0xFF; hdr[o++] = (bb::kRecordSize >> 8) & 0xFF;
        if (std::fwrite(hdr, 1, sizeof(hdr), fp_) != sizeof(hdr)) { close(); return false; }
    }
    std::fflush(fp_);
    ::fsync(::fileno(fp_));
    lastSyncS_ = monoNowS();
    return true;
}

void BlackBox::log(const WorldState& s) {
    if (!fp_) return;

    uint8_t rec[bb::kRecordSize];
    bb::packRecord(s, rec);
    if (std::fwrite(rec, 1, sizeof(rec), fp_) != sizeof(rec)) {
        // Storage gone (full/unplugged) — disable rather than spin on errors.
        close();
        return;
    }
    ++nRecords_;

    const double now = monoNowS();
    if (now - lastSyncS_ >= fsyncIntervalS_) {
        std::fflush(fp_);
        ::fsync(::fileno(fp_));
        lastSyncS_ = now;
    }
}

void BlackBox::close() {
    if (!fp_) return;
    std::fflush(fp_);
    ::fsync(::fileno(fp_));
    std::fclose(fp_);
    fp_ = nullptr;
}

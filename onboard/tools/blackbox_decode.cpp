// blackbox_decode — offline decoder for a BlackBox flight-data file.
//
//   blackbox_decode <file.kbb>            # CSV to stdout
//   blackbox_decode <file.kbb> --summary  # one-line summary only
//
// Reads the header, then walks fixed-size records. A record whose sync word or
// CRC is wrong (a torn tail after a crash/brownout, or a spliced file) is
// skipped and the decoder resynchronises by scanning forward for the next sync
// word, so one bad record never costs the rest of the log.
//
// This is intentionally a separate, non-flight tool: nothing here runs on the
// aircraft. It has no dependency on OpenCV or the runtime — just the black-box
// format constants.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "black_box.hpp"

static const char* behaviorName(uint8_t b) {
    static const char* names[] = {"MANUAL","IDLE","NAVIGATE","ROAD_FOLLOW",
                                  "TRACK","SEARCH","EVADE","HOLD","RTL"};
    return (b < sizeof(names)/sizeof(names[0])) ? names[b] : "?";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.kbb> [--summary]\n", argv[0]);
        return 2;
    }
    const std::string path = argv[1];
    const bool summaryOnly = (argc >= 3 && std::string(argv[2]) == "--summary");

    std::FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); return 1; }

    // ---- read whole file (black boxes are small — minutes of ~120-byte records)
    std::fseek(fp, 0, SEEK_END);
    const long sz = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz > 0 ? sz : 0);
    if (sz > 0 && std::fread(buf.data(), 1, sz, fp) != (size_t)sz) {
        std::fprintf(stderr, "short read\n"); std::fclose(fp); return 1;
    }
    std::fclose(fp);

    // ---- header
    if ((long)buf.size() < bb::kHeaderSize ||
        std::memcmp(buf.data(), bb::kMagic, 4) != 0) {
        std::fprintf(stderr, "not a black-box file (bad magic)\n");
        return 1;
    }
    const uint16_t version = buf[4] | (buf[5] << 8);
    const uint16_t recSize = buf[6] | (buf[7] << 8);
    if (recSize != bb::kRecordSize) {
        std::fprintf(stderr,
            "record size %u in file != decoder's %d (version %u) — "
            "decoder built against a different format\n",
            recSize, bb::kRecordSize, version);
        return 1;
    }

    // ---- walk records, resyncing past any torn/garbage record
    long pos = bb::kHeaderSize;
    uint32_t good = 0, bad = 0;
    double firstT = 0, lastT = 0;

    if (!summaryOnly) {
        std::printf("t_s,frame,behavior,opmode,phase,ctl_roll,ctl_pitch,ctl_yaw,"
                    "ctl_thr,ctl_active,mission_active,mission_go,est_valid,"
                    "gps_denied,corridor_valid,armed,link,est_pe,est_pn,est_pu,"
                    "est_ve,est_vn,est_vu,est_eph_m,batt_v,alt_m,roll_deg,"
                    "pitch_deg,yaw_deg,lat,lon,sats,fix,corridor_open\n");
    }

    while (pos + bb::kRecordSize <= (long)buf.size()) {
        bb::Record r;
        if (bb::unpackRecord(&buf[pos], r)) {
            if (good == 0) firstT = r.tMonoS;
            lastT = r.tMonoS;
            ++good;
            if (!summaryOnly) {
                auto bit = [&](uint8_t f){ return (r.flags & f) ? 1 : 0; };
                std::printf("%.3f,%u,%s,%s,%s,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,"
                            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,"
                            "%.7f,%.7f,%u,%u,%.3f\n",
                    r.tMonoS, r.frameId, behaviorName(r.behavior), r.opMode, r.phase,
                    r.ctlRoll, r.ctlPitch, r.ctlYaw, r.ctlThrottle,
                    bit(bb::F_CONTROL_ACTIVE), bit(bb::F_MISSION_ACTIVE), bit(bb::F_MISSION_GO),
                    bit(bb::F_EST_VALID), bit(bb::F_EST_GPS_DENIED), bit(bb::F_CORRIDOR_VALID),
                    bit(bb::F_VEH_ARMED), bit(bb::F_VEH_LINK),
                    r.estPe, r.estPn, r.estPu, r.estVe, r.estVn, r.estVu, r.estEphM,
                    r.vehBattV, r.vehAltM, r.vehRollDeg, r.vehPitchDeg, r.vehYawDeg,
                    r.vehLat, r.vehLon, r.vehSats, r.vehFix, r.corridorOpen);
            }
            pos += bb::kRecordSize;
        } else {
            // Torn/garbage record: resync to the next sync word after `pos`.
            ++bad;
            long next = pos + 1;
            const uint8_t lo = bb::kRecordSync & 0xFF, hi = (bb::kRecordSync >> 8) & 0xFF;
            while (next + 1 < (long)buf.size() && !(buf[next] == lo && buf[next+1] == hi))
                ++next;
            if (next + bb::kRecordSize > (long)buf.size()) break;
            pos = next;
        }
    }

    const double dur = (good > 1) ? (lastT - firstT) : 0.0;
    std::fprintf(stderr,
        "[blackbox] %u records, %u skipped (torn/garbage), %.1f s span, format v%u\n",
        good, bad, dur, version);
    return 0;
}

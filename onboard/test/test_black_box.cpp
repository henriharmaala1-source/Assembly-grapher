// test_black_box — the black box must survive the exact thing it exists for:
// a torn final record from a crash/brownout must not corrupt or hide the
// records written before it.
//
// Covers:
//   1. round-trip — pack a WorldState, unpack it, fields match.
//   2. file I/O — log N records, decode the file, get N records back in order.
//   3. crash survivability — truncate the file mid-final-record, confirm the
//      first N-1 records still decode and the torn one is rejected (not misread).
//   4. corruption — flip a byte inside a record, confirm its CRC rejects it.
//   5. resync — splice garbage between two records, confirm both still decode.
//
// Uses a CHECK macro (not assert) so the checks survive a Release/NDEBUG build,
// and keeps all file I/O OUT of the checks for the same reason.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "black_box.hpp"
#include "world_model.hpp"

#define CHECK(cond) do { if (!(cond)) { \
    std::fprintf(stderr, "CHECK failed: %s  (%s:%d)\n", #cond, __FILE__, __LINE__); \
    return 1; } } while (0)

static WorldState makeState(int i) {
    WorldState s;
    s.tickMonoS    = 100.0 + i * 0.05;
    s.frameId      = i;
    s.behavior     = Behavior::NAVIGATE;
    s.opMode       = "AUTONOMY";
    s.missionPhase = "MOVE";
    s.control.roll     = 0.10f * i;
    s.control.pitch    = -0.20f;
    s.control.yaw      = 0.05f;
    s.control.throttle = 0.5f;
    s.controlActive = (i % 2 == 0);
    s.missionActive = true;
    s.missionGo     = true;
    s.estValid      = true;
    s.estPe = 1.5f * i; s.estPn = -2.0f * i; s.estPu = 3.0f;
    s.estVe = 0.4f; s.estVn = 0.6f; s.estVu = 0.f;
    s.estEphM = 2.25f;
    s.vehBattV = 22.2f; s.vehAltM = 30.f;
    s.vehRollDeg = 5.f; s.vehPitchDeg = -3.f; s.vehYawDeg = 90.f + i;
    s.vehLat = 60.1699 + i * 1e-6; s.vehLon = 24.9384 - i * 1e-6;
    s.vehSats = 12; s.vehFix = 3;
    s.vehArmed = true; s.vehLink = true;
    s.corridorValid = true; s.corridorOpen = 0.8f;
    return s;
}

static bool eq(const WorldState& s, const bb::Record& r) {
    if (r.frameId != (uint32_t)s.frameId) return false;
    if (std::string(r.opMode) != s.opMode) return false;
    if (std::string(r.phase)  != s.missionPhase) return false;
    if (std::fabs(r.tMonoS - s.tickMonoS) > 1e-9) return false;
    if (std::fabs(r.ctlRoll - s.control.roll) > 1e-6) return false;
    if (std::fabs(r.estPe - s.estPe) > 1e-3) return false;
    if (std::fabs(r.estEphM - s.estEphM) > 1e-6) return false;
    if (std::fabs(r.vehLat - s.vehLat) > 1e-9) return false;
    if (r.vehSats != s.vehSats || r.vehFix != s.vehFix) return false;
    return ((r.flags & bb::F_CONTROL_ACTIVE) != 0) == s.controlActive;
}

// Read a whole file into memory. Returns false on any I/O error (checked by
// the caller, not swallowed in an assert).
static bool slurp(const std::string& path, std::vector<uint8_t>& out) {
    std::FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) return false;
    std::fseek(fp, 0, SEEK_END);
    long sz = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    out.resize(sz > 0 ? sz : 0);
    bool ok = (sz <= 0) || (std::fread(out.data(), 1, sz, fp) == (size_t)sz);
    std::fclose(fp);
    return ok;
}

static bool spew(const std::string& path, const uint8_t* data, long len) {
    std::FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) return false;
    bool ok = (std::fwrite(data, 1, len, fp) == (size_t)len);
    std::fclose(fp);
    return ok;
}

// Decode a byte buffer into records, resyncing past torn/garbage ones — a
// self-contained mirror of blackbox_decode's walk (no exec needed).
static std::vector<bb::Record> decode(const std::vector<uint8_t>& buf, int& skipped) {
    std::vector<bb::Record> out; skipped = 0;
    if ((long)buf.size() < bb::kHeaderSize) return out;
    if (std::memcmp(buf.data(), bb::kMagic, 4) != 0) return out;
    long pos = bb::kHeaderSize;
    while (pos + bb::kRecordSize <= (long)buf.size()) {
        bb::Record r;
        if (bb::unpackRecord(&buf[pos], r)) { out.push_back(r); pos += bb::kRecordSize; }
        else {
            ++skipped;
            long next = pos + 1;
            const uint8_t lo = bb::kRecordSync & 0xFF, hi = (bb::kRecordSync >> 8) & 0xFF;
            while (next + 1 < (long)buf.size() && !(buf[next] == lo && buf[next+1] == hi)) ++next;
            if (next + bb::kRecordSize > (long)buf.size()) break;
            pos = next;
        }
    }
    return out;
}

int main() {
    const std::string path = "test_blackbox.tmp";
    const int N = 20;

    // ---- 1. round-trip (no file) -----------------------------------------
    {
        uint8_t rec[bb::kRecordSize];
        WorldState s = makeState(7);
        bb::packRecord(s, rec);
        bb::Record r;
        CHECK(bb::unpackRecord(rec, r));
        CHECK(eq(s, r));
    }

    // ---- 2. file I/O: log N, decode N ------------------------------------
    std::remove(path.c_str());
    {
        BlackBox box;
        CHECK(box.open(path));
        for (int i = 0; i < N; ++i) box.log(makeState(i));
        CHECK(box.recordCount() == (uint32_t)N);
        box.close();

        std::vector<uint8_t> buf;
        CHECK(slurp(path, buf));
        int skipped = 0;
        auto recs = decode(buf, skipped);
        CHECK((int)recs.size() == N);
        CHECK(skipped == 0);
        for (int i = 0; i < N; ++i) CHECK(eq(makeState(i), recs[i]));
    }

    // ---- 3. crash survivability: truncate mid-final-record ----------------
    {
        std::vector<uint8_t> buf;
        CHECK(slurp(path, buf));
        CHECK((long)buf.size() > 5);
        CHECK(spew(path, buf.data(), (long)buf.size() - 5));   // torn tail

        std::vector<uint8_t> buf2;
        CHECK(slurp(path, buf2));
        int skipped = 0;
        auto recs = decode(buf2, skipped);
        // The final record is now short, so the walk stops before it:
        // N-1 clean records, none misread as valid.
        CHECK((int)recs.size() == N - 1);
        for (int i = 0; i < N - 1; ++i) CHECK(eq(makeState(i), recs[i]));
    }

    // ---- 4. corruption: flip a byte inside a record, CRC must reject -------
    {
        std::remove(path.c_str());
        BlackBox box;
        CHECK(box.open(path));
        for (int i = 0; i < 3; ++i) box.log(makeState(i));
        box.close();

        std::vector<uint8_t> buf;
        CHECK(slurp(path, buf));
        const long mid = bb::kHeaderSize + bb::kRecordSize + 10;   // payload of record 1
        CHECK(mid < (long)buf.size());
        buf[mid] ^= 0xFF;

        int skipped = 0;
        auto recs = decode(buf, skipped);
        // Records 0 and 2 decode; corrupted record 1 is skipped.
        CHECK((int)recs.size() == 2);
        CHECK(skipped >= 1);
        CHECK(recs[0].frameId == 0);
        CHECK(recs[1].frameId == 2);
    }

    // ---- 5. resync: garbage spliced between two good records --------------
    {
        std::vector<uint8_t> buf;
        uint8_t hdr[bb::kHeaderSize] = {'K','B','B','1',
            bb::kVersion & 0xFF, (bb::kVersion >> 8) & 0xFF,
            bb::kRecordSize & 0xFF, (bb::kRecordSize >> 8) & 0xFF};
        buf.insert(buf.end(), hdr, hdr + sizeof(hdr));
        uint8_t r0[bb::kRecordSize], r1[bb::kRecordSize];
        bb::packRecord(makeState(0), r0);
        bb::packRecord(makeState(1), r1);
        buf.insert(buf.end(), r0, r0 + sizeof(r0));
        const uint8_t junk[7] = {1,2,3,4,5,6,7};   // spliced garbage, no sync word
        buf.insert(buf.end(), junk, junk + sizeof(junk));
        buf.insert(buf.end(), r1, r1 + sizeof(r1));

        int skipped = 0;
        auto recs = decode(buf, skipped);
        CHECK((int)recs.size() == 2);
        CHECK(recs[0].frameId == 0);
        CHECK(recs[1].frameId == 1);
    }

    std::remove(path.c_str());
    std::printf("test_black_box: OK\n");
    return 0;
}

#include "depth_record.hpp"

#include <cstdio>
#include <cstring>

namespace sim {

namespace {

constexpr char kMagic[8] = {'K', 'D', 'E', 'P', 'T', 'H', '0', '1'};

// Explicit little-endian pack/unpack rather than fwrite of a struct. A struct
// write bakes in this compiler's padding and this CPU's byte order, and the
// file is meant to move between a Windows laptop, a Pi and whatever reads it in
// five years. Sixty-four bytes is not worth being clever about.
void put32(uint8_t* p, uint32_t v) {
    p[0] = uint8_t(v); p[1] = uint8_t(v >> 8);
    p[2] = uint8_t(v >> 16); p[3] = uint8_t(v >> 24);
}
uint32_t get32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8)
         | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
void putf(uint8_t* p, float v) { uint32_t t; std::memcpy(&t, &v, 4); put32(p, t); }
float getf(const uint8_t* p) { uint32_t t = get32(p); float v; std::memcpy(&v, &t, 4); return v; }

void packHeader(const DepthRecordHeader& h, uint8_t* b) {
    std::memset(b, 0, DepthRecordHeader::HEADER_BYTES);
    std::memcpy(b, kMagic, 8);
    put32(b + 8,  h.width);
    put32(b + 12, h.height);
    put32(b + 16, h.frames);
    putf (b + 20, h.depthScale);
    putf (b + 24, h.fx);
    putf (b + 28, h.fy);
    putf (b + 32, h.ppx);
    putf (b + 36, h.ppy);
    putf (b + 40, h.baselineM);
    put32(b + 44, h.flags);
    // 45..63 reserved, already zeroed.
}

bool unpackHeader(const uint8_t* b, DepthRecordHeader& h) {
    if (std::memcmp(b, kMagic, 8) != 0) return false;
    h.width      = get32(b + 8);
    h.height     = get32(b + 12);
    h.frames     = get32(b + 16);
    h.depthScale = getf (b + 20);
    h.fx         = getf (b + 24);
    h.fy         = getf (b + 28);
    h.ppx        = getf (b + 32);
    h.ppy        = getf (b + 36);
    h.baselineM  = getf (b + 40);
    h.flags      = get32(b + 44);
    return true;
}

}  // namespace

bool DepthRecordReader::open(const std::string& path, std::string* err) {
    close();
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) { if (err) *err = "cannot open " + path; return false; }
    uint8_t hdr[DepthRecordHeader::HEADER_BYTES];
    if (std::fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
        std::fclose(f);
        if (err) *err = "short file -- not a .kdr recording";
        return false;
    }
    if (!unpackHeader(hdr, h_)) {
        std::fclose(f);
        if (err) *err = "bad magic -- not a .kdr recording";
        return false;
    }
    // A truncated recording is the normal outcome of a cable being pulled, so
    // trust the FILE LENGTH over the header's frame count rather than reading
    // past the end. The writer patches the count on close; a killed writer
    // never gets to.
    std::fseek(f, 0, SEEK_END);
    const long bytes = std::ftell(f);
    const long perFrame = long(h_.width) * h_.height * 2;
    if (h_.width == 0 || h_.height == 0 || perFrame <= 0) {
        std::fclose(f);
        if (err) *err = "degenerate frame size in header";
        return false;
    }
    const uint32_t actual = uint32_t((bytes - DepthRecordHeader::HEADER_BYTES) / perFrame);
    if (actual < h_.frames) h_.frames = actual;   // truncated: believe the bytes
    if (h_.frames == 0) {
        std::fclose(f);
        if (err) *err = "recording contains no complete frames";
        return false;
    }
    f_ = f;
    idx_ = 0;
    raw_.assign(size_t(h_.width) * h_.height, 0);
    return true;
}

void DepthRecordReader::close() {
    if (f_) { std::fclose(static_cast<FILE*>(f_)); f_ = nullptr; }
}

bool DepthRecordReader::readFrame(int i, std::vector<float>& out) {
    if (!f_ || i < 0 || i >= int(h_.frames)) return false;
    FILE* f = static_cast<FILE*>(f_);
    const long perFrame = long(h_.width) * h_.height * 2;
    if (std::fseek(f, DepthRecordHeader::HEADER_BYTES + long(i) * perFrame, SEEK_SET) != 0)
        return false;
    if (std::fread(raw_.data(), 2, raw_.size(), f) != raw_.size()) return false;
    out.resize(raw_.size());
    const float s = h_.depthScale;
    for (size_t k = 0; k < raw_.size(); ++k)
        out[k] = raw_[k] ? float(raw_[k]) * s : -1.f;   // 0 stored = invalid
    idx_ = i;
    return true;
}

int DepthRecordReader::readNext(std::vector<float>& out) {
    const int i = idx_;
    if (!readFrame(i, out)) return -1;
    idx_ = (i + 1) % int(h_.frames);
    return i;
}

bool DepthRecordWriter::open(const std::string& path, const DepthRecordHeader& h,
                             std::string* err) {
    close();
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) { if (err) *err = "cannot create " + path; return false; }
    h_ = h;
    h_.frames = 0;
    uint8_t hdr[DepthRecordHeader::HEADER_BYTES];
    packHeader(h_, hdr);
    if (std::fwrite(hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
        std::fclose(f);
        if (err) *err = "cannot write header";
        return false;
    }
    f_ = f;
    written_ = 0;
    return true;
}

bool DepthRecordWriter::writeFrame(const uint16_t* px) {
    if (!f_) return false;
    const size_t n = size_t(h_.width) * h_.height;
    if (std::fwrite(px, 2, n, static_cast<FILE*>(f_)) != n) return false;
    ++written_;
    return true;
}

bool DepthRecordWriter::close() {
    if (!f_) return true;
    FILE* f = static_cast<FILE*>(f_);
    // Patch the frame count in place. Written last on purpose: a recording
    // killed mid-capture then has frames=0 in the header, and the reader falls
    // back to the file length -- so an interrupted capture stays readable
    // instead of becoming a corrupt file.
    uint8_t b[4];
    put32(b, written_);
    bool ok = (std::fseek(f, 16, SEEK_SET) == 0) && (std::fwrite(b, 1, 4, f) == 4);
    std::fclose(f);
    f_ = nullptr;
    return ok;
}

}  // namespace sim

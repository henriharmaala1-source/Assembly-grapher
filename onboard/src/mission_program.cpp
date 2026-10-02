#include "mission_program.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace kms {

const char* opName(Op o) {
    static const char* N[] = {"END", "HOLD", "GOTO", "TURN_TO", "TURN_BY", "CLIMB", "SEARCH",
                              "FACE", "APPROACH", "EXPLORE", "RUN", "WAIT", "JUMP",
                              "JUMP_IFNOT", "SET_REG", "LOOP", "TIMER", "JUMP_TIMEUP", "MARK", "MARK_SEEN",
                              "SAY", "LAND", "RTL", "RESUME", "GO_STATE", "TRACK", "UNTRACK",
                              "TURN_REF", "CLIMB_BY", "APPROACH_TO", "NAV"};
    static_assert(sizeof(N) / sizeof(N[0]) == size_t(Op::COUNT_), "opName table");
    const unsigned i = unsigned(o);
    return i < unsigned(Op::COUNT_) ? N[i] : "?";
}

uint32_t crc32(const uint8_t* data, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) {
        c ^= data[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

// ------------------------------------------------------------ byte writer
// Little-endian, field by field: never a struct memcpy, so the file means the
// same thing to the x86 that wrote it and the ARM that reads it.
namespace {
struct W {
    std::vector<uint8_t> b;
    void u8(uint8_t v) { b.push_back(v); }
    void u32(uint32_t v) { for (int i = 0; i < 4; ++i) b.push_back(uint8_t(v >> (8 * i))); }
    void i32(int32_t v) { u32(uint32_t(v)); }
    void f32(float v) { uint32_t u; std::memcpy(&u, &v, 4); u32(u); }
    void f64(double v) {
        uint64_t u; std::memcpy(&u, &v, 8);
        for (int i = 0; i < 8; ++i) b.push_back(uint8_t(u >> (8 * i)));
    }
    void str(const std::string& s) {
        const uint32_t n = uint32_t(std::min<size_t>(s.size(), 4096));
        u32(n);
        b.insert(b.end(), s.begin(), s.begin() + n);
    }
};
struct R {
    const std::vector<uint8_t>& b;
    size_t i = 0;
    bool bad = false;
    explicit R(const std::vector<uint8_t>& v) : b(v) {}
    bool need(size_t n) { if (i + n > b.size()) bad = true; return !bad; }
    uint8_t u8() { if (!need(1)) return 0; return b[i++]; }
    uint32_t u32() {
        if (!need(4)) return 0;
        uint32_t v = 0;
        for (int k = 0; k < 4; ++k) v |= uint32_t(b[i++]) << (8 * k);
        return v;
    }
    int32_t i32() { return int32_t(u32()); }
    float f32() { const uint32_t u = u32(); float v; std::memcpy(&v, &u, 4); return v; }
    double f64() {
        if (!need(8)) return 0;
        uint64_t u = 0;
        for (int k = 0; k < 8; ++k) u |= uint64_t(b[i++]) << (8 * k);
        double v; std::memcpy(&v, &u, 8); return v;
    }
    std::string str() {
        const uint32_t n = u32();
        if (n > 4096 || !need(n)) { bad = true; return {}; }
        std::string s(b.begin() + long(i), b.begin() + long(i + n));
        i += n;
        return s;
    }
    // A count that is about to size a vector: bounded by what is left, so a
    // corrupt count cannot ask for gigabytes.
    uint32_t count(size_t minBytesEach) {
        const uint32_t n = u32();
        if (bad || size_t(n) * minBytesEach > b.size() - i) { bad = true; return 0; }
        return n;
    }
};
}  // namespace

std::vector<uint8_t> serialize(const Program& p) {
    W w;
    w.u32(kMagic); w.u32(kVersion);
    w.u32(0);                                  // checksum, filled below
    w.str(p.name); w.str(p.sourceName); w.u32(p.sourceHash);
    w.u32(p.caps); w.f32(p.fenceM); w.f32(p.timeoutS); w.i32(p.registers);
    w.u32(uint32_t(p.strings.size()));
    for (const auto& s : p.strings) w.str(s);
    w.u32(uint32_t(p.targets.size()));
    for (const auto& t : p.targets) { w.u8(t.kind); w.f64(t.x); w.f64(t.y); w.i32(t.base); w.i32(t.name); }
    w.u32(uint32_t(p.condOps.size()));
    for (const auto& c : p.condOps) { w.u8(c.kind); w.u8(c.var); w.u8(c.cmp); w.i32(c.arg); w.f32(c.value); }
    w.u32(uint32_t(p.conds.size()));
    for (const auto& c : p.conds) { w.i32(c.start); w.i32(c.count); }
    w.u32(uint32_t(p.handlers.size()));
    for (const auto& h : p.handlers) { w.i32(h.cond); w.i32(h.pc); w.i32(h.line); }
    w.u32(uint32_t(p.states.size()));
    for (const auto& st : p.states) { w.i32(st.name); w.i32(st.pc); w.i32(st.transStart); w.i32(st.transCount); }
    w.u32(uint32_t(p.transitions.size()));
    for (const auto& t : p.transitions) { w.i32(t.cond); w.i32(t.to); w.i32(t.line); }
    w.u32(uint32_t(p.sizes.size()));
    for (const auto& z : p.sizes) { w.i32(z.label); w.f32(z.heightM); w.u8(z.assumed); }
    w.u32(uint32_t(p.code.size()));
    for (const auto& in : p.code) {
        w.u8(uint8_t(in.op)); w.i32(in.line); w.i32(in.target); w.i32(in.jump);
        w.i32(in.cond); w.i32(in.text); w.f32(in.a); w.f32(in.b); w.u8(in.flags);
    }
    const uint32_t c = crc32(w.b.data() + 12, w.b.size() - 12);
    for (int i = 0; i < 4; ++i) w.b[8 + size_t(i)] = uint8_t(c >> (8 * i));
    return w.b;
}

bool verify(const Program& p, std::string* err) {
    auto fail = [&](const std::string& m) { if (err) *err = m; return false; };
    const int nS = int(p.strings.size()), nT = int(p.targets.size()),
              nC = int(p.conds.size()), nI = int(p.code.size()),
              nO = int(p.condOps.size());
    if (nI == 0) return fail("no instructions");
    if (p.registers < 0 || p.registers > 256) return fail("register count out of range");
    if (!(p.fenceM > 0.f) || !(p.timeoutS > 0.f)) return fail("fence/timeout not positive");
    for (int i = 0; i < nT; ++i) {
        const Target& t = p.targets[size_t(i)];
        if (t.kind > Target::RUNTIME) return fail("target " + std::to_string(i) + ": bad kind");
        if (t.name >= nS) return fail("target " + std::to_string(i) + ": bad name");
        // OFFSET bases point BACKWARDS only, so resolving one cannot loop.
        if ((t.kind == Target::OFFSET || t.kind == Target::REL) && (t.base < 0 || t.base >= i))
            return fail("target " + std::to_string(i) + ": bad base");
    }
    for (int k = 0; k < nC; ++k) {
        const CondRange& r = p.conds[size_t(k)];
        if (r.start < 0 || r.count <= 0 || r.start + r.count > nO)
            return fail("condition " + std::to_string(k) + ": bad range");
        int depth = 0;
        for (int j = r.start; j < r.start + r.count; ++j) {
            const CondOp& c = p.condOps[size_t(j)];
            switch (c.kind) {
                case CondOp::TRUE_: case CondOp::POSITIONED: ++depth; break;
                case CondOp::SEEN: case CondOp::TRACKING:
                    if (c.arg < 0 || c.arg >= nS) return fail("condition: bad label");
                    ++depth; break;
                case CondOp::RANGE:
                    if (c.arg < 0 || c.arg >= nS || c.cmp > CondOp::GE) return fail("condition: bad range");
                    ++depth; break;
                case CondOp::VAR:
                    if (c.var > CondOp::HEADING || c.cmp > CondOp::GE) return fail("condition: bad var");
                    ++depth; break;
                case CondOp::DIST:
                    if (c.arg < 0 || c.arg >= nT || c.cmp > CondOp::GE) return fail("condition: bad target");
                    ++depth; break;
                case CondOp::NOT: if (depth < 1) return fail("condition: stack underflow"); break;
                case CondOp::AND: case CondOp::OR:
                    if (depth < 2) return fail("condition: stack underflow");
                    --depth; break;
                default: return fail("condition: bad op");
            }
        }
        if (depth != 1) return fail("condition " + std::to_string(k) + ": does not reduce to one value");
    }
    for (const ObjectSize& z : p.sizes)
        if (z.label < 0 || z.label >= nS || !(z.heightM > 0.f)) return fail("object size out of range");
    const int nSt = int(p.states.size()), nTr = int(p.transitions.size());
    for (const State& st : p.states)
        if (st.name < 0 || st.name >= nS || st.pc < 0 || st.pc >= nI || st.transStart < 0 ||
            st.transCount < 0 || st.transStart + st.transCount > nTr)
            return fail("state out of range");
    for (const Transition& t : p.transitions)
        if (t.cond < 0 || t.cond >= nC || t.to < 0 || t.to >= nSt) return fail("transition out of range");
    for (const Handler& h : p.handlers)
        if (h.cond < 0 || h.cond >= nC || h.pc < 0 || h.pc >= nI) return fail("handler out of range");
    for (int i = 0; i < nI; ++i) {
        const Instr& in = p.code[size_t(i)];
        const std::string at = "instruction " + std::to_string(i) + ": ";
        if (unsigned(in.op) >= unsigned(Op::COUNT_)) return fail(at + "bad opcode");
        if (in.jump < -1 || in.jump >= nI) return fail(at + "jump out of range");
        if (in.cond < -1 || in.cond >= nC) return fail(at + "condition out of range");
        if (in.text < -1 || in.text >= nS) return fail(at + "text out of range");
        switch (in.op) {
            case Op::GO_STATE:
                if (in.target < 0 || in.target >= int(p.states.size())) return fail(at + "state out of range");
                break;
            case Op::GOTO: case Op::MARK: case Op::MARK_SEEN: case Op::TURN_REF:
                if (in.target < 0 || in.target >= nT) return fail(at + "target out of range");
                break;
            case Op::SET_REG: case Op::LOOP: case Op::TIMER: case Op::JUMP_TIMEUP:
                if (in.target < 0 || in.target >= p.registers) return fail(at + "register out of range");
                break;
            default: break;
        }
        if ((in.op == Op::JUMP || in.op == Op::LOOP || in.op == Op::JUMP_TIMEUP) && in.jump < 0)
            return fail(at + "jump missing");
        if (in.op == Op::JUMP_IFNOT && (in.cond < 0 || in.jump < 0)) return fail(at + "branch incomplete");
        if (in.op == Op::WAIT && in.cond < 0) return fail(at + "wait without condition");
        if ((in.op == Op::SEARCH || in.op == Op::FACE || in.op == Op::APPROACH ||
             in.op == Op::SAY || in.op == Op::RUN || in.op == Op::MARK_SEEN ||
             in.op == Op::TRACK || in.op == Op::APPROACH_TO) && in.text < 0)
            return fail(at + "missing text");
        // Every op that takes time has a bound: nothing waits for ever.
        const bool timed = in.op == Op::GOTO || in.op == Op::TURN_TO || in.op == Op::TURN_BY ||
                           in.op == Op::CLIMB || in.op == Op::APPROACH || in.op == Op::TURN_REF ||
                           in.op == Op::CLIMB_BY || in.op == Op::APPROACH_TO;
        if (timed && !(in.b > 0.f)) return fail(at + "no timeout");
        if ((in.op == Op::HOLD || in.op == Op::EXPLORE || in.op == Op::RUN ||
             in.op == Op::SEARCH || in.op == Op::FACE || in.op == Op::WAIT ||
             in.op == Op::TRACK) && !(in.a > 0.f))
            return fail(at + "no duration");
    }
    return true;
}

bool deserialize(const std::vector<uint8_t>& bytes, Program& out, std::string* err) {
    auto fail = [&](const std::string& m) { if (err) *err = m; return false; };
    if (bytes.size() < 12) return fail("too short to be a .kmb");
    R r(bytes);
    if (r.u32() != kMagic) return fail("not a compiled mission (bad magic)");
    const uint32_t ver = r.u32();
    if (ver != kVersion)
        return fail("compiled for format v" + std::to_string(ver) + ", this build reads v" +
                    std::to_string(kVersion) + " -- recompile it");
    const uint32_t want = r.u32();
    if (crc32(bytes.data() + 12, bytes.size() - 12) != want)
        return fail("checksum mismatch: the file is damaged or truncated");
    Program p;
    p.name = r.str(); p.sourceName = r.str(); p.sourceHash = r.u32();
    p.caps = r.u32(); p.fenceM = r.f32(); p.timeoutS = r.f32(); p.registers = r.i32();
    for (uint32_t n = r.count(4), i = 0; i < n; ++i) p.strings.push_back(r.str());
    for (uint32_t n = r.count(25), i = 0; i < n && !r.bad; ++i) {
        Target t; t.kind = Target::Kind(r.u8()); t.x = r.f64(); t.y = r.f64();
        t.base = r.i32(); t.name = r.i32(); p.targets.push_back(t);
    }
    for (uint32_t n = r.count(11), i = 0; i < n && !r.bad; ++i) {
        CondOp c; c.kind = CondOp::Kind(r.u8()); c.var = r.u8(); c.cmp = r.u8();
        c.arg = r.i32(); c.value = r.f32(); p.condOps.push_back(c);
    }
    for (uint32_t n = r.count(8), i = 0; i < n && !r.bad; ++i) {
        CondRange c; c.start = r.i32(); c.count = r.i32(); p.conds.push_back(c);
    }
    for (uint32_t n = r.count(12), i = 0; i < n && !r.bad; ++i) {
        Handler h; h.cond = r.i32(); h.pc = r.i32(); h.line = r.i32(); p.handlers.push_back(h);
    }
    for (uint32_t n = r.count(16), i = 0; i < n && !r.bad; ++i) {
        State st; st.name = r.i32(); st.pc = r.i32(); st.transStart = r.i32(); st.transCount = r.i32();
        p.states.push_back(st);
    }
    for (uint32_t n = r.count(12), i = 0; i < n && !r.bad; ++i) {
        Transition t; t.cond = r.i32(); t.to = r.i32(); t.line = r.i32(); p.transitions.push_back(t);
    }
    for (uint32_t n = r.count(9), i = 0; i < n && !r.bad; ++i) {
        ObjectSize z; z.label = r.i32(); z.heightM = r.f32(); z.assumed = r.u8(); p.sizes.push_back(z);
    }
    for (uint32_t n = r.count(30), i = 0; i < n && !r.bad; ++i) {
        Instr in;
        in.op = Op(r.u8()); in.line = r.i32(); in.target = r.i32(); in.jump = r.i32();
        in.cond = r.i32(); in.text = r.i32(); in.a = r.f32(); in.b = r.f32(); in.flags = r.u8();
        p.code.push_back(in);
    }
    if (r.bad) return fail("truncated");
    if (r.i != bytes.size()) return fail("trailing bytes after the program");
    if (!verify(p, err)) return false;
    out = std::move(p);
    return true;
}

bool saveProgram(const Program& p, const std::string& path, std::string* err) {
    const std::vector<uint8_t> b = serialize(p);
    std::ofstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "cannot write " + path; return false; }
    f.write(reinterpret_cast<const char*>(b.data()), std::streamsize(b.size()));
    if (!f) { if (err) *err = "write failed: " + path; return false; }
    return true;
}

bool loadProgram(const std::string& path, Program& out, std::string* err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "cannot open " + path; return false; }
    std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (b.size() > (8u << 20)) { if (err) *err = "too large to be a mission"; return false; }
    return deserialize(b, out, err);
}

namespace {
std::string cv_fmt(const char* f, ...) {
    char b[160];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    return b;
}
}  // namespace

std::string disassemble(const Program& p) {
    std::ostringstream o;
    char buf[256];
    auto S = [&](int i) { return i >= 0 && i < int(p.strings.size()) ? p.strings[size_t(i)] : std::string("?"); };
    auto T = [&](int i) -> std::string {
        if (i < 0 || i >= int(p.targets.size())) return "?";
        const Target& t = p.targets[size_t(i)];
        if (t.name >= 0) return S(t.name);
        return "#" + std::to_string(i);
    };
    std::snprintf(buf, sizeof buf, "mission \"%s\"  fence %.0f m  timeout %.0f s  needs:%s%s%s\n",
                  p.name.c_str(), p.fenceM, p.timeoutS,
                  (p.caps & Program::NEEDS_POSITION) ? " position" : "",
                  (p.caps & Program::NEEDS_DETECTOR) ? " detector" : "",
                  (p.caps & Program::NEEDS_GPS) ? " gps" : "");
    o << buf;
    static const char* KIND[] = {"start", "enu", "gps", "offset", "rel", "runtime"};
    for (size_t i = 0; i < p.targets.size(); ++i) {
        const Target& t = p.targets[i];
        if (t.kind == Target::OFFSET || t.kind == Target::REL)
            std::snprintf(buf, sizeof buf, "  target %-10s %s %s %+.2f %s %+.2f %s\n",
                          T(int(i)).c_str(), KIND[t.kind], T(t.base).c_str(), t.x,
                          t.kind == Target::REL ? "ahead" : "E", t.y,
                          t.kind == Target::REL ? "right" : "N");
        else
            std::snprintf(buf, sizeof buf, "  target %-10s %s %.6g %.6g\n", T(int(i)).c_str(),
                          KIND[t.kind < 6 ? t.kind : 0], t.x, t.y);
        o << buf;
    }
    for (const ObjectSize& z : p.sizes) {
        std::snprintf(buf, sizeof buf, "  size %-10s %.2f m tall%s\n", S(z.label).c_str(), z.heightM,
                      z.assumed ? " (assumed: typical, not declared)" : "");
        o << buf;
    }
    for (const Handler& h : p.handlers) {
        std::snprintf(buf, sizeof buf, "  on cond#%d -> %d   (line %d)\n", h.cond, h.pc, h.line);
        o << buf;
    }
    for (const State& st : p.states) {
        std::snprintf(buf, sizeof buf, "  state %-12s -> %d\n", S(st.name).c_str(), st.pc);
        o << buf;
        for (int k = st.transStart; k < st.transStart + st.transCount; ++k) {
            const Transition& t = p.transitions[size_t(k)];
            std::snprintf(buf, sizeof buf, "    when cond#%d -> state %s   (line %d)\n", t.cond,
                          S(p.states[size_t(t.to)].name).c_str(), t.line);
            o << buf;
        }
    }
    for (size_t i = 0; i < p.code.size(); ++i) {
        const Instr& in = p.code[i];
        std::string arg;
        switch (in.op) {
            case Op::GOTO: arg = T(in.target) + cv_fmt(" r=%.2f t=%.0f", in.a, in.b); break;
            case Op::MARK: case Op::TURN_REF: arg = T(in.target); break;
            case Op::GO_STATE:
                arg = in.target >= 0 && in.target < int(p.states.size())
                          ? S(p.states[size_t(in.target)].name) : "?";
                break;
            case Op::TRACK: arg = "\"" + S(in.text) + "\"" + cv_fmt(" %.0f s", in.a); break;
            case Op::MARK_SEEN: arg = T(in.target) + " = seen \"" + S(in.text) + "\""; break;
            case Op::SEARCH: case Op::FACE: case Op::APPROACH: case Op::RUN: case Op::SAY:
            case Op::APPROACH_TO:
                arg = "\"" + S(in.text) + "\"" + cv_fmt(" %.2f %.2f", in.a, in.b); break;
            case Op::SET_REG: case Op::LOOP: case Op::TIMER: case Op::JUMP_TIMEUP:
                arg = cv_fmt("r%d %.2f", in.target, in.a); break;
            case Op::WAIT: case Op::JUMP_IFNOT:
                arg = cv_fmt("cond#%d %.2f", in.cond, in.a); break;
            default: arg = cv_fmt("%.2f %.2f", in.a, in.b); break;
        }
        std::snprintf(buf, sizeof buf, "  %3zu  line %-4d %-11s %s%s%s\n", i, in.line, opName(in.op),
                      arg.c_str(), (in.flags & FLAG_NEW) ? " (new only)" : "",
                      in.jump >= 0 ? (" -> " + std::to_string(in.jump)).c_str() : "");
        o << buf;
    }
    return o.str();
}

}  // namespace kms

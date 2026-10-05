#include "mission_compile.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

namespace kms {

namespace {

constexpr double kPi = 3.14159265358979;

// ------------------------------------------------------------------ lexer
enum class Tk { IDENT, NUMBER, STRING, SYM, NEWLINE, END };
struct Token {
    Tk kind = Tk::END;
    std::string text;      // IDENT/STRING/SYM
    double num = 0;
    int line = 1, col = 1;
};

std::vector<Token> lex(const std::string& src, std::vector<Diagnostic>& diags) {
    std::vector<Token> out;
    int line = 1, col = 1;
    size_t i = 0;
    auto push = [&](Tk k, std::string t, double n, int l, int c) {
        Token tk; tk.kind = k; tk.text = std::move(t); tk.num = n; tk.line = l; tk.col = c;
        out.push_back(tk);
    };
    while (i < src.size()) {
        const char ch = src[i];
        if (ch == '#') { while (i < src.size() && src[i] != '\n') ++i; continue; }
        if (ch == '\n') { push(Tk::NEWLINE, "", 0, line, col); ++i; ++line; col = 1; continue; }
        if (ch == ' ' || ch == '\t' || ch == '\r') { ++i; ++col; continue; }
        const int l = line, c = col;
        if (std::isdigit((unsigned char)ch) || (ch == '.' && i + 1 < src.size() &&
                                                std::isdigit((unsigned char)src[i + 1]))) {
            size_t j = i;
            while (j < src.size() && (std::isdigit((unsigned char)src[j]) || src[j] == '.')) ++j;
            push(Tk::NUMBER, src.substr(i, j - i), std::atof(src.substr(i, j - i).c_str()), l, c);
            col += int(j - i); i = j;
            continue;
        }
        if (std::isalpha((unsigned char)ch) || ch == '_') {
            size_t j = i;
            while (j < src.size() && (std::isalnum((unsigned char)src[j]) || src[j] == '_')) ++j;
            // "m/s" is one unit word.
            std::string w = src.substr(i, j - i);
            if (w == "m" && j + 1 < src.size() && src[j] == '/' && src[j + 1] == 's') { w = "m/s"; j += 2; }
            push(Tk::IDENT, w, 0, l, c);
            col += int(j - i); i = j;
            continue;
        }
        if (ch == '"') {
            size_t j = i + 1;
            while (j < src.size() && src[j] != '"' && src[j] != '\n') ++j;
            if (j >= src.size() || src[j] != '"') {
                diags.push_back({l, c, true, "unterminated string"});
                i = j; continue;
            }
            push(Tk::STRING, src.substr(i + 1, j - i - 1), 0, l, c);
            col += int(j + 1 - i); i = j + 1;
            continue;
        }
        if (ch == '-' && i + 1 < src.size() && src[i + 1] == '>') {
            push(Tk::SYM, "->", 0, l, c); i += 2; col += 2; continue;
        }
        if ((ch == '<' || ch == '>') && i + 1 < src.size() && src[i + 1] == '=') {
            push(Tk::SYM, std::string(1, ch) + "=", 0, l, c); i += 2; col += 2; continue;
        }
        if (std::string("{}(),;=+-<>%").find(ch) != std::string::npos) {
            push(Tk::SYM, std::string(1, ch), 0, l, c); ++i; ++col; continue;
        }
        diags.push_back({l, c, true, std::string("unexpected character '") + ch + "'"});
        ++i; ++col;
    }
    push(Tk::END, "", 0, line, col);
    return out;
}

// ------------------------------------------------------------------ units
enum class Unit { NONE, LEN, TIME, ANGLE, PCT, SPEED };
const char* unitName(Unit u) {
    switch (u) {
        case Unit::LEN: return "a distance (m)";
        case Unit::TIME: return "a time (s, min)";
        case Unit::ANGLE: return "an angle (deg)";
        case Unit::PCT: return "a percentage (%)";
        case Unit::SPEED: return "a speed (m/s)";
        default: return "a plain number";
    }
}

struct Section { std::vector<Instr> code; };

// A name the program can fly to, and what is known about it before flight.
struct Place {
    int index = -1;
    double boundM = -1;        // upper bound on its distance from the start; < 0 unknown
    int line = 0;
};

struct ParseError {};

std::string cv_like(const char* f, const char* a, double b) {
    char buf[200];
    std::snprintf(buf, sizeof buf, f, a, b);
    return buf;
}

class Compiler {
public:
    Compiler(const std::string& src, const std::string& name, CompileResult& r)
        : res_(r) {
        toks_ = lex(src, res_.diags);
        prog().sourceName = name;
        prog().sourceHash = crc32(reinterpret_cast<const uint8_t*>(src.data()), src.size());
        prog().name = name;
        // Target 0 is always the start.
        Target st; st.kind = Target::START; st.name = str("start");
        prog().targets.push_back(st);
        places_["start"] = {0, 0.0, 0};
    }

    void run() {
        sec_ = &main_;
        bool lastTerminal = false;
        while (!at(Tk::END)) {
            if (skipSeparators()) continue;
            try {
                lastTerminal = statement(true);
            } catch (const ParseError&) {
                recover();
            }
        }
        if (!prog().states.empty()) {
            // With states, the top level is SETUP: it runs once and then the
            // first state begins.
            if (!lastTerminal) {
                const int g = emit(Op::GO_STATE, peek().line);
                at_(g).target = 0;
            }
        } else {
            if (!lastTerminal)
                warn(peek().line, 1, "the program ends without land or rtl: when it finishes "
                                     "the aircraft hovers where it is (an `end`)");
            emit(Op::END, peek().line);
        }
        for (size_t i = 0; i < prog().states.size(); ++i)
            if (stateLine_[i] == 0)
                err(stateRefLine_[i], 1, "no state called '" +
                                             prog().strings[size_t(prog().states[i].name)] + "'");
        // Handlers after the main program, their jumps relocated.
        Program& p = prog();
        p.code = main_.code;
        for (auto& h : handlerSecs_) {
            const int off = int(p.code.size());
            for (Instr in : h.second.code) {
                if (in.jump >= 0) in.jump += off;
                p.code.push_back(in);
            }
            p.handlers[size_t(h.first)].pc = off;
        }
        for (auto& st : stateSecs_) {
            const int off = int(p.code.size());
            for (Instr in : st.second.code) {
                if (in.jump >= 0) in.jump += off;
                p.code.push_back(in);
            }
            p.states[size_t(st.first)].pc = off;
        }
        for (size_t i = 0; i < p.states.size(); ++i) {
            p.states[i].transStart = int(p.transitions.size());
            p.states[i].transCount = int(stateTrans_[i].size());
            for (const Transition& t : stateTrans_[i]) p.transitions.push_back(t);
        }
        // SIZES: what the program declared, then -- for labels it uses but
        // did not size -- typical heights, stated as warnings so a range read
        // off a 1.7 m guess is never mistaken for a measurement.
        static const std::pair<const char*, float> TYPICAL[] = {
            {"person", 1.7f}, {"door", 2.0f}, {"chair", 0.9f}, {"car", 1.5f},
            {"bicycle", 1.1f}, {"dog", 0.6f}, {"cat", 0.3f}, {"truck", 3.0f},
            {"bus", 3.2f}, {"motorcycle", 1.1f}, {"bottle", 0.25f}, {"backpack", 0.5f}};
        for (const auto& kv : sizes_) {
            ObjectSize z; z.label = str(kv.first); z.heightM = kv.second; p.sizes.push_back(z);
        }
        for (const std::string& l : labelsUsed_) {
            if (sizes_.count(l)) continue;
            bool known = false;
            for (const auto& t : TYPICAL)
                if (l == t.first) {
                    ObjectSize z; z.label = str(l); z.heightM = t.second; z.assumed = 1;
                    p.sizes.push_back(z);
                    known = true;
                    if (p.caps & Program::NEEDS_RANGE)
                        warn(1, 1, cv_like("no `size %s`: its range falls back to a typical %.2f m height when "
                                           "there is no depth and no ground plane",
                                           l.c_str(), t.second));
                }
            if (!known && (p.caps & Program::NEEDS_RANGE))
                warn(1, 1, "no `size " + l + "`: its range needs depth or the ground plane "
                           "(declare e.g. `size " + l + " 1.0 m`)");
        }
        p.registers = regs_;
        if (p.code.size() > 20000) err(1, 1, "program too large (over 20000 instructions)");
        bool anyErr = false;
        for (const auto& d : res_.diags) anyErr |= d.error;
        std::string why;
        if (!anyErr && !verify(p, &why)) {
            err(1, 1, "internal: compiled program failed verification: " + why);
            anyErr = true;
        }
        res_.ok = !anyErr;
    }

private:
    CompileResult& res_;
    std::vector<Token> toks_;
    size_t pos_ = 0;
    Section main_;
    Section* sec_ = nullptr;
    std::vector<std::pair<int, Section>> handlerSecs_;
    std::vector<std::pair<int, Section>> stateSecs_;
    std::map<std::string, int> stateIdx_;           // name -> states index
    std::vector<int> stateLine_;                    // where each was defined (0 = only referenced)
    std::vector<std::vector<Transition>> stateTrans_;
    std::vector<int> stateRefLine_;
    int curState_ = -1;                             // the state being compiled
    std::map<std::string, float> sizes_;            // `size LABEL N m`
    bool warnedDirect_ = false;
    bool warnedDown_ = false;
    std::vector<std::string> labelsUsed_;           // every object label the program names
    std::map<std::string, Place> places_;
    std::map<std::string, int> strIdx_;
    int regs_ = 0;
    bool inHandler_ = false;
    int depth_ = 0;

    Program& prog() { return res_.program; }

    // ---- diagnostics
    void err(int line, int col, const std::string& m) { res_.diags.push_back({line, col, true, m}); }
    void warn(int line, int col, const std::string& m) { res_.diags.push_back({line, col, false, m}); }
    [[noreturn]] void fail(const Token& t, const std::string& m) {
        err(t.line, t.col, m);
        throw ParseError();
    }
    void recover() {
        // Skip to the end of the line, keeping braces balanced so a broken
        // line inside a block does not swallow the block's end.
        while (!at(Tk::END) && !at(Tk::NEWLINE)) {
            if (isSym("{")) ++depth_;
            if (isSym("}")) { if (depth_ == 0) return; --depth_; }
            ++pos_;
        }
    }

    // ---- token helpers
    const Token& peek(int k = 0) const {
        const size_t i = std::min(pos_ + size_t(k), toks_.size() - 1);
        return toks_[i];
    }
    bool at(Tk k) const { return peek().kind == k; }
    bool isSym(const char* s, int k = 0) const { return peek(k).kind == Tk::SYM && peek(k).text == s; }
    bool isWord(const char* w, int k = 0) const { return peek(k).kind == Tk::IDENT && peek(k).text == w; }
    const Token& next() { const Token& t = peek(); if (pos_ < toks_.size() - 1) ++pos_; return t; }
    bool skipSeparators() {
        bool any = false;
        while (at(Tk::NEWLINE) || isSym(";")) { next(); any = true; }
        return any;
    }
    void skipNewlines() { while (at(Tk::NEWLINE)) next(); }
    void expectSym(const char* s, const char* what) {
        if (!isSym(s)) fail(peek(), std::string("expected '") + s + "' " + what);
        next();
    }
    bool acceptWord(const char* w) { if (isWord(w)) { next(); return true; } return false; }
    std::string ident(const char* what) {
        if (!at(Tk::IDENT)) fail(peek(), std::string("expected ") + what);
        return next().text;
    }
    // A statement ends at a newline, ';', '}' or the end -- anything else left
    // on the line is a mistake worth naming rather than silently dropping.
    void endStatement() {
        if (at(Tk::NEWLINE) || isSym(";") || isSym("}") || at(Tk::END)) return;
        fail(peek(), "unexpected '" + (peek().kind == Tk::NUMBER ? peek().text : peek().text) +
                         "' after the statement");
    }

    int str(const std::string& s) {
        auto it = strIdx_.find(s);
        if (it != strIdx_.end()) return it->second;
        const int i = int(prog().strings.size());
        prog().strings.push_back(s);
        strIdx_[s] = i;
        return i;
    }

    // A number with an optional unit; converted to the unit's base (m, s,
    // deg, %, m/s). A bare number is accepted as the expected unit.
    double number(Unit want, const char* what) {
        bool neg = false;
        if (isSym("-")) { next(); neg = true; }
        if (!at(Tk::NUMBER)) fail(peek(), std::string("expected ") + what);
        const Token& t = next();
        double v = neg ? -t.num : t.num;
        Unit got = Unit::NONE;
        double scale = 1;
        if (isSym("%")) { next(); got = Unit::PCT; }
        else if (at(Tk::IDENT)) {
            const std::string& u = peek().text;
            if (u == "m") got = Unit::LEN;
            else if (u == "cm") { got = Unit::LEN; scale = 0.01; }
            else if (u == "s" || u == "sec") got = Unit::TIME;
            else if (u == "min") { got = Unit::TIME; scale = 60; }
            else if (u == "deg") got = Unit::ANGLE;
            else if (u == "m/s") got = Unit::SPEED;
            if (got != Unit::NONE) next();
        }
        if (got != Unit::NONE && got != want)
            fail(t, std::string(what) + " should be " + unitName(want) + ", not " + unitName(got));
        return v * scale;
    }
    double positive(Unit u, const char* what, double lo, double hi) {
        const Token& t = peek();
        const double v = number(u, what);
        if (!(v >= lo && v <= hi)) {
            char b[160];
            std::snprintf(b, sizeof b, "%s must be between %g and %g (got %g)", what, lo, hi, v);
            fail(t, b);
        }
        return v;
    }
    std::string label() {
        const std::string l = at(Tk::STRING) ? next().text : ident("an object label (e.g. person)");
        if (std::find(labelsUsed_.begin(), labelsUsed_.end(), l) == labelsUsed_.end())
            labelsUsed_.push_back(l);
        return l;
    }

    // ---- emission
    int emit(Op op, int line) {
        Instr in; in.op = op; in.line = line;
        sec_->code.push_back(in);
        return int(sec_->code.size()) - 1;
    }
    Instr& at_(int pc) { return sec_->code[size_t(pc)]; }
    int here() const { return int(sec_->code.size()); }
    int reg() { return regs_++; }

    // ---- places
    int addTarget(Target t) {
        prog().targets.push_back(t);
        return int(prog().targets.size()) - 1;
    }
    static double addBound(double base, double d) { return base < 0 ? -1 : base + d; }
    void checkFence(const Token& t, const std::string& name, double bound) {
        if (bound > prog().fenceM + 1e-6) {
            char b[200];
            std::snprintf(b, sizeof b, "'%s' can be %.1f m from the start, outside the %.0f m fence",
                          name.c_str(), bound, prog().fenceM);
            fail(t, b);
        }
    }
    Place placeRef(const char* what) {
        const Token& t = peek();
        const std::string n = ident(what);
        auto it = places_.find(n);
        if (it == places_.end()) fail(t, "unknown place '" + n + "' (define it with let)");
        return it->second;
    }
    // A place expression, possibly with an offset: NAME [+ (east 2, north 1)]
    // or NAME + (ahead 3, right -1). Returns the target index and its bound.
    Place placeExpr(const char* what, const std::string& nameForNew = "") {
        const Token& t0 = peek();
        Place base = placeRef(what);
        if (!isSym("+")) return base;
        next();
        expectSym("(", "to start an offset like (ahead 3, right 1)");
        double a = 0, b = 0;
        int frame = -1;      // 0 = east/north, 1 = ahead/right
        for (;;) {
            const Token& wt = peek();
            const std::string w = ident("east, north, ahead or right");
            int f; double sign = 1; double* slot;
            if (w == "east" || w == "west") { f = 0; slot = &a; sign = w == "west" ? -1 : 1; }
            else if (w == "north" || w == "south") { f = 0; slot = &b; sign = w == "south" ? -1 : 1; }
            else if (w == "ahead" || w == "behind") { f = 1; slot = &a; sign = w == "behind" ? -1 : 1; }
            else if (w == "right" || w == "left") { f = 1; slot = &b; sign = w == "left" ? -1 : 1; }
            else fail(wt, "'" + w + "' is not an offset direction (east, north, ahead, right, ...)");
            if (frame >= 0 && frame != f)
                fail(wt, "an offset is either east/north or ahead/right, not both");
            frame = f;
            *slot += sign * number(Unit::LEN, "an offset distance");
            if (isSym(",")) { next(); continue; }
            break;
        }
        expectSym(")", "to close the offset");
        Target tg;
        tg.kind = frame == 1 ? Target::REL : Target::OFFSET;
        tg.base = base.index; tg.x = a; tg.y = b;
        if (!nameForNew.empty()) tg.name = str(nameForNew);
        Place p;
        p.index = addTarget(tg);
        p.boundM = addBound(base.boundM, std::hypot(a, b));
        p.line = t0.line;
        checkFence(t0, nameForNew.empty() ? "that offset" : nameForNew, p.boundM);
        return p;
    }
    // An anonymous point relative to a place, for orbit/survey expansion.
    int relPoint(const Place& base, double ahead, double right) {
        Target tg; tg.kind = Target::REL; tg.base = base.index; tg.x = ahead; tg.y = right;
        return addTarget(tg);
    }

    // ---- heights
    // `height H m` (above the ground) or `above H m` (above the place's own
    // height -- an object's top). Returns false if neither is next.
    bool heightOpt(uint8_t& flag, double& v, int line, int col) {
        if (acceptWord("height")) { flag = FLAG_ALT_ABS; v = positive(Unit::LEN, "the height", 0, 200); }
        else if (acceptWord("above")) { flag = FLAG_ALT_REL; v = number(Unit::LEN, "the height above it"); }
        else return false;
        warnDown(line, col);
        return true;
    }
    void warnDown(int line, int col) {
        if (warnedDown_) return;
        warnedDown_ = true;
        warn(line, col, "changing height: the forward camera checks a glide no steeper than it has "
                        "seen; a steeper rest is a vertical move NOTHING checks (never below the "
                        "runtime's floor, except land)");
    }

    // ---- states
    int stateRef(const std::string& name, int line) {
        auto it = stateIdx_.find(name);
        if (it != stateIdx_.end()) return it->second;
        State st; st.name = str(name);
        prog().states.push_back(st);
        const int i = int(prog().states.size()) - 1;
        stateIdx_[name] = i;
        stateLine_.push_back(0);
        stateRefLine_.push_back(line);
        stateTrans_.emplace_back();
        return i;
    }
    // `-> NAME`, `go NAME` or `go to NAME`.
    int stateTarget() {
        acceptWord("to");
        acceptWord("state");
        const Token& t = peek();
        return stateRef(ident("a state name"), t.line);
    }

    // ---- conditions (postfix)
    int condition() {
        CondRange r;
        r.start = int(prog().condOps.size());
        condOr();
        r.count = int(prog().condOps.size()) - r.start;
        prog().conds.push_back(r);
        return int(prog().conds.size()) - 1;
    }
    void pushOp(CondOp::Kind k) { CondOp c; c.kind = k; prog().condOps.push_back(c); }
    void condOr() {
        condAnd();
        while (isWord("or")) { next(); condAnd(); pushOp(CondOp::OR); }
    }
    void condAnd() {
        condNot();
        while (isWord("and")) { next(); condNot(); pushOp(CondOp::AND); }
    }
    void condNot() {
        if (isWord("not")) { next(); condNot(); pushOp(CondOp::NOT); return; }
        condAtom();
    }
    CondOp::Cmp cmp() {
        const Token& t = peek();
        if (isSym("<")) { next(); return CondOp::LT; }
        if (isSym("<=")) { next(); return CondOp::LE; }
        if (isSym(">")) { next(); return CondOp::GT; }
        if (isSym(">=")) { next(); return CondOp::GE; }
        fail(t, "expected a comparison (<, <=, >, >=)");
    }
    void condAtom() {
        const Token& t = peek();
        if (isSym("(")) { next(); condOr(); expectSym(")", "to close the condition"); return; }
        if (acceptWord("true")) { pushOp(CondOp::TRUE_); return; }
        if (acceptWord("positioned")) { pushOp(CondOp::POSITIONED); return; }
        if (acceptWord("range")) {
            acceptWord("to");
            CondOp c; c.kind = CondOp::RANGE; c.arg = str(label());
            c.cmp = uint8_t(cmp());
            c.value = float(number(Unit::LEN, "a range"));
            prog().condOps.push_back(c);
            prog().caps |= Program::NEEDS_DETECTOR | Program::NEEDS_RANGE;
            return;
        }
        if (acceptWord("tracking")) {
            CondOp c; c.kind = CondOp::TRACKING; c.arg = str(label());
            prog().condOps.push_back(c);
            prog().caps |= Program::NEEDS_DETECTOR;
            return;
        }
        if (acceptWord("seen")) {
            CondOp c; c.kind = CondOp::SEEN;
            if (acceptWord("new")) c.var = FLAG_NEW;
            c.arg = str(label());
            prog().condOps.push_back(c);
            prog().caps |= Program::NEEDS_DETECTOR;
            return;
        }
        if (isWord("distance") || isWord("dist")) {
            next();
            acceptWord("to");
            // A PLACE (let ...), or an OBJECT label: then the horizontal
            // distance to where it is remembered to be -- which goes to 0
            // right over it, out of the camera's view.
            if ((at(Tk::IDENT) && !places_.count(peek().text)) || at(Tk::STRING)) {
                CondOp c; c.kind = CondOp::OBJDIST; c.arg = str(label());
                c.cmp = uint8_t(cmp());
                c.value = float(number(Unit::LEN, "a distance"));
                prog().condOps.push_back(c);
                prog().caps |= Program::NEEDS_POSITION | Program::NEEDS_DETECTOR | Program::NEEDS_RANGE;
                return;
            }
            const Place p = placeRef("a place");
            CondOp c; c.kind = CondOp::DIST; c.arg = p.index;
            c.cmp = uint8_t(cmp());
            c.value = float(number(Unit::LEN, "a distance"));
            prog().condOps.push_back(c);
            prog().caps |= Program::NEEDS_POSITION;
            return;
        }
        struct V { const char* w; CondOp::Var v; Unit u; };
        static const V VARS[] = {{"battery", CondOp::BATTERY, Unit::PCT},
                                 {"alt", CondOp::ALT, Unit::LEN},
                                 {"altitude", CondOp::ALT, Unit::LEN},
                                 {"time", CondOp::TIME, Unit::TIME},
                                 {"speed", CondOp::SPEED, Unit::SPEED},
                                 {"heading", CondOp::HEADING, Unit::ANGLE}};
        for (const V& v : VARS)
            if (isWord(v.w)) {
                next();
                CondOp c; c.kind = CondOp::VAR; c.var = uint8_t(v.v);
                c.cmp = uint8_t(cmp());
                c.value = float(number(v.u, v.w));
                prog().condOps.push_back(c);
                return;
            }
        fail(t, "expected a condition: seen LABEL, tracking LABEL, range to LABEL < N, positioned, battery/alt/time/speed/heading "
                "< N, distance to PLACE < N, not/and/or");
    }

    // ---- blocks
    // Returns whether the block's last statement ends the program (land/rtl/end).
    bool block() {
        skipNewlines();
        expectSym("{", "to open a block");
        ++depth_;
        bool lastTerminal = false;
        for (;;) {
            if (skipSeparators()) continue;
            if (isSym("}")) { next(); --depth_; break; }
            if (at(Tk::END)) fail(peek(), "missing '}': the block is never closed");
            try {
                lastTerminal = statement(false);
            } catch (const ParseError&) {
                recover();
            }
        }
        return lastTerminal;
    }
    // `else { ... }` after a statement that can fail. Emits the failure branch
    // and returns the pc the failing op must jump to, or -1 if there is none
    // (then a failure STOPS the mission -- stated in the doc).
    int elseBranch(int failingPc) {
        // `else` may sit on the next line after a closing brace-less statement.
        size_t save = pos_;
        skipNewlines();
        if (!isWord("else")) { pos_ = save; return -1; }
        next();
        const int jumpOver = emit(Op::JUMP, peek().line);
        const int failPc = here();
        block();
        at_(jumpOver).jump = here();
        at_(failingPc).jump = failPc;
        return failPc;
    }

    // ---- statements. Returns true if it ends the program (land/rtl/end).
    bool statement(bool topLevel) {
        const Token t = peek();
        if (isSym("->")) {                     // `-> STATE`: go there
            next();
            const int g = emit(Op::GO_STATE, t.line);
            at_(g).target = stateTarget();
            endStatement(); return true;
        }
        if (t.kind != Tk::IDENT) fail(t, "expected a statement");
        const std::string w = t.text;
        const int L = t.line;

        if (w == "mission") {
            next();
            if (!topLevel) fail(t, "'mission' belongs at the top of the file");
            if (!at(Tk::STRING)) fail(peek(), "expected the mission's name in quotes");
            prog().name = next().text;
            endStatement(); return false;
        }
        if (w == "follow") {
            // FOLLOW THE LOCK at a distance: closer than that and it backs
            // off, farther and it closes, centred on an aim point; optionally
            // at a height. Ends after `for T`, or `until COND` (timeout T).
            next();
            const std::string lab = label();
            double dist = -1, speed = 2.0, aim = 0, tm = -1;
            bool strafe = false, forT = false;
            uint8_t altFlag = 0; double altV = 0;
            int until = -1;
            for (;;) {
                if (acceptWord("at")) dist = positive(Unit::LEN, "the following distance", 0.5, 100);
                else if (acceptWord("max")) speed = positive(Unit::SPEED, "the top speed", 0.1, 15);
                else if (acceptWord("aim")) {
                    double sign = 1;
                    if (acceptWord("left")) sign = -1;
                    else if (!acceptWord("right")) acceptWord("centre");
                    if (at(Tk::NUMBER)) aim = sign * positive(Unit::NONE, "the aim (box widths)", 0, 20);
                }
                else if (acceptWord("strafe")) strafe = true;
                else if (heightOpt(altFlag, altV, L, t.col)) {}
                else if (acceptWord("for")) { tm = positive(Unit::TIME, "how long", 0.5, 3600); forT = true; }
                else if (acceptWord("until")) until = condition();
                else if (acceptWord("timeout")) tm = positive(Unit::TIME, "the timeout", 0.5, 3600);
                else break;
            }
            if (dist < 0) fail(t, "follow needs a distance: follow person at 4 m");
            if (!forT && until < 0) fail(t, "follow needs an end: `for 60 s` or `until CONDITION`");
            if (tm < 0) tm = 120;
            prog().caps |= Program::NEEDS_DETECTOR | Program::NEEDS_RANGE;
            if (!warnedDirect_) {
                warnedDirect_ = true;
                warn(L, t.col, "follow flies at the object with NOTHING checking the way for "
                               "obstacles -- keep the fence tight");
            }
            const int pc = emit(Op::FOLLOW, L);
            at_(pc).text = str(lab);
            at_(pc).a = float(dist); at_(pc).b = float(speed); at_(pc).c = float(tm);
            at_(pc).d = float(aim); at_(pc).e = float(altV);
            at_(pc).cond = until;
            at_(pc).flags = uint8_t((strafe ? FLAG_STRAFE : 0) | altFlag | (forT ? FLAG_FOR : 0));
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "steer") {
            // STEER ON THE LOCK: aim at a point on the object's box, in box
            // widths so it holds still as the box grows; a speed; a way to
            // turn; and what ends it.
            next();
            const std::string lab = label();
            double aim = 0, aimUp = 0, speed = 1.0, timeout = 30;
            bool strafe = false, dive = false, ray = false, forT = false;
            int until = -1;
            for (;;) {
                if (acceptWord("aim")) {
                    double sign = 1;
                    if (acceptWord("left")) sign = -1;
                    else if (!acceptWord("right")) acceptWord("centre");
                    if (at(Tk::NUMBER)) aim = sign * positive(Unit::NONE, "the aim (box widths)", 0, 20);
                    // ...and up/down of the box's centre, in box HEIGHTS: the
                    // crosshair is then a point in 3D, and it flies at it.
                    if (isWord("up") || isWord("down")) {
                        const double vs = next().text == "up" ? 1 : -1;
                        aimUp = vs * positive(Unit::NONE, "the aim (box heights)", 0, 20);
                        ray = true;
                    }
                } else if (acceptWord("throttle")) {
                    speed = positive(Unit::NONE, "the throttle (0..1)", 0, 1);
                    ray = true;
                } else if (acceptWord("speed")) {
                    speed = positive(Unit::SPEED, "the speed", 0, 15);
                } else if (acceptWord("strafe")) {
                    strafe = true;
                } else if (acceptWord("dive")) {
                    dive = true;
                    warnDown(L, t.col);
                } else if (acceptWord("until")) {
                    until = condition();
                } else if (acceptWord("for")) {
                    timeout = positive(Unit::TIME, "how long", 0.1, 600);
                    forT = true;
                } else if (acceptWord("timeout")) {
                    timeout = positive(Unit::TIME, "the timeout", 0.5, 600);
                } else break;
            }
            prog().caps |= Program::NEEDS_DETECTOR;
            if (!warnedDirect_) {
                warnedDirect_ = true;
                warn(L, t.col, "steer flies at the object with NOTHING checking the way for "
                               "obstacles -- keep it short, give it an until, keep the fence tight");
            }
            const int pc = emit(Op::STEER, L);
            at_(pc).text = str(lab);
            at_(pc).a = float(aim); at_(pc).b = float(speed); at_(pc).c = float(timeout);
            at_(pc).cond = until;
            at_(pc).flags = uint8_t((strafe ? FLAG_STRAFE : 0) | (dive ? FLAG_DIVE : 0) |
                                    (ray ? FLAG_RAY : 0) | (forT ? FLAG_FOR : 0));
            at_(pc).d = float(aimUp);
            if (ray && !(speed <= 1.0))
                fail(t, "with a crosshair, set `throttle 0..1` rather than a speed");
            if (ray) warnDown(L, t.col);
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "gains") {
            // `gains yaw kp 1.2 kd 0.2` -- how hard each loop onto the target
            // point pulls (P), damps (D) and trims (I), from here on. Put it
            // at the top of a state to give that mode its own.
            next();
            const Token at0 = peek();
            const std::string ax = ident("yaw, strafe, dive or range");
            int axis = ax == "yaw" ? 0 : ax == "strafe" ? 1 : (ax == "dive" || ax == "vertical") ? 2
                     : ax == "range" ? 3 : ax == "track" ? 4 : -1;
            if (axis < 0) fail(at0, "'" + ax + "': gains are for yaw, strafe, dive, range or track");
            const int pc = emit(Op::GAINS, L);
            at_(pc).target = axis;
            uint8_t which = 0;
            for (;;) {
                if (acceptWord("kp")) { at_(pc).a = float(positive(Unit::NONE, "kp", 0, 100)); which |= 1; }
                else if (acceptWord("ki")) { at_(pc).b = float(positive(Unit::NONE, "ki", 0, 100)); which |= 2; }
                else if (acceptWord("kd")) { at_(pc).c = float(positive(Unit::NONE, "kd", 0, 100)); which |= 4; }
                else if (acceptWord("filter")) { at_(pc).d = float(positive(Unit::TIME, "the derivative filter", 0, 5)); which |= 8; }
                else break;
            }
            if (!which) fail(peek(), "gains needs at least one of kp, ki, kd, filter");
            at_(pc).flags = which;
            endStatement(); return false;
        }
        if (w == "param") {
            // `param min_alt 0.5 m`: a runtime knob, from here on.
            next();
            const Token nt = peek();
            const std::string n = ident("a parameter name");
            int id = -1;
            for (int k = 0; k < P_COUNT; ++k) if (n == paramName(k)) id = k;
            if (id < 0) {
                std::string all;
                for (int k = 0; k < P_COUNT; ++k) all += std::string(k ? ", " : "") + paramName(k);
                fail(nt, "'" + n + "' is not a parameter: " + all);
            }
            static const Unit U[] = {Unit::LEN, Unit::NONE, Unit::ANGLE, Unit::SPEED, Unit::SPEED,
                                     Unit::TIME, Unit::LEN, Unit::NONE, Unit::NONE, Unit::TIME};
            const int pc = emit(Op::PARAM, L);
            at_(pc).target = id;
            at_(pc).a = float(positive(U[id], n.c_str(), 0, 1000));
            endStatement(); return false;
        }
        if (w == "pass") {
            // PASS OVER an object without stopping, at a height, out N m past.
            next();
            if (!acceptWord("over")) fail(peek(), "expected `pass over LABEL`");
            const std::string lab = label();
            double speed = 2.0, beyond = 3.0, tm = 60;
            uint8_t altFlag = 0; double altV = 0;
            for (;;) {
                if (heightOpt(altFlag, altV, L, t.col)) continue;
                if (acceptWord("speed")) speed = positive(Unit::SPEED, "the speed", 0.1, 20);
                else if (acceptWord("beyond")) beyond = positive(Unit::LEN, "how far past it", 0, 100);
                else if (acceptWord("timeout")) tm = positive(Unit::TIME, "the timeout", 1, 600);
                else break;
            }
            prog().caps |= Program::NEEDS_POSITION | Program::NEEDS_DETECTOR | Program::NEEDS_RANGE;
            if (!warnedDirect_) {
                warnedDirect_ = true;
                warn(L, t.col, "pass over flies straight at it with NOTHING checking the way -- "
                               "keep it above the obstacles");
            }
            const int pc = emit(Op::PASS_OVER, L);
            at_(pc).text = str(lab);
            at_(pc).a = float(beyond); at_(pc).b = float(speed); at_(pc).c = float(tm);
            at_(pc).e = float(altV); at_(pc).flags = altFlag;
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "cruise") {
            // CRUISE: straight ahead, level, at a speed, holding the height it
            // has -- "just going forward", whatever the camera's tilt.
            next();
            double speed = 2.0, tm = -1;
            bool forT = false;
            int until = -1;
            for (;;) {
                if (acceptWord("speed")) speed = positive(Unit::SPEED, "the speed", 0, 20);
                else if (acceptWord("for")) { tm = positive(Unit::TIME, "how long", 0.1, 3600); forT = true; }
                else if (acceptWord("until")) until = condition();
                else if (acceptWord("timeout")) tm = positive(Unit::TIME, "the timeout", 0.1, 3600);
                else break;
            }
            if (!forT && until < 0) fail(t, "cruise needs an end: `for 60 s` or `until CONDITION`");
            if (tm < 0) tm = 120;
            if (!warnedDirect_) {
                warnedDirect_ = true;
                warn(L, t.col, "cruise flies straight ahead with NOTHING checking the way -- "
                               "keep it above the obstacles and the fence tight");
            }
            const int pc = emit(Op::CRUISE, L);
            at_(pc).a = float(speed); at_(pc).c = float(tm); at_(pc).cond = until;
            at_(pc).flags = forT ? FLAG_FOR : 0;
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "fly") {
            // FLY AT THE CROSSHAIR: a fixed point in the camera image and a
            // throttle; the aircraft flies along the ray through it. The
            // simplest control there is, and the one the 3D playground draws.
            next();
            if (!acceptWord("crosshair")) fail(peek(), "expected `fly crosshair X Y throttle T for S s`");
            const double x = number(Unit::NONE, "the crosshair x (-1..1, + right)");
            if (isSym(",")) next();
            const double y = number(Unit::NONE, "the crosshair y (-1..1, + up)");
            if (std::fabs(x) > 1.0 || std::fabs(y) > 1.0) fail(t, "the crosshair must be inside the frame: -1..1");
            double thr = 0.5, tm = -1;
            bool forT = false;
            int until = -1;
            for (;;) {
                if (acceptWord("throttle")) thr = positive(Unit::NONE, "the throttle (0..1)", 0, 1);
                else if (acceptWord("for")) { tm = positive(Unit::TIME, "how long", 0.1, 600); forT = true; }
                else if (acceptWord("until")) until = condition();
                else if (acceptWord("timeout")) tm = positive(Unit::TIME, "the timeout", 0.1, 600);
                else break;
            }
            if (!forT && until < 0) fail(t, "fly needs an end: `for 3 s` or `until CONDITION`");
            if (tm < 0) tm = 30;
            if (!warnedDirect_) {
                warnedDirect_ = true;
                warn(L, t.col, "fly flies where the crosshair points with NOTHING checking the way -- "
                               "keep it short and the fence tight");
            }
            warnDown(L, t.col);
            const int pc = emit(Op::FLY, L);
            at_(pc).a = float(x); at_(pc).d = float(y); at_(pc).b = float(thr); at_(pc).c = float(tm);
            at_(pc).cond = until;
            at_(pc).flags = forT ? FLAG_FOR : 0;
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "path") {
            // A PATH ANCHORED ON A TRACKED OBJECT -- what the editor's
            // first-person pane writes. Points are AHEAD/RIGHT of the object
            // along the line of sight at the start; the object is re-measured
            // all the way, and the points move with it.
            next();
            const std::string lab = label();
            bool facing = false;
            double radius = 0.75, timeout = 60;
            for (;;) {
                if (acceptWord("facing")) facing = true;
                else if (acceptWord("radius")) radius = positive(Unit::LEN, "the arrival radius", 0.3, 20);
                else if (acceptWord("timeout")) timeout = positive(Unit::TIME, "the timeout per point", 1, 600);
                else break;
            }
            prog().caps |= Program::NEEDS_POSITION | Program::NEEDS_DETECTOR | Program::NEEDS_RANGE;
            Target an; an.kind = Target::RUNTIME;
            const int ai = addTarget(an);
            const int anc = emit(Op::ANCHOR, L);
            at_(anc).target = ai; at_(anc).text = str(lab);
            skipNewlines();
            expectSym("{", "to open the path's points");
            int points = 0;
            for (;;) {
                if (skipSeparators()) continue;
                if (isSym("}")) { next(); break; }
                if (at(Tk::END)) fail(peek(), "missing '}': the path is never closed");
                const Token st = peek();
                if (acceptWord("to")) {
                    double a = 0, r = 0;
                    for (;;) {
                        const Token dt = peek();
                        if (!at(Tk::IDENT)) break;
                        const std::string d = peek().text;
                        double sign = 1, *slot = nullptr;
                        if (d == "ahead" || d == "behind") { slot = &a; sign = d == "behind" ? -1 : 1; }
                        else if (d == "right" || d == "left") { slot = &r; sign = d == "left" ? -1 : 1; }
                        else if (d == "radius" || d == "timeout" || d == "above" || d == "height") break;
                        else fail(dt, "'" + d + "': a path point is `to ahead A right R`");
                        next();
                        *slot += sign * number(Unit::LEN, "a distance");
                        if (isSym(",")) next();
                    }
                    double pr = radius;
                    uint8_t altFlag = 0; double altV = 0;
                    for (;;) {
                        if (heightOpt(altFlag, altV, st.line, st.col)) continue;
                        if (acceptWord("radius")) { pr = positive(Unit::LEN, "the arrival radius", 0.3, 20); continue; }
                        break;
                    }
                    Target off; off.kind = Target::REL; off.base = ai; off.x = a; off.y = r;
                    const int ti = addTarget(off);
                    const int g = emit(Op::GOTO, st.line);
                    at_(g).target = ti; at_(g).a = float(pr); at_(g).b = float(timeout);
                    at_(g).flags = altFlag; at_(g).c = float(altV);
                    if (facing) {
                        const int fc = emit(Op::TURN_TO_PLACE, st.line);
                        at_(fc).target = ai; at_(fc).b = 20.f;
                    }
                    ++points;
                } else if (acceptWord("hold")) {
                    const int h = emit(Op::HOLD, st.line);
                    at_(h).a = float(positive(Unit::TIME, "the hold", 0.1, 600));
                } else {
                    fail(st, "inside a path: `to ahead A right R` or `hold N s`");
                }
                endStatement();
            }
            if (!points) fail(t, "a path needs at least one `to` point");
            emit(Op::UNANCHOR, L);
            elseBranch(anc);
            endStatement(); return false;
        }
        if (w == "nav") {
            next();
            const Token mt = peek();
            const std::string m = ident("certified or direct");
            if (m != "certified" && m != "direct")
                fail(mt, "nav takes certified (every leg checked by the planner) or direct "
                         "(straight at the target, unchecked)");
            const int pc = emit(Op::NAV, L);
            at_(pc).a = m == "direct" ? 1.f : 0.f;
            if (m == "direct" && !warnedDirect_) {
                warnedDirect_ = true;
                warn(L, t.col, "nav direct: goto, over and approach fly straight lines that NOTHING "
                               "checks for obstacles -- fly above them (climb) and keep the fence tight");
            }
            endStatement(); return false;
        }
        if (w == "size") {
            // `size door 2.0 m`: how TALL the thing is, so its range can be
            // read off its box (f * height / pixels) with no depth camera.
            next();
            if (!topLevel) fail(t, "'size' belongs at the top level");
            const std::string lab = label();
            sizes_[lab] = float(positive(Unit::LEN, "the object's height", 0.02, 100));
            endStatement(); return false;
        }
        if (w == "fence") {
            next();
            if (!topLevel) fail(t, "'fence' belongs at the top level");
            if (!places_.empty() && places_.size() > 1)
                fail(t, "set the fence before defining places, so they can be checked against it");
            prog().fenceM = float(positive(Unit::LEN, "the fence radius", 2, 2000));
            endStatement(); return false;
        }
        if (w == "timeout") {
            next();
            if (!topLevel) fail(t, "the mission 'timeout' belongs at the top level");
            prog().timeoutS = float(positive(Unit::TIME, "the mission timeout", 10, 3600));
            endStatement(); return false;
        }
        if (w == "let") {
            next();
            const Token nt = peek();
            const std::string name = ident("a name for the place");
            if (places_.count(name)) fail(nt, "'" + name + "' is already defined");
            static const char* RESERVED[] = {"start", "here", "seen", "enu", "fwd", "gps"};
            for (const char* r : RESERVED)
                if (name == r) fail(nt, "'" + name + "' is a reserved word");
            expectSym("=", "after the place's name");
            Place p; p.line = L;
            const Token vt = peek();
            if (acceptWord("here")) {
                Target tg; tg.kind = Target::RUNTIME; tg.name = str(name);
                p.index = addTarget(tg);
                const int pc = emit(Op::MARK, L);
                at_(pc).target = p.index;
                prog().caps |= Program::NEEDS_POSITION;
            } else if (acceptWord("seen")) {
                const bool isNew = acceptWord("new");
                const std::string lab = label();
                Target tg; tg.kind = Target::RUNTIME; tg.name = str(name);
                p.index = addTarget(tg);
                const int pc = emit(Op::MARK_SEEN, L);
                at_(pc).target = p.index;
                at_(pc).text = str(lab);
                if (isNew) at_(pc).flags = FLAG_NEW;
                prog().caps |= Program::NEEDS_POSITION | Program::NEEDS_DETECTOR |
                               Program::NEEDS_RANGE;
                // Defined before its else so the else may still test it --
                // a place that failed to mark is UNSET, and flying to one
                // fails rather than guessing.
                places_[name] = p;
                elseBranch(pc);
                return false;
            } else if (isWord("enu") || isWord("fwd") || isWord("gps")) {
                const std::string k = next().text;
                expectSym("(", ("after " + k).c_str());
                Target tg; tg.name = str(name);
                if (k == "gps") {
                    tg.kind = Target::GPS;
                    tg.x = number(Unit::NONE, "a latitude");
                    expectSym(",", "between latitude and longitude");
                    tg.y = number(Unit::NONE, "a longitude");
                    if (std::fabs(tg.x) > 90 || std::fabs(tg.y) > 180) fail(vt, "not a latitude/longitude");
                    prog().caps |= Program::NEEDS_GPS;
                } else {
                    const double a = number(Unit::LEN, k == "enu" ? "metres east" : "metres ahead");
                    expectSym(",", "between the two distances");
                    const double b = number(Unit::LEN, k == "enu" ? "metres north" : "metres right");
                    if (k == "enu") { tg.kind = Target::ENU; tg.x = a; tg.y = b; }
                    else { tg.kind = Target::REL; tg.base = 0; tg.x = a; tg.y = b; }
                    p.boundM = std::hypot(a, b);
                }
                expectSym(")", "to close the place");
                p.index = addTarget(tg);
                checkFence(nt, name, p.boundM);
            } else if (at(Tk::IDENT)) {
                // Another place, maybe offset; a plain alias (let home =
                // start) shares its target.
                p = placeExpr("a place", name);
            } else {
                fail(vt, "expected here, seen LABEL, enu(e, n), fwd(ahead, right), "
                         "gps(lat, lon) or another place");
            }
            places_[name] = p;
            endStatement(); return false;
        }
        if (w == "set") {
            // RE-MARK a place declared with `let NAME = here` (or seen): in a
            // state that loops, or anywhere after its first definition.
            next();
            const Token nt = peek();
            const std::string name = ident("a place to set");
            auto it = places_.find(name);
            if (it == places_.end()) fail(nt, "unknown place '" + name + "' (declare it with let first)");
            if (prog().targets[size_t(it->second.index)].kind != Target::RUNTIME)
                fail(nt, "'" + name + "' is a fixed place; only places marked in flight "
                             "(let NAME = here / seen LABEL) can be set again");
            expectSym("=", "after the place's name");
            if (acceptWord("here")) {
                const int pc = emit(Op::MARK, L);
                at_(pc).target = it->second.index;
                prog().caps |= Program::NEEDS_POSITION;
            } else if (acceptWord("seen")) {
                const bool isNew = acceptWord("new");
                const std::string lab = label();
                const int pc = emit(Op::MARK_SEEN, L);
                at_(pc).target = it->second.index;
                at_(pc).text = str(lab);
                if (isNew) at_(pc).flags = FLAG_NEW;
                prog().caps |= Program::NEEDS_POSITION | Program::NEEDS_DETECTOR |
                               Program::NEEDS_RANGE;
                elseBranch(pc);
                return false;
            } else {
                fail(peek(), "expected here or seen LABEL");
            }
            endStatement(); return false;
        }
        if (w == "goto" || w == "over") {
            next();
            const Place p = placeExpr("a place to fly to");
            prog().caps |= Program::NEEDS_POSITION;
            double radius = w == "over" ? 1.0 : 1.5, timeout = 120, hold = w == "over" ? 3 : 0;
            uint8_t altFlag = 0; double altV = 0;
            for (;;) {
                if (heightOpt(altFlag, altV, L, t.col)) continue;
                if (acceptWord("radius")) radius = positive(Unit::LEN, "the arrival radius", 0.3, 20);
                else if (acceptWord("timeout")) timeout = positive(Unit::TIME, "the timeout", 1, 3600);
                else if (w == "over" && acceptWord("hold")) hold = positive(Unit::TIME, "the hold", 0, 600);
                else break;
            }
            const int pc = emit(Op::GOTO, L);
            at_(pc).target = p.index; at_(pc).a = float(radius); at_(pc).b = float(timeout);
            at_(pc).flags = altFlag; at_(pc).c = float(altV);
            elseBranch(pc);
            if (hold > 0) { const int h = emit(Op::HOLD, L); at_(h).a = float(hold); }
            endStatement(); return false;
        }
        if (w == "orbit") {
            next();
            const Place c = placeExpr("the place to orbit");
            prog().caps |= Program::NEEDS_POSITION;
            double R = -1, laps = 1, pts = 8, dir = 1;
            for (;;) {
                if (acceptWord("radius")) R = positive(Unit::LEN, "the orbit radius", 1, 100);
                else if (acceptWord("laps")) laps = positive(Unit::NONE, "laps", 0.25, 20);
                else if (acceptWord("points")) pts = positive(Unit::NONE, "points", 3, 36);
                else if (acceptWord("cw")) dir = 1;
                else if (acceptWord("ccw")) dir = -1;
                else break;
            }
            if (R < 0) fail(t, "orbit needs a radius: orbit PLACE radius 3 m");
            checkFence(t, "the orbit", addBound(c.boundM, R));
            // TRANSLATED HERE into gotos round the place, starting on the NEAR
            // side of its reference heading -- for a place marked from a
            // detection, the side the aircraft is looking from.
            const int n = int(std::lround(pts * laps));
            for (int k = 0; k <= n; ++k) {
                const double th = kPi + dir * 2 * kPi * k / pts;
                const int tg = relPoint(c, R * std::cos(th), R * std::sin(th));
                const int pc = emit(Op::GOTO, L);
                at_(pc).target = tg; at_(pc).a = float(std::min(1.0, std::max(0.75, R * 0.3)));
                at_(pc).b = 60.f;
            }
            endStatement(); return false;
        }
        if (w == "survey") {
            next();
            const Place c = placeExpr("the survey's corner");
            prog().caps |= Program::NEEDS_POSITION;
            double W = -1, Lm = -1, S = -1;
            for (;;) {
                if (acceptWord("width")) W = positive(Unit::LEN, "the width", 0, 500);
                else if (acceptWord("length")) Lm = positive(Unit::LEN, "the length", 1, 500);
                else if (acceptWord("spacing")) S = positive(Unit::LEN, "the lane spacing", 0.5, 100);
                else break;
            }
            if (W < 0 || Lm < 0 || S < 0)
                fail(t, "survey needs width, length and spacing: survey PLACE width 10 m length 20 m spacing 3 m");
            checkFence(t, "the survey", addBound(c.boundM, std::hypot(W, Lm)));
            const int lanes = int(std::floor(W / S + 1e-6)) + 1;
            if (lanes > 100) fail(t, "survey would need more than 100 lanes");
            // Lanes run AHEAD along the place's reference heading, stepping RIGHT.
            for (int i = 0; i < lanes; ++i) {
                const double r = i * S;
                const double a0 = (i % 2) ? Lm : 0, a1 = (i % 2) ? 0 : Lm;
                for (double a : {a0, a1}) {
                    const int pc = emit(Op::GOTO, L);
                    at_(pc).target = relPoint(c, a, r);
                    at_(pc).a = 1.0f; at_(pc).b = 120.f;
                }
            }
            endStatement(); return false;
        }
        if (w == "hold") {
            next();
            const int pc = emit(Op::HOLD, L);
            at_(pc).a = float(positive(Unit::TIME, "the hold time", 0.1, 600));
            endStatement(); return false;
        }
        if (w == "turn") {
            next();
            double timeout = 20;
            int pc;
            if (acceptWord("to")) {
                pc = emit(Op::TURN_TO, L);
                double h = number(Unit::ANGLE, "a heading");
                h = std::fmod(std::fmod(h, 360.0) + 360.0, 360.0);
                at_(pc).a = float(h);
            } else {
                double sign = 1;
                if (acceptWord("left")) sign = -1;
                else acceptWord("right");
                pc = emit(Op::TURN_BY, L);
                at_(pc).a = float(sign * positive(Unit::ANGLE, "the turn", -720, 720));
            }
            if (acceptWord("timeout")) timeout = positive(Unit::TIME, "the timeout", 1, 120);
            at_(pc).b = float(timeout);
            endStatement(); return false;
        }
        if (w == "climb") {
            next();
            acceptWord("to");
            const int pc = emit(Op::CLIMB, L);
            at_(pc).a = float(positive(Unit::LEN, "the altitude", 0.3, 120));
            at_(pc).b = 30.f;
            if (acceptWord("timeout")) at_(pc).b = float(positive(Unit::TIME, "the timeout", 1, 300));
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "search" || w == "face" || w == "approach") {
            next();
            const bool isNew = w == "search" && acceptWord("new");
            const std::string lab = label();
            prog().caps |= Program::NEEDS_DETECTOR;
            Op op = w == "search" ? Op::SEARCH : w == "face" ? Op::FACE : Op::APPROACH;
            // `approach door to 2 m`: stop at a RANGE (from its size, or depth),
            // rather than when it fills a fraction of the frame.
            double stopAt = -1;
            if (op == Op::APPROACH && acceptWord("to")) {
                stopAt = positive(Unit::LEN, "the stopping range", 0.5, 100);
                op = Op::APPROACH_TO;
                prog().caps |= Program::NEEDS_RANGE;
            }
            const int pc = emit(op, L);
            at_(pc).text = str(lab);
            if (isNew) at_(pc).flags = FLAG_NEW;
            double timeout = w == "search" ? 30 : w == "face" ? 15 : 90, fill = 0.4, dir = 1;
            for (;;) {
                if (acceptWord("timeout")) timeout = positive(Unit::TIME, "the timeout", 1, 600);
                else if (w == "search" && acceptWord("left")) dir = -1;
                else if (w == "search" && acceptWord("right")) dir = 1;
                else if (op == Op::APPROACH && acceptWord("fill"))
                    fill = positive(Unit::NONE, "the fill fraction", 0.05, 0.95);
                else break;
            }
            if (op == Op::APPROACH_TO) { at_(pc).a = float(stopAt); at_(pc).b = float(timeout); }
            else if (op == Op::APPROACH) { at_(pc).a = float(fill); at_(pc).b = float(timeout); }
            else { at_(pc).a = float(timeout); at_(pc).b = float(dir); }
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "explore") {
            next();
            const int pc = emit(Op::EXPLORE, L);
            at_(pc).a = float(positive(Unit::TIME, "how long to explore", 1, 3600));
            prog().caps |= Program::NEEDS_POSITION;
            endStatement(); return false;
        }
        if (w == "run") {
            next();
            const Token mt = peek();
            std::string mode = ident("a control mode name (e.g. FOLLOW_ROAD)");
            for (char& ch : mode) ch = char(std::toupper((unsigned char)ch));
            if (mode == "SCRIPT") fail(mt, "a script cannot run SCRIPT");
            if (!acceptWord("for")) fail(peek(), "expected 'for': run MODE for 30 s");
            const int pc = emit(Op::RUN, L);
            at_(pc).text = str(mode);
            at_(pc).a = float(positive(Unit::TIME, "how long", 1, 3600));
            endStatement(); return false;
        }
        if (w == "wait") {
            next();
            if (!acceptWord("until")) fail(peek(), "expected 'until': wait until CONDITION");
            const int c = condition();
            double timeout = 60;
            if (acceptWord("timeout")) timeout = positive(Unit::TIME, "the timeout", 0.1, 3600);
            const int pc = emit(Op::WAIT, L);
            at_(pc).cond = c; at_(pc).a = float(timeout);
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "if") {
            next();
            const int c = condition();
            const int br = emit(Op::JUMP_IFNOT, L);
            at_(br).cond = c;
            const bool thenTerm = block();
            size_t save = pos_;
            skipNewlines();
            if (isWord("else")) {
                next();
                const int over = emit(Op::JUMP, L);
                at_(br).jump = here();
                bool elseTerm;
                if (isWord("if")) elseTerm = statement(topLevel);
                else elseTerm = block();
                at_(over).jump = here();
                return thenTerm && elseTerm;
            }
            pos_ = save;
            at_(br).jump = here();
            return false;
        }
        if (w == "repeat") {
            next();
            const Token nt = peek();
            const double n = positive(Unit::NONE, "the repeat count", 1, 1000);
            if (n != std::floor(n)) fail(nt, "repeat takes a whole number");
            const int r = reg();
            const int s = emit(Op::SET_REG, L);
            at_(s).target = r; at_(s).a = float(n);
            const int top = here();
            block();
            const int lp = emit(Op::LOOP, L);
            at_(lp).target = r; at_(lp).jump = top;
            return false;
        }
        if (w == "while") {
            next();
            const int c = condition();
            double timeout = 120;
            if (acceptWord("timeout")) timeout = positive(Unit::TIME, "the timeout", 1, 3600);
            const int r = reg();
            const int tm = emit(Op::TIMER, L);
            at_(tm).target = r;
            const int top = here();
            const int br = emit(Op::JUMP_IFNOT, L);
            at_(br).cond = c;
            const int tu = emit(Op::JUMP_TIMEUP, L);
            at_(tu).target = r; at_(tu).a = float(timeout);
            block();
            const int back = emit(Op::JUMP, L);
            at_(back).jump = top;
            at_(br).jump = here();
            at_(tu).jump = here();
            return false;
        }
        if (w == "on") {
            next();
            if (!topLevel) fail(t, "'on' handlers belong at the top level");
            const int c = condition();
            Handler h; h.cond = c; h.line = L;
            prog().handlers.push_back(h);
            const int hi = int(prog().handlers.size()) - 1;
            handlerSecs_.emplace_back(hi, Section());
            Section* saved = sec_;
            sec_ = &handlerSecs_.back().second;
            inHandler_ = true;
            bool term = false;
            try { term = block(); } catch (...) { inHandler_ = false; sec_ = saved; throw; }
            const bool resumes = !sec_->code.empty() && sec_->code.back().op == Op::RESUME;
            if (!term && !resumes) {
                warn(L, t.col, "this handler ends without land, rtl or resume: when it fires "
                               "the mission stops and the aircraft hovers");
                emit(Op::END, L);
            }
            inHandler_ = false;
            sec_ = saved;
            return false;
        }
        if (w == "state") {
            next();
            if (!topLevel) fail(t, "a 'state' belongs at the top level, not inside a block");
            const Token nt = peek();
            const std::string name = ident("the state's name");
            const int si = stateRef(name, nt.line);
            if (stateLine_[size_t(si)] != 0)
                fail(nt, "state '" + name + "' is already defined (line " +
                             std::to_string(stateLine_[size_t(si)]) + ")");
            stateLine_[size_t(si)] = L;
            stateSecs_.emplace_back(si, Section());
            Section* saved = sec_;
            sec_ = &stateSecs_.back().second;
            curState_ = si;
            bool term = false;
            try { term = block(); } catch (...) { curState_ = -1; sec_ = saved; throw; }
            const bool loops = !sec_->code.empty() && sec_->code.back().op == Op::GO_STATE;
            if (!term && !loops) {
                warn(L, t.col, "state '" + name + "' ends without going to another state, land "
                               "or rtl: when its steps run out the aircraft hovers there "
                               "(its `when` triggers stay live)");
                emit(Op::END, L);
            }
            curState_ = -1;
            sec_ = saved;
            return false;
        }
        if (w == "when") {
            next();
            if (curState_ < 0)
                fail(t, "'when' is a state's trigger -- put it inside a state { } "
                        "(for the whole mission use 'on')");
            const int c = condition();
            if (!isSym("->") && !isWord("go"))
                fail(peek(), "expected '-> STATE' after the condition");
            next();
            Transition tr; tr.cond = c; tr.line = L; tr.to = stateTarget();
            stateTrans_[size_t(curState_)].push_back(tr);
            endStatement(); return false;
        }
        if (w == "go") {
            next();
            const int g = emit(Op::GO_STATE, L);
            at_(g).target = stateTarget();
            endStatement(); return true;
        }
        if (w == "move") {
            next();
            const Token dt = peek();
            const std::string d = ident("forward, back, left or right");
            double ahead = 0, right = 0;
            const double m = positive(Unit::LEN, "the distance", 0.2, 200);
            if (d == "forward" || d == "ahead") ahead = m;
            else if (d == "back" || d == "backward") ahead = -m;
            else if (d == "right") right = m;
            else if (d == "left") right = -m;
            else if (d == "up" || d == "down")
                fail(dt, "up/down are not moves: use `up 1 m` (climb); descending is the FC's job (land)");
            else fail(dt, "'" + d + "' is not a direction (forward, back, left, right)");
            double radius = 0.4, timeout = 60;
            for (;;) {
                if (acceptWord("radius")) radius = positive(Unit::LEN, "the arrival radius", 0.3, 20);
                else if (acceptWord("timeout")) timeout = positive(Unit::TIME, "the timeout", 1, 600);
                else break;
            }
            prog().caps |= Program::NEEDS_POSITION;
            if (m > prog().fenceM) fail(dt, "that move is longer than the fence");
            // TRANSLATED: mark here (with the heading it has now), fly to the
            // point offset from it, turn back to that heading. Every leg on the
            // way is certified like any other -- a sideways move turns to face
            // its way first, because the camera only clears what it can see.
            Target base; base.kind = Target::RUNTIME;
            const int bi = addTarget(base);
            const int mk = emit(Op::MARK, L);
            at_(mk).target = bi;
            Target off; off.kind = Target::REL; off.base = bi; off.x = ahead; off.y = right;
            const int oi = addTarget(off);
            const int g = emit(Op::GOTO, L);
            at_(g).target = oi; at_(g).a = float(radius); at_(g).b = float(timeout);
            elseBranch(g);
            const int tb = emit(Op::TURN_REF, L);
            at_(tb).target = bi; at_(tb).b = 20.f;
            endStatement(); return false;
        }
        if (w == "yaw") {
            next();
            int pc;
            if (acceptWord("to")) {
                pc = emit(Op::TURN_TO, L);
                double h = number(Unit::ANGLE, "a heading");
                at_(pc).a = float(std::fmod(std::fmod(h, 360.0) + 360.0, 360.0));
            } else {
                double sign = 1;
                if (acceptWord("left")) sign = -1;
                else if (!acceptWord("right")) fail(peek(), "expected left, right or to");
                pc = emit(Op::TURN_BY, L);
                at_(pc).a = float(sign * positive(Unit::ANGLE, "the yaw", 0, 720));
            }
            at_(pc).b = 20.f;
            if (acceptWord("timeout")) at_(pc).b = float(positive(Unit::TIME, "the timeout", 1, 120));
            endStatement(); return false;
        }
        if (w == "up") {
            next();
            const int pc = emit(Op::CLIMB_BY, L);
            at_(pc).a = float(positive(Unit::LEN, "how far up", 0.1, 50));
            at_(pc).b = 30.f;
            if (acceptWord("timeout")) at_(pc).b = float(positive(Unit::TIME, "the timeout", 1, 300));
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "down") {
            next();
            const int pc = emit(Op::CLIMB_BY, L);
            at_(pc).a = -float(positive(Unit::LEN, "how far down", 0.1, 100));
            at_(pc).b = 30.f;
            if (acceptWord("timeout")) at_(pc).b = float(positive(Unit::TIME, "the timeout", 1, 300));
            warnDown(L, t.col);
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "altitude") {
            // To a height above the ground, up or down.
            next();
            acceptWord("to");
            const int pc = emit(Op::CLIMB, L);
            at_(pc).a = float(positive(Unit::LEN, "the height", 0.3, 200));
            at_(pc).b = 30.f;
            if (acceptWord("timeout")) at_(pc).b = float(positive(Unit::TIME, "the timeout", 1, 300));
            warnDown(L, t.col);
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "track") {
            next();
            const std::string lab = label();
            prog().caps |= Program::NEEDS_DETECTOR;
            const int pc = emit(Op::TRACK, L);
            at_(pc).text = str(lab);
            at_(pc).a = 10.f;
            if (acceptWord("timeout")) at_(pc).a = float(positive(Unit::TIME, "the timeout", 0.5, 120));
            elseBranch(pc);
            endStatement(); return false;
        }
        if (w == "untrack" || w == "release") {
            next();
            emit(Op::UNTRACK, L);
            endStatement(); return false;
        }
        if (w == "say") {
            next();
            if (!at(Tk::STRING)) fail(peek(), "expected a message in quotes");
            const int pc = emit(Op::SAY, L);
            at_(pc).text = str(next().text);
            endStatement(); return false;
        }
        if (w == "land" || w == "rtl" || w == "end") {
            next();
            emit(w == "land" ? Op::LAND : w == "rtl" ? Op::RTL : Op::END, L);
            endStatement(); return true;
        }
        if (w == "resume") {
            next();
            if (!inHandler_) fail(t, "'resume' only means something inside an 'on' handler");
            emit(Op::RESUME, L);
            endStatement(); return true;
        }
        fail(t, "unknown statement '" + w + "'");
    }
};

}  // namespace

std::string CompileResult::report(const std::string& file) const {
    std::ostringstream o;
    for (const Diagnostic& d : diags)
        o << file << ":" << d.line << ":" << d.col << ": " << (d.error ? "error" : "warning")
          << ": " << d.message << "\n";
    return o.str();
}

CompileResult compile(const std::string& source, const std::string& sourceName) {
    CompileResult r;
    Compiler c(source, sourceName, r);
    c.run();
    return r;
}

CompileResult compileFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        CompileResult r;
        r.diags.push_back({0, 0, true, "cannot open " + path});
        return r;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string name = path;
    const size_t s = name.find_last_of("/\\");
    if (s != std::string::npos) name = name.substr(s + 1);
    return compile(ss.str(), name);
}

}  // namespace kms

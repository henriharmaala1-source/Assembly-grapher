#pragma once
// ---------------------------------------------------------------------------
// A COMPILED MISSION -- what the Pi runs, and all it runs.
//
// Missions are written as text (.kms, see mission_compile.hpp and
// docs/mission-scripts.md) and TRANSLATED ON THE MACHINE THEY ARE WRITTEN ON:
// `kestrel mission compile` turns the text into this program and writes it as
// a .kmb file. The Pi never sees the text. It has no parser, no expression
// evaluator over strings, nothing that can meet a typo in the air: it loads a
// flat table of instructions, checks it (loadProgram), and steps through it
// (script_mode.hpp).
//
// Everything that CAN be decided before flight is decided by the compiler:
// names resolved to indices, units converted, patterns (orbit, survey)
// expanded into plain gotos, loops bounded, every jump resolved. What is left
// for the Pi is only what can be known in the air -- where the aircraft is,
// what it sees, how long something took.
//
// The loader still trusts nothing: a .kmb is a file that crossed a cable, so
// every index in it is bounds-checked and every condition's stack depth
// verified before the first instruction runs. A program that passes cannot
// make the interpreter read outside its tables.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>
#include <vector>

namespace kms {

constexpr uint32_t kMagic   = 0x31424D4Bu;   // "KMB1"
constexpr uint32_t kVersion = 6;

enum class Op : uint8_t {
    END = 0,      // finished: hover and report done
    HOLD,         // a = seconds
    GOTO,         // target, a = arrival radius m, b = timeout s, jump = on failure
    TURN_TO,      // a = heading deg (0 = N, cw), b = timeout s
    TURN_BY,      // a = degrees, + = clockwise, b = timeout s
    CLIMB,        // a = altitude m above takeoff, b = timeout s, jump = on failure
    SEARCH,       // text = label, a = timeout s, b = yaw direction (+1/-1), jump = not found
    FACE,         // text = label, a = timeout s, jump = lost
    APPROACH,     // text = label, a = fill fraction of frame height, b = timeout s, jump = lost
    EXPLORE,      // a = seconds of safe travel, no goal
    RUN,          // text = control mode name, a = seconds
    WAIT,         // cond, a = timeout s, jump = on timeout (-1: carry on)
    JUMP,         // jump
    JUMP_IFNOT,   // cond, jump
    SET_REG,      // reg, a = value
    LOOP,         // reg: decrement, and jump while it is still > 0
    TIMER,        // reg := now
    JUMP_TIMEUP,  // reg, a = seconds: jump once that long has passed since TIMER
    MARK,         // target := where the aircraft is now (reference: its heading)
    MARK_SEEN,    // target := where the object `text` is (reference: the line
                  // of sight to it), from its box in the image and its range;
                  // jump = not seen, or no range to be had
    SAY,          // text
    LAND,         // hand the FC its LAND; the program ends
    RTL,          // hand the FC its return-to-launch; the program ends
    RESUME,       // end of an `on` handler: go back to what was interrupted
    GO_STATE,     // target = state: enter it (its transitions become live)
    TRACK,        // text = label, a = timeout s, jump = never locked: hand the
                  // best detection's box to the lightweight tracker and wait
                  // for it to lock -- from then on the tracker, not the
                  // detector, says where the object is, every frame
    UNTRACK,      // release the tracker
    TURN_REF,     // target: turn back to that place's reference heading, b = timeout
    CLIMB_BY,     // a = metres up from where this starts, b = timeout, jump = on failure
    APPROACH_TO,  // text = label, a = stop at this range m, b = timeout s, jump = lost
    NAV,          // a = 0 certified (every leg checked by the planner), 1 direct
                  // (straight at the target on heading + pitch, NOTHING checked:
                  // outdoors, above the obstacles, inside the fence)
    // A PATH ANCHORED ON A TRACKED OBJECT (`path door { to ahead A right R }`).
    ANCHOR,       // target (RUNTIME) := where `text` is, as MARK_SEEN -- and from
                  // now until UNANCHOR, RE-MEASURED every tick it is in view with
                  // a range, so the path's points move with a better estimate.
                  // Its reference heading (the line of sight at the start)
                  // stays fixed, so "ahead" and "right" do not swing as the
                  // aircraft moves round it. jump = not seen / no range.
    UNANCHOR,     // stop re-measuring it
    TURN_TO_PLACE,// target: face that place, b = timeout s
    // STEER ON THE LOCK, in the image: aim at a point on the object's box
    // (a = box widths right of its centre, - = left), fly at b m/s, until
    // `cond` holds (or c seconds, which fails). FLAG_STRAFE: keep the aim
    // point centred by rolling sideways instead of yawing. Unchecked by any
    // planner, like `nav direct`. jump = lost it.
    STEER,
    COUNT_
};

const char* opName(Op o);

// A place. Positions are LOCAL ENU metres from where the mission started, the
// frame WorldState::estPe/estPn are in -- whichever source is filling them
// (FC flow, VIO, SLAM, GPS through the Pi estimator).
struct Target {
    enum Kind : uint8_t {
        START = 0,   // where GO was pressed; its reference heading is the
                     // heading the aircraft had then
        ENU,         // x east, y north of the start
        GPS,         // x latitude, y longitude (needs a fix at the start)
        OFFSET,      // base + x east, y north
        REL,         // base + x AHEAD, y RIGHT along the base's reference
                     // heading: for START that is the heading at GO (indoors,
                     // "12 m ahead" means something and "12 m east" does not);
                     // for a place marked from a detection it is the LINE OF
                     // SIGHT to the object, so "ahead 3" is 3 m beyond it
        RUNTIME,     // set in the air: MARK (here) or MARK_SEEN (an object)
    };
    Kind     kind = START;
    double   x = 0, y = 0;
    int32_t  base = -1;      // OFFSET: the target it is relative to
    int32_t  name = -1;      // strings index, -1 for an anonymous point
};

// Conditions are postfix: atoms push a truth value, AND/OR/NOT combine them.
// The loader checks every condition leaves exactly one value.
struct CondOp {
    enum Kind : uint8_t { TRUE_ = 0, SEEN, VAR, DIST, POSITIONED, NOT, AND, OR,
                          TRACKING,     // arg = label: the tracker locked on it
                          RANGE };      // arg = label: its estimated range vs value
    enum Var  : uint8_t { BATTERY = 0, ALT, TIME, SPEED, HEADING };
    enum Cmp  : uint8_t { LT = 0, LE, GT, GE };
    Kind    kind = TRUE_;
    uint8_t var = 0, cmp = 0;   // SEEN: var = FLAG_NEW for `seen new LABEL`
    int32_t arg = -1;        // SEEN: label string; DIST: target
    float   value = 0.f;     // VAR/DIST: compared against
};
struct CondRange { int32_t start = 0, count = 0; };

struct Instr {
    Op      op = Op::END;
    int32_t line = 0;        // source line, for telemetry and errors
    int32_t target = -1;     // target, or register for SET_REG/LOOP/TIMER/JUMP_TIMEUP
    int32_t jump = -1;       // pc to go to (JUMP, failures); -1 = none
    int32_t cond = -1;       // condition index
    int32_t text = -1;       // strings index
    float   a = 0.f, b = 0.f, c = 0.f;
    // FLAG_NEW (search / seen / mark): only objects NOT already visited --
    // whose ground position is not within newRadiusM of a place this mission
    // already marked for that label. "The next one", for any chain of them.
    uint8_t flags = 0;
};
constexpr uint8_t FLAG_NEW = 1;
constexpr uint8_t FLAG_STRAFE = 2;    // STEER: roll to it, do not yaw

// `on COND { ... }`: checked every tick while the main program runs; fires
// once. Its body ends in LAND/RTL/END, or RESUME back to what it interrupted.
struct Handler { int32_t cond = -1, pc = -1, line = 0; };

// A STATE MACHINE on top of the instructions. A state is a named entry point;
// while it is current its transitions are checked every tick, and the first
// that holds switches state -- abandoning whatever the state was doing. That
// is the "object seen -> behaviour" trigger: the behaviour is the state's
// code, the trigger its `when` line.
struct Transition { int32_t cond = -1, to = -1, line = 0; };
struct State {
    int32_t name = -1;               // strings index
    int32_t pc = -1;                 // its first instruction
    int32_t transStart = 0, transCount = 0;
};

// HOW BIG A THING IS, for range from its box: distance = f * height / box
// height in pixels. Declared with `size door 2.0 m`; the compiler fills in
// common labels it was not told about (mission_compile.cpp) and says so.
// `assumed`: a typical height the compiler filled in, not one the mission
// declared -- the runtime then prefers the ground plane over it.
struct ObjectSize { int32_t label = -1; float heightM = 0.f; uint8_t assumed = 0; };

struct Program {
    enum Cap : uint32_t {
        NEEDS_POSITION = 1u,   // goto/over/orbit/survey/dist/fence
        NEEDS_DETECTOR = 2u,   // search/face/approach/seen
        NEEDS_GPS      = 4u,   // a gps(...) target
        NEEDS_RANGE    = 8u,   // a place taken from a detection
    };
    std::string name;
    std::string sourceName;          // the .kms it came from
    uint32_t    sourceHash = 0;      // CRC32 of its text: which version this is
    uint32_t    caps = 0;
    float       fenceM = 50.f;       // never further than this from the start
    float       timeoutS = 600.f;    // the whole mission
    int32_t     registers = 0;
    std::vector<std::string> strings;
    std::vector<Target>      targets;
    std::vector<CondOp>      condOps;
    std::vector<CondRange>   conds;
    std::vector<Handler>     handlers;
    std::vector<State>       states;
    std::vector<Transition>  transitions;
    std::vector<ObjectSize>  sizes;
    std::vector<Instr>       code;
};

// Binary form. save/load round-trip exactly; load verifies magic, version,
// checksum and every index (see the header comment) and says what failed.
std::vector<uint8_t> serialize(const Program& p);
bool deserialize(const std::vector<uint8_t>& bytes, Program& out, std::string* err);
bool saveProgram(const Program& p, const std::string& path, std::string* err);
bool loadProgram(const std::string& path, Program& out, std::string* err);
// The structural checks on their own: what the loader runs after decoding.
bool verify(const Program& p, std::string* err);

// A readable listing, one instruction a line with its source line --
// `kestrel mission check` prints it so what will fly can be read.
std::string disassemble(const Program& p);

uint32_t crc32(const uint8_t* data, size_t n);

}  // namespace kms

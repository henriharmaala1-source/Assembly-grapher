// MISSION SCRIPTS end to end: text -> compiler -> .kmb bytes -> loader ->
// SCRIPT mode flying a point-mass aircraft through the REAL MissionController.
//
//   compile      a good program compiles; mistakes are errors with a line
//   loader       a damaged or tampered .kmb is refused, never interpreted
//   goto         a place 10 m away is reached and the program moves on
//   seen         `let p = seen person`: a box low in the image, at altitude,
//                becomes a ground position on the right bearing at the right
//                range -- and a path written relative to it is flown
//   handlers     battery low fires `on`, which hands RTL to the FC
//   failure      a goto that cannot finish takes its else; without one the
//                program STOPS (hovers), it does not carry on
//   fence        leaving the fence stops it
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "control_mode.hpp"
#include "mission_compile.hpp"
#include "mission_program.hpp"
#include "script_mode.hpp"

static int fails = 0;
static void check(const char* what, bool ok, const std::string& got = "") {
    std::printf("  %-60s %s  %s\n", what, ok ? "ok  " : "FAIL", got.c_str());
    if (!ok) ++fails;
}

static kms::Program compileOk(const std::string& src) {
    kms::CompileResult r = kms::compile(src, "test.kms");
    if (!r.ok) std::printf("%s", r.report("test.kms").c_str());
    // Through the bytes, as the Pi gets it.
    kms::Program p;
    std::string err;
    if (!kms::deserialize(kms::serialize(r.program), p, &err)) std::printf("load: %s\n", err.c_str());
    return p;
}

// A point-mass aircraft: pitch -> forward speed (lagged), yaw -> yaw rate. The
// corridor is always open, the estimate is the truth.
struct Sim {
    WorldState s;
    float v = 0.f;
    double t = 0;
    void init() {
        s.estValid = true; s.estEphM = 0.5f;
        s.vehAltM = 5.f; s.vehBattery = 0.9f; s.vehLink = true; s.vehArmed = true;
        s.corridorValid = true; s.corridorOpen = 1.f; s.corridorOffset = 0.f;
        s.missionGo = true;
    }
    void step(ScriptMode& m, int frameW = 640, int frameH = 480) {
        const float dt = 0.05f;
        t += dt;
        s.tickMonoS = t; s.corridorStampS = t;
        ControlCtx c; c.dt = dt; c.frameW = frameW; c.frameH = frameH;
        const ControlCmd cmd = m.update(s, c);
        if (cmd.valid) {
            s.vehYawDeg += cmd.yaw * 90.f * dt;
            while (s.vehYawDeg >= 360.f) s.vehYawDeg -= 360.f;
            while (s.vehYawDeg < 0.f) s.vehYawDeg += 360.f;
            v += (cmd.pitch * 4.f - v) * dt / 0.35f;
            s.vehAltM += cmd.throttle * 1.f * dt;
        } else {
            v *= 0.9f;
        }
        const float a = s.vehYawDeg * 3.14159265f / 180.f;
        s.estPe += std::sin(a) * v * dt; s.estPn += std::cos(a) * v * dt;
        s.estSpeed = std::fabs(v); s.vehGroundspeed = std::fabs(v);
    }
};

static ScriptMode::Params params() {
    ScriptMode::Params p;
    p.mission.useMap = false;          // corridor only: open everywhere
    p.mission.settleSec = 0.3f;
    p.detHfovDeg = 60.f;
    p.detTiltDeg = -30.f;              // looking down 30 deg
    return p;
}

int main() {
    std::printf("mission scripts\n");

    // ------------------------------------------------------------ compile
    {
        kms::CompileResult r = kms::compile(
            "fence 30 m\nlet a = enu(40, 0)\ngoto b\nclimb 3 s\nresume\n", "bad.kms");
        int errs = 0;
        for (const auto& d : r.diags) errs += d.error;
        check("mistakes are errors, each with its line", !r.ok && errs == 4,
              std::to_string(errs) + " errors");
        const std::string rep = r.report("bad.kms");
        check("  ...and the fence one names the place and distance",
              rep.find("bad.kms:2:") != std::string::npos && rep.find("40.0 m") != std::string::npos);
    }

    // ------------------------------------------------------------- loader
    {
        kms::CompileResult r = kms::compile("let a = enu(5, 0)\ngoto a\nland\n", "t");
        std::vector<uint8_t> b = kms::serialize(r.program);
        kms::Program p; std::string err;
        check("a compiled program loads back", kms::deserialize(b, p, &err), err);
        std::vector<uint8_t> bad = b; bad[bad.size() / 2] ^= 0x40;
        check("one flipped bit is refused (checksum)", !kms::deserialize(bad, p, &err), err);
        // Tampered AND re-checksummed: an out-of-range jump must still be caught.
        kms::Program evil = r.program;
        evil.code[0].jump = 999;
        check("an out-of-range jump is refused even with a valid checksum",
              !kms::deserialize(kms::serialize(evil), p, &err), err);
        std::vector<uint8_t> trunc(b.begin(), b.begin() + long(b.size() - 3));
        check("a truncated file is refused", !kms::deserialize(trunc, p, &err), err);
    }

    // --------------------------------------------------------------- goto
    {
        ScriptMode m(params());
        std::string err;
        m.load(compileOk("let a = enu(10, 0)\ngoto a radius 1 m\nhold 1 s\nend\n"), &err);
        Sim sim; sim.init();
        sim.s.vehYawDeg = 0.f;            // facing north; the place is east
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 20 * 120 && !m.finished(); ++i) sim.step(m);
        const double d = std::hypot(sim.s.estPe - 10.0, sim.s.estPn);
        char b[96]; std::snprintf(b, sizeof b, "%.2f m off, %.0f s, %s", d, sim.t, m.status().c_str());
        check("goto reaches a place 10 m away (turns to it first)", m.finished() && !m.failed() && d < 1.6, b);
    }

    // --------------------------------------------- seen -> a path around it
    {
        ScriptMode m(params());
        std::string err;
        const bool ok = m.load(compileOk(
            "let p = seen person else { end }\n"
            "let past = p + (ahead 2)\n"
            "goto past radius 1 m\n"
            "end\n"), &err);
        check("a detection-relative program loads", ok, err);
        Sim sim; sim.init();
        sim.s.vehYawDeg = 90.f;           // facing east
        // A person: box centred horizontally at +80 px, bottom edge at row 400.
        // f = 320/tan(30) = 554 px; bottom is 160 px below centre = 16.1 deg
        // below the optical axis, which itself is 30 deg down: 46.1 deg below
        // the horizon at 5 m altitude -> 4.81 m out; bearing 90 + atan(80/554).
        Detection d; d.label = "person"; d.confidence = 0.9f;
        d.box = cv::Rect(360, 300, 80, 100);
        sim.s.detections = {d};
        sim.s.detStampS = 0.0;
        m.onEnter(sim.s); sim.s.missionGo = true;
        sim.s.detStampS = 0.05;
        sim.step(m);
        double pe = 0, pn = 0;
        const bool marked = m.targetPos(1, pe, pn);
        const double brg = 90.0 + std::atan(80.0 / 554.26) * 180.0 / 3.14159265;
        const double dep = 30.0 + std::atan(160.0 / 554.26) * 180.0 / 3.14159265;
        const double rng = 5.0 / std::tan(dep * 3.14159265 / 180.0);
        const double we = rng * std::sin(brg * 3.14159265 / 180.0);
        const double wn = rng * std::cos(brg * 3.14159265 / 180.0);
        char b[128];
        std::snprintf(b, sizeof b, "at (%.2f, %.2f), want (%.2f, %.2f)", pe, pn, we, wn);
        check("the box becomes a ground position (bearing + altitude range)",
              marked && std::hypot(pe - we, pn - wn) < 0.05, b);
        sim.s.detections.clear();
        for (int i = 0; i < 20 * 120 && !m.finished(); ++i) sim.step(m);
        const double wantE = we + 2.0 * std::sin(brg * 3.14159265 / 180.0);
        const double wantN = wn + 2.0 * std::cos(brg * 3.14159265 / 180.0);
        const double off = std::hypot(sim.s.estPe - wantE, sim.s.estPn - wantN);
        std::snprintf(b, sizeof b, "%.2f m off, %s", off, m.status().c_str());
        check("  ...and `p + (ahead 2)` -- past it on the line of sight -- is flown to",
              m.finished() && !m.failed() && off < 1.6, b);
    }

    // ---------------------------------------------- not seen -> its else
    {
        ScriptMode m(params());
        std::string err;
        m.load(compileOk("let p = seen person else { say \"none\"; rtl }\nend\n"), &err);
        Sim sim; sim.init();
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 10 && !m.finished(); ++i) sim.step(m);
        check("nothing in view: its else runs (here: rtl to the FC)",
              m.finished() && sim.s.fcRequest == WorldState::FcRequest::RTL, m.status());
    }

    // ----------------------------------------------------------- handlers
    {
        ScriptMode m(params());
        std::string err;
        m.load(compileOk("on battery < 25% { rtl }\nhold 60 s\nend\n"), &err);
        Sim sim; sim.init();
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 40; ++i) sim.step(m);
        const bool early = !m.finished();
        sim.s.vehBattery = 0.2f;
        for (int i = 0; i < 5; ++i) sim.step(m);
        check("`on battery < 25%` fires mid-hold and hands RTL to the FC",
              early && m.finished() && sim.s.fcRequest == WorldState::FcRequest::RTL, m.status());
        // ...and the manager turns that into its RTH path.
        ModeManager mm;
        auto* sm = new ScriptMode(params());
        sm->load(compileOk("rtl\n"), &err);
        mm.add(std::unique_ptr<IControlMode>(sm));
        WorldState w; w.missionGo = false;
        mm.select("SCRIPT", w);
        w.missionGo = true; w.estValid = true;
        ControlCtx c; c.dt = 0.05f;
        bool rth = false;
        mm.tick(w, c, rth);
        check("  ...which the ModeManager turns into its RTH trigger", rth);
    }

    // ------------------------------------------------------------ failure
    {
        ScriptMode m(params());
        std::string err;
        m.load(compileOk("let a = enu(10, 0)\ngoto a timeout 2 s else { say \"too slow\"; end }\nend\n"), &err);
        Sim sim; sim.init();
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 20 * 5 && !m.finished(); ++i) sim.step(m);
        check("a goto that times out takes its else", m.finished() && !m.failed(), m.status());

        ScriptMode m2(params());
        m2.load(compileOk("let a = enu(10, 0)\ngoto a timeout 2 s\nhold 30 s\nland\n"), &err);
        Sim sim2; sim2.init();
        m2.onEnter(sim2.s); sim2.s.missionGo = true;
        for (int i = 0; i < 20 * 5 && !m2.finished(); ++i) sim2.step(m2);
        check("without an else it STOPS -- does not carry on to land",
              m2.finished() && m2.failed() && sim2.s.fcRequest == WorldState::FcRequest::NONE,
              m2.status());
    }

    // -------------------------------------------------------------- fence
    {
        ScriptMode m(params());
        std::string err;
        m.load(compileOk("fence 5 m\nexplore 60 s\nend\n"), &err);
        Sim sim; sim.init();
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 20 * 60 && !m.finished(); ++i) sim.step(m);
        const double d = std::hypot(sim.s.estPe, sim.s.estPn);
        char b[96]; std::snprintf(b, sizeof b, "%.1f m out: %s", d, m.status().c_str());
        check("exploring past the fence stops it", m.finished() && m.failed() && d < 7.0, b);
    }

    // ----------------------------------------------------- nothing before GO
    {
        ScriptMode m(params());
        std::string err;
        m.load(compileOk("let a = enu(10, 0)\ngoto a\nend\n"), &err);
        Sim sim; sim.init();
        m.onEnter(sim.s);                 // GO is off
        for (int i = 0; i < 100; ++i) sim.step(m);
        check("before GO it only hovers", std::hypot(sim.s.estPe, sim.s.estPn) < 0.01 && !m.finished(),
              m.status());
    }

    // ------------------------------------------------- move left, keep heading
    {
        ScriptMode m(params());
        std::string err;
        m.load(compileOk("move right 2 m\nmove forward 3 m\nend\n"), &err);
        Sim sim; sim.init();
        sim.s.vehYawDeg = 0.f;
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 20 * 120 && !m.finished(); ++i) sim.step(m);
        char b[128];
        std::snprintf(b, sizeof b, "at (%.2f, %.2f) heading %.0f, %s", sim.s.estPe, sim.s.estPn,
                      sim.s.vehYawDeg, m.status().c_str());
        const double hd = std::fabs(std::remainder(sim.s.vehYawDeg, 360.0));
        check("`move right 2 m; move forward 3 m` ends 2 E 3 N, facing north again",
              m.finished() && !m.failed() && std::hypot(sim.s.estPe - 2, sim.s.estPn - 3) < 0.9 &&
                  hd < 6, b);
    }

    // ------------------------------- THE DOOR: detect -> track -> state -> behaviour
    // The detector (YOLO) runs every 10th tick; the tracker every tick once a
    // script hands it a box. A state machine patrols, a `when` trigger fires
    // on the door, the tracker takes over, and the behaviour flies.
    {
        const char* src =
            "mission \"door\"\n"
            "state patrol {\n"
            "  when seen door -> acquire\n"
            "  yaw right 90\n"
            "  hold 1 s\n"
            "  -> patrol\n"
            "}\n"
            "state acquire {\n"
            "  track door timeout 5 s else { -> patrol }\n"
            "  -> go_through\n"
            "}\n"
            // A trigger belongs to the state where it MATTERS: losing the
            // door while approaching it means start again; losing it while
            // stepping aside (which turns the camera away) is the plan.
            "state go_through {\n"
            "  when not tracking door -> patrol\n"
            "  approach door fill 0.5 timeout 60 s\n"
            "  -> step_aside\n"
            "}\n"
            "state step_aside {\n"
            "  untrack\n"
            "  move left 1 m\n"
            "  land\n"
            "}\n";
        kms::CompileResult cr = kms::compile(src, "door.kms");
        check("the door state machine compiles", cr.ok, cr.report("door.kms"));
        ScriptMode m(params());
        std::string err;
        m.load(compileOk(src), &err);
        Sim sim; sim.init();
        sim.s.vehYawDeg = 0.f;
        const double doorE = 8.0, doorN = 0.5;     // east of the start: patrol must turn to it
        const int W = 640, H = 480;
        const double f = 320.0 / std::tan(30.0 * 3.14159265 / 180.0);
        int detTicks = 0, trackerLocks = 0, lastReq = 0;
        bool locked = false, sawTracking = false;
        std::vector<std::string> states;
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 20 * 180 && !m.finished(); ++i) {
            // Where the door is in the image, if at all.
            const double de = doorE - sim.s.estPe, dn = doorN - sim.s.estPn;
            const double dist = std::hypot(de, dn);
            double off = std::atan2(de, dn) * 180.0 / 3.14159265 - sim.s.vehYawDeg;
            while (off > 180) off -= 360;
            while (off <= -180) off += 360;
            const bool inView = std::fabs(off) < 28.0;
            cv::Rect box;
            if (inView) {
                const int cx = int(W / 2 + f * std::tan(off * 3.14159265 / 180.0));
                const int h = int(std::min(470.0, H / dist)), w = h / 2;
                box = cv::Rect(cx - w / 2, H / 2 - h / 2, w, h);
            }
            // THE DETECTOR: slow.
            if (i % 10 == 0) {
                ++detTicks;
                sim.s.detections.clear();
                if (inView) {
                    Detection d; d.label = "door"; d.confidence = 0.8f; d.box = box;
                    sim.s.detections.push_back(d);
                }
                sim.s.detStampS = sim.t;
            }
            // THE TRACKER: locks on a handed box, then follows every tick.
            if (sim.s.trackRequestSeq != lastReq) { lastReq = sim.s.trackRequestSeq; locked = true; ++trackerLocks; }
            sim.s.targetValid = sim.s.targetLocked = locked && inView;
            if (locked && inView) {
                sim.s.targetBox = box;
                sim.s.targetStampS = sim.t;
                sim.s.targetFixAgeS = 0.f;
            }
            sawTracking |= m.stateName() == "go_through";
            if ((states.empty() || states.back() != m.stateName()) && states.size() < 12)
                states.push_back(m.stateName());
            sim.step(m, W, H);
        }
        std::string path;
        for (const auto& st : states) path += (path.empty() ? "" : " > ") + st;
        check("patrol -> acquire -> go_through -> step_aside, once",
              path == "patrol > acquire > go_through > step_aside", path);
        check("the detector handed off ONCE; the tracker did the rest",
              trackerLocks == 1 && sawTracking, std::to_string(trackerLocks) + " lock(s)");
        const double dE = doorE - sim.s.estPe, dN = doorN - sim.s.estPn;
        char b[160];
        std::snprintf(b, sizeof b, "%.2f m from the door, %s", std::hypot(dE, dN), m.status().c_str());
        check("it approached to fill 0.5 (2 m), stepped left, and handed LAND to the FC",
              m.finished() && sim.s.fcRequest == WorldState::FcRequest::LAND &&
                  std::hypot(dE, dN) > 1.4 && std::hypot(dE, dN) < 3.2, b);
    }

    // ------------------------------------------------ RANGE FROM ITS SIZE
    // No depth: `size person 1.7 m` and a 94 px tall box at the optical
    // centre is f * 1.7 / 94 = 10.0 m away, on the heading.
    {
        ScriptMode::Params pp = params();
        pp.detTiltDeg = 0.f;
        ScriptMode m(pp);
        std::string err;
        m.load(compileOk("size person 1.7 m\nlet p = seen person else { end }\nend\n"), &err);
        Sim sim; sim.init();
        sim.s.vehYawDeg = 90.f;
        Detection d; d.label = "person"; d.confidence = 0.9f;
        d.box = cv::Rect(320 - 20, 240 - 47, 40, 94);
        sim.s.detections = {d};
        m.onEnter(sim.s); sim.s.missionGo = true;
        sim.s.detStampS = 0.05;
        sim.step(m);
        double pe = 0, pn = 0;
        const bool ok = m.targetPos(1, pe, pn);
        const double want = 554.256 * 1.7 / 94.0;
        char b[96]; std::snprintf(b, sizeof b, "at (%.2f, %.2f), want (%.2f, 0)", pe, pn, want);
        check("`size person 1.7 m`: a 94 px box is placed 10.0 m out on its bearing",
              ok && std::fabs(pe - want) < 0.05 && std::fabs(pn) < 0.05, b);
    }

    // ------------------- RANGE THROUGH THE TRACKER: detector once, scale after
    // The detector sees the door ONCE and is never run again. The tracker's
    // box is square and its own size, but it SCALES with the door -- so the
    // range at hand-off, carried by that scale, says when it is 3 m away.
    for (int scales = 1; scales >= 0; --scales) {
        ScriptMode::Params pp = params();
        pp.detTiltDeg = 0.f;
        ScriptMode m(pp);
        std::string err;
        m.load(compileOk("size door 2.0 m\n"
                         "track door timeout 5 s else { end }\n"
                         "approach door to 3 m timeout 60 s else { say \"no range\"; end }\n"
                         "land\n"), &err);
        Sim sim; sim.init();
        sim.s.vehYawDeg = 90.f;
        const double doorE = 9.0, f = 554.256;
        int lastReq = 0; bool locked = false;
        sim.s.targetCore = scales ? "fused" : "mosse";
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 20 * 90 && !m.finished(); ++i) {
            const double dist = doorE - sim.s.estPe;
            const int h = int(f * 2.0 / dist);
            const cv::Rect det(320 - h / 4, 240 - h / 2, h / 2, h);
            // THE DETECTOR: only in the first half second.
            sim.s.detections.clear();
            if (i < 10) {
                Detection d; d.label = "door"; d.confidence = 0.9f; d.box = det;
                sim.s.detections.push_back(d);
                sim.s.detStampS = sim.t;
            }
            if (sim.s.trackRequestSeq != lastReq) { lastReq = sim.s.trackRequestSeq; locked = true; }
            sim.s.targetValid = sim.s.targetLocked = locked;
            if (locked) {
                // Square, and NOT the door's size -- a scaling core keeps its
                // CHANGE right; a fixed one does not change at all.
                const int side = scales ? int(150.0 * 4.0 / dist) : 60;
                sim.s.targetBox = cv::Rect(320 - side / 2, 240 - side / 2, side, side);
                sim.s.targetStampS = sim.t; sim.s.targetFixAgeS = 0.f;
            }
            sim.step(m, 640, 480);
        }
        const double dist = doorE - sim.s.estPe;
        char b[128]; std::snprintf(b, sizeof b, "stopped %.2f m from it: %s", dist, m.status().c_str());
        if (scales)
            check("a SCALING tracker carries the range: `approach door to 3 m` stops at 3 m",
                  sim.s.fcRequest == WorldState::FcRequest::LAND && std::fabs(dist - 3.0) < 0.5, b);
        else
            check("  ...a fixed-size tracker cannot, and it says so instead of guessing",
                  m.finished() && sim.s.fcRequest == WorldState::FcRequest::NONE && dist > 3.4, b);
    }

    // ------------------------------- LIGHTPOLES: no voxel map, pole after pole
    // missions/lightpoles.kms against three poles in a line north of the
    // start. The "detector" sees a pole only from the camera geometry: in
    // front, inside the FoV, within 40 m, its box's foot where the ground
    // is at that range from 12 m up with the camera 30 deg down.
    {
        ScriptMode m(params());
        std::string err;
        const kms::CompileResult cr = kms::compileFile(std::string(KESTREL_MISSIONS_DIR) + "/lightpoles.kms");
        check("missions/lightpoles.kms compiles (with its `nav direct` warning)", cr.ok,
              cr.report("lightpoles.kms"));
        kms::Program pr;
        kms::deserialize(kms::serialize(cr.program), pr, &err);
        m.load(pr, &err);
        Sim sim; sim.init();
        sim.s.vehYawDeg = 0.f;
        const double poles[3][2] = {{1.0, 15.0}, {-1.5, 32.0}, {0.5, 50.0}};
        double best[3] = {1e9, 1e9, 1e9};
        const double f = 554.256, tilt = 30.0, W = 640, H = 480;
        m.onEnter(sim.s); sim.s.missionGo = true;
        for (int i = 0; i < 20 * 400 && !m.finished(); ++i) {
            sim.s.detections.clear();
            for (const auto& p : poles) {
                const double de = p[0] - sim.s.estPe, dn = p[1] - sim.s.estPn;
                const double d = std::hypot(de, dn);
                double off = std::atan2(de, dn) * 57.2958 - sim.s.vehYawDeg;
                while (off > 180) off -= 360;
                while (off <= -180) off += 360;
                if (d < 1.0 || d > 40.0 || std::fabs(off) > 28.0) continue;
                const double alt = sim.s.vehAltM;
                const double depFoot = std::atan2(alt, d) * 57.2958;          // below horizon
                const double depTop  = std::atan2(alt - 6.0, d) * 57.2958;
                const double yFoot = H / 2 + f * std::tan((depFoot - tilt) / 57.2958);
                const double yTop  = H / 2 + f * std::tan((depTop - tilt) / 57.2958);
                if (yFoot > H - 1 || yFoot < 0) continue;                  // foot out of view
                const double cx = W / 2 + f * std::tan(off / 57.2958);
                Detection dd; dd.label = "lightpole"; dd.confidence = 0.8f;
                const int top = int(std::max(0.0, yTop));
                dd.box = cv::Rect(int(cx) - 4, top, 8, std::max(4, int(yFoot) - top));
                sim.s.detections.push_back(dd);
            }
            sim.s.detStampS = sim.t;
            if (m.status().find("hold") != std::string::npos)
                for (int k = 0; k < 3; ++k)
                    best[k] = std::min(best[k], std::hypot(poles[k][0] - sim.s.estPe,
                                                           poles[k][1] - sim.s.estPn));
            sim.step(m, 640, 480);
        }
        char b[160];
        std::snprintf(b, sizeof b, "hovered %.2f / %.2f / %.2f m from them; %s", best[0], best[1],
                      best[2], m.status().c_str());
        check("over each lightpole in turn, then home (no voxel map, no depth)",
              best[0] < 1.6 && best[1] < 1.6 && best[2] < 1.6 &&
                  sim.s.fcRequest == WorldState::FcRequest::RTL, b);
    }

    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}

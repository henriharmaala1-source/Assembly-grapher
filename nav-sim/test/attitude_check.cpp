// Gravity levels; the gyro carries between levellings. Both halves, and the
// gate between them, pinned here -- this is pure arithmetic, so it is the one
// piece of the live IMU path that can be verified without the camera.
#include <cmath>
#include <cstdio>
#include <string>

#include "attitude_filter.hpp"

using namespace sim;

static int failures = 0;
static void check(const char* what, bool ok, const std::string& d = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++failures;
}
static std::string f2(float v) { char b[48]; std::snprintf(b, sizeof b, "%.2f", v); return b; }

// Gravity as seen in camera axes at a given roll/pitch. Level = (0, g, 0).
static void gravityAt(float rollDeg, float pitchDeg, float& ax, float& ay, float& az) {
    const float r = rollDeg * 3.14159265f / 180.f, p = pitchDeg * 3.14159265f / 180.f;
    const float g = 9.81f;
    ax =  g * std::sin(r) * std::cos(p);
    ay =  g * std::cos(r) * std::cos(p);
    az = -g * std::sin(p);
}

int main() {
    std::printf("attitude from an IMU\n");
    AttitudeParams ap;

    {   // level
        AttitudeFilter f; f.init(ap);
        float ax, ay, az; gravityAt(0.f, 0.f, ax, ay, az);
        f.seed(ax, ay, az);
        check("level accelerometer reads zero roll and pitch",
              std::fabs(f.rollDeg()) < 0.1f && std::fabs(f.pitchDeg()) < 0.1f,
              "r " + f2(f.rollDeg()) + "  p " + f2(f.pitchDeg()));
    }
    {   // a static tilt is recovered, both axes, both signs
        const float cases[][2] = {{20.f, 0.f}, {-15.f, 0.f}, {0.f, 25.f}, {0.f, -30.f}, {12.f, 18.f}};
        bool all = true; std::string worst;
        for (const auto& c : cases) {
            AttitudeFilter f; f.init(ap);
            float ax, ay, az; gravityAt(c[0], c[1], ax, ay, az);
            f.seed(ax, ay, az);
            const float er = std::fabs(f.rollDeg() - c[0]), ep = std::fabs(f.pitchDeg() - c[1]);
            if (er > 0.5f || ep > 0.5f) { all = false; worst = f2(er) + "/" + f2(ep); }
        }
        check("a static tilt is recovered on both axes", all, worst);
    }
    {   // pure rotation with the accelerometer held level: the gyro must carry,
        // and the accelerometer must pull it back -- so the reading lands
        // between the two, nearer the truth the accelerometer reports.
        AttitudeFilter f; f.init(ap);
        float ax, ay, az; gravityAt(0.f, 0.f, ax, ay, az);
        f.seed(ax, ay, az);
        for (int i = 0; i < 200; ++i) f.update(0.f, 1.0f, 0.f, ax, ay, az, 0.005f);
        check("yaw integrates from the gyro alone",
              std::fabs(f.yawDeg() - 57.3f) < 2.f, f2(f.yawDeg()) + " deg after 1 rad");
        check("and gravity holds roll and pitch level meanwhile",
              std::fabs(f.rollDeg()) < 1.f && std::fabs(f.pitchDeg()) < 1.f,
              "r " + f2(f.rollDeg()) + "  p " + f2(f.pitchDeg()));
    }
    {   // THE ONE THAT MATTERS. Under sustained lateral acceleration the
        // accelerometer is not a level, and a filter that believes it will
        // report a tilt the vehicle does not have.
        AttitudeFilter f; f.init(ap);
        float ax, ay, az; gravityAt(0.f, 0.f, ax, ay, az);
        f.seed(ax, ay, az);
        // 4 m/s^2 sideways for a second, vehicle genuinely level, gyro silent.
        for (int i = 0; i < 200; ++i) f.update(0.f, 0.f, 0.f, ax + 4.f, ay, az, 0.005f);
        check("a manoeuvre does not tilt the horizon", std::fabs(f.rollDeg()) < 2.f,
              "roll " + f2(f.rollDeg()) + " deg under 4 m/s^2");
        check("and the filter says it stopped trusting the accelerometer",
              !f.accelTrusted());
        // and it recovers once the manoeuvre ends
        for (int i = 0; i < 400; ++i) f.update(0.f, 0.f, 0.f, ax, ay, az, 0.005f);
        check("it levels again afterwards", std::fabs(f.rollDeg()) < 0.5f,
              "roll " + f2(f.rollDeg()));
    }
    {   // an accelerometer-only filter would have failed the case above
        AttitudeParams loose = ap;
        loose.accelGateMs2 = 100.f;                       // magnitude gate off
        loose.disagreeFullDeg = loose.disagreeNoneDeg = 1e3f;  // and disagreement off
        AttitudeFilter f; f.init(loose);
        float ax, ay, az; gravityAt(0.f, 0.f, ax, ay, az);
        f.seed(ax, ay, az);
        for (int i = 0; i < 200; ++i) f.update(0.f, 0.f, 0.f, ax + 4.f, ay, az, 0.005f);
        check("WITHOUT the gates, the same manoeuvre tilts it badly",
              std::fabs(f.rollDeg()) > 15.f, "roll " + f2(f.rollDeg()) + " deg");
    }

    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}

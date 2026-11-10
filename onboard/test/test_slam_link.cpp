// SlamLink / SlamClient, and -- when it is built -- ORB-SLAM3 itself on
// rendered stereo IR.
//
// Part 1 (always): a fake SLAM server on a local socket. Pins the client's
// contract: frames are delivered newest-first with at most one in flight,
// a replaced frame's IMU samples are carried into the next one (never lost),
// replies come back matched by sequence, and a server restart is survived.
//
// Part 2 (KESTREL_ORBSLAM=<kestrel-orbslam> KESTREL_ORBVOC=<ORBvoc.txt>): starts
// the real bridge, flies known trajectories through navcore's band-limited IR
// render -- a LEFT and a RIGHT image, 50 mm apart, as the D435i's pair -- and
// scores ORB-SLAM3's stereo trajectory against truth after anchoring it the
// way VoxelNavModule does (slam_anchor.hpp). Skipped, and said so, otherwise.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "depth_camera.hpp"
#include "emitter_gate.hpp"
#include "imu_sync.hpp"
#include "slam_anchor.hpp"
#include "slam_client.hpp"
#include "slam_link.hpp"
#include "voxel_world.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// A fake SLAM: answers each frame after `delayMs`, echoing the frame's time as
// x-translation, and counts the IMU samples it was given.
struct FakeServer {
    std::string path;
    int delayMs = 50;
    std::atomic<long> frames{0}, imu{0};
    std::atomic<bool> stop{false};
    std::thread thr;
    int lfd = -1;
    std::atomic<int> cfd{-1};             // the live connection, to cut it on end()
    void start() {
        lfd = slamlink::listenUnix(path);
        thr = std::thread([this] {
            while (!stop.load()) {
                const int fd = ::accept(lfd, nullptr, nullptr);
                if (fd < 0) return;
                cfd.store(fd);
                slamlink::FrameHeader h;
                std::vector<uint8_t> l, r;
                std::vector<slamlink::ImuSample> s;
                while (!stop.load() && slamlink::recvFrame(fd, h, l, r, s)) {
                    frames.fetch_add(1);
                    imu.fetch_add(long(s.size()));
                    sleepMs(delayMs);
                    slamlink::PoseReply p;
                    p.seq = h.seq; p.state = slamlink::kOk;
                    p.Twc[3] = float(h.tS);
                    if (!slamlink::sendPose(fd, p)) break;
                }
                cfd.store(-1);
                ::close(fd);
            }
        });
    }
    void end() {
        stop.store(true);
        const int c = cfd.load();
        if (c >= 0) ::shutdown(c, SHUT_RDWR);
        ::shutdown(lfd, SHUT_RDWR); ::close(lfd);
        if (thr.joinable()) thr.join();
    }
};

void box(sim::VoxelWorld& w, float x0, float y0, float z0, float x1, float y1, float z1,
         float tex) {
    int ax, ay, az, bx, by, bz;
    w.worldToCell(x0, y0, z0, ax, ay, az);
    w.worldToCell(x1, y1, z1, bx, by, bz);
    for (int z = az; z <= bz; ++z)
        for (int y = ay; y <= by; ++y)
            for (int x = ax; x <= bx; ++x) { w.set(x, y, z); w.setTex(x, y, z, tex); }
}

struct Waypoint { float e, n, u, yawDeg; };

std::vector<Waypoint> densify(const std::vector<Waypoint>& key, float dt, float vel,
                              float yawRate) {
    std::vector<Waypoint> out;
    for (size_t i = 0; i + 1 < key.size(); ++i) {
        const Waypoint& a = key[i]; const Waypoint& b = key[i + 1];
        const float dist = std::hypot(std::hypot(b.e - a.e, b.n - a.n), b.u - a.u);
        const float dy = b.yawDeg - a.yawDeg;
        const float T = std::max(dist / vel, std::fabs(dy) / yawRate);
        const int n = std::max(1, int(T / dt));
        for (int k = 0; k < n; ++k) {
            const float s = float(k) / float(n);
            out.push_back({a.e + (b.e - a.e) * s, a.n + (b.n - a.n) * s,
                           a.u + (b.u - a.u) * s, a.yawDeg + dy * s});
        }
    }
    out.push_back(key.back());
    return out;
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    // ------------------------------------------------------------------- 1
    std::printf("SlamClient against a fake server\n");
    {
        FakeServer srv;
        srv.path = "/tmp/kestrel-slamtest-" + std::to_string(::getpid()) + ".sock";
        srv.delayMs = 50;
        srv.start();
        SlamClient cli(srv.path);
        for (int i = 0; i < 30 && !cli.connected(); ++i) sleepMs(50);
        CHECK(cli.connected());
        slamlink::FrameHeader h;
        h.width = 64; h.height = 48;
        std::vector<uint8_t> img(64 * 48, 7);
        // 40 frames at 5 ms against a 50 ms server: most must be DROPPED, and
        // every one of the 40 x 3 IMU samples must still arrive.
        long imuSent = 0;
        uint32_t lastSeq = 0;
        for (int i = 0; i < 40; ++i) {
            h.tS = 0.1 * i;
            std::vector<slamlink::ImuSample> s(3);
            imuSent += 3;
            lastSeq = cli.submit(h, img.data(), img.data(), s);
            sleepMs(5);
        }
        slamlink::PoseReply rep;
        uint32_t lastReply = 0;
        for (int i = 0; i < 60; ++i) {
            if (cli.takeReply(rep)) lastReply = rep.seq;
            if (lastReply == lastSeq) break;
            sleepMs(20);
        }
        std::printf("  40 submitted: %ld sent, %ld dropped, IMU %ld of %ld delivered, "
                    "last reply seq %u (newest %u)\n", cli.framesSent(),
                    cli.framesDropped(), srv.imu.load(), imuSent, lastReply, lastSeq);
        CHECK(cli.framesDropped() > 20);
        CHECK(cli.framesSent() + cli.framesDropped() == 40);
        CHECK(srv.imu.load() == imuSent);             // carried, never lost
        CHECK(lastReply == lastSeq);                  // the newest frame is answered
        CHECK(std::fabs(rep.Twc[3] - 3.9f) < 1e-4f);  // ...and its payload is intact
        srv.end();
        // The server goes away: the client says so, and reconnects when it returns.
        for (int i = 0; i < 40 && cli.connected(); ++i) {
            cli.submit(h, img.data(), img.data(), {});
            sleepMs(50);
        }
        CHECK(!cli.connected());
        FakeServer srv2;
        srv2.path = srv.path; srv2.delayMs = 1;
        srv2.start();
        for (int i = 0; i < 40 && !cli.connected(); ++i) sleepMs(50);
        CHECK(cli.connected());
        srv2.end();
        std::printf("  server restart: disconnected, then reconnected\n");
    }

    // ------------------------------------------------------------------ 1b
    std::printf("DarkFrameGate: the strobe's lit frames, from the image alone\n");
    {
        // A smooth scene (blurred noise) that drifts slowly, and the
        // projector's dots on odd frames at a given contrast.
        cv::RNG rng(4);
        cv::Mat base(240, 424, CV_8U);
        rng.fill(base, cv::RNG::UNIFORM, 40, 200);
        cv::GaussianBlur(base, base, cv::Size(0, 0), 2.5);
        auto frame = [&](int k, bool lit, int dotGain) {
            cv::Mat f;
            const cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, 0.3 * k, 0, 1, 0);
            cv::warpAffine(base, f, M, base.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
            if (lit)
                for (int v = 2; v < f.rows; v += 6)
                    for (int u = (v * 5) % 6; u < f.cols; u += 6)
                        f.at<uchar>(v, u) = cv::saturate_cast<uchar>(f.at<uchar>(v, u) + dotGain);
            return f;
        };
        // Strobing, dots clearly visible: every lit frame refused, dark ones used.
        sim::DarkFrameGate g;
        int litUsed = 0, darkUsed = 0, darkFrames = 0;
        for (int k = 0; k < 60; ++k) {
            const bool lit = k % 2 == 1;
            const bool use = sim::DarkFrameGate::usable(g.classify(frame(k, lit, 60)));
            if (lit && use) ++litUsed;
            if (!lit) { ++darkFrames; if (use) ++darkUsed; }
        }
        std::printf("  strobing: %d lit frames used (must be 0), %d of %d dark used\n",
                    litUsed, darkUsed, darkFrames);
        CHECK(litUsed == 0);
        CHECK(darkUsed >= darkFrames - 2);
        // A DROPPED dark frame puts two lit frames in a row: the second must
        // still be refused (alternation is never taken as proof).
        sim::DarkFrameGate g2;
        int litUsed2 = 0;
        const bool seq[] = {false, true, false, true, true, false, true, false, true, true, true, false};
        for (int k = 0; k < 12; ++k)
            if (seq[k] && sim::DarkFrameGate::usable(g2.classify(frame(k, true, 60)))) ++litUsed2;
            else if (!seq[k]) g2.classify(frame(k, false, 60));
        CHECK(litUsed2 == 0);
        // No visible dots (sunlight): usable once 6 frames show no strobe.
        sim::DarkFrameGate g3;
        int used3 = 0;
        for (int k = 0; k < 30; ++k)
            if (sim::DarkFrameGate::usable(g3.classify(frame(k, false, 0)))) ++used3;
        std::printf("  dropped frame: %d lit used (must be 0); no dots: %d of 30 used\n",
                    litUsed2, used3);
        CHECK(used3 >= 24);
    }
    std::printf("ImuSync: accel interpolated onto gyro times\n");
    {
        // accel along x = t (a ramp, so linear interpolation is exact), 250 Hz;
        // gyro at 400 Hz. Every gyro sample inside the accel span comes out
        // paired and right; the ones past the last accel wait.
        std::vector<sim::ImuRaw> raw;
        for (int i = 0; i <= 250; ++i) { sim::ImuRaw a; a.tS = i / 250.0; a.x = float(a.tS); raw.push_back(a); }
        for (int i = 0; i <= 420; ++i) { sim::ImuRaw gy; gy.gyro = true; gy.tS = i / 400.0; gy.z = 1.f; raw.push_back(gy); }
        ImuSync sync;
        sync.push(raw);
        std::vector<slamlink::ImuSample> out;
        sync.drain(out);
        float worst = 0.f;
        for (const auto& o : out) worst = std::max(worst, std::fabs(o.ax - float(o.tS)));
        std::printf("  %zu of 421 gyro samples paired (those after 1.0 s wait), worst accel "
                    "error %.2g\n", out.size(), double(worst));
        CHECK(out.size() == 401);
        CHECK(worst < 1e-4f);
        CHECK(!out.empty() && out.back().tS <= 1.0 && std::fabs(out.back().gz - 1.f) < 1e-6f);
    }

    // ------------------------------------------------------------------- 2
    const char* bin = std::getenv("KESTREL_ORBSLAM");
    const char* voc = std::getenv("KESTREL_ORBVOC");
    if (!bin || !voc) {
        std::printf("ORB-SLAM3 on rendered stereo: SKIPPED (set KESTREL_ORBSLAM and "
                    "KESTREL_ORBVOC; onboard/orbslam/README.md)\n");
        std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
        return fails ? 1 : 0;
    }
    std::printf("ORB-SLAM3 on rendered stereo IR\n");
    sim::VoxelWorld w;
    {
        const float S = 20.f, H = 5.f, c = 0.1f;
        w.init(c, 0.f, 0.f, 0.f, int(S / c), int(S / c), int(H / c));
        box(w, 0, 0, 0, S, S, 0.1f, 0.5f);
        box(w, 0, 0, 0, 0.3f, S, H, 0.4f); box(w, S - 0.3f, 0, 0, S, S, H, 0.4f);
        box(w, 0, 0, 0, S, 0.3f, H, 0.4f); box(w, 0, S - 0.3f, 0, S, S, H, 0.4f);
        std::srand(5);
        for (int i = 0; i < 30; ++i) {
            const float x = 2.f + 16.f * float(std::rand()) / float(RAND_MAX);
            const float y = 2.f + 16.f * float(std::rand()) / float(RAND_MAX);
            if (std::hypot(x - 10.f, y - 10.f) < 3.5f) continue;
            box(w, x - 0.2f, y - 0.2f, 0, x + 0.2f, y + 0.2f, H, 0.55f);
        }
    }
    sim::CamParams cp;
    // 848x480, as the bridge is meant to run. At 424x240 stereo start takes
    // ~99 frames (ORB-SLAM3 wants > 500 features with depth in ONE frame) and
    // the "tracking within a second" bar below fails every time -- measured in
    // 39 of 39 launches. SLAMTEST_LOWRES keeps it for looking at that.
    cp.width = 848; cp.height = 480; cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
    cp.irBandLimit = true;
    if (std::getenv("SLAMTEST_LOWRES")) { cp.width = 424; cp.height = 240; }
    const sim::DepthCamera cam(cp);

    struct Case { const char* name; std::vector<Waypoint> key; float pitch; float vel, yawRate; };
    const std::vector<Case> cases = {
        {"straight 8 m", {{10, 7, 1.5f, 0}, {10, 15, 1.5f, 0}}, -10.f, 1.f, 36.f},
        {"square 5x5, turns in place",
         {{8, 8, 1.5f, 0}, {8, 13, 1.5f, 0}, {8, 13, 1.5f, 90}, {13, 13, 1.5f, 90},
          {13, 13, 1.5f, 180}, {13, 8, 1.5f, 180}, {13, 8, 1.5f, 270},
          {8, 8, 1.5f, 270}}, -15.f, 1.f, 36.f},
        {"hover, then a leg, then hover", {{10, 7, 1.5f, 0}, {10, 7, 1.5f, 0.01f},
          {10, 11, 1.5f, 0.01f}, {10, 11, 1.5f, 0.02f}}, -15.f, 1.f, 0.002f},
    };
    const float dt = 1.f / 15.f;
    for (size_t ci = 0; ci < cases.size(); ++ci) {
        const Case& c = cases[ci];
        if (const char* only = std::getenv("SLAMTEST_ONLY"))   // e.g. "straight"
            if (!std::strstr(c.name, only)) continue;
        // A fresh bridge per case: each flight starts its own map.
        const std::string sock = "/tmp/kestrel-orbtest-" + std::to_string(::getpid()) +
                                 "-" + std::to_string(ci) + ".sock";
        const pid_t pid = ::fork();
        if (pid == 0) {
            if (!std::getenv("SLAMTEST_VERBOSE")) {
                if (!std::freopen("/dev/null", "w", stdout)) _exit(126);
            }
            ::execl(bin, bin, "--vocab", voc, "--socket", sock.c_str(), (char*)nullptr);
            _exit(127);
        }
        SlamClient cli(sock);
        for (int i = 0; i < 600 && !cli.connected(); ++i) sleepMs(50);
        CHECK(cli.connected());
        const std::vector<Waypoint> traj = densify(c.key, dt, c.vel, c.yawRate);
        SlamAnchor anchor;
        int ok = 0, lost = 0, notInit = 0;
        float finalErr = 0, maxErr = 0, yawErr = 0, path = 0, msSum = 0;
        Waypoint prev = traj.front();
        for (size_t i = 0; i < traj.size(); ++i) {
            const Waypoint& q = traj[i];
            sim::CamPose L;
            L.e = q.e; L.n = q.n; L.u = q.u; L.yawDeg = q.yawDeg; L.pitchDeg = c.pitch;
            const cv::Matx33d Rwc = SlamAnchor::rotWc(L);
            sim::CamPose R = L;                     // right imager, +x by the baseline
            R.e += float(Rwc(0, 0) * cp.baselineM);
            R.n += float(Rwc(1, 0) * cp.baselineM);
            R.u += float(Rwc(2, 0) * cp.baselineM);
            const cv::Mat il = cam.renderIR(w, L), ir = cam.renderIR(w, R);
            slamlink::FrameHeader h;
            h.tS = double(i) * dt;
            h.width = cp.width; h.height = cp.height;
            h.fx = cam.fpx(); h.fy = cam.fy(); h.cx = cam.ppx(); h.cy = cam.ppy();
            h.baselineM = cp.baselineM; h.fps = 1.f / dt;
            const uint32_t seq = cli.submit(h, il.data, ir.data, {});
            slamlink::PoseReply rep;
            bool got = false;
            // The first reply waits on the vocabulary load (145 MB of text:
            // tens of seconds on a desk, longer on a Pi).
            const auto tWait = std::chrono::steady_clock::now();
            const double limitS = i == 0 ? 300.0 : 30.0;
            while (!got && std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                         tWait).count() < limitS) {
                got = cli.takeReply(rep) && rep.seq == seq;
                if (!got) sleepMs(2);
            }
            if (i == 0) std::printf("  (bridge ready after %.1f s)\n",
                                    std::chrono::duration<double>(
                                        std::chrono::steady_clock::now() - tWait).count());
            CHECK(got);
            if (!got) break;
            if (i > 0) msSum += rep.ms;
            path += std::hypot(std::hypot(q.e - prev.e, q.n - prev.n), q.u - prev.u);
            prev = q;
            const bool isOk = rep.state == slamlink::kOk || rep.state == slamlink::kOkKlt;
            if (!isOk) { if (rep.state == slamlink::kNotInitialized) ++notInit; else ++lost; continue; }
            ++ok;
            // Anchor at the first tracked frame to the TRUE pose: the test
            // scores ORB-SLAM3's drift, not the IMU's or the compass's.
            if (!anchor.anchored()) anchor.anchor(rep.Twc, L);
            float e, n, u, yaw;
            anchor.toEnu(rep.Twc, e, n, u, yaw);
            const float err = std::hypot(std::hypot(e - q.e, n - q.n), u - q.u);
            finalErr = err; maxErr = std::max(maxErr, err);
            float ye = yaw - q.yawDeg;
            while (ye > 180.f) ye -= 360.f;
            while (ye <= -180.f) ye += 360.f;
            yawErr = std::fabs(ye);
        }
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        ::unlink(sock.c_str());
        const int n = int(traj.size());
        const float drift = 100.f * finalErr / std::max(0.5f, path);
        std::printf("  %-30s %dx%d  path %5.1f m  tracked %d/%d (init %d, lost %d)  "
                    "final err %.3f m (%.1f %%)  max %.3f m  yaw err %.1f deg  %.1f ms/frame\n",
                    c.name, cp.width, cp.height, path, ok, n, notInit, lost, finalErr,
                    drift, maxErr, yawErr, msSum / float(std::max(1, n - 1)));
        // Bars: tracking within a second (ORB-SLAM3's stereo start needs more
        // than 500 features with depth in one frame), never lost in a textured
        // room, and position within 3 % of the path (0.1 m when it barely moves).
        CHECK(notInit <= 15);
        CHECK(lost == 0);
        CHECK(finalErr < std::max(0.1f, 0.03f * path));
        CHECK(yawErr < 3.f);
    }
    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}

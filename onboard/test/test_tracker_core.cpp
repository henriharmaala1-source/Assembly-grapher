// BOTH TRACKER CORES, THROUGH THE ONE SEAM THEY NOW SHARE.
//
// The gap this closes: TrackModule instantiated LockOnTracker, the recorder
// built LockTracker, and every tracker test exercised the second. So a green
// tracker suite said nothing about the tracker running inside kestrel. That
// was deliberate staging rather than an accident, but it meant the test
// results did not cover the deployed code.
//
// This drives BOTH through ITrackerCore on the SAME synthetic sequence. It is
// not an accuracy shoot-out -- the two are different trackers and are expected
// to differ -- it pins the contract every consumer relies on: designate takes,
// update returns a box that follows the target, and the observation carries
// the provenance a consumer needs to decide how much to trust it.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "tracker_core.hpp"

namespace {
int failures = 0;
void check(const char* what, bool ok, const std::string& detail = "") {
    std::printf("    %-50s %s  %s\n", what, ok ? "ok  " : "FAIL", detail.c_str());
    if (!ok) ++failures;
}

// A textured target on a textured background, so a correlation tracker has
// something to lock and something to be distracted by.
cv::Mat frameAt(int w, int h, float tx, float ty, int size) {
    cv::Mat m(h, w, CV_8UC3);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const int v = 90 + int(30.0 * std::sin(x * 0.07) * std::cos(y * 0.05));
            m.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(v), uchar(v), uchar(v));
        }
    const cv::Rect box(int(tx) - size / 2, int(ty) - size / 2, size, size);
    for (int y = box.y; y < box.y + box.height; ++y)
        for (int x = box.x; x < box.x + box.width; ++x) {
            if (x < 0 || y < 0 || x >= w || y >= h) continue;
            const int u = x - box.x, vv = y - box.y;
            const int c = ((u * 37 + vv * 91) ^ (u * 13)) & 0xFF;
            m.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(c), uchar(255 - c), uchar(c / 2 + 60));
        }
    return m;
}

void exercise(const char* label, std::unique_ptr<track::ITrackerCore> core) {
    std::printf("  %s\n", label);
    const int W = 320, H = 240, SZ = 48;
    float tx = 120.f, ty = 120.f;
    double clock = 100.0;                  // deliberately not starting at zero
    const double dt = 1.0 / 30.0;

    core->designate(frameAt(W, H, tx, ty, SZ), cv::Point(int(tx), int(ty)), SZ);
    check("names itself", std::string(core->name()).size() > 0, core->name());

    float worst = 0.f;
    track::TrackObs o;
    for (int i = 0; i < 40; ++i) {
        tx += 2.f;                          // 60 px/s at 30 fps
        clock += dt;
        o = core->update(frameAt(W, H, tx, ty, SZ), clock);
        if (o.valid) {
            const float ex = std::hypot(o.box.x + o.box.width * 0.5f - tx,
                                        o.box.y + o.box.height * 0.5f - ty);
            worst = std::max(worst, ex);
        }
    }
    // A DIVERGENCE CHECK, NOT AN ACCURACY ONE. The two cores are different
    // trackers and are expected to differ; asserting a tight bound here would
    // be asserting something about the scene I invented for this file rather
    // than about either tracker. Per-core accuracy belongs in each core's own
    // suite, against the sequences those were built around. What the SEAM has
    // to guarantee is that a core driven through it produces a box in the
    // right part of the image rather than garbage.
    std::printf("      followed to worst err %d px, state %s\n",
                int(worst), o.state);
    check("produces a box that has not diverged", o.valid && worst < 60.f,
          "worst err " + std::to_string(int(worst)) + " px");

    // PROVENANCE. Every one of these was unavailable before the seam existed,
    // and each answers a question a consumer of a box actually has.
    check("echoes the capture time it was given",
          std::fabs(o.captureSec - clock) < 1e-9, std::to_string(o.captureSec));
    check("reports which core produced it",
          std::string(o.core) == core->name(), o.core);
    check("reports how long since anything was SEEN",
          o.ageOfFixSec >= 0.f && o.ageOfFixSec < 1.f,
          std::to_string(o.ageOfFixSec) + " s");

    // VELOCITY IS PER SECOND. The target moves 2 px a frame at 30 fps, which
    // is 60 px/s -- and the whole point of the timestamp work is that this
    // number means the same thing whatever the frame rate.
    if (o.locked)
        check("velocity is in pixels per SECOND",
              std::fabs(o.vel.x) > 15.f && std::fabs(o.vel.x) < 200.f,
              std::to_string(o.vel.x) + " px/s");

    // AND A FEATURELESS FRAME MUST NOT BE MISTAKEN FOR A FIX. Not "the target
    // moved away" -- an image with nothing in it at all, where no tracker can
    // honestly claim to be seeing anything. This is the weakest possible
    // version of the check and every core has to pass it.
    cv::Mat blank(H, W, CV_8UC3, cv::Scalar(128, 128, 128));
    for (int i = 0; i < 12; ++i) {
        clock += dt;
        o = core->update(blank, clock);
    }
    std::printf("      after 12 blank frames: state %s, fix age %.2f s\n",
                o.state, o.ageOfFixSec);
    check("does not claim a current fix on a blank frame",
          !o.locked || o.ageOfFixSec > 0.f,
          std::string(o.state) + ", fix age " + std::to_string(o.ageOfFixSec));
}
}  // namespace

int main() {
    std::printf("tracker cores, through the shared seam\n");
    exercise("fused (OpenCV-free core)", track::makeFusedCore());
    exercise("legacy FLOW (the one that flies)", track::makeLegacyCore(Backend::FLOW));
    std::printf(failures ? "FAILURES (%d)\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}

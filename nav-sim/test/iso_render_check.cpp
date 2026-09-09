// Headless check of the voxel-model renderer.
//
// The pane looked like static and the reason turned out to be display pitch,
// not the map: 240 cells across a 440 px image is one pixel per cube. That is
// not a bug you can see in a unit test of the map, and it is not one you can
// see in a screenshot description either -- so this dumps PNGs at several
// pitches and prints how many pixels a block actually occupies, which is the
// number that decides whether cubes are visible at all.
//
//   ./iso_render_check /tmp/iso        writes /tmp/iso_0.5m.png ... etc
// Includes are explicit rather than transitive on purpose: MSVC does not ship
// <cmath>, <vector> or <algorithm> inside <string> the way libstdc++ happens
// to, and that difference alone cost this repository three Windows CI rounds.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

int main(int argc, char** argv) {
    const std::string out = argc > 1 ? argv[1] : "/tmp/iso";

    VoxelWorld W;
    ForestParams fp; fp.cell = 0.25f; fp.seed = 1;
    std::vector<Trail> trails;
    genForest(W, fp, &trails);
    std::printf("forest %dx%dx%d, %zu trails\n", W.nx(), W.ny(), W.nz(), trails.size());

    // Fly a short straight leg so the map holds real accumulated structure --
    // rendering an empty map proves nothing about the renderer.
    float px = trails.empty() ? 15.f : trails[0].front()[0];
    float py = trails.empty() ? 10.f : trails[0].front()[1];
    float pz = 4.5f;
    CamParams cp; DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = fp.cell;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    VoxelMap M; M.init(mp, px, py, pz);

    // Fly the trail polyline itself at a constant speed. Not a planner -- this
    // is a renderer check, and using the planner here would mean a render bug
    // and a planning bug produce the same picture.
    std::vector<cv::Point3f> flown;
    float yaw = 0;
    if (!trails.empty()) {
        const Trail& t = trails[0];
        size_t seg = 0; float u = 0;
        for (int s = 0; s < 400 && seg + 1 < t.size(); ++s) {
            float ax=t[seg][0], ay=t[seg][1], bx=t[seg+1][0], by=t[seg+1][1];
            float L = std::hypot(bx-ax, by-ay);
            px = ax + (bx-ax)*u; py = ay + (by-ay)*u;
            yaw = std::atan2(bx-ax, by-ay) * 180.f / sim::PI_F;
            CamPose pose; pose.e = px; pose.n = py; pose.u = pz;
            pose.yawDeg = yaw; pose.pitchDeg = -5;
            M.integrate(cam.renderStereo(W, pose, nullptr), cam, pose);
            M.recentre(px, py, pz);
            flown.push_back({px, py, pz});
            u += 0.3f / std::max(0.1f, L);
            while (u >= 1.f && seg + 1 < t.size()) { u -= 1.f; ++seg; }
        }
    }
    std::printf("flew %zu steps along trail 0, ending (%.0f,%.0f)\n", flown.size(), px, py);

    struct V { float blockM, spanM; };
    for (V v : {V{0.5f,20.f}, V{1.0f,32.f}, V{1.5f,44.f}, V{2.0f,60.f}}) {
        const float b = v.blockM;
        VoxelMap::IsoView iv;
        cv::Mat img = M.isoImage(440, 40.f, 30.f, &iv, b, v.spanM);
        // Overlay the flown path through the SAME IsoView the cubes used. If
        // the projection and the renderer ever disagree this line drifts off
        // the model, which is visible instantly and silent otherwise.
        for (size_t i = flown.size() > 200 ? flown.size()-200 : 1; i < flown.size(); ++i)
            cv::line(img, iv.project(flown[i-1].x, flown[i-1].y, flown[i-1].z),
                          iv.project(flown[i].x,   flown[i].y,   flown[i].z),
                     {60,60,225}, 2, cv::LINE_AA);
        // Occupied-pixel fraction: an all-background image means the map is
        // empty or the projection is off-screen, and both look identical in a
        // filename.
        long nonbg = 0;
        for (int y = 0; y < img.rows; ++y)
            for (int x = 0; x < img.cols; ++x) {
                cv::Vec3b c = img.at<cv::Vec3b>(y, x);
                if (c != cv::Vec3b(238, 240, 244)) ++nonbg;
            }
        char path[256];
        std::snprintf(path, sizeof path, "%s_%.0fm_%.2fm.png", out.c_str(), v.spanM, b);
        cv::imwrite(path, img);
        std::printf("  span %2.0f m block %.2f m -> %5.1f px per cube, "
                    "%4.1f%% of pane drawn   %s\n",
                    v.spanM, b, iv.s, 100.0 * double(nonbg) / (img.rows * img.cols), path);
    }

    // Overlay register: a point at the map centre must land near the pane
    // centre, and one 20 m away must not. If projection and cubes disagree the
    // path overlay silently draws in the wrong place.
    VoxelMap::IsoView iv;
    M.isoImage(440, 40.f, 0.f, &iv, 1.5f, 40.f);
    cv::Point2f c0 = iv.project(px, py, pz);
    cv::Point2f c1 = iv.project(px + 20.f, py, pz);
    std::printf("  project: vehicle -> (%.0f,%.0f), +20 m east -> (%.0f,%.0f), "
                "%.1f px per 20 m\n", c0.x, c0.y, c1.x, c1.y, cv::norm(c1 - c0));
    return 0;
}

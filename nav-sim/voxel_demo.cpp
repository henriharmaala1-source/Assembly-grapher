// Fly a trajectory through a real-ish world, build a voxel map from simulated
// stereo, and score the map against ground truth.
//
//   ./voxel_demo --world forest --frames 120 --out /tmp/vox
//   ./voxel_demo --world city   --frames 120 --out /tmp/vox
//   ./voxel_demo --world osm --buildings helsinki.txt
//   ./voxel_demo --world lidar --points plot.xyz
//
// The point of this program is the LAST TWO NUMBERS it prints. IoU says whether
// the map resembles the world; false-free says whether it will fly you into
// something. They are not the same question and the second one is the one that
// matters.
#include <cstdio>
#include <cstring>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

static cv::Mat depthToColour(const cv::Mat& d, float maxR) {
    cv::Mat vis(d.rows, d.cols, CV_8UC3, cv::Scalar(60, 60, 60));
    for (int y = 0; y < d.rows; ++y)
        for (int x = 0; x < d.cols; ++x) {
            float r = d.at<float>(y, x);
            if (!(r > 0.f)) continue;                     // invalid stays grey
            float f = std::min(1.f, r / maxR);
            vis.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(255 * (1 - f)), uchar(80 + 100 * f),
                                                uchar(255 * f));
        }
    return vis;
}

int main(int argc, char** argv) {
    std::string world = "forest", out = "/tmp/vox", bfile, pfile;
    int frames = 120;
    // --truth integrates PERFECT depth instead of simulated stereo. This is the
    // control: if the map is bad with truth depth, the mapper is broken; if it
    // is good with truth and bad with stereo, the depth is the problem and the
    // number you are looking at is a real sensor limitation, not a bug.
    bool useTruth = false;
    float integM = -1.f;
    float cell = 0.25f;
    for (int i = 1; i < argc; ++i) {
        auto next = [&](const char* d) { return (i + 1 < argc) ? argv[++i] : d; };
        if (!std::strcmp(argv[i], "--world"))      world = next("forest");
        else if (!std::strcmp(argv[i], "--frames"))frames = std::atoi(next("120"));
        else if (!std::strcmp(argv[i], "--out"))   out = next("/tmp/vox");
        else if (!std::strcmp(argv[i], "--cell"))  cell = float(std::atof(next("0.25")));
        else if (!std::strcmp(argv[i], "--buildings")) bfile = next("");
        else if (!std::strcmp(argv[i], "--points"))    pfile = next("");
        else if (!std::strcmp(argv[i], "--truth"))     useTruth = true;
        else if (!std::strcmp(argv[i], "--integ"))     integM = float(std::atof(next("25")));
    }

    VoxelWorld W;
    float startE, startN, startU, hdg = 0.f;
    if (world == "city") {
        CityParams p; p.cell = cell; genCity(W, p);
        startE = p.streetM * 0.5f + 3.f; startN = 6.f; startU = 4.f; hdg = 0.f;
    } else if (world == "osm") {
        if (!loadOsmBuildings(W, bfile, cell)) return 1;
        startE = W.nx() * cell * 0.1f; startN = W.ny() * cell * 0.1f; startU = 8.f;
    } else if (world == "lidar") {
        if (!loadLidarXyz(W, pfile, cell)) return 1;
        startE = W.nx() * cell * 0.2f; startN = W.ny() * cell * 0.2f; startU = 12.f;
    } else {
        ForestParams p; p.cell = cell; genForest(W, p);
        startE = 20.f; startN = 10.f; startU = 6.f; hdg = 0.f;
    }
    std::printf("world '%s': %dx%dx%d cells @ %.2f m, %.1f%% solid\n",
                world.c_str(), W.nx(), W.ny(), W.nz(), W.cell(),
                100.0 * double(W.solidCount()) / (double(W.nx()) * W.ny() * W.nz()));

    CamParams cp; cp.width = 320; cp.height = 240;
    DepthCamera cam(cp);
    std::printf("camera: %dx%d, f=%.1f px, B=%.2f m, blind zone %.2f m, "
                "err@20m %.1f%%\n",
                cp.width, cp.height, cam.fpx(), cp.baselineM,
                cam.fpx() * cp.baselineM / cp.maxDisp,
                100.f * 20.f * cp.subpixelPx / (cam.fpx() * cp.baselineM));

    VoxelMapParams mp; mp.cell = cell;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    if (integM > 0) mp.maxIntegM = integM;
    VoxelMap M; M.init(mp, startE, startN, startU);

    // A simple lawnmower sweep with yaw scanning. Not a planner — the planner
    // is what this world exists to test later; for now we just need coverage.
    CamPose pose; pose.e = startE; pose.n = startN; pose.u = startU;
    double tDepth = 0, tInteg = 0;
    float meanValid = 0;
    for (int f = 0; f < frames; ++f) {
        pose.yawDeg = hdg + 55.f * std::sin(f * 0.21f);
        pose.pitchDeg = -6.f + 4.f * std::sin(f * 0.13f);
        pose.rollDeg = 8.f * std::sin(f * 0.17f);

        int64 t0 = cv::getTickCount();
        float vf = 0;
        cv::Mat d = useTruth ? cam.renderTruth(W, pose) : cam.renderStereo(W, pose, &vf);
        if (useTruth) { vf = 1.f; }
        int64 t1 = cv::getTickCount();
        M.integrate(d, cam, pose);
        int64 t2 = cv::getTickCount();
        tDepth += double(t1 - t0) / cv::getTickFrequency();
        tInteg += double(t2 - t1) / cv::getTickFrequency();
        meanValid += vf;

        // advance along +N, drifting east, staying above the local surface
        pose.n += 0.55f;
        pose.e += 0.12f;

        if (f == frames / 2 || f == frames - 1) {
            cv::imwrite(out + "_depth_" + std::to_string(f) + ".png",
                        depthToColour(d, cp.maxRangeM));
            cv::imwrite(out + "_truth_" + std::to_string(f) + ".png",
                        depthToColour(cam.renderTruth(W, pose), cp.maxRangeM));
            cv::imwrite(out + "_slice_" + std::to_string(f) + ".png",
                        M.sliceImage(pose.u));
        }
    }
    meanValid /= float(frames);

    VoxelMap::Score s = M.score(W, pose.e, pose.n, pose.u, 25.f, 30.f);
    cv::imwrite(out + "_compare.png", compareImage(W, M, s, 640));

    std::printf("\nmode: %s   integrate to %.1f m\n",
                useTruth ? "GROUND-TRUTH depth (control)" : "simulated stereo", mp.maxIntegM);
    std::printf("%d frames   depth %.1f ms/frame   integrate %.1f ms/frame\n",
                frames, 1000 * tDepth / frames, 1000 * tInteg / frames);
    std::printf("valid stereo pixels: %.1f%%   (the rest is sky, blind zone, "
                "occlusion band, or untextured surface)\n", 100 * meanValid);
    std::printf("\nMAP vs TRUTH within 25 m:\n");
    std::printf("  occupied IoU        %.3f\n", s.iou());
    std::printf("  unknown             %.1f%%\n", 100.0 * double(s.unknown) / std::max(1L, s.total));
    std::printf("  FALSE-FREE cells    %ld  (%.3f%% of cells called free)\n",
                s.falseFree, 100.0 * s.falseFreeRate());
    std::printf("     ^ solid in truth, FREE in the map. Each one is a place the\n"
                "       planner would happily fly through a real obstacle.\n");
    std::printf("\nwrote %s_compare.png and per-frame depth/slice images\n", out.c_str());
    return 0;
}

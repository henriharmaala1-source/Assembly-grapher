// Closed-loop navigation harness over a voxel world.
//
//   sense -> map -> plan -> move -> repeat,  scored against ground truth.
//
//   ./voxel_sim --world forest --goal 150 170 8
//   ./voxel_sim --world city --cell 0.4 --truth        # perfect-depth control
//   ./voxel_sim --world forest --general-only          # no A*, reactive only
//
// This is the file that answers "does the planner work", and it answers it the
// only way that means anything: by flying the thing and checking whether it hit
// something REAL. The aircraft plans against its own noisy voxel map; collisions
// are detected against VoxelWorld. Those are different data structures on
// purpose -- a harness that checks the plan against the map it planned with
// would pass no matter how wrong the map is.
//
// Always run --truth as a control. If the run fails with perfect depth, the
// planner is broken. If it succeeds with perfect depth and fails with stereo,
// the map is the limit and the number is a sensor result.
#include <cstdio>
#include <cstring>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#if SIM_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "ompl_planner.hpp"
#include "voxel_planner.hpp"
#include "voxel_world.hpp"

using namespace sim;

// True clearance from the world, used ONLY for scoring -- never fed to the
// planner.
//
// EXACT, by scanning the actual voxels in a box around the point. The first
// version sampled 26 rays outward and took the first hit, and it was wrong in
// the same way the planner was: at r = 0.6 m the 26 sample points are ~0.6 m
// apart on the sphere, so a 0.2 m tree trunk sits between them and reads as
// clear. It reported 3.00 m of clearance one step before a collision 0.36 m
// away -- geometrically impossible, and the tell that the DETECTOR was broken
// rather than the planner.
//
// A collision detector that can miss obstacles makes every number the harness
// prints meaningless, so this one is exhaustive: 125 cells at 0.25 m and a
// 0.6 m radius, which costs nothing at sim rates.
static float trueClearance(const VoxelWorld& w, float x, float y, float z, float maxR) {
    int cx, cy, cz;
    w.worldToCell(x, y, z, cx, cy, cz);
    const int R = int(std::ceil(maxR / w.cell()));
    float best = maxR;
    for (int dz = -R; dz <= R; ++dz)
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                if (!w.solid(cx + dx, cy + dy, cz + dz)) continue;
                float wx, wy, wz;
                w.cellCentre(cx + dx, cy + dy, cz + dz, wx, wy, wz);
                // distance to the cell's nearest face, not its centre
                float ex = std::max(0.f, std::fabs(wx - x) - w.cell() * 0.5f);
                float ey = std::max(0.f, std::fabs(wy - y) - w.cell() * 0.5f);
                float ez = std::max(0.f, std::fabs(wz - z) - w.cell() * 0.5f);
                best = std::min(best, std::sqrt(ex * ex + ey * ey + ez * ez));
            }
    return best;
}

int main(int argc, char** argv) {
    std::string world = "forest", out = "/tmp/nav";
    int steps = 600;
    float cell = 0.25f, dt = 0.1f;
    float goalE = 150, goalN = 170, goalU = 8;
    bool useTruth = false, generalOnly = false, display = false;
    int replanEvery = 25;
    float lHit=-1, lMiss=-1, occT=-99, freeT=-99;
    for (int i = 1; i < argc; ++i) {
        auto next = [&](const char* d) { return (i + 1 < argc) ? argv[++i] : d; };
        if (!std::strcmp(argv[i], "--world")) world = next("forest");
        else if (!std::strcmp(argv[i], "--steps")) steps = std::atoi(next("600"));
        else if (!std::strcmp(argv[i], "--cell")) cell = float(std::atof(next("0.25")));
        else if (!std::strcmp(argv[i], "--out")) out = next("/tmp/nav");
        else if (!std::strcmp(argv[i], "--truth")) useTruth = true;
        else if (!std::strcmp(argv[i], "--general-only")) generalOnly = true;
        else if (!std::strcmp(argv[i], "--display")) display = true;
        else if (!std::strcmp(argv[i], "--lhit")) lHit = float(std::atof(next("0.85")));
        else if (!std::strcmp(argv[i], "--lmiss")) lMiss = float(std::atof(next("0.4")));
        else if (!std::strcmp(argv[i], "--occt")) occT = float(std::atof(next("0")));
        else if (!std::strcmp(argv[i], "--freet")) freeT = float(std::atof(next("-0.4")));
        else if (!std::strcmp(argv[i], "--replan")) replanEvery = std::atoi(next("25"));
        else if (!std::strcmp(argv[i], "--goal")) {
            goalE = float(std::atof(next("150")));
            goalN = float(std::atof(next("170")));
            goalU = float(std::atof(next("8")));
        }
    }

    VoxelWorld W;
    float px, py, pz;
    if (world == "city") {
        CityParams p; p.cell = cell; genCity(W, p);
        px = p.streetM * 0.5f; py = 5.f; pz = 6.f;
    } else {
        ForestParams p; p.cell = cell; genForest(W, p);
        px = 15.f; py = 10.f; pz = 6.f;
    }
    // VALIDATE THE SPAWN before blaming the planner for anything. A fixed start
    // point in a procedurally generated forest lands inside a tree often enough
    // that "collision at 0.4 m travelled" is far more likely to be a bad initial
    // condition than a planning failure -- and a harness that cannot tell those
    // apart is worse than no harness.
    {
        float c0 = trueClearance(W, px, py, pz, 3.0f);
        if (c0 < 1.5f) {
            printf("  spawn clearance only %.2f m -- searching for a clear start\n", c0);
            bool ok = false;
            for (float rad = 1.f; rad <= 25.f && !ok; rad += 1.f)
                for (int a = 0; a < 24 && !ok; ++a)
                    for (float dzs = 0.f; dzs <= 8.f && !ok; dzs += 1.f) {
                        float th = a * float(M_PI) / 12.f;
                        float tx = px + rad * std::cos(th), ty = py + rad * std::sin(th),
                              tz = pz + dzs;
                        if (trueClearance(W, tx, ty, tz, 3.0f) >= 1.5f) {
                            px = tx; py = ty; pz = tz; ok = true;
                        }
                    }
            if (!ok) { printf("  !! no clear spawn found within 25 m -- aborting\n"); return 3; }
        }
        printf("  spawn clearance %.2f m at (%.1f,%.1f,%.1f)\n",
               trueClearance(W, px, py, pz, 3.0f), px, py, pz);
    }
    printf("world '%s' %dx%dx%d @ %.2f m   start (%.1f,%.1f,%.1f) -> goal (%.0f,%.0f,%.0f)\n",
           world.c_str(), W.nx(), W.ny(), W.nz(), W.cell(), px, py, pz, goalE, goalN, goalU);

    CamParams cp; DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = cell;
    if (lHit  > 0)   mp.lHit = lHit;
    if (lMiss > 0)   mp.lMiss = lMiss;
    if (occT  > -90) mp.occThresh = occT;
    if (freeT > -90) mp.freeThresh = freeT;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    VoxelMap M; M.init(mp, px, py, pz);   // after spawn validation, not before

    GeneralParams gp; gp.robotR = 0.6f;
    GeneralPlanner gen(gp);
    ForwardParams fwp; fwp.robotR = gp.robotR;
    ForwardPath path;
    printf("  forward planner: %s\n", forwardPlannerName());

    float vx = 0, vy = 0, vz = 0, yaw = 0;
    float travelled = 0, minClear = 1e9f;
    int collisions = 0, stopped = 0, noPath = 0, replans = 0;
    bool reached = false;
    std::vector<cv::Point2f> trail;
    double tPlan = 0, tSense = 0, tGen = 0, tPrec = 0, tInteg = 0;
    int nPrec = 0;

    const float startDist = std::hypot(goalE - px, goalN - py);

#if SIM_HAVE_HIGHGUI
    // Static truth backdrop for the live top-down view, rendered once.
    cv::Mat truthTop;
    if (display) {
        cv::namedWindow("kestrel voxel sim", cv::WINDOW_AUTOSIZE);
        truthTop = cv::Mat(W.ny(), W.nx(), CV_8UC3, cv::Scalar(250, 248, 245));
    }
    bool paused = false;
#endif

    for (int s = 0; s < steps; ++s) {
        // --- sense -----------------------------------------------------------
        CamPose pose; pose.e = px; pose.n = py; pose.u = pz;
        pose.yawDeg = yaw; pose.pitchDeg = -5.f; pose.rollDeg = 0.f;
        int64 t0 = cv::getTickCount();
        cv::Mat d = useTruth ? cam.renderTruth(W, pose) : cam.renderStereo(W, pose, nullptr);
        int64 tm = cv::getTickCount();
        M.integrate(d, cam, pose);
        tInteg += double(cv::getTickCount() - tm) / cv::getTickFrequency();
        tSense += double(cv::getTickCount() - t0) / cv::getTickFrequency();
        M.recentre(px, py, pz);

        // --- precise plan, occasionally ---------------------------------------
        int64 t1 = cv::getTickCount();
        if (!generalOnly && (s % replanEvery == 0)) {
            // Plan AHEAD along the mission bearing, not to the distant goal.
            float mAz = std::atan2(goalE - px, goalN - py) * 180.f / float(M_PI);
            float mEl = std::atan2(goalU - pz,
                                   std::hypot(goalE - px, goalN - py)) * 180.f / float(M_PI);
            int64 tp = cv::getTickCount();
            path = planForward(M, px, py, pz, mAz, mEl, fwp);
            tPrec += double(cv::getTickCount() - tp) / cv::getTickFrequency(); ++nPrec;
            ++replans;
            if (!path.found) ++noPath;
        }
        // Direction the precise planner would like: the first waypoint far
        // enough ahead to be meaningful. If there is no path, fall back to the
        // straight-line goal bearing -- the reactive layer still keeps us safe,
        // we just explore rather than follow a route.
        float tgtE = goalE, tgtN = goalN, tgtU = goalU;
        if (path.found) {
            for (const auto& w : path.pts) {
                float dd = std::hypot(w[0] - px, w[1] - py);
                if (dd > 3.0f) { tgtE = w[0]; tgtN = w[1]; tgtU = w[2]; break; }
            }
        }
        float gAz = std::atan2(tgtE - px, tgtN - py) * 180.f / float(M_PI);
        float gEl = std::atan2(tgtU - pz, std::hypot(tgtE - px, tgtN - py)) * 180.f / float(M_PI);

        // --- general plan, every step ----------------------------------------
        int64 tg = cv::getTickCount();
        GeneralResult gr = gen.plan(M, px, py, pz, gAz, gEl);
        tGen += double(cv::getTickCount() - tg) / cv::getTickFrequency();
        tPlan += double(cv::getTickCount() - t1) / cv::getTickFrequency();
        if (gr.speed <= 0.01f) ++stopped;

        // --- move -------------------------------------------------------------
        float dx, dy, dz;
        {
            float a = gr.azDeg * float(M_PI) / 180.f, e = gr.elDeg * float(M_PI) / 180.f;
            dx = std::cos(e) * std::sin(a); dy = std::cos(e) * std::cos(a); dz = std::sin(e);
        }
        // First-order lag toward the commanded velocity: an aircraft cannot
        // change direction instantly, and a planner that assumes it can will
        // look far better in sim than in the air.
        const float tau = 0.35f;
        float k = std::min(1.f, dt / tau);
        vx += (dx * gr.speed - vx) * k;
        vy += (dy * gr.speed - vy) * k;
        vz += (dz * gr.speed - vz) * k;
        px += vx * dt; py += vy * dt; pz += vz * dt;
        travelled += std::sqrt(vx * vx + vy * vy + vz * vz) * dt;
        if (std::hypot(vx, vy) > 0.2f) yaw = std::atan2(vx, vy) * 180.f / float(M_PI);
        trail.push_back({px, py});

        // --- score against TRUTH ---------------------------------------------
        float clr = trueClearance(W, px, py, pz, 2.0f);
        minClear = std::min(minClear, clr);
        if (clr <= gp.robotR * 0.5f) {
            ++collisions;
            printf("  !! COLLISION at step %d, (%.1f, %.1f, %.1f), clearance %.2f m\n",
                   s, px, py, pz, clr);
            break;
        }
        if (std::hypot(goalE - px, goalN - py) < 4.f && std::fabs(goalU - pz) < 4.f) {
            reached = true;
            printf("  reached goal at step %d\n", s);
            break;
        }
#if SIM_HAVE_HIGHGUI
        if (display) {
            // Repaint the truth slice at the CURRENT flight height each frame --
            // the world is 3D, so a fixed-height backdrop would be misleading.
            int zc; { int a, b; W.worldToCell(0, 0, pz, a, b, zc); }
            truthTop.setTo(cv::Scalar(250, 248, 245));
            for (int y = 0; y < W.ny(); ++y)
                for (int x = 0; x < W.nx(); ++x)
                    if (W.solid(x, y, zc))
                        truthTop.at<cv::Vec3b>(W.ny() - 1 - y, x) = cv::Vec3b(70, 70, 70);
            for (size_t i = 1; i < trail.size(); ++i) {
                int x0, y0, z0, x1, y1, z1;
                W.worldToCell(trail[i-1].x, trail[i-1].y, pz, x0, y0, z0);
                W.worldToCell(trail[i].x, trail[i].y, pz, x1, y1, z1);
                cv::line(truthTop, {x0, W.ny()-1-y0}, {x1, W.ny()-1-y1}, {40,40,220}, 2);
            }
            if (path.found)
                for (size_t i = 1; i < path.pts.size(); ++i) {
                    int x0,y0,z0,x1,y1,z1;
                    W.worldToCell(path.pts[i-1][0], path.pts[i-1][1], pz, x0,y0,z0);
                    W.worldToCell(path.pts[i][0], path.pts[i][1], pz, x1,y1,z1);
                    cv::line(truthTop, {x0, W.ny()-1-y0}, {x1, W.ny()-1-y1}, {40,170,40}, 1);
                }
            { int gx,gy,gz; W.worldToCell(goalE, goalN, pz, gx,gy,gz);
              cv::circle(truthTop, {gx, W.ny()-1-gy}, 9, {40,180,40}, 2); }
            cv::Mat topV; cv::resize(truthTop, topV, cv::Size(480,480), 0,0, cv::INTER_AREA);
            cv::Mat sliceV = M.sliceImage(pz, 480);
            cv::Mat depthV(480, 480, CV_8UC3, cv::Scalar(60,60,60));
            { cv::Mat dv(d.rows, d.cols, CV_8UC3, cv::Scalar(60,60,60));
              for (int y=0;y<d.rows;++y) for (int x=0;x<d.cols;++x) {
                  float r = d.at<float>(y,x); if(!(r>0.f)) continue;
                  float f = std::min(1.f, r/cp.maxRangeM);
                  dv.at<cv::Vec3b>(y,x) = cv::Vec3b(uchar(255*(1-f)), uchar(80+100*f), uchar(255*f)); }
              cv::resize(dv, depthV, cv::Size(480,480), 0,0, cv::INTER_NEAREST); }
            cv::putText(topV,  "TRUTH + flown path", {10,22}, cv::FONT_HERSHEY_SIMPLEX,0.6,{30,30,30},2);
            cv::putText(sliceV,"VOXEL MAP slice",   {10,22}, cv::FONT_HERSHEY_SIMPLEX,0.6,{30,30,30},2);
            cv::putText(depthV,useTruth?"DEPTH (truth)":"DEPTH (stereo)", {10,22},
                        cv::FONT_HERSHEY_SIMPLEX,0.6,{240,240,240},2);
            cv::Mat row; cv::hconcat(std::vector<cv::Mat>{topV, sliceV, depthV}, row);
            char hud[240];
            std::snprintf(hud, sizeof hud,
                "step %d  pos %.0f,%.0f,%.1f  v %.2f m/s  free %.2f m  open %.2f m  %s  path %s  [space]=pause [q]=quit",
                s, px, py, pz, std::hypot(vx,vy), gr.freeM, gr.openM,
                gr.blocked?"BLOCKED":"ok", path.found?"yes":"NONE");
            cv::Mat bar(34, row.cols, CV_8UC3, cv::Scalar(25,25,30));
            cv::putText(bar, hud, {10,23}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {235,235,235}, 1);
            cv::Mat full; cv::vconcat(row, bar, full);
            cv::imshow("kestrel voxel sim", full);
            int key = cv::waitKey(paused ? 0 : 1);
            if (key == 'q' || key == 27) break;
            if (key == ' ') paused = !paused;
        }
#endif
        if (s % 40 == 0)
            printf("  step %4d  pos (%6.1f,%6.1f,%5.1f)  v %.2f  cmd %.2f  free %.2f  "
                   "open %.2f  %s  path %s(%zu wp)\n", s, px, py, pz,
                   std::hypot(vx, vy), gr.speed, gr.freeM, gr.openM,
                   gr.blocked ? "BLOCKED" : "ok    ",
                   path.found ? "" : "NONE ", path.pts.size());
    }

    // --- report -------------------------------------------------------------
    const float endDist = std::hypot(goalE - px, goalN - py);
    printf("\n--- %s depth, %s ---\n",
           useTruth ? "GROUND-TRUTH" : "simulated stereo",
           generalOnly ? "reactive only" : "reactive + A*");
    printf("  outcome            %s\n",
           collisions ? "COLLIDED" : (reached ? "reached goal" : "ran out of steps"));
    printf("  distance to goal   %.1f m  (started %.1f m away)\n", endDist, startDist);
    printf("  path travelled     %.1f m   (straight line %.1f m, ratio %.2f)\n",
           travelled, startDist, travelled / std::max(1.f, startDist));
    printf("  min true clearance %.2f m   <- scored against the WORLD, not the map\n", minClear);
    printf("  stopped on         %d of %d steps\n", stopped, steps);
    printf("  A* replans         %d, of which no path %d\n", replans, noPath);
    // Split out what would actually run ON THE AIRCRAFT. The depth RENDER is
    // sim-only -- on the Pi that is the stereo matcher, benchmarked separately.
    // Map integration and both planners are real onboard cost.
    int nsteps = std::max(1, (int)trail.size());
    printf("  --- onboard cost (per step unless noted) ---\n");
    printf("  map integrate      %6.2f ms\n", 1000 * tInteg / nsteps);
    printf("  general planner    %6.2f ms\n", 1000 * tGen / nsteps);
    printf("  forward planner    %6.2f ms per replan (%d replans, every %d steps)\n",
           nPrec ? 1000 * tPrec / nPrec : 0.0, nPrec, replanEvery);
    printf("  ONBOARD TOTAL      %6.2f ms/step amortised\n",
           1000 * (tInteg + tGen + tPrec) / nsteps);
    printf("  [sim-only] depth render %.1f ms/step\n",
           1000 * (tSense - tInteg) / nsteps);

    VoxelMap::Score sc = M.score(W, px, py, pz, 25.f, 30.f);
    printf("  map false-free     %.3f%%\n", 100.0 * sc.falseFreeRate());

    // Top-down: truth occupancy at flight height, plus the flown trail.
    cv::Mat top(W.ny(), W.nx(), CV_8UC3, cv::Scalar(250, 248, 245));
    int zc; { int a, b; W.worldToCell(0, 0, pz, a, b, zc); }
    for (int y = 0; y < W.ny(); ++y)
        for (int x = 0; x < W.nx(); ++x)
            if (W.solid(x, y, zc))
                top.at<cv::Vec3b>(W.ny() - 1 - y, x) = cv::Vec3b(70, 70, 70);
    for (size_t i = 1; i < trail.size(); ++i) {
        int x0, y0, z0, x1, y1, z1;
        W.worldToCell(trail[i - 1].x, trail[i - 1].y, pz, x0, y0, z0);
        W.worldToCell(trail[i].x, trail[i].y, pz, x1, y1, z1);
        cv::line(top, {x0, W.ny() - 1 - y0}, {x1, W.ny() - 1 - y1}, {40, 40, 220}, 2);
    }
    { int gx, gy, gz; W.worldToCell(goalE, goalN, pz, gx, gy, gz);
      cv::circle(top, {gx, W.ny() - 1 - gy}, 8, {40, 180, 40}, 2); }
    cv::Mat topOut; cv::resize(top, topOut, cv::Size(700, 700), 0, 0, cv::INTER_AREA);
    cv::putText(topOut, collisions ? "COLLIDED" : (reached ? "REACHED GOAL" : "TIMEOUT"),
                {14, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                collisions ? cv::Scalar(30, 30, 220) : cv::Scalar(30, 140, 30), 2);
    cv::imwrite(out + "_top.png", topOut);
    cv::imwrite(out + "_slice.png", M.sliceImage(pz));
    printf("  wrote %s_top.png (flown path over truth) and %s_slice.png\n",
           out.c_str(), out.c_str());
    return collisions ? 2 : (reached ? 0 : 1);
}

// Point-and-click front end for the voxel navigation sim.
//
//   ./voxel_gui            (Windows: voxel_gui.exe)
//
// No flags, no typing. A menu appears: click a world, click Fly. During flight
// you can pause, restart, or go back and pick another world without quitting.
//
// voxel_sim remains the scriptable entry point -- sweep.sh drives it for
// multi-seed batches, which is where the numbers that matter come from. This
// file is for looking at one run with your eyes.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "depth_camera.hpp"
#include "ompl_planner.hpp"
#include "voxel_map.hpp"
#include "voxel_planner.hpp"
#include "voxel_world.hpp"

using namespace sim;

static const char* WIN = "kestrel voxel sim";

struct WorldChoice { const char* label; const char* kind; const char* file; float cell; };
static const std::vector<WorldChoice> WORLDS = {
    {"Forest",           "forest", "",                            0.25f},
    {"City (generated)", "city",   "",                            0.40f},
    {"Hervanta",         "osm",    "worlds/hervanta.txt",         0.50f},
    {"Tampere centre",   "osm",    "worlds/tampere_centre.txt",   0.50f},
    {"Helsinki centre",  "osm",    "worlds/helsinki_centre.txt",  0.50f},
};

struct Cfg {
    int   world = 0;
    int   seed = 1;
    bool  truth = false;      // perfect depth control
    int   steps = 900;
    bool  trail = false;      // forest only: start on a trail, fly its length
};

struct Btn { cv::Rect r; std::string label; int id; bool on = false; };

static struct { int x = 0, y = 0; bool clicked = false; } g_mouse;
static void onMouse(int ev, int x, int y, int, void*) {
    g_mouse.x = x; g_mouse.y = y;
    if (ev == cv::EVENT_LBUTTONDOWN) g_mouse.clicked = true;
}

static void drawBtn(cv::Mat& img, const Btn& b) {
    cv::Scalar bg = b.on ? cv::Scalar(70, 140, 60) : cv::Scalar(58, 58, 66);
    cv::rectangle(img, b.r, bg, cv::FILLED);
    cv::rectangle(img, b.r, cv::Scalar(120, 120, 130), 1);
    int base = 0;
    cv::Size ts = cv::getTextSize(b.label, cv::FONT_HERSHEY_SIMPLEX, 0.52, 1, &base);
    cv::putText(img, b.label,
                {b.r.x + (b.r.width - ts.width) / 2, b.r.y + (b.r.height + ts.height) / 2},
                cv::FONT_HERSHEY_SIMPLEX, 0.52, {238, 238, 240}, 1, cv::LINE_AA);
}

// Returns false if the user closed the window.
static bool menu(Cfg& cfg) {
    const int W = 1000, H = 560;
    for (;;) {
        cv::Mat img(H, W, CV_8UC3, cv::Scalar(30, 30, 36));
        cv::putText(img, "kestrel  -  voxel navigation sim", {28, 46},
                    cv::FONT_HERSHEY_SIMPLEX, 0.95, {240, 240, 245}, 2, cv::LINE_AA);
        cv::putText(img, "pick a world, then Fly", {30, 74},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {150, 150, 160}, 1, cv::LINE_AA);

        std::vector<Btn> btns;
        int y = 104;
        for (size_t i = 0; i < WORLDS.size(); ++i) {
            btns.push_back({cv::Rect(30, y, 260, 40), WORLDS[i].label, int(i),
                            cfg.world == int(i)});
            y += 48;
        }
        // depth mode
        cv::putText(img, "depth", {330, 122}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    {150, 150, 160}, 1, cv::LINE_AA);
        btns.push_back({cv::Rect(330, 134, 190, 40), "Simulated stereo", 100, !cfg.truth});
        btns.push_back({cv::Rect(330, 182, 190, 40), "Perfect (control)", 101, cfg.truth});
        cv::putText(img, "Run 'Perfect' too: if it fails there,", {330, 246},
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, {150, 150, 160}, 1, cv::LINE_AA);
        cv::putText(img, "the planner is at fault, not the sensor.", {330, 264},
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, {150, 150, 160}, 1, cv::LINE_AA);

        // seed
        cv::putText(img, "world seed", {560, 122}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    {150, 150, 160}, 1, cv::LINE_AA);
        btns.push_back({cv::Rect(560, 134, 40, 40), "-", 200});
        btns.push_back({cv::Rect(650, 134, 40, 40), "+", 201});
        cv::putText(img, std::to_string(cfg.seed), {612, 162},
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, {240, 240, 245}, 2, cv::LINE_AA);
        cv::putText(img, "changes the generated world", {560, 198},
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, {150, 150, 160}, 1, cv::LINE_AA);

        // Trail mode. Only meaningful in the forest, so say so rather than
        // silently doing nothing when a city is selected.
        bool isForest = !std::strcmp(WORLDS[cfg.world].kind, "forest");
        cv::putText(img, "forest trail", {760, 122}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    {150, 150, 160}, 1, cv::LINE_AA);
        btns.push_back({cv::Rect(760, 134, 200, 40),
                        cfg.trail ? "Follow a trail" : "Open stand", 500,
                        cfg.trail && isForest});
        cv::putText(img, isForest ? "start on a cleared corridor"
                                  : "(forest only)", {760, 198},
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, {150, 150, 160}, 1, cv::LINE_AA);

        // steps
        cv::putText(img, "steps", {560, 246}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    {150, 150, 160}, 1, cv::LINE_AA);
        btns.push_back({cv::Rect(560, 258, 40, 40), "-", 300});
        btns.push_back({cv::Rect(680, 258, 40, 40), "+", 301});
        cv::putText(img, std::to_string(cfg.steps), {612, 286},
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, {240, 240, 245}, 2, cv::LINE_AA);

        btns.push_back({cv::Rect(30, y + 24, 260, 52), "FLY", 400, true});
        cv::putText(img, "space pause   r restart   m menu   q quit   -/+ playback speed",
                    {30, H - 26}, cv::FONT_HERSHEY_SIMPLEX, 0.46,
                    {150, 150, 160}, 1, cv::LINE_AA);

        for (const auto& b : btns) drawBtn(img, b);
        cv::imshow(WIN, img);
        int k = cv::waitKey(20);
        if (k == 'q' || k == 27) return false;

        if (g_mouse.clicked) {
            g_mouse.clicked = false;
            for (const auto& b : btns) {
                if (!b.r.contains({g_mouse.x, g_mouse.y})) continue;
                if (b.id < 100) cfg.world = b.id;
                else if (b.id == 100) cfg.truth = false;
                else if (b.id == 101) cfg.truth = true;
                else if (b.id == 200) cfg.seed = std::max(1, cfg.seed - 1);
                else if (b.id == 201) cfg.seed = std::min(99, cfg.seed + 1);
                else if (b.id == 300) cfg.steps = std::max(200, cfg.steps - 200);
                else if (b.id == 301) cfg.steps = std::min(4000, cfg.steps + 200);
                else if (b.id == 500) cfg.trail = !cfg.trail;
                else if (b.id == 400) return true;
            }
        }
    }
}

// EXACT clearance against the world -- scoring only, never given to the planner.
static float trueClearance(const VoxelWorld& w, float x, float y, float z, float maxR) {
    int cx, cy, cz; w.worldToCell(x, y, z, cx, cy, cz);
    const int R = int(std::ceil(maxR / w.cell()));
    float best = maxR;
    for (int dz = -R; dz <= R; ++dz)
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                if (!w.solid(cx+dx, cy+dy, cz+dz)) continue;
                float wx, wy, wz; w.cellCentre(cx+dx, cy+dy, cz+dz, wx, wy, wz);
                float ex = std::max(0.f, std::fabs(wx-x) - w.cell()*0.5f);
                float ey = std::max(0.f, std::fabs(wy-y) - w.cell()*0.5f);
                float ez = std::max(0.f, std::fabs(wz-z) - w.cell()*0.5f);
                best = std::min(best, std::sqrt(ex*ex + ey*ey + ez*ez));
            }
    return best;
}

// Returns: 0 quit, 1 back to menu, 2 restart same config.
static int fly(const Cfg& cfg) {
    const WorldChoice& wc = WORLDS[cfg.world];
    const float cell = wc.cell;
    VoxelWorld W;
    float px, py, pz, goalE, goalN, goalU;

    cv::Mat splash(560, 1000, CV_8UC3, cv::Scalar(30, 30, 36));
    cv::putText(splash, std::string("building ") + wc.label + " ...", {30, 60},
                cv::FONT_HERSHEY_SIMPLEX, 0.8, {240, 240, 245}, 2);
    cv::imshow(WIN, splash); cv::waitKey(1);

    std::vector<Trail> trails;
    if (!std::strcmp(wc.kind, "forest")) {
        ForestParams p; p.cell = cell; p.seed = unsigned(cfg.seed);
        genForest(W, p, &trails);
        px = 15; py = 10; pz = 6; goalE = 120; goalN = 150; goalU = 8;
        // Trail mode places the aircraft at one end of a cleared corridor and
        // puts the goal at the other. Following a corridor is a different test
        // from crossing open stand: the planner has to turn DOWN a lane that is
        // narrower than the gaps either side of it.
        if (cfg.trail && !trails.empty()) {
            const Trail& t = trails[0];
            px = t.front()[0]; py = t.front()[1]; pz = 4.5f;
            goalE = t.back()[0]; goalN = t.back()[1]; goalU = 4.5f;
        }
    } else if (!std::strcmp(wc.kind, "city")) {
        CityParams p; p.cell = cell; p.seed = unsigned(cfg.seed); genCity(W, p);
        px = p.streetM * 0.5f; py = 5; pz = 6; goalE = 160; goalN = 190; goalU = 8;
    } else {
        if (!loadOsmBuildings(W, wc.file, cell)) {
            cv::Mat e(200, 900, CV_8UC3, cv::Scalar(30, 30, 36));
            cv::putText(e, std::string("cannot open ") + wc.file, {20, 60},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, {80, 80, 240}, 2);
            cv::putText(e, "run: python3 worlds/make_fi_cities.py", {20, 110},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, {220, 220, 230}, 1);
            cv::putText(e, "any key to return", {20, 160},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {150, 150, 160}, 1);
            cv::imshow(WIN, e); cv::waitKey(0); return 1;
        }
        px = W.nx()*cell*0.12f; py = W.ny()*cell*0.12f; pz = 12;
        goalE = W.nx()*cell*0.85f; goalN = W.ny()*cell*0.85f; goalU = 20;
    }

    // Spawn validation -- never start inside a building or a tree.
    if (trueClearance(W, px, py, pz, 3.f) < 1.5f) {
        bool ok = false;
        for (float rad = 1; rad <= 30 && !ok; rad += 1)
            for (int a = 0; a < 24 && !ok; ++a)
                for (float dz = 0; dz <= 10 && !ok; dz += 1) {
                    float th = a * sim::PI_F / 12.f;
                    float tx = px + rad*std::cos(th), ty = py + rad*std::sin(th), tz = pz + dz;
                    if (trueClearance(W, tx, ty, tz, 3.f) >= 1.5f) { px=tx; py=ty; pz=tz; ok=true; }
                }
    }

    CamParams cp; DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = cell;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    VoxelMap M; M.init(mp, px, py, pz);
    GeneralParams gp; gp.robotR = 0.6f;
    GeneralPlanner gen(gp);
    ForwardParams fwp; fwp.robotR = gp.robotR;
    ForwardPath path;

    float vx=0, vy=0, vz=0, yaw=0, travelled=0, minClear=1e9f;
    bool paused = false, collided = false;
    float isoYaw = 0.f; bool spin = true;   // voxel-model view angle
    std::vector<cv::Point2f> trail;
    std::vector<cv::Point3f> trail3;        // same path, with height, for the 3D view
    const float dt = 0.1f;
    // PLAYBACK RATE, separate from the aircraft's speed, and the two were being
    // confused. One sim step is dt = 0.1 s of flight, and a step costs roughly
    // 80 ms to compute, so the window has been running at very nearly 1x real
    // time all along -- what looked "very high" was the commanded 6 m/s, now
    // 3 m/s. Rate is here so you can slow the world down to watch a decision,
    // which is a different need from making the aircraft fly slower.
    //   rate < 0  -> as fast as the machine will go (batch-like)
    static const float RATES[] = {0.25f, 0.5f, 1.f, 2.f, 4.f, -1.f};
    int rateIdx = 2;
    // Display pitch of the voxel-model pane, metres per drawn block. Not the
    // map resolution -- see VoxelMap::isoImage. Coarser means bigger, clearer
    // blocks and more of the map on screen; finer means the real detail the
    // map holds, at the cost of legibility.
    // Block pitch and render distance are stepped TOGETHER: a 20 m view wants
    // small blocks and a 60 m view wants big ones, and letting them drift apart
    // just gives you the two bad combinations as well as the two good ones.
    struct View { float blockM, spanM; const char* name; };
    static const View VIEWS[] = {
        {0.5f, 20.f, "20 m / 0.5 m"},
        {1.0f, 32.f, "32 m / 1.0 m"},
        {1.5f, 44.f, "44 m / 1.5 m"},
        {2.0f, 60.f, "60 m / 2.0 m"},
    };
    int blockIdx = 2;
    cv::Mat top(W.ny(), W.nx(), CV_8UC3);
    (void)0;

    for (int s = 0; s < cfg.steps; ++s) {
        const int64 tStep = cv::getTickCount();
        CamPose pose; pose.e=px; pose.n=py; pose.u=pz;
        pose.yawDeg=yaw; pose.pitchDeg=-5; pose.rollDeg=0;
        cv::Mat d = cfg.truth ? cam.renderTruth(W, pose) : cam.renderStereo(W, pose, nullptr);
        M.integrate(d, cam, pose);
        M.recentre(px, py, pz);

        if (s % 25 == 0) {
            float mAz = std::atan2(goalE-px, goalN-py) * 180.f/sim::PI_F;
            float mEl = std::atan2(goalU-pz, std::hypot(goalE-px, goalN-py)) * 180.f/sim::PI_F;
            path = planForward(M, px, py, pz, mAz, mEl, fwp);
        }
        float tE=goalE, tN=goalN, tU=goalU;
        if (path.found)
            for (const auto& w : path.pts)
                if (std::hypot(w[0]-px, w[1]-py) > 3.f) { tE=w[0]; tN=w[1]; tU=w[2]; break; }
        float gAz = std::atan2(tE-px, tN-py) * 180.f/sim::PI_F;
        float gEl = std::atan2(tU-pz, std::hypot(tE-px, tN-py)) * 180.f/sim::PI_F;
        GeneralResult gr = gen.plan(M, px, py, pz, gAz, gEl);

        float a = gr.azDeg*sim::PI_F/180.f, e = gr.elDeg*sim::PI_F/180.f;
        float dx = std::cos(e)*std::sin(a), dy = std::cos(e)*std::cos(a), dz = std::sin(e);
        float k = std::min(1.f, dt/0.35f);
        vx += (dx*gr.speed - vx)*k; vy += (dy*gr.speed - vy)*k; vz += (dz*gr.speed - vz)*k;
        px += vx*dt; py += vy*dt; pz += vz*dt;
        travelled += std::sqrt(vx*vx+vy*vy+vz*vz)*dt;
        if (std::hypot(vx,vy) > 0.2f) yaw = std::atan2(vx,vy)*180.f/sim::PI_F;
        trail.push_back({px, py});
        trail3.push_back({px, py, pz});
        float clr = trueClearance(W, px, py, pz, 2.f);
        minClear = std::min(minClear, clr);
        if (clr <= gp.robotR*0.5f) collided = true;

        // ---- draw ----
        int zc; { int aa,bb; W.worldToCell(0,0,pz,aa,bb,zc); }
        top.setTo(cv::Scalar(250,248,245));
        for (int yy=0; yy<W.ny(); ++yy)
            for (int xx=0; xx<W.nx(); ++xx)
                if (W.solid(xx,yy,zc)) top.at<cv::Vec3b>(W.ny()-1-yy,xx) = cv::Vec3b(70,70,70);
        // Trails first, under everything else -- they are terrain, not a result.
        for (const auto& tr : trails)
            for (size_t i=1;i<tr.size();++i){int x0,y0,z0,x1,y1,z1;
                W.worldToCell(tr[i-1][0],tr[i-1][1],pz,x0,y0,z0);
                W.worldToCell(tr[i][0],tr[i][1],pz,x1,y1,z1);
                cv::line(top,{x0,W.ny()-1-y0},{x1,W.ny()-1-y1},{150,205,225},
                         std::max(2,int(3.5f/cell)));}
        if (path.found)
            for (size_t i=1;i<path.pts.size();++i){int x0,y0,z0,x1,y1,z1;
                W.worldToCell(path.pts[i-1][0],path.pts[i-1][1],pz,x0,y0,z0);
                W.worldToCell(path.pts[i][0],path.pts[i][1],pz,x1,y1,z1);
                cv::line(top,{x0,W.ny()-1-y0},{x1,W.ny()-1-y1},{40,170,40},2);}
        for (size_t i=1;i<trail.size();++i){int x0,y0,z0,x1,y1,z1;
            W.worldToCell(trail[i-1].x,trail[i-1].y,pz,x0,y0,z0);
            W.worldToCell(trail[i].x,trail[i].y,pz,x1,y1,z1);
            cv::line(top,{x0,W.ny()-1-y0},{x1,W.ny()-1-y1},{40,40,220},2);}
        { int gx,gy,gz; W.worldToCell(goalE,goalN,pz,gx,gy,gz);
          cv::circle(top,{gx,W.ny()-1-gy},9,{40,180,40},2); }
        cv::Mat topV; cv::resize(top, topV, {440,440}, 0,0, cv::INTER_AREA);
        cv::Mat sliceV = M.sliceImage(pz, 440);
        cv::Mat dv(d.rows,d.cols,CV_8UC3,cv::Scalar(60,60,60));
        for (int yy=0;yy<d.rows;++yy) for (int xx=0;xx<d.cols;++xx){
            float r=d.at<float>(yy,xx); if(!(r>0))continue; float f=std::min(1.f,r/cp.maxRangeM);
            dv.at<cv::Vec3b>(yy,xx)=cv::Vec3b(uchar(255*(1-f)),uchar(80+100*f),uchar(255*f));}
        cv::Mat depthV; cv::resize(dv, depthV, {440,440}, 0,0, cv::INTER_NEAREST);
        cv::putText(topV,"TRUTH + path",{10,22},cv::FONT_HERSHEY_SIMPLEX,0.55,{30,30,30},2);
        cv::putText(sliceV,"VOXEL MAP  (grey=unknown)",{10,22},cv::FONT_HERSHEY_SIMPLEX,0.5,{30,30,30},2);
        cv::putText(depthV,cfg.truth?"DEPTH (perfect)":"DEPTH (stereo)",{10,22},
                    cv::FONT_HERSHEY_SIMPLEX,0.55,{240,240,240},2);
        // THE VOXEL MODEL ITSELF, in 3D. The slice above shows one height; this
        // shows the whole structure the aircraft has actually built, which is
        // the only view where "the map looks nothing like the world" is obvious
        // at a glance.
        if (spin) isoYaw += 0.4f;
        VoxelMap::IsoView iv;
        cv::Mat isoV = M.isoImage(440, 40.f, isoYaw, &iv,
                                  VIEWS[blockIdx].blockM, VIEWS[blockIdx].spanM);
        // The plan, drawn in the same 3D space as the blocks. Without this the
        // voxel pane shows WHAT the aircraft believes but not what it intends,
        // and the two together are the only way to see a bad decision being
        // made against a correct map -- which is the failure that matters.
        {
            // Only draw what is inside the MAP, not merely inside the image.
            // The map is a window that scrolls with the aircraft, so a point
            // the aircraft passed a minute ago still projects to a perfectly
            // reasonable pixel -- over empty background, implying the map holds
            // structure there when it has long since been scrolled away.
            auto onScreen = [&](const cv::Point2f& q) {
                return q.x > 0 && q.y > 0 && q.x < isoV.cols && q.y < isoV.rows;
            };
            auto inMap = [&](float wx, float wy, float wz) {
                int cx, cy, cz; M.worldToCell(wx, wy, wz, cx, cy, cz);
                return M.inBounds(cx, cy, cz);
            };
            size_t from = trail3.size() > 600 ? trail3.size() - 600 : 0;
            for (size_t i = from + 1; i < trail3.size(); ++i) {
                if (!inMap(trail3[i].x, trail3[i].y, trail3[i].z)) continue;
                cv::Point2f a0 = iv.project(trail3[i-1].x, trail3[i-1].y, trail3[i-1].z);
                cv::Point2f a1 = iv.project(trail3[i].x,   trail3[i].y,   trail3[i].z);
                if (!onScreen(a0) || !onScreen(a1)) continue;
                cv::line(isoV, a0, a1, {60, 60, 225}, 2, cv::LINE_AA);
            }
            if (path.found)
                for (size_t i = 1; i < path.pts.size(); ++i) {
                    cv::Point2f a0 = iv.project(path.pts[i-1][0], path.pts[i-1][1], path.pts[i-1][2]);
                    cv::Point2f a1 = iv.project(path.pts[i][0],   path.pts[i][1],   path.pts[i][2]);
                    if (!onScreen(a0) || !onScreen(a1)) continue;
                    cv::line(isoV, a0, a1, {40, 190, 40}, 2, cv::LINE_AA);
                }
            // Commanded heading: a stub from the aircraft along the direction
            // the reactive layer actually chose, length = the confirmed-free
            // run. This is the single most diagnostic thing in the window --
            // when it swings wildly you are watching the churn directly.
            {
                float ca = gr.azDeg*sim::PI_F/180.f, ce = gr.elDeg*sim::PI_F/180.f;
                float L = std::max(1.f, gr.freeM);
                cv::Point2f a0 = iv.project(px, py, pz);
                cv::Point2f a1 = iv.project(px + std::cos(ce)*std::sin(ca)*L,
                                            py + std::cos(ce)*std::cos(ca)*L,
                                            pz + std::sin(ce)*L);
                if (onScreen(a0) && onScreen(a1))
                    cv::arrowedLine(isoV, a0, a1, {20, 140, 245}, 2, cv::LINE_AA, 0, 0.25);
                if (onScreen(a0)) cv::circle(isoV, a0, 5, {20, 20, 30}, cv::FILLED, cv::LINE_AA);
            }
        }
        char isoLbl[96];
        std::snprintf(isoLbl,sizeof isoLbl,"VOXEL MODEL (built)   %s",
                      VIEWS[blockIdx].name);
        cv::putText(isoV,isoLbl,{10,22},cv::FONT_HERSHEY_SIMPLEX,0.5,{30,30,30},2);
        cv::putText(isoV,"red = at your altitude   green below   blue above",{10,400},
                    cv::FONT_HERSHEY_SIMPLEX,0.40,{110,110,120},1);
        cv::putText(isoV,"blue line flown   green plan   orange commanded",{10,416},
                    cv::FONT_HERSHEY_SIMPLEX,0.40,{110,110,120},1);
        cv::putText(isoV,"[<-/->] rotate  [s] spin  [ / ] view distance",{10,432},
                    cv::FONT_HERSHEY_SIMPLEX,0.42,{110,110,120},1);
        cv::Mat rowA, rowB, row;
        cv::hconcat(std::vector<cv::Mat>{topV,isoV}, rowA);
        cv::hconcat(std::vector<cv::Mat>{sliceV,depthV}, rowB);
        cv::vconcat(rowA, rowB, row);
        cv::Mat bar(64, row.cols, CV_8UC3, cv::Scalar(25,25,30));
        char l1[260], l2[260];
        char rl[24];
        if (RATES[rateIdx] < 0) std::snprintf(rl,sizeof rl,"max");
        else std::snprintf(rl,sizeof rl,"%gx", RATES[rateIdx]);
        std::snprintf(l1,sizeof l1,"%s  seed %d  %s%s   step %d/%d   %.1f m/s   flown %.0f m   [-/+] %s",
                      wc.label, cfg.seed, cfg.truth?"perfect depth":"stereo",
                      (cfg.trail && !trails.empty())?"  trail":"",
                      s, cfg.steps, std::hypot(vx,vy), travelled, rl);
        std::snprintf(l2,sizeof l2,"free %.1f m   open %.1f m   %s   path %s   min clearance %.2f m%s",
                      gr.freeM, gr.openM, gr.blocked?"BLOCKED":"ok",
                      path.found?"yes":"none", minClear, collided?"   *** COLLIDED ***":"");
        cv::putText(bar,l1,{12,24},cv::FONT_HERSHEY_SIMPLEX,0.46,{235,235,235},1,cv::LINE_AA);
        cv::putText(bar,l2,{12,48},cv::FONT_HERSHEY_SIMPLEX,0.46,
                    collided?cv::Scalar(90,90,250):cv::Scalar(190,190,200),1,cv::LINE_AA);
        cv::Mat full; cv::vconcat(row, bar, full);
        cv::imshow(WIN, full);

        // Spend whatever is left of this step's wall-clock budget inside
        // waitKey, so the window stays responsive while it waits. A step costs
        // ~80 ms of real compute against a 100 ms budget at 1x, so at 2x and
        // above this is a no-op and the sim simply runs flat out.
        int wait = 1;
        if (RATES[rateIdx] > 0) {
            double spentMs = double(cv::getTickCount() - tStep)
                           / cv::getTickFrequency() * 1000.0;
            double budgetMs = double(dt) * 1000.0 / RATES[rateIdx];
            wait = std::max(1, int(budgetMs - spentMs));
        }
        int key = cv::waitKey(paused ? 0 : wait);
        if (key=='q'||key==27) return 0;
        if (key=='m') return 1;
        if (key=='r') return 2;
        if (key==' ') paused = !paused;
        if (key=='s') spin = !spin;
        if (key=='-'||key=='_') rateIdx = std::max(0, rateIdx-1);
        if (key=='+'||key=='=') rateIdx = std::min(int(sizeof(RATES)/sizeof(*RATES))-1, rateIdx+1);
        if (key=='[') blockIdx = std::max(0, blockIdx-1);
        if (key==']') blockIdx = std::min(int(sizeof(VIEWS)/sizeof(*VIEWS))-1, blockIdx+1);
        if (key==81 || key=='a') { isoYaw -= 6.f; spin = false; }   // left arrow
        if (key==83 || key=='d') { isoYaw += 6.f; spin = false; }   // right arrow
        if (collided) {                       // hold on the crash so it can be seen
            for (;;) { int kk = cv::waitKey(0);
                if (kk=='q'||kk==27) return 0; if (kk=='m') return 1; if (kk=='r') return 2; }
        }
    }
    for (;;) { int kk = cv::waitKey(0);
        if (kk=='q'||kk==27) return 0; if (kk=='m') return 1; if (kk=='r') return 2; }
}

int main() {
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(WIN, onMouse);
    Cfg cfg;
    for (;;) {
        if (!menu(cfg)) break;
        int r;
        do { r = fly(cfg); } while (r == 2);
        if (r == 0) break;
    }
    cv::destroyAllWindows();
    return 0;
}

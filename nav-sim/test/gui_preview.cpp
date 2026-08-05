// Render the voxel_gui flight layout to a PNG, headlessly.
//
// The GUI is the deliverable most likely to be judged by looking at it, and it
// is the one thing in this tree that cannot be checked over ssh, in CI, or by
// anyone who has not built it yet. So produce the same 2x2 composite the window
// shows, write it to a file, and let a picture be reviewable evidence rather
// than a claim.
//
// This is deliberately NOT voxel_gui refactored into a library: the GUI's own
// loop stays simple, and a preview that shares no code with it also cannot
// inherit its bugs silently -- if the two ever disagree, that is information.
//
//   ./gui_preview /tmp/preview.png [seed] [steps]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "depth_camera.hpp"
#include "ompl_planner.hpp"
#include "voxel_map.hpp"
#include "voxel_planner.hpp"
#include "voxel_world.hpp"

using namespace sim;

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

int main(int argc, char** argv) {
    const std::string out = argc > 1 ? argv[1] : "/tmp/preview.png";
    const unsigned seed   = argc > 2 ? unsigned(std::atoi(argv[2])) : 1u;
    const int steps       = argc > 3 ? std::atoi(argv[3]) : 320;
    const float cell = 0.25f;

    // Deterministic sampling planner: same seed -> same flight.
    setPlannerSeed(unsigned(seed));
    VoxelWorld W;
    std::vector<Trail> trails;
    ForestParams fp; fp.cell = cell; fp.seed = seed;
    genForest(W, fp, &trails);

    float px = 15, py = 10, pz = 6, goalE = 120, goalN = 150, goalU = 8;
    if (!trails.empty()) {
        px = trails[0].front()[0]; py = trails[0].front()[1]; pz = 5.5f;
        goalE = trails[0].back()[0]; goalN = trails[0].back()[1]; goalU = 5.5f;
    }

    CamParams cp; DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = cell;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    VoxelMap M; M.init(mp, px, py, pz);
    GeneralParams gp; gp.robotR = 0.6f;
    GeneralPlanner gen(gp);
    ForwardParams fwp; fwp.robotR = gp.robotR;
    ForwardPath path;
    BearingFilter gfilt;
    StallMonitor stall;

    float vx=0, vy=0, vz=0, yaw=0, travelled=0, minClear=1e9f;
    const float dt = 0.1f;
    std::vector<cv::Point2f> trail;
    std::vector<cv::Point3f> trail3;
    cv::Mat d;
    GeneralResult gr;

    for (int s = 0; s < steps; ++s) {
        CamPose pose; pose.e=px; pose.n=py; pose.u=pz;
        pose.yawDeg=yaw; pose.pitchDeg=-5;
        d = cam.renderStereo(W, pose, nullptr);
        M.integrate(d, cam, pose);
        M.recentre(px, py, pz);
        // Same steering as voxel_sim and voxel_gui: replan on demand, aim by
        // pure pursuit, low-pass the reference bearing.
        float mAz = std::atan2(goalE-px, goalN-py) * 180.f/sim::PI_F;
        float mEl = std::atan2(goalU-pz, std::hypot(goalE-px, goalN-py)) * 180.f/sim::PI_F;
        // The router is a FALLBACK, not the normal case. Following a routed
        // path measures worse than pointing at the goal in open forest; a 12 m
        // reactive horizon cannot see out of a dead end. So run reactive, and
        // call the router only once progress has actually stalled.
        stall.update(std::hypot(goalE-px, goalN-py));
        if (!stall.engaged) path = ForwardPath();
        else if (!pathStillGood(M, path, px, py, pz, mAz, fwp))
            path = planForward(M, px, py, pz, mAz, mEl, fwp);
        float tE=goalE, tN=goalN, tU=goalU;
        if (path.found) pursuitPoint(path, px, py, pz, 6.f, tE, tN, tU);
        gfilt.update(std::atan2(tE-px, tN-py) * 180.f/sim::PI_F,
                     std::atan2(tU-pz, std::hypot(tE-px, tN-py)) * 180.f/sim::PI_F, 0.25f);
        gr = gen.plan(M, px, py, pz, gfilt.azDeg, gfilt.elDeg);
        float a = gr.azDeg*sim::PI_F/180.f, e = gr.elDeg*sim::PI_F/180.f;
        float dx = std::cos(e)*std::sin(a), dy = std::cos(e)*std::cos(a), dz = std::sin(e);
        float k = std::min(1.f, dt/0.35f);
        vx += (dx*gr.speed - vx)*k; vy += (dy*gr.speed - vy)*k; vz += (dz*gr.speed - vz)*k;
        px += vx*dt; py += vy*dt; pz += vz*dt;
        travelled += std::sqrt(vx*vx+vy*vy+vz*vz)*dt;
        if (std::hypot(vx,vy) > 0.2f) yaw = std::atan2(vx,vy)*180.f/sim::PI_F;
        trail.push_back({px,py}); trail3.push_back({px,py,pz});
        minClear = std::min(minClear, trueClearance(W, px, py, pz, 2.f));
    }

    // --- the same four panes voxel_gui shows ---
    int zc; { int aa,bb; W.worldToCell(0,0,pz,aa,bb,zc); }
    // Band + dilate, matching voxel_gui -- see the comment there. A single
    // slice resized 1.8x renders a 1200 stems/ha forest as an empty field.
    cv::Mat top(W.ny(), W.nx(), CV_8UC3, cv::Scalar(250,248,245));
    {
        const int band = std::max(1, int(1.5f / cell));
        cv::Mat occ(W.ny(), W.nx(), CV_8U, cv::Scalar(0));
        for (int yy=0; yy<W.ny(); ++yy) {
            uchar* orow = occ.ptr<uchar>(W.ny()-1-yy);
            for (int xx=0; xx<W.nx(); ++xx)
                for (int dz=-band; dz<=band; ++dz)
                    if (W.solid(xx,yy,zc+dz)) { orow[xx] = 255; break; }
        }
        cv::dilate(occ, occ, cv::Mat());
        top.setTo(cv::Scalar(70,70,70), occ);
    }
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
            cv::line(top,{x0,W.ny()-1-y0},{x1,W.ny()-1-y1},{40,170,40},3);}
    for (size_t i=1;i<trail.size();++i){int x0,y0,z0,x1,y1,z1;
        W.worldToCell(trail[i-1].x,trail[i-1].y,pz,x0,y0,z0);
        W.worldToCell(trail[i].x,trail[i].y,pz,x1,y1,z1);
        cv::line(top,{x0,W.ny()-1-y0},{x1,W.ny()-1-y1},{40,40,220},3);}
    { int gx,gy,gz; W.worldToCell(goalE,goalN,pz,gx,gy,gz);
      cv::circle(top,{gx,W.ny()-1-gy},9,{40,180,40},3); }
    cv::Mat topV; cv::resize(top, topV, {440,440}, 0,0, cv::INTER_AREA);

    VoxelMap::IsoView iv;
    cv::Mat isoV = M.isoImage(440, 40.f, 30.f, &iv, 1.5f, 44.f);
    auto on = [&](const cv::Point2f& q){ return q.x>0 && q.y>0 && q.x<440 && q.y<440; };
    auto inMap = [&](const cv::Point3f& p){
        int a,b,c; M.worldToCell(p.x,p.y,p.z,a,b,c); return M.inBounds(a,b,c); };
    for (size_t i = trail3.size()>600 ? trail3.size()-600 : 1; i<trail3.size(); ++i) {
        if (!inMap(trail3[i])) continue;
        cv::Point2f a0=iv.project(trail3[i-1].x,trail3[i-1].y,trail3[i-1].z);
        cv::Point2f a1=iv.project(trail3[i].x,trail3[i].y,trail3[i].z);
        if (on(a0)&&on(a1)) cv::line(isoV,a0,a1,{60,60,225},2,cv::LINE_AA);
    }
    if (path.found)
        for (size_t i=1;i<path.pts.size();++i){
            cv::Point2f a0=iv.project(path.pts[i-1][0],path.pts[i-1][1],path.pts[i-1][2]);
            cv::Point2f a1=iv.project(path.pts[i][0],path.pts[i][1],path.pts[i][2]);
            if (on(a0)&&on(a1)) cv::line(isoV,a0,a1,{40,190,40},2,cv::LINE_AA);
        }
    {   float ca=gr.azDeg*sim::PI_F/180.f, ce=gr.elDeg*sim::PI_F/180.f;
        float L=std::max(1.f,gr.freeM);
        cv::Point2f a0=iv.project(px,py,pz);
        cv::Point2f a1=iv.project(px+std::cos(ce)*std::sin(ca)*L,
                                  py+std::cos(ce)*std::cos(ca)*L, pz+std::sin(ce)*L);
        if (on(a0)&&on(a1)) cv::arrowedLine(isoV,a0,a1,{20,140,245},2,cv::LINE_AA,0,0.25);
        if (on(a0)) cv::circle(isoV,a0,5,{20,20,30},cv::FILLED,cv::LINE_AA);
    }

    // First person, through the map. maxRange is set from what the map can
    // honestly know rather than from what looks good -- see fpvImage.
    cv::Mat fpv = M.fpvImage(px, py, pz, yaw, -5.f, 440, 90.f, 25.f);

    cv::Mat sliceV = M.sliceImage(pz, 440);
    cv::Mat dv(d.rows,d.cols,CV_8UC3,cv::Scalar(60,60,60));
    for (int yy=0;yy<d.rows;++yy) for (int xx=0;xx<d.cols;++xx){
        float r=d.at<float>(yy,xx); if(!(r>0))continue;
        float f=std::min(1.f,r/cp.maxRangeM);
        dv.at<cv::Vec3b>(yy,xx)=cv::Vec3b(uchar(255*(1-f)),uchar(80+100*f),uchar(255*f));}
    cv::Mat depthV; cv::resize(dv, depthV, {440,440}, 0,0, cv::INTER_NEAREST);

    cv::putText(topV,"TRUTH + path",{10,22},cv::FONT_HERSHEY_SIMPLEX,0.55,{30,30,30},2);
    cv::putText(isoV,"VOXEL MODEL (built)   44 m / 1.5 m",{10,22},
                cv::FONT_HERSHEY_SIMPLEX,0.5,{30,30,30},2);
    cv::putText(isoV,"red = at your altitude   green below   blue above",{10,400},
                cv::FONT_HERSHEY_SIMPLEX,0.40,{110,110,120},1);
    cv::putText(isoV,"blue line flown   green plan   orange commanded",{10,416},
                cv::FONT_HERSHEY_SIMPLEX,0.40,{110,110,120},1);
    cv::putText(sliceV,"VOXEL MAP  (grey=unknown)",{10,22},
                cv::FONT_HERSHEY_SIMPLEX,0.5,{30,30,30},2);
    cv::putText(depthV,"DEPTH (stereo)",{10,22},cv::FONT_HERSHEY_SIMPLEX,0.55,{240,240,240},2);

    cv::putText(fpv,"FIRST PERSON (the map, from the aircraft)",{10,22},
                cv::FONT_HERSHEY_SIMPLEX,0.45,{30,30,30},2);
    cv::putText(fpv,"pale = unknown, not empty",{10,42},
                cv::FONT_HERSHEY_SIMPLEX,0.42,{60,60,70},1);

    // Legend rather than a dead pane. The colour keys are the part of these
    // views a newcomer gets wrong, and a legend beside them costs nothing.
    cv::Mat key(440,440,CV_8UC3,cv::Scalar(30,30,36));
    {
        const char* lines[] = {
            "WHAT YOU ARE LOOKING AT",
            "",
            "TRUTH        the real world at flight height.",
            "             pale corridors are forest trails.",
            "             red flown, green planned, circle goal.",
            "",
            "VOXEL MODEL  the map the aircraft built, from",
            "             outside. blocks are a display pitch,",
            "             not the map resolution.",
            "",
            "FIRST PERSON the same map, from inside it.",
            "             this is what the aircraft 'sees'.",
            "             pale is UNKNOWN, never empty --",
            "             fog you can see beats a false-free",
            "             percentage you have to interpret.",
            "",
            "DEPTH        what stereo returned. grey = no match.",
            "",
            "VOXEL MAP    horizontal slice. white free,",
            "             black occupied, GREY UNKNOWN.",
            "",
            "COLOUR KEY   red is at your altitude and is what",
            "             you would hit. green below, blue above.",
        };
        int y = 40;
        for (const char* s : lines) {
            cv::putText(key, s, {16, y}, cv::FONT_HERSHEY_SIMPLEX, 0.38,
                        s[0] && s[1] && s[0] >= 'A' && s[0] <= 'Z' && s[1] >= 'A' && s[1] <= 'Z'
                            ? cv::Scalar(235,235,240) : cv::Scalar(165,165,180),
                        1, cv::LINE_AA);
            y += 18;
        }
    }

    cv::Mat rowA, rowB, rowC, row;
    cv::hconcat(std::vector<cv::Mat>{topV,isoV}, rowA);
    cv::hconcat(std::vector<cv::Mat>{fpv,depthV}, rowB);
    cv::hconcat(std::vector<cv::Mat>{sliceV,key}, rowC);
    cv::vconcat(std::vector<cv::Mat>{rowA, rowB, rowC}, row);
    cv::Mat bar(64, row.cols, CV_8UC3, cv::Scalar(25,25,30));
    char l1[260], l2[260];
    std::snprintf(l1,sizeof l1,"Forest  seed %u  stereo  trail   step %d/%d   %.1f m/s   flown %.0f m   [-/+] 1x",
                  seed, steps, steps, std::hypot(vx,vy), travelled);
    std::snprintf(l2,sizeof l2,"free %.1f m   open %.1f m   %s   path %s   min clearance %.2f m",
                  gr.freeM, gr.openM, gr.blocked?"BLOCKED":"ok",
                  path.found?"yes":"none", minClear);
    cv::putText(bar,l1,{12,24},cv::FONT_HERSHEY_SIMPLEX,0.46,{235,235,235},1,cv::LINE_AA);
    cv::putText(bar,l2,{12,48},cv::FONT_HERSHEY_SIMPLEX,0.46,{190,190,200},1,cv::LINE_AA);
    cv::Mat full; cv::vconcat(row, bar, full);
    cv::imwrite(out, full);
    std::printf("wrote %s  (%dx%d)  flown %.0f m, min clearance %.2f m\n",
                out.c_str(), full.cols, full.rows, travelled, minClear);

    // Score the map against truth, and count what is actually MARKED. The
    // first-person view showed fog where the depth image plainly had trunks,
    // and "that is the honest sensor horizon" is a comfortable explanation that
    // has to be checked rather than assumed -- an under-marking map would look
    // exactly the same and would be a real bug.
    VoxelMap::Score sc = M.score(W, px, py, pz, 12.f, pz + 10.f);
    long occCells = 0;
    for (int z = 0; z < mp.nz; ++z)
        for (int y = 0; y < mp.ny; ++y)
            for (int x = 0; x < mp.nx; ++x)
                if (M.logAt(x,y,z) > mp.occThresh) ++occCells;
    std::printf("  map: %ld cells marked OCCUPIED of %d\n",
                occCells, mp.nx*mp.ny*mp.nz);
    std::printf("  within 12 m: observed %ld of %ld (%.0f%% coverage), "
                "occupied TP %ld FP %ld FN %ld, IoU %.2f, false-free %.2f%%\n",
                sc.observed, sc.total, 100.0*sc.coverage(),
                sc.occTP, sc.occFP, sc.occFN, sc.iou(), 100.0*sc.falseFreeRate());
    return 0;
}

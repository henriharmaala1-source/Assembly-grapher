#include "nav_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

VoxelMapParams fineMapParams(const DepthCamera& cam, float cell, int stride) {
    const CamParams& cp = cam.params();
    VoxelMapParams mp;
    mp.cell = cell;
    mp.depthSigCoef = 0.25f / (cam.fpx() * cp.baselineM);
    mp.maxIntegM = std::sqrt(cell * cam.fpx() * cp.baselineM / 0.25f) * 0.75f;
    mp.integrateStride = stride;
    mp.minIntegM = cam.fpx() * cp.baselineM / 126.f * 1.2f;
    return mp;
}

void NavPipeline::init(const DepthCamera& cam, const NavPipelineParams& p,
                       const CamPose& origin) {
    cam_ = &cam;
    p_ = p;
    mp_ = fineMapParams(cam, p.cell, p.stride);

    TrajParams tp;
    tp.robotR = p.robotR;
    tp.vMax = p.vMax;
    tp.dt = 0.1f;
    tp.goalWeight = p.goalWeight;
    tp.coreFrac = p.coreFrac;
    traj_.reset(new TrajectoryPlanner(tp));

    reset(origin);
}

void NavPipeline::reset(const CamPose& origin) {
    map_.init(mp_, origin.e, origin.n, origin.u);
    if (p_.seedBodyM > 0.f)
        map_.seedFree(origin.e, origin.n, origin.u, p_.seedBodyM);
    BearingFieldParams bp;
    bp.maxRangeM = 30.f;
    bp.minFillFrac = p_.fillFrac;
    bfield_.init(bp);
    // The planner too: it holds its last choice for hysteresis, and a choice
    // made from another vantage is not a reason to prefer anything here.
    if (traj_) traj_.reset(new TrajectoryPlanner(traj_->params()));
    frames_ = 0;
}

float NavPipeline::straightFreeM(const CamPose& from, float azDeg,
                                 float maxM) const {
    if (!cam_) return 0.f;
    // NOTHING PAST THE MARKING RANGE. Returns beyond maxIntegM are never
    // marked OCCUPIED -- that is what the honest range means -- but the rays
    // that pass BESIDE an object out there still carve FREE up to maxCarveM.
    // So a trunk at 3 m, with a 2.5 m marking range, sits in the map as a
    // column of UNKNOWN inside a sea of FREE, and a coreFrac-0 ball walks
    // straight past it. Found by test_voxel_nav's closed loop: legs certified
    // to 5.25 m whose ground-truth clearance was 0.10 m. A ball is only
    // evidence where every cell it touches COULD have been marked.
    maxM = std::min(maxM, mp_.maxIntegM - p_.robotR);
    const float a = azDeg * 3.14159265f / 180.f;
    const float de = std::sin(a), dn = std::cos(a);
    const float step = mp_.cell * 0.5f;     // no sample skips a whole cell
    float d = 0.f;
    for (float t = step; t <= maxM + 1e-4f; t += step) {
        const float x = from.e + de * t, y = from.n + dn * t, z = from.u;
        if (!map_.sphereClear(x, y, z, p_.robotR, p_.coreFrac)) break;
        if (map_.stateAt(x, y, z) != VoxelMap::FREE) break;
        if (p_.legCoreM > 0.f &&
            !coreFree(x, y, z, p_.legCoreM, from.e, from.n)) break;
        d = t;
    }
    return d;
}

bool NavPipeline::coreFree(float x, float y, float z, float r,
                           float sx, float sy) const {
    const float h = mp_.cell * 0.5f;
    int x0, y0, x1, y1, cz, dummy;
    map_.worldToCell(x - r, y - r, z, x0, y0, cz);
    map_.worldToCell(x + r, y + r, z, x1, y1, dummy);
    for (int cy = y0; cy <= y1; ++cy)
        for (int cx = x0; cx <= x1; ++cx) {
            float bx, by, bz; map_.cellCentre(cx, cy, cz, bx, by, bz);
            // Nearest point of the cell's square to (x,y): box, not centre --
            // the same correction sphereClear needed.
            const float nx = std::max(0.f, std::fabs(x - bx) - h);
            const float ny = std::max(0.f, std::fabs(y - by) - h);
            if (nx * nx + ny * ny > r * r) continue;
            const float qx = std::max(0.f, std::fabs(sx - bx) - h);
            const float qy = std::max(0.f, std::fabs(sy - by) - h);
            if (qx * qx + qy * qy <= r * r) continue;         // standing in it
            if (!map_.inBounds(cx, cy, cz)) return false;     // unknown
            if (map_.stateAt(bx, by, bz) != VoxelMap::FREE) return false;
        }
    return true;
}

GeneralResult NavPipeline::step(const cv::Mat& depthM, const CamPose& pose) {
    if (!cam_ || depthM.empty()) {
        GeneralResult r; r.blocked = true; return r;
    }
    map_.integrate(depthM, *cam_, pose);
    // A no-op while the pose is fixed -- recentre returns early on a zero
    // shift -- and required the moment a translation estimate exists.
    map_.recentre(pose.e, pose.n, pose.u);
    bfield_.update(depthM, *cam_, pose, 1);
    ++frames_;
    // Awareness comes from the bearing field; it scores a DIRECTION and
    // nothing else. Every safety test in plan() reads map_ alone.
    const TrajectoryPlanner::FarBearings fb{&bfield_, p_.farRangeM};
    return traj_->plan(map_, pose.e, pose.n, pose.u, pose.yawDeg,
                       pose.yawDeg, 0.f, {}, &fb);
}

}  // namespace sim

#include "nav_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

VoxelMapParams fineMapParams(const DepthCamera& cam, float cell, int stride,
                             float subpixelPx) {
    const CamParams& cp = cam.params();
    const float s = std::max(0.02f, subpixelPx);
    VoxelMapParams mp;
    mp.cell = cell;
    mp.depthSigCoef = s / (cam.fpx() * cp.baselineM);
    mp.maxIntegM = std::sqrt(cell * cam.fpx() * cp.baselineM / s) * 0.75f;
    mp.integrateStride = stride;
    mp.minIntegM = cam.fpx() * cp.baselineM / 126.f * 1.2f;
    return mp;
}

std::vector<float> obstacleDistanceFromFrame(const cv::Mat& depthM,
                                             const DepthCamera& cam,
                                             const CamPose& attitude,
                                             int bins, float elBandDeg,
                                             int minPixels, int stride,
                                             float maxM) {
    bins = std::max(1, bins);
    std::vector<float> out(size_t(bins), -1.f);
    if (depthM.empty() || depthM.type() != CV_32F) return out;
    // Body frame: position at the origin, heading zero, so an azimuth is a
    // bearing from the nose. Roll and pitch stay: they decide which pixels
    // look level.
    CamPose body;
    body.rollDeg = attitude.rollDeg;
    body.pitchDeg = attitude.pitchDeg;
    // Per-sector histogram at 0.1 m: robust k-th nearest without sorting.
    const float res = 0.1f;
    const int nb = std::max(1, int(maxM / res) + 1);
    std::vector<int> hist(size_t(bins) * nb, 0), count(size_t(bins), 0);
    const float sinBand = std::sin(elBandDeg * 3.14159265f / 180.f);
    stride = std::max(1, stride);
    for (int v = 0; v < depthM.rows; v += stride) {
        const float* row = depthM.ptr<float>(v);
        for (int u = 0; u < depthM.cols; u += stride) {
            const float r = row[u];
            if (!(r > 0.f)) continue;
            float dx, dy, dz;
            cam.rayFor(body, u, v, dx, dy, dz);        // unit ray, body frame
            if (std::fabs(dz) > sinBand) continue;       // not at our height
            const float horiz = r * std::sqrt(std::max(0.f, 1.f - dz * dz));
            if (horiz > maxM) continue;
            float az = std::atan2(dx, dy) * 180.f / 3.14159265f;   // cw from nose
            if (az < 0.f) az += 360.f;
            const int b = int(az / (360.f / float(bins)) + 0.5f) % bins;
            ++hist[size_t(b) * nb + std::min(nb - 1, int(horiz / res))];
            ++count[size_t(b)];
        }
    }
    for (int b = 0; b < bins; ++b) {
        if (count[size_t(b)] < minPixels) continue;      // unknown, not clear
        int acc = 0;
        for (int i = 0; i < nb; ++i) {
            acc += hist[size_t(b) * nb + i];
            if (acc >= minPixels) { out[size_t(b)] = (float(i) + 0.5f) * res; break; }
        }
    }
    return out;
}

void NavPipeline::init(const DepthCamera& cam, const NavPipelineParams& p,
                       const CamPose& origin) {
    cam_ = &cam;
    p_ = p;
    mp_ = fineMapParams(cam, p.cell, p.stride, p.subpixelPx);

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

cv::Mat NavPipeline::renderFpv(const CamPose& pose, int w, int h,
                               float hfovDeg) const {
    return renderFpv(map_, mp_.maxIntegM, bfield_, p_.farRangeM, pose, w, h, hfovDeg);
}

cv::Mat NavPipeline::renderFpv(const VoxelMap& map, float maxIntegM,
                               const BearingField& field, float farRangeM,
                               const CamPose& pose, int w, int h, float hfovDeg) {
    const std::vector<VoxelMap::Layer> fine{{&map, 0.f, maxIntegM}};
    cv::Mat maskNear, maskFar;
    cv::Mat fpv = VoxelMap::renderLadder(fine, pose.e, pose.n, pose.u,
                                         pose.yawDeg, pose.pitchDeg, w, h,
                                         hfovDeg, FpvStyle(), &maskNear);
    const cv::Mat far_ = BearingField::render(field, pose.yawDeg,
                                              pose.pitchDeg, w, h, hfovDeg,
                                              maxIntegM, farRangeM,
                                              pose.u, &maskFar);
    for (int v = 0; v < fpv.rows; ++v) {
        const uchar* mn = maskNear.ptr<uchar>(v);
        const uchar* mf = maskFar.ptr<uchar>(v);
        const cv::Vec3b* fr = far_.ptr<cv::Vec3b>(v);
        cv::Vec3b* out = fpv.ptr<cv::Vec3b>(v);
        for (int u = 0; u < fpv.cols; ++u)
            if (!mn[u] && mf[u]) out[u] = fr[u];
    }
    return fpv;
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

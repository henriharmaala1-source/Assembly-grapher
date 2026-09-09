// Side-by-side: what the fine map knows versus what the coarse map knows.
//
//   ./far_map_demo out.png [seed] [steps]
//
// The claim being illustrated is about RANGE, so a first-person view is the
// wrong picture -- it shows what is close, which is the part both maps agree
// about. This renders both maps isometrically at the same span with range rings
// drawn on, and prints occupied cells as a function of distance, which is the
// number the picture is a picture of.
//
// Why a coarse map is not a compromise: depth error grows as Z^2, so
// Z_max = sqrt(cell*f*B/sigma). At 0.25 m cells on a 12 cm baseline that is
// 5.2 m; at 2 m cells it is 14.8 m. A return at 12 m genuinely has metres of
// uncertainty along the ray, and a 0.25 m voxel claims a precision the
// measurement does not contain. Sizing the cell to the uncertainty is the
// honest thing to do and it happens to triple the range.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

namespace {

// WHAT DOES EACH MAP KNOW ABOUT THE SPACE AHEAD?
//
// The first version of this counted occupied cells by range and was wrong
// twice over. It reported fine-map cells at 24 m -- impossible for an 8 m
// marking limit -- because the map is egocentric and scrolls, so those were
// marked when the aircraft was closer and left behind. And comparing raw cell
// COUNTS between maps of different resolution is meaningless anyway: one 2 m
// cell is 512 of the 0.25 m ones by volume.
//
// The question that matters is coverage: of the volume in front of the
// aircraft, how much does each map have an OPINION about? Unknown space is
// what the planner refuses to fly into, so unknown-ahead is the cost being
// paid, and it is what a longer honest range is supposed to buy back.
struct Coverage { long total = 0, known = 0, occupied = 0; };

Coverage forwardCoverage(const VoxelMap& m, float px, float py, float pz,
                         float yawDeg, float nearM, float farM) {
    Coverage c;
    const float a = yawDeg * sim::PI_F / 180.f;
    const float fx = std::sin(a), fy = std::cos(a);
    // A wedge +-35 deg about the heading, +-4 m vertically -- roughly the
    // volume the aircraft could reach in the next few seconds.
    for (float r = nearM; r <= farM; r += 0.5f)
        for (float th = -35.f; th <= 35.f; th += 2.f)
            for (float dz = -4.f; dz <= 4.f; dz += 0.5f) {
                float t = a + th * sim::PI_F / 180.f;
                float qx = px + std::sin(t) * r, qy = py + std::cos(t) * r, qz = pz + dz;
                ++c.total;
                VoxelMap::State s = m.stateAt(qx, qy, qz);
                if (s != VoxelMap::UNKNOWN) ++c.known;
                if (s == VoxelMap::OCCUPIED) ++c.occupied;
            }
    (void)fx; (void)fy;
    return c;
}

void drawRing(cv::Mat& img, const VoxelMap::IsoView& iv,
              float px, float py, float pz, float radiusM,
              const cv::Scalar& col, const char* label) {
    std::vector<cv::Point> pts;
    for (int a = 0; a <= 72; ++a) {
        float th = a * 2.f * sim::PI_F / 72.f;
        cv::Point2f q = iv.project(px + radiusM * std::cos(th),
                                   py + radiusM * std::sin(th), pz);
        pts.push_back(cv::Point(int(q.x), int(q.y)));
    }
    for (size_t i = 1; i < pts.size(); ++i)
        cv::line(img, pts[i-1], pts[i], col, 1, cv::LINE_AA);
    cv::putText(img, label, pts[18] + cv::Point(4, -4),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string out = argc > 1 ? argv[1] : "/tmp/far_map.png";
    const unsigned seed   = argc > 2 ? unsigned(std::atoi(argv[2])) : 1u;
    const int steps       = argc > 3 ? std::atoi(argv[3]) : 400;

    VoxelWorld W;
    ForestParams fp; fp.cell = 0.25f; fp.seed = seed;
    std::vector<Trail> trails;
    genForest(W, fp, &trails);

    float px = 15, py = 10, pz = 6;
    if (!trails.empty()) { px = trails[0].front()[0]; py = trails[0].front()[1]; pz = 5.5f; }

    CamParams cp; DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = 0.25f;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    VoxelMap M; M.init(mp, px, py, pz);

    VoxelMapParams fp2;
    fp2.cell = 2.0f; fp2.nx = 128; fp2.ny = 128; fp2.nz = 40;
    fp2.maxIntegM = 14.f; fp2.maxCarveM = 40.f;
    fp2.integrateStride = 4; fp2.carveWinPx = 0;
    fp2.depthSigCoef = mp.depthSigCoef;
    VoxelMap Mfar; Mfar.init(fp2, px, py, pz);

    // Fly the trail on rails. This demo is about what the two maps CONTAIN, and
    // running the planner would make the answer depend on where it chose to go.
    float yaw = 0;
    if (!trails.empty()) {
        const Trail& t = trails[0];
        size_t seg = 0; float u = 0;
        for (int s = 0; s < steps && seg + 1 < t.size(); ++s) {
            float ax=t[seg][0], ay=t[seg][1], bx=t[seg+1][0], by=t[seg+1][1];
            float L = std::hypot(bx-ax, by-ay);
            px = ax + (bx-ax)*u; py = ay + (by-ay)*u;
            yaw = std::atan2(bx-ax, by-ay) * 180.f / sim::PI_F;
            CamPose pose; pose.e=px; pose.n=py; pose.u=pz;
            pose.yawDeg=yaw; pose.pitchDeg=-5;
            cv::Mat d = cam.renderStereo(W, pose, nullptr);
            M.integrate(d, cam, pose);      M.recentre(px, py, pz);
            Mfar.integrate(d, cam, pose);   Mfar.recentre(px, py, pz);
            u += 0.3f / std::max(0.1f, L);
            while (u >= 1.f && seg + 1 < t.size()) { u -= 1.f; ++seg; }
        }
    }

    std::printf("coverage of the forward wedge -- what each map has an OPINION about\n\n");
    std::printf("%-12s%12s%12s%12s%12s\n", "band", "fine known", "fine occ", "coarse known", "coarse occ");
    float bands[][2] = {{2,6},{6,10},{10,14},{14,20}};
    long fKnownFar = 0, cKnownFar = 0, fTotFar = 0;
    for (auto& b : bands) {
        Coverage cf = forwardCoverage(M,    px, py, pz, yaw, b[0], b[1]);
        Coverage cc = forwardCoverage(Mfar, px, py, pz, yaw, b[0], b[1]);
        std::printf("%-12s%11.0f%%%11.0f%%%11.0f%%%11.0f%%\n",
            (std::to_string(int(b[0])) + "-" + std::to_string(int(b[1])) + " m").c_str(),
            100.0*cf.known/std::max(1L,cf.total), 100.0*cf.occupied/std::max(1L,cf.total),
            100.0*cc.known/std::max(1L,cc.total), 100.0*cc.occupied/std::max(1L,cc.total));
        if (b[0] >= 8) { fKnownFar += cf.known; cKnownFar += cc.known; fTotFar += cf.total; }
    }
    std::printf("\nbeyond 8 m: fine knows %.0f%% of the wedge, coarse knows %.0f%%\n",
                100.0*fKnownFar/std::max(1L,fTotFar), 100.0*cKnownFar/std::max(1L,fTotFar));

    const int S = 560;
    VoxelMap::IsoView ivF, ivC;
    cv::Mat fine   = M.isoImage(S, 40.f, 30.f, &ivF, 0.5f, 40.f);
    cv::Mat coarse = Mfar.isoImage(S, 40.f, 30.f, &ivC, 2.0f, 40.f);

    // Range rings at each map's honest limit, so the extent claim is legible
    // rather than asserted.
    drawRing(fine,   ivF, px, py, pz, mp.maxIntegM,  {90, 90, 220}, "8 m");
    drawRing(fine,   ivF, px, py, pz, fp2.maxIntegM, {170,170,190}, "14 m");
    drawRing(coarse, ivC, px, py, pz, mp.maxIntegM,  {90, 90, 220}, "8 m");
    drawRing(coarse, ivC, px, py, pz, fp2.maxIntegM, {40, 150, 40}, "14 m");

    cv::putText(fine,  "FINE  0.25 m voxels   honest to 5.2 m, marks to 8 m",
                {12, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.48, {30,30,30}, 1, cv::LINE_AA);
    cv::putText(coarse,"COARSE  2 m voxels    honest to 14.8 m, marks to 14 m",
                {12, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.48, {30,30,30}, 1, cv::LINE_AA);

    cv::Mat row; cv::hconcat(fine, coarse, row);
    cv::Mat bar(96, row.cols, CV_8UC3, cv::Scalar(25,25,30));
    char l1[300], l2[300];
    Coverage cfF = forwardCoverage(M,    px, py, pz, yaw, 10.f, 20.f);
    Coverage ccF = forwardCoverage(Mfar, px, py, pz, yaw, 10.f, 20.f);
    long fBeyond = lround(100.0*cfF.known/std::max(1L,cfF.total));
    long cBeyond = lround(100.0*ccF.known/std::max(1L,ccF.total));
    std::snprintf(l1, sizeof l1,
        "Same flight, same depth images, two voxel sizes.  Z_max = sqrt(cell*f*B/sigma):"
        "  0.25 m -> 5.2 m,   2 m -> 14.8 m");
    std::snprintf(l2, sizeof l2,
        "coverage of the wedge 10-20 m ahead:   fine %ld%%    coarse %ld%%"
        "        (the coarse cell is not less accurate out there -- the measurement is)",
        fBeyond, cBeyond);
    cv::putText(bar, l1, {14, 34}, cv::FONT_HERSHEY_SIMPLEX, 0.46, {225,225,230}, 1, cv::LINE_AA);
    cv::putText(bar, l2, {14, 66}, cv::FONT_HERSHEY_SIMPLEX, 0.46, {150,210,150}, 1, cv::LINE_AA);
    cv::Mat full; cv::vconcat(row, bar, full);
    cv::imwrite(out, full);
    std::printf("wrote %s (%dx%d)\n", out.c_str(), full.cols, full.rows);
    return 0;
}

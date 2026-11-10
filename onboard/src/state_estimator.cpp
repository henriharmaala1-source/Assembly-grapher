#include "state_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kEarthR = 6378137.0;          // WGS84 equatorial radius (m)
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

void StateEstimator::reset() {
    x_ = cv::Mat::zeros(6, 1, CV_64F);
    P_ = cv::Mat::eye(6, 6, CV_64F) * 1e6;     // unknown until first fix
    origin_ = Origin{};
    init_        = false;
    headingDeg_  = 0.f;
    gpsAgeS_     = 1e9f;
    everGps_     = false;
    glitchCount_ = 0;
}

// ----------------------------------------------------------------- geodesy
void StateEstimator::geoToEnu_(double lat, double lon, float altM,
                               float& pe, float& pn, float& pu) const {
    const double clat = std::cos(origin_.lat0 * kDeg2Rad);
    pe = (float)((lon - origin_.lon0) * kDeg2Rad * kEarthR * clat);
    pn = (float)((lat - origin_.lat0) * kDeg2Rad * kEarthR);
    pu = altM - origin_.alt0;
}

void StateEstimator::enuToGeo_(float pe, float pn, float pu,
                               double& lat, double& lon, float& altM) const {
    const double clat = std::cos(origin_.lat0 * kDeg2Rad);
    lat  = origin_.lat0 + (pn / kEarthR) * kRad2Deg;
    lon  = origin_.lon0 + (pe / (kEarthR * clat)) * kRad2Deg;
    altM = origin_.alt0 + pu;
}

// ----------------------------------------------------------------- predict
void StateEstimator::predict(float dt) {
    if (!init_ || dt <= 0.f) { if (init_) gpsAgeS_ += std::max(dt, 0.f); return; }
    const double d = dt;

    // F = [[I3, dt*I3],[0, I3]] — constant-velocity.
    cv::Mat F = cv::Mat::eye(6, 6, CV_64F);
    for (int i = 0; i < 3; ++i) F.at<double>(i, i + 3) = d;

    x_ = F * x_;

    // Q from continuous white-noise acceleration (PSD = sigmaAccel²), blocked
    // by [position | velocity] grouping rather than per-axis interleave.
    const double q   = (double)p_.sigmaAccel * p_.sigmaAccel;
    const double qpp = q * d * d * d * d / 4.0;
    const double qpv = q * d * d * d / 2.0;
    const double qvv = q * d * d;
    cv::Mat Q = cv::Mat::zeros(6, 6, CV_64F);
    for (int i = 0; i < 3; ++i) {
        Q.at<double>(i, i)         = qpp;
        Q.at<double>(i, i + 3)     = qpv;
        Q.at<double>(i + 3, i)     = qpv;
        Q.at<double>(i + 3, i + 3) = qvv;
    }

    P_ = F * P_ * F.t() + Q;
    gpsAgeS_ += dt;
}

// ------------------------------------------------------------- generic update
void StateEstimator::update_(const cv::Mat& H, const cv::Mat& z, const cv::Mat& R) {
    const cv::Mat y = z - H * x_;               // innovation
    const cv::Mat S = H * P_ * H.t() + R;
    const cv::Mat K = P_ * H.t() * S.inv(cv::DECOMP_SVD);
    x_ += K * y;
    cv::Mat I = cv::Mat::eye(6, 6, CV_64F);
    P_ = (I - K * H) * P_;
    P_ = 0.5 * (P_ + P_.t());                   // keep symmetric
}

// ----------------------------------------------------------------- GPS
void StateEstimator::initFromGps_(double lat, double lon, float altM) {
    origin_ = Origin{lat, lon, altM, true};
    x_ = cv::Mat::zeros(6, 1, CV_64F);          // we are at the origin
    P_ = cv::Mat::eye(6, 6, CV_64F);
    for (int i = 0; i < 3; ++i) P_.at<double>(i, i)         = 4.0;    // 2 m pos
    for (int i = 3; i < 6; ++i) P_.at<double>(i, i)         = 1.0;    // 1 m/s vel
    init_    = true;
    everGps_ = true;
    gpsAgeS_ = 0.f;
}

void StateEstimator::updateGps(double lat, double lon, float altM, int fixType,
                               bool haveVel, float velN, float velE, float velD,
                               float ephM, float epvM) {
    if (fixType < 3) return;                     // iNAV-style: need a 3-D fix
    if (!origin_.set) { initFromGps_(lat, lon, altM); return; }

    float pe, pn, pu;
    geoToEnu_(lat, lon, altM, pe, pn, pu);

    // Glitch gate: reject an implausible jump while GPS has been continuous,
    // mirroring iNAV's 2.5 m radius. A jump after a long gap is a re-acquire and
    // is allowed through. If many fixes in a row "glitch", our estimate is the
    // thing that's wrong — snap the position to the GPS and re-acquire.
    const float dx = pe - (float)x_.at<double>(0);
    const float dy = pn - (float)x_.at<double>(1);
    const bool continuous = gpsAgeS_ < 1.0f;
    bool glitch = continuous &&
                  std::sqrt(dx * dx + dy * dy) > p_.glitchRadiusM;

    if (glitch && ++glitchCount_ >= 5) {        // persistent → estimate is stale
        x_.at<double>(0) = pe; x_.at<double>(1) = pn; x_.at<double>(2) = pu;
        for (int i = 0; i < 3; ++i) P_.at<double>(i, i) = 4.0;   // re-inflate pos cov
        glitch = false;
        glitchCount_ = 0;
    }
    if (!glitch) {
        glitchCount_ = 0;
        cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
        for (int i = 0; i < 3; ++i) H.at<double>(i, i) = 1.0;
        cv::Mat z = (cv::Mat_<double>(3, 1) << pe, pn, pu);
        const double eh = std::max(0.3f, ephM), ev = std::max(0.5f, epvM);
        cv::Mat R = (cv::Mat_<double>(3, 3) <<
                     eh * eh, 0, 0,  0, eh * eh, 0,  0, 0, ev * ev);
        update_(H, z, R);
    }

    if (haveVel) {                               // ENU = (E, N, U) from NED
        cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
        for (int i = 0; i < 3; ++i) H.at<double>(i, i + 3) = 1.0;
        cv::Mat z = (cv::Mat_<double>(3, 1) << velE, velN, -velD);
        const double s = p_.sigmaFcVel;
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F) * (s * s);
        update_(H, z, R);
    }

    gpsAgeS_ = 0.f;
    everGps_ = true;
}

void StateEstimator::updateBaro(float altM) {
    if (!init_) return;
    cv::Mat H = cv::Mat::zeros(1, 6, CV_64F);
    H.at<double>(0, 2) = 1.0;                     // pu
    cv::Mat z = (cv::Mat_<double>(1, 1) << (altM - origin_.alt0));
    cv::Mat R = (cv::Mat_<double>(1, 1) << (double)p_.sigmaBaro * p_.sigmaBaro);
    update_(H, z, R);
}

void StateEstimator::updateFcVelocity(float velN, float velE, float velD) {
    if (!init_) return;
    cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
    for (int i = 0; i < 3; ++i) H.at<double>(i, i + 3) = 1.0;
    cv::Mat z = (cv::Mat_<double>(3, 1) << velE, velN, -velD);
    const double s = p_.sigmaFcVel;
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F) * (s * s);
    update_(H, z, R);
}

void StateEstimator::updateVisionVelocity(float vRight, float vFwd, float vUp) {
    if (!init_) return;
    const float h = headingDeg_ * (float)kDeg2Rad;   // 0 = North, clockwise
    const float s = std::sin(h), c = std::cos(h);
    const float ve = vFwd * s + vRight * c;          // forward=(sin,cos), right=(cos,-sin)
    const float vn = vFwd * c - vRight * s;
    cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
    for (int i = 0; i < 3; ++i) H.at<double>(i, i + 3) = 1.0;
    cv::Mat z = (cv::Mat_<double>(3, 1) << ve, vn, vUp);
    const double sv = p_.sigmaVoVel;
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F) * (sv * sv);
    update_(H, z, R);
}

void StateEstimator::updateVisionPose(float pe, float pn, float pu, float sigmaM) {
    if (!init_) return;
    cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
    for (int i = 0; i < 3; ++i) H.at<double>(i, i) = 1.0;
    cv::Mat z = (cv::Mat_<double>(3, 1) << pe, pn, pu);
    const double s = std::max(0.1f, sigmaM);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F) * (s * s);
    update_(H, z, R);
}

// ----------------------------------------------------------------- outputs
StateEstimator::State StateEstimator::state() const {
    State st;
    if (!init_) return st;
    st.valid = true;
    st.pe = (float)x_.at<double>(0); st.pn = (float)x_.at<double>(1);
    st.pu = (float)x_.at<double>(2);
    st.ve = (float)x_.at<double>(3); st.vn = (float)x_.at<double>(4);
    st.vu = (float)x_.at<double>(5);
    st.headingDeg = headingDeg_;
    st.speedMs    = std::sqrt(st.ve * st.ve + st.vn * st.vn);
    st.ephM = (float)std::sqrt(0.5 * (P_.at<double>(0, 0) + P_.at<double>(1, 1)));
    st.epvM = (float)std::sqrt(P_.at<double>(2, 2));
    st.gpsDenied = gpsAgeS_ > p_.gpsTimeoutS;
    st.gpsAgeS   = gpsAgeS_;
    enuToGeo_(st.pe, st.pn, st.pu, st.lat, st.lon, st.altM);
    return st;
}

bool StateEstimator::makeExtGps(ExtGps& out) const {
    if (!init_) return false;
    const State st = state();
    out.lat     = st.lat;
    out.lon     = st.lon;
    out.altMslM = st.altM;
    out.velN    = st.vn;
    out.velE    = st.ve;
    out.velD    = -st.vu;
    out.ephM    = clampf(st.ephM, 0.5f, 50.f);   // honest, but iNAV-trustable
    out.epvM    = clampf(st.epvM, 0.8f, 50.f);
    out.hdop    = clampf(st.ephM / 2.0f, 0.7f, 9.9f);
    out.yawDeg  = st.headingDeg;
    out.fixType = 3;
    out.sats    = 12;
    return true;
}

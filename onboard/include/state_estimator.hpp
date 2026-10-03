#pragma once

#include <opencv2/core.hpp>

#include "control_types.hpp"

// Loosely-coupled Kalman state estimator for the companion computer.
//
// This is NOT a re-implementation of the FC's own filter. iNAV already fuses
// raw IMU + GPS + baro into a good attitude and a complementary-filter position
// estimate. The Pi-side estimator fuses the things the FC can't on its own —
// chiefly VISUAL ODOMETRY / SLAM — with GPS and baro into ONE locally-smooth,
// globally-drift-corrected pose at loop rate, and produces a synthetic GPS fix
// (ExtGps) that can be fed back to iNAV so its nav modes work GPS-denied.
//
//   state x = [pe, pn, pu, ve, vn, vu]   (local ENU metres, m/s)
//   - origin anchored at the first 3-D fix; pe=East, pn=North, pu=Up.
//   - constant-velocity process model; unmodelled accel becomes process noise.
//   - roll/pitch/yaw are taken DIRECTLY from FC telemetry (not estimated) —
//     the FC's Mahony AHRS beats anything we'd derive casually. Heading is
//     stored so vision body-frame velocities can be rotated into ENU.
//
// All the nonlinearity (geodetic projection, body→world rotation) is handled at
// the measurement boundary, so the core filter stays linear and well-conditioned
// — a deliberate engineering choice over a fragile fully-nonlinear EKF.
//
// Measurement sources (each optional, fused when available):
//   updateGps()            — absolute position (+velocity), glitch-gated
//   updateBaro()           — vertical position
//   updateFcVelocity()     — FC ground velocity as a velocity pseudo-measurement
//   updateVisionVelocity() — body-frame VO velocity rotated to ENU   [P5 hook]
//   updateVisionPose()     — absolute canopy-localization fix         [P5 hook]
class StateEstimator {
public:
    struct Params {
        float sigmaAccel   = 0.6f;   // process accel noise (m/s²) — model slack
        float sigmaBaro    = 0.6f;   // baro altitude meas noise (m)
        float sigmaFcVel   = 0.6f;   // FC ground-velocity meas noise (m/s)
        float sigmaVoVel   = 0.25f;  // vision-odometry velocity noise (m/s)
        float gpsTimeoutS  = 1.5f;   // matches iNAV INAV_GPS_TIMEOUT_MS
        float glitchRadiusM = 2.5f;  // matches iNAV INAV_GPS_GLITCH_RADIUS
    };

    StateEstimator() { reset(); }
    explicit StateEstimator(Params p) : p_(p) { reset(); }

    void reset();

    // --- time update -------------------------------------------------------
    void predict(float dt);

    // --- measurement updates ----------------------------------------------
    // GPS: lat/lon in degrees, altM metres. Sets the origin on the first 3-D
    // fix. velN/E/D optional (haveVel). Position jumps beyond the glitch radius
    // are rejected (the estimate keeps coasting), mirroring iNAV.
    void updateGps(double lat, double lon, float altM, int fixType,
                   bool haveVel, float velN, float velE, float velD,
                   float ephM, float epvM);

    void updateBaro(float altM);
    void updateFcVelocity(float velN, float velE, float velD);

    // VO body-frame velocity (x=right, y=forward, z=up) rotated into ENU using
    // the current heading. The integration point for the P5 SLAM front-end.
    void updateVisionVelocity(float vRight, float vFwd, float vUp);

    // Absolute world-frame position fix (e.g. canopy-silhouette localization).
    void updateVisionPose(float pe, float pn, float pu, float sigmaM);

    void setHeading(float yawDeg) { headingDeg_ = yawDeg; }

    // --- outputs -----------------------------------------------------------
    struct State {
        bool   valid     = false;  // origin set and filter initialised
        double lat        = 0.0;   // estimate reprojected to global
        double lon        = 0.0;
        float  altM       = 0.f;
        float  pe = 0, pn = 0, pu = 0;   // local ENU position (m)
        float  ve = 0, vn = 0, vu = 0;   // local ENU velocity (m/s)
        float  headingDeg = 0.f;
        float  ephM = 0.f, epvM = 0.f;   // 1σ position uncertainty (m)
        float  speedMs    = 0.f;         // horizontal speed
        bool   gpsDenied  = false;       // coasting without a GPS fix
        float  gpsAgeS    = 0.f;         // time since last accepted GPS
    };
    State state() const;

    bool hasOrigin() const { return origin_.set; }

    // Build a synthetic GPS fix from the current estimate for MSP2_SENSOR_GPS.
    // Returns false until the estimate is usable (origin set, recent data).
    bool makeExtGps(ExtGps& out) const;

private:
    struct Origin { double lat0 = 0, lon0 = 0; float alt0 = 0; bool set = false; };

    void initFromGps_(double lat, double lon, float altM);
    void update_(const cv::Mat& H, const cv::Mat& z, const cv::Mat& R);
    void geoToEnu_(double lat, double lon, float altM,
                   float& pe, float& pn, float& pu) const;
    void enuToGeo_(float pe, float pn, float pu,
                   double& lat, double& lon, float& altM) const;

    Params   p_;
    Origin   origin_;
    cv::Mat  x_;          // 6×1 state
    cv::Mat  P_;          // 6×6 covariance
    bool     init_       = false;
    float    headingDeg_ = 0.f;
    float    gpsAgeS_    = 1e9f;
    bool     everGps_    = false;
    int      glitchCount_ = 0;     // consecutive rejected fixes → forces re-acquire
};

#include "kalman_center.hpp"

KalmanCenter::KalmanCenter() : kf_(4, 2, 0, CV_32F) {
    // Constant-velocity transition: x' = x + vx, y' = y + vy
    kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Observe only position, not velocity
    kf_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    cv::setIdentity(kf_.processNoiseCov,     cv::Scalar(3e-2f));  // Q
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(1e-1f));  // R
    cv::setIdentity(kf_.errorCovPost,        cv::Scalar(1.0f));   // P
}

void KalmanCenter::init(cv::Point2f pos) {
    kf_.statePost  = (cv::Mat_<float>(4, 1) << pos.x, pos.y, 0.f, 0.f);
    kf_.statePre   = kf_.statePost.clone();
    cv::setIdentity(kf_.errorCovPost, cv::Scalar(1.0f));
    initialized_ = true;
}

cv::Point2f KalmanCenter::predict() {
    cv::Mat p = kf_.predict();
    return {p.at<float>(0), p.at<float>(1)};
}

cv::Point2f KalmanCenter::correct(cv::Point2f m) {
    cv::Mat meas = (cv::Mat_<float>(2, 1) << m.x, m.y);
    cv::Mat s    = kf_.correct(meas);
    return {s.at<float>(0), s.at<float>(1)};
}

cv::Point2f KalmanCenter::position() const {
    return {kf_.statePost.at<float>(0), kf_.statePost.at<float>(1)};
}

cv::Point2f KalmanCenter::velocity() const {
    return {kf_.statePost.at<float>(2), kf_.statePost.at<float>(3)};
}

cv::Point2f KalmanCenter::project(float steps) const {
    const auto p = position(), v = velocity();
    return {p.x + v.x * steps, p.y + v.y * steps};
}

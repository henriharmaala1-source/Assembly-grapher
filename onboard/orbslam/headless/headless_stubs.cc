// SPDX-License-Identifier: GPL-3.0-or-later
// HEADLESS BUILD of ORB-SLAM3: Viewer and MapDrawer without drawing.
//
// Replaces src/Viewer.cc and src/MapDrawer.cc (see fetch_orbslam3.sh). The
// core calls into both from Tracking and System; every call is kept and does
// nothing. The Viewer is never constructed -- the bridge passes
// bUseViewer=false -- but its methods must still link.
#include "MapDrawer.h"
#include "Viewer.h"

namespace ORB_SLAM3 {

MapDrawer::MapDrawer(Atlas* pAtlas, const string&, Settings*) : mpAtlas(pAtlas) {}
void MapDrawer::newParameterLoader(Settings*) {}
bool MapDrawer::ParseViewerParamFile(cv::FileStorage&) { return true; }
void MapDrawer::DrawMapPoints() {}
void MapDrawer::DrawKeyFrames(const bool, const bool, const bool, const bool) {}
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix&) {}
void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f& Tcw) {
    std::unique_lock<std::mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}
void MapDrawer::SetReferenceKeyFrame(KeyFrame*) {}
void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix& M,
                                             pangolin::OpenGlMatrix& MOw) {
    M.SetIdentity(); MOw.SetIdentity();
}

Viewer::Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer,
               Tracking* pTracking, const string&, Settings*)
    : both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer), mpTracker(pTracking), mbFinishRequested(false),
      mbFinished(true), mbStopped(true), mbStopRequested(false) {}
void Viewer::newParameterLoader(Settings*) {}
bool Viewer::ParseViewerParamFile(cv::FileStorage&) { return true; }
void Viewer::Run() { SetFinish(); }
void Viewer::RequestFinish() { std::unique_lock<std::mutex> l(mMutexFinish); mbFinishRequested = true; }
bool Viewer::CheckFinish() { std::unique_lock<std::mutex> l(mMutexFinish); return mbFinishRequested; }
void Viewer::SetFinish() { std::unique_lock<std::mutex> l(mMutexFinish); mbFinished = true; }
bool Viewer::isFinished() { std::unique_lock<std::mutex> l(mMutexFinish); return mbFinished; }
void Viewer::RequestStop() { std::unique_lock<std::mutex> l(mMutexStop); mbStopRequested = true; }
bool Viewer::isStopped() { std::unique_lock<std::mutex> l(mMutexStop); return true; }
bool Viewer::isStepByStep() { return false; }
bool Viewer::Stop() { return true; }
void Viewer::Release() { std::unique_lock<std::mutex> l(mMutexStop); mbStopped = false; }

}  // namespace ORB_SLAM3

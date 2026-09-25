// THE DEMO. Four things at once, on a laptop, in one window.
//
//   kestrel demo                  sim everywhere -- runs on any machine
//   kestrel demo --live           depth and the detector from a RealSense
//   kestrel demo --replay w.kdr   a recording, for when the camera is packed
//
// WHAT IT SHOWS, and why these four:
//
//   SIM DEMONSTRATION a planner flying the simulated world: FPV FOOTAGE of
//                     the true scene (navcore/footage.hpp), with the map it is
//                     building inset -- first-person, as it BELIEVES, and from
//                     above with its track. Pale is UNKNOWN. The gap between
//                     footage and map is what the aircraft has measured. It is
//                     the only pane that needs no camera. Flies freeM, the
//                     best planner measured here, unless --model names a
//                     learned policy -- and the caption always says which.
//   LIVE DEPTH        what the real sensor returns, colourised. The one part
//                     of the stack that can lie, and the one that has to be
//                     watched rather than trusted.
//   LIVE VOXEL        FIRST PERSON, from the camera: that live depth through
//                     navcore's NavPipeline -- the aircraft's own map and far
//                     tier -- drawn as voxel_live draws it, the fine map to
//                     its honest range and the bearing field beyond. Grey is
//                     unknown here too. With no camera it is a simulated
//                     D435i on the sim aircraft, running the same path.
//   HUMANS            people found in the camera image, each labelled with a
//                     range read out of the depth frame. A box alone is a
//                     webcam trick; a box that says "2.3 m" is this stack.
//
// FOUR PANES, FOUR THREADS, and that is not decoration. Each stage has its own
// natural rate -- the sim steps as fast as the planner allows, the camera
// arrives at 30 Hz whatever we do, a person detector is tens of milliseconds
// per frame, and the window wants to redraw smoothly regardless. Running them
// in one loop makes every pane as slow as the slowest, which on a laptop is
// the detector, and a 6 Hz FPV looks broken. Each stage therefore owns a
// thread and publishes its latest finished frame; the compositor takes what is
// there and never waits.
//
// NOTHING HERE IMPLEMENTS ANYTHING. The planner pane steps `VoxelEnv`, the
// depth comes from `FrameSource`, the map is `VoxelMap`, the FPV is
// `renderFrame` -- the same code the measurements in docs/ were taken with.
// A demo that reimplements the stack is a demo of the demo.
//
// IT IS CHECKABLE WITHOUT A CAMERA OR A DISPLAY. `demo --shot PREFIX` writes
// every pane and the composed window to PNG from synthetic input, and
// `demo --check` asserts the layout, both with no device attached. That is the
// same property `gui --check` and `report --check` have, and it exists for the
// same reason: this is reviewed over ssh.
#pragma once

#include <string>
#include <vector>

namespace kdemo {

// Parsed `kestrel demo` arguments. Defaults are the ones that run anywhere.
// How long the tour stays on one map when nothing ends it sooner. Long enough
// to watch a room-to-room route develop, short enough that a viewer who walks
// up sees the other world within a few minutes.
constexpr double kTourMapS = 180.0;

// The world episode `ep` of a demo flies: `world` itself, or for "tour" the
// showcase worlds in turn.
std::string worldForEpisode(const std::string& world, int ep);

struct Options {
    // Where the depth for the two live panes comes from.
    enum Source { SIM = 0, REPLAY, LIVE };
    int         source = SIM;
    std::string replayPath;

    // The policy for the sim pane. Empty means "no network available" and the
    // pane flies a classical planner, saying so on screen. A demo that silently
    // shows freeM while the caption says "learned" is the worst thing this
    // file could do.
    std::string model;          // .onnx exported from the checkpoint
    // freeM, not cover: it is the planner CLAUDE.md measures as best on every
    // column that matters -- travel, net displacement, coverage, no crashes.
    // cover was fifth of ten. The showcase flies the best thing there is.
    std::string fallback = "freeM";   // a BaselinePolicy name

    // GALLERY BY DEFAULT, not forest. Through the 2.0/1.0/0.25 m FPV ladder a
    // forest is mush -- a 0.3 m trunk is one voxel on the fine rung and absent
    // on the coarse one -- so the view appears to change as you approach it,
    // which is indistinguishable from a broken map. gallery is built on the
    // coarse rung's own lattice; see GalleryParams in voxel_world.hpp.
    //
    // "tour" (the default) flies BOTH worlds built for this view in turn:
    // gallery, then hall, then gallery again on a new layout, and so on until
    // the window is closed. A map ends on a collision, after kTourMapS, or on
    // [r]; a named world stays that world and only its layout changes.
    std::string world = "tour";
    unsigned    seed = 101;
    int         maxSteps = 0;   // 0 = never stop; the demo loops forever

    // Camera stream the detector reads. AUTO prefers colour, then infrared,
    // then any webcam videoio can open -- so an unplugged camera costs the
    // demo one pane and not the demo.
    enum Eyes { AUTO = 0, COLOUR, INFRARED, WEBCAM, NOEYES };
    int  eyes = AUTO;
    int  webcamIndex = 0;
    std::string detector;       // .onnx person detector; empty = built-in HOG

    // CUDA FOR THE TWO NETWORKS, when there is a GPU and OpenCV was built to
    // use it. AUTO takes it if it is there, which is what a demo wants; ON
    // fails loudly if it is not, which is what a rehearsal wants; OFF is the
    // control you need to claim a speedup at all.
    //
    // It does NOT touch the sim's depth renderer. That has its own switch
    // (NAVSIM_WITH_CUDA) guarding a kernel whose own header says it has never
    // been compiled or run, gated by cuda_depth_check -- turning that on from
    // here would be shipping an unvalidated kernel inside the one command
    // whose job is to be believed.
    enum Cuda { CUDA_AUTO = 0, CUDA_ON, CUDA_OFF };
    int  cuda = CUDA_AUTO;

    int  camW = 640, camH = 480, camFps = 30;
    bool emitter = true;

    // WHERE THE CAMERA IS for the live voxel pane: "fixed" (the map is built
    // where the camera stands -- move it and the map smears), "vio" (DepthVio
    // on its IR), "slam" (ORB-SLAM3 via a running kestrel-orbslam). navcore's
    // VisualPose, the object the aircraft runs. With the emitter on and a
    // tracker, the emitter STROBES and only dark frames are tracked.
    std::string pose = "fixed";
    std::string slamSocket = "/tmp/kestrel-slam.sock";

    int  paneW = 480, paneH = 360;   // one pane; the window is 2x2 of these
    bool mirror = true;              // the human pane, so waving matches
};

// Parse; returns false and fills `err` on a bad argument.
bool parse(const std::vector<std::string>& args, Options& o, std::string& err);

// Run the window until q/ESC. Returns a process exit code, or -1 when this
// build has no highgui -- the caller then says so rather than exiting, exactly
// as `gui` does.
int run(const Options& o);

// Every pane plus the composed window, as PREFIX_<name>.png, from synthetic
// input. No camera, no display, no policy. Returns how many were written.
int shot(const Options& o, const std::string& prefix);

// Assert the layout: panes inside the canvas, no pane overlapping another,
// every caption inside its pane, no caption elided to nothing. Prints each
// violation and returns the count. Runs in ctest.
int check();

}  // namespace kdemo

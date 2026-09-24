# kestrel-orbslam: ORB-SLAM3 for the aircraft

ORB-SLAM3 (stereo, or stereo-inertial) on the D435i's IR pair, as the
aircraft's position estimate when there is no GPS. It is the most widely used
ready-made SLAM with an official D435i example, and it tolerates our flight
profile well: slow legs and full stops do not starve it, and stereo gets
metric scale from the 50 mm baseline without needing motion to initialise.

## How it fits

```
 D435i ──► onboard (kestrel --voxel --voxel-slam)
             │  depth ──► voxel map, proximity        (as before)
             │  IR left + IR right (+ IMU), dark frames only
             ▼  Unix socket, navcore/slam_link.hpp
          kestrel-orbslam  (this directory, GPLv3)
             │  pose + tracking state, one reply per frame
             ▼
          onboard: anchored into ENU (navcore/slam_anchor.hpp) ──► WorldState vio*
             ──► the mission's displacement (fc_odometry.hpp)
             ──► EKF3 as ExternalNav with --voxel-vio-fc (VISION_POSITION_ESTIMATE)
```

- **Separate process, on purpose.** ORB-SLAM3 is GPLv3; linking it into
  `kestrel` would put all of onboard under GPLv3. The two share only the
  protocol header. The same socket would take Basalt or OpenVINS.
- **Onboard owns the camera** (only one process can stream it) and forwards
  the pair with its device timestamp and intrinsics; the bridge writes its
  ORB-SLAM3 settings from the first frame, so there is no calibration file to
  keep in step with the camera.
- **Flow control:** at most one frame in flight, newest wins, IMU samples are
  never dropped (`navcore/slam_client.hpp`). ORB-SLAM3 on a Pi 5 is slower than
  the camera; at walking pace tracking every second or third frame is fine.
- **Emitter:** the projector's dots move with the camera and must not be
  tracked. `nav.vox_emitter = strobe` (default) alternates them and a gate
  picks the dark frames **from the image** (`navcore/emitter_gate.hpp`) —
  librealsense's per-frame metadata has been reported with inverted polarity
  in this mode. `off` is the simplest choice outdoors, where the sun swamps
  the dots anyway.
- **New map = new frame.** After a loss ORB-SLAM3 may start a new map; onboard
  re-anchors it to the last estimate and bumps the reset counter so EKF3 treats
  the discontinuity as one. Loop closures bump it too.

## Build (on the Pi, or any Linux)

```bash
sudo apt install libeigen3-dev libboost-serialization-dev libssl-dev libopencv-dev
onboard/orbslam/fetch_orbslam3.sh                 # pinned commit, headless patch
cmake -S onboard/orbslam -B onboard/orbslam/build \
      -DORB_SLAM3_DIR=onboard/orbslam/third_party/ORB_SLAM3
cmake --build onboard/orbslam/build -j4
```

The patch (in `fetch_orbslam3.sh`) changes the build, not the SLAM: no
Pangolin/OpenGL viewer (`headless/`), C++14, one read-only accessor for the
active map id, and the removal of a settings printout that segfaults upstream
for rectified stereo.

## Run

```bash
onboard/orbslam/build/kestrel-orbslam \
    --vocab onboard/orbslam/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt &
./build/kestrel --voxel --voxel-fps=30 --voxel-slam --voxel-vio-fc --fc=mavlink --auto
```

Config keys: `nav.vox_slam`, `nav.vox_slam_socket` (default
`/tmp/kestrel-slam.sock`), `nav.vox_slam_inertial`, `nav.vox_emitter`
(`strobe`/`off`/`on`), `nav.vox_vio_to_fc`. ArduPilot parameters for the EKF3
feed: `docs/gnss-denied-setup.md` §11.

The vocabulary is 145 MB of text and loads in ~5 s on a desktop, longer on a
Pi; the first reply waits for it. Start the bridge before takeoff.

## Measured (simulation)

`test/test_slam_link.cpp` with `KESTREL_ORBSLAM` and `KESTREL_ORBVOC` set runs
this bridge on navcore's band-limited stereo IR render, anchored like the
module anchors it. 848x480, 15 Hz:

| flight | tracked | drift | ms/frame (desktop x86) |
|---|---|---|---|
| straight 8 m | 120/120 | 0.3 % | 18.8 |
| 5x5 m square, turns in place | 408/408 | 0.2 % | 20.1 |
| hover, 4 m leg, hover | 208/208 | 1.8 cm | 21.9 |

At 424x240 stereo initialisation can take seconds (ORB-SLAM3 wants more than
500 features with depth in one frame); run it at 640 or 848 wide.

**Closed loop** (`test_voxel_nav` with `VOXTEST_SLAM`): the whole mission
flown for 150 s on ORB-SLAM3's estimate, the four worlds every other
estimator here was measured on, stereo depth for the map, two runs at a time
on 4 cores. Raw: `../docs/orbslam_sweep_2026-09-24.txt`.

| | travel | cells | SLAM error at end | collisions |
|---|---|---|---|---|
| per-stop maps, no estimator (baseline) | 38.5 m | 38 | -- | 0 |
| DepthVio, mapping during legs | 59.9 m | 64 | 0.7-5.7 % | 0 |
| **ORB-SLAM3**, stops only | 48.5 m | 36 | 0.01-1.07 m | 0 |
| **ORB-SLAM3**, mapping during legs | 57.2 m | 59 | 0.11-1.46 m | 0 |

What the traces show, and what was done about it:

- **Every tracking loss began in a turn in place** (the turn onto a leg, or
  SCAN): ORB-SLAM3 reports "recently lost" for ~3 s, then starts a new map,
  which onboard re-anchors. The residual error -- up to 1.5 m -- is the ground
  covered during those seconds.
- **Before the jump guard, three of eight runs ended 10-19 m off** after such
  losses, the SLAM frame having moved under the estimate. The guard
  (`slam_anchor.hpp`: a pose that implies more than 4 m/s since the last one
  is re-anchored, not believed) fired three times in the rerun and no run
  exceeded 1.5 m.
- The harness keeps flying legs on the frozen estimate while SLAM is lost; on
  the aircraft `vioLocalEstimate` drops the estimate and the mission stops, so
  this is the pessimistic case.
- Turn rate: `mission.max_yaw_stick` caps the turn onto a leg. Its effect is
  measured below when the run completes.

So: as accurate as the drift-free ideal when tracking (0.01-0.5 m over 60 m),
with losses concentrated in fast turns -- slow the turns.

Not known until it flies: real IR (auto-exposure, blur, a misbehaving strobe),
the Pi 5's frame rate with the voxel pipeline running beside it, and latency
(set `VISO_DELAY_MS` from it).

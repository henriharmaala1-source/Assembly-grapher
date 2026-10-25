# VIO from a downward camera: what tilt, rotation, field of view, speed, height and the ground do

Measured 2026-10-25 with `nav-sim/test/downvio_sweep` (grid: `downvio_sweep.sh`,
raw: `downward_vio_sweep.csv`, 344 runs). The estimator is the aircraft's own
**DepthVio** (navcore/vio.hpp), unchanged except for the fix below; the
camera looks straight down from the airframe (image top = nose) and is
rendered every frame at the TRUE body attitude through navcore's IR model.
The IMU gives the VIO that attitude with 0.3 deg of noise. 640x400 global
shutter, 30 fps unless stated, 4 seeds per condition, numbers are mean drift
(final horizontal error as % of distance) +- standard error.

Two depth sources:

- **mono + rangefinder** ("plane"): a wide-angle global-shutter camera and a
  downward rangefinder. Per-pixel range from the rangefinder, the IMU's
  attitude and a flat-ground assumption -- the cheap downward option.
- **downward stereo**: a second D435-class depth camera (navcore's stereo
  failure model).

## First: DepthVio could not do this at all

DepthVio held the attitude as Euler angles. A downward camera is at pitch -90,
that representation's gimbal lock, and 0.3 deg of IMU noise swung the yaw/roll
split by tens of degrees per frame: **102 % drift** on a level 6 m flight
(test_vio, with its 0.5 deg/s gyro bias), every pose reported valid. Fixed by
composing the IMU attitude as a matrix (commit "DepthVio: attitude as a
matrix"); the forward camera is unchanged in effect (test_vio at 848x480: same
to the centimetre in all 12 cases). Everything below is with the fix.

## Tilt: does not matter (flat ground, attitude known)

2 m/s, 1.5 m up. Drift %, mono + rangefinder / stereo:

| tilt | 60 deg lens | 90 deg | 120 deg | 140 deg |
|---|---|---|---|---|
| 0 | 0.34 / 0.42 | 0.44 / 0.40 | 0.40 / 0.46 | 0.52 / 1.09 |
| 20 | 0.25 / 0.47 | 0.89 / 0.49 | 0.37 / 0.57 | 0.53 / 0.94 |
| 40 | 0.53 / 0.85 | 0.29 / 0.19 | 0.76 / 0.59 | 0.61 / 0.96 |

No lost frames anywhere in the table. A nose-down quad's belly camera looks
BACKWARD; at 40 deg a 140 deg lens has the horizon in view (80 % floor) and
still holds. (The wide stereo is ~1 %: its depth is less precise per pixel.)

## Rotation rate: does not matter

Roll/pitch wobble at 1.5 Hz on a 15 deg tilt: peak 94 and 188 deg/s give
0.46-0.82 % at every lens. DepthVio takes rotation from the IMU and solves
only heading and position, so rotation is not something it has to see.

## FPV cruise: fine at 60 fps

5 m up, tilt and speed together (10 deg/5 m/s ... 40 deg/18 m/s): 0.33-1.00 %
at every lens, no lost frames. At 30 fps the same speeds need the per-frame
jump gate raised (kestrel's 0.5 m is set for its slow legs).

## What DOES matter

**1. Attitude accuracy -- for mono + rangefinder only.** A constant roll/pitch
bias (level flight):

| bias | mono 60 deg | mono 140 deg | stereo 60 deg | stereo 140 deg |
|---|---|---|---|---|
| 0.5 deg | 5.1 % | 3.6 % | 0.22 % | 1.02 % |
| 1 deg | 10.4 % | 5.9 % | 0.44 % | 0.91 % |
| 2 deg | 20.9 % | 10.3 % | 0.45 % | 0.72 % |

Linear, ~10 % per degree at 60 deg, ~6 % at 140 deg (standard errors
0.1-0.4). Stereo does not care. The mono setup builds its depth FROM the
attitude, so an attitude error is a tilted ground plane, and DepthVio's
keyframe odometry turns it into drift. An EKF's tilt is good to a few tenths
of a degree in steady flight and worse when accelerating, so through DepthVio
this is the error that will dominate. A flow estimator that takes gyro RATES
(ArduPilot's own optical-flow fusion) is far less exposed to it; so would be
estimating the ground plane from the image rather than from the IMU.

**2. Ground that is not flat -- mono + rangefinder.** Boxes 0.3-1.5 m tall
over 15 % of the floor: mono 4.0-10.3 % (worst: 140 deg lens at 30 deg tilt,
which sees the most of it obliquely), stereo 0.95-2.8 %. Flat-ground depth is
wrong exactly where the ground is not flat.

**3. Height -- for a wide lens.** 20 deg tilt, 4 m/s:

| height | 60 deg | 140 deg |
|---|---|---|
| 1.5 m | 0.79 % | 0.56 % |
| 5 m | 0.33 % | 0.52 % |
| 15 m | 0.74 % | **10.4 %** |

A 140 deg lens spreads 640 px over 140 deg: from 15 m that is ~5 cm of ground
per pixel at the centre and far worse at the oblique edge, where flat-ground
depth is also least certain. Wide is right low, narrow is right high.

Low texture (smooth concrete, richness 0.15) changed nothing here -- but the
IR model is band-limited value noise with structure at every scale, kinder
than real smooth floors; do not read that row as a guarantee.

## Not modelled

Pinhole lenses only (real >120 deg lenses are fisheye: better centre, worse
edges than this); no motion blur, rolling shutter, exposure changes or
vibration; IMU error as white noise plus a constant bias; straight lines of
20-60 m; one texture model; 4 seeds per condition.

# The ArduPilot bridge — architecture PLAN

Status: **design only.** The MAVLink v2 codec exists and is pinned against
pymavlink golden frames; `mavlink_backend.cpp` speaks it. What does not exist is
a coherent answer to *where state estimation lives* under ArduPilot, and the one
currently implied by `state_estimator.hpp` is the iNAV answer.

Written 2026-08-12, after finding that the flight-stack decision changed and the
state model did not. See `NOTES.md` for that finding and `PROJECT_CV.md` §5.

---

## 1. The problem with the current shape

`state_estimator.hpp` is an **iNAV-era design**, and every load-bearing choice in
it says so:

* constants copied from iNAV — `gpsTimeoutS` "matches INAV_GPS_TIMEOUT_MS",
  `glitchRadiusM` "matches INAV_GPS_GLITCH_RADIUS"
* premise — "iNAV already fuses raw IMU + GPS + baro into a good attitude and a
  complementary-filter position estimate... the Pi-side estimator fuses the
  things the FC can't"
* output — "build a synthetic GPS fix from the current estimate for
  **`MSP2_SENSOR_GPS`**", fed back "so its nav modes work GPS-denied"

**Under ArduPilot that architecture is not merely non-idiomatic, it is
statistically wrong.** EKF3 would receive an *already-filtered* Pi estimate and
treat it as an independent measurement. Two filters in series, each smoothing,
neither aware of the other: lag, over-confidence, and a covariance that means
nothing. This is a known trap and it is worth stating plainly, because the code
compiles, passes its tests, and reads as deliberate.

## 2. Three possible architectures

| | who estimates position | Pi sends | verdict |
|---|---|---|---|
| **A** | the Pi; FC consumes it | synthetic GPS / vision pose | the current implied design. **Retire.** Double filtering. |
| **B** | the FC; Pi sends measurements | odometry increments + covariance | **the right v2**, once VO exists |
| **C** | **nobody** | body-frame attitude or velocity commands | **the right v1**, and it deletes the estimator from the flight path |

### Why C is right for v1

The thesis needs **no global position**. The planner is body-frame by
construction (`toWorld` rotates primitives by the current heading), the map is
local and short-lived, and the output is a bearing and a speed. Nothing in
`THESIS.md` §4 requires knowing where the aircraft is in the world.

So v1 should not estimate position **anywhere** — not on the Pi, not by feeding
ArduPilot a fiction. The Pi does perception and local planning; the FC does
attitude and rate control, which it is far better at than we would be.

That is also the cheapest possible answer, which is the thesis.

### Why B is right later

When scan-matching odometry exists (`nav-sim/docs/POSE_AND_OPENNESS_PLAN.md`
§5), the correct handover is **raw-ish increments, not a filtered pose**:
`VISION_POSITION_DELTA` or `ODOMETRY` with honest covariance, and
`EK3_SRC1_POSXY = ExternalNav`. One filter, correct statistics, and attitude
still comes from EKF3 where the high-rate IMU lives.

**The Pi-side Kalman does not survive either architecture.** Under C it is
unnecessary; under B it is harmful.

## 3. The bridge, message by message

### 3.1 Downlink — FC to Pi

| msg | id | why | status |
|---|---|---|---|
| `ATTITUDE` | 30 | roll/pitch/yaw to gravity-align the depth and rotate the map | **in the enum, no decoder** |
| `HEARTBEAT` | 0 | mode, armed, link health | present |
| `RC_CHANNELS` | 65 | assist switch, pilot intent | present |
| `SYS_STATUS` | 1 | battery, for the speed budget and abort | present |
| `EKF_STATUS_REPORT` | 193 | **"do I trust myself"** | **absent** |
| `GPS_RAW_INT` | 24 | **logged, never routed to the planner** | present |

Two of these deserve emphasis.

**`ATTITUDE` is the one v1 actually needs**, and it has no decoder. Roll and
pitch are what let the map be gravity-aligned without an IMU filter of our own,
and heading is what the body→world rotation needs. Adding it is ~30 lines plus a
golden-frame test, in a codec that already carries 14 messages.

**`EKF_STATUS_REPORT` closes the self-trust gap** that `PROJECT_CV.md` records:
the project has a health signal (valid-pixel fraction) that nothing consumes, and
every safety mechanism assumes the *map* might be wrong while none assumes the
*system* might be. The FC's own estimator health is a free, independent second
opinion, and it should gate the speed budget.

### 3.2 Uplink — Pi to FC

**v1: attitude, not velocity.** `SET_ATTITUDE_TARGET` (82), already in the enum.

This is the consequence most likely to bite on the field day, so it is stated
here rather than discovered: **GUIDED velocity setpoints require a horizontal
velocity estimate, and GNSS-denied there isn't one.** No GPS, no external nav,
no optical flow means EKF3 has IMU and baro only — enough for attitude, not for
velocity. So `SET_POSITION_TARGET_LOCAL_NED` in a velocity mask **will not work
for v1**, and the planner's speed command has to be expressed as pitch angle.

Crude, and correct for the constraint.

**v1.5: optical flow + downward rangefinder** → `EK3_SRC1_VELXY = OpticalFlow`,
which unlocks velocity setpoints and a much better control interface for maybe
€40 of hardware. This is probably the highest-value small purchase in the
project and it is not currently on any list.

**v2: external nav** → position control, the full GUIDED interface.

### 3.3 The elegant one: `OBSTACLE_DISTANCE` (330)

ArduPilot's own proximity/avoidance layer consumes a **72-element array of
distances by bearing**.

That is *exactly* the angular openness map designed in
`POSE_AND_OPENNESS_PLAN.md` §1 — same structure, same units, already on the
plan for our own planner. Publishing it costs one message and buys **a second,
independent avoidance layer running inside ArduPilot**, with different code,
different failure modes, and no dependence on our planner being correct.

For a project whose safety argument is built on independent paths to a veto,
that is close to free defence in depth.

## 4. Failure and handover

The four link states in `THESIS.md` §1.0.1 map onto ArduPilot mechanisms that
already exist, and mostly onto ones we do not have to write:

* **Pi stops sending** → ArduPilot times out the offboard setpoints and reverts
  to the pilot's mode. A failsafe we get for free.
* **`SYSID_MYGCS` gating** — already implemented in `mavlink_backend.cpp`.
* **Autonomy engaged** → mode change by `COMMAND_LONG`, gated on the RC switch.
* **Pilot flying** → the Pi sends nothing, or `OBSTACLE_DISTANCE` only, so the
  autonomy is a veto rather than a driver.

## 5. GNSS exclusion, made checkable

The `THESIS.md` §1.0 claim is that no satellite information reaches the
navigation solution. Under this architecture that becomes trivially verifiable
rather than a matter of trust:

* `EK3_SRC1_POSXY` / `VELXY` / `YAW` logged per flight, and never set to GPS
* `GPS_RAW_INT` is **received and written to the black box, and routed nowhere
  else** — one grep proves it
* the controlled pair: same mission with the receiver connected-but-not-sourced
  and again physically unplugged, identical behaviour

Under architecture **C** the claim is stronger still, because there is no
position estimate anywhere for GNSS to contaminate.

## 6. Order of work

1. **`ATTITUDE` decoder + golden-frame test.** ~30 lines. v1 needs it.
2. **Retire the Pi-side Kalman from the flight path.** Not delete — it is
   correct code for architecture B and should be kept, marked, and left unwired.
3. **`SET_ATTITUDE_TARGET` command path**, with the pitch-angle speed mapping.
4. **`EKF_STATUS_REPORT` decoder**, gating the speed budget.
5. **`OBSTACLE_DISTANCE` publisher**, once the angular map exists.
6. **Non-GPS ArduPilot setup written down** — `EK3_SRC*`, pre-arm checks, and
   what fails first. Currently an open item with no document.

Items 1 and 3 are the v1 flight blockers. The rest can follow.

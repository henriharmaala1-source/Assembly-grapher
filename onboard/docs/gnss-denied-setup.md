# GNSS-denied ArduPilot setup

Written 2026-08-17 to close the open item that said this was "unwritten, and the
symptom looks like a broken link". It is the configuration side of architecture
**C** in `MAVLINK_BRIDGE_PLAN.md`: **nobody estimates position.**

> **Verify every enum value against the docs for YOUR firmware version.** These
> numbers move between releases, and a wrong source enum fails in a way that
> looks exactly like dead hardware. The *reasoning* below is stable; the
> integers are not.

---

## 1. Why GUIDED will refuse, and what to expect

The failure mode to expect first is **not** a crash or an error message. It is
mode entry being silently denied, or a pre-arm refusal that names something
apparently unrelated. ArduPilot will not enter a position-controlled mode
without a position solution it trusts, and GNSS-denied it does not have one.

So: use **GUIDED_NOGPS**, not GUIDED. It exists precisely for attitude-only
offboard control and it is the mode the `SET_ATTITUDE_TARGET` uplink targets.

## 2. The EKF source set

The point of this section is that the claim "no satellite information reaches
the navigation solution" becomes **checkable by grep** rather than trusted.

| parameter | set to | why |
|---|---|---|
| `AHRS_EKF_TYPE` | 3 | EKF3 |
| `EK3_ENABLE` | 1 | |
| `EK3_SRC1_POSXY` | **None** | nobody estimates horizontal position |
| `EK3_SRC1_VELXY` | **None** | ...or velocity, until optical flow exists |
| `EK3_SRC1_POSZ` | Baro | altitude is the one thing available |
| `EK3_SRC1_VELZ` | None | |
| `EK3_SRC1_YAW` | Compass | **the weak link — see §5** |
| `GPS_TYPE` | 0 | no driver at all, if no receiver is fitted |

Log `EK3_SRC1_*` with every flight. If none of them ever reads GPS, the
exclusion claim is evidence rather than assertion.

**If a receiver IS fitted for logging only:** leave `GPS_TYPE` at its driver
value, keep every `EK3_SRC*` off GPS, and confirm `GPS_RAW_INT` is received,
written to the black box, and routed nowhere else. One grep proves it.

## 3. Pre-arm checks

`ARMING_CHECK` is a bitmask. Do **not** set it to 0 — that disables the checks
that are still meaningful (battery, RC, INS, compass) along with the GPS ones.
Clear only the GPS/position bits and leave the rest armed.

Expect to iterate here. Read the pre-arm text the FC emits rather than guessing:
it names the failing check, and the name is usually the answer.

## 4. Serial and streams

For whichever port the Pi is wired to (SERIAL2 shown):

    SERIAL2_PROTOCOL = 2      # MAVLink2
    SERIAL2_BAUD     = 115    # 115200
    SR2_RC_CHAN      = 10     # Hz -- 0 means never sent
    SR2_EXTRA1       = 50     # ATTITUDE, the rate the control loop wants
    SR2_EXTENDED_STATUS = 2   # SYS_STATUS, EKF_STATUS_REPORT

**`rc_probe` exists to test exactly this.** If the link is up and heartbeats
arrive but no RC frames do, the stream rate is the cause and the tool says so by
name rather than showing an empty table.

## 5. The genuinely weak link: yaw

GNSS-denied, **heading comes from the compass alone**. ArduPilot's GPS-derived
yaw fallback is not available to you.

A compass on a small frame, near motor currents, under a carbon plate, in a
forest, is the least trustworthy sensor in the aircraft — and a yaw error
rotates the entire world model, because `SET_ATTITUDE_TARGET` commands yaw
ABSOLUTELY as heading plus an increment.

Budget real time for compass calibration and interference checking. `COMPASS_*`
offsets and a CompassMot run are not optional here. `EKF_STATUS_REPORT`'s
`compass_variance` is now decoded and is the honest place to watch it.

## 6. Failure and handover — mostly free

* **Pi stops sending** → ArduPilot times out the offboard setpoints and reverts
  to the pilot's mode. A failsafe obtained by doing nothing.
* **`SYSID_MYGCS` gating** → already implemented in `mavlink_backend.cpp`.
* **Pilot takes over** → the RC override path deliberately writes 0 to every
  channel it is not driving, so the mode switch is always the pilot's.
* **Stale command** → `FcLink` substitutes a NEUTRAL hover rather than repeating
  the last motion command.

## 7. Order of bring-up

1. **SITL first.** Get GUIDED_NOGPS to engage and hold an attitude on a laptop
   with nothing to crash. This converts the scariest unknown into a desk task.
2. `rc_probe` against the real FC — confirms wiring, protocol and stream rates.
3. Attitude decode: confirm roll/pitch/yaw track the airframe when tilted.
4. `SET_ATTITUDE_TARGET` on the bench, **props off**, watching the FC's own
   attitude target in the logs.
5. Only then fly, with the pilot on the switch.

## 8. What is NOT solved by any of this

Position still drifts without bound, because nothing measures it. Architecture C
does not fix that — it declines to pretend otherwise. Optical flow plus a
downward rangefinder is the v1.5 upgrade that bounds velocity error and makes
`EK3_SRC1_VELXY = OpticalFlow` meaningful; external nav is v2.

## 9. Optical flow is not optional for the voxel mission (measured, 2026-09-23)

§8 called flow "the v1.5 upgrade". Two findings move it to **required** for
`--voxel`:

1. **The mission cannot fly a leg without a displacement.** The Pi-side
   estimator initialises from a GPS fix only, so GNSS-denied `estValid` never
   became true and the move-stop-sense cycle sat in `SETTLE(no-est)` for ever.
   `fc_odometry.hpp` now takes the FC's own `LOCAL_POSITION_NED` as the
   displacement **only when EKF3 vouches for it** (`POS_HORIZ_REL` set,
   `CONST_POS_MODE` clear). With no horizontal source EKF3 never sets that
   flag, so the aircraft hovers -- the safe failure, now a stated one.
2. **The per-stop map assumes a stationary hover; ALT_HOLD is not stationary.**
   In `test_voxel_nav`'s closed loop, unmeasured hover drift gave collisions
   that grew with it -- 1 / 3 / 15 at 0.05 / 0.1 / 0.2 m/s on perfect depth,
   3 / 3 / 16 on simulated stereo, 4 worlds x 150 s each -- and **all but two
   happened while HOVERING**, not on certified legs. Nothing at the Pi can stop
   an aircraft sliding into a pillar while it thinks; position hold can.
   Estimates for tilt-only hover drift under a canopy are 0.1-0.5 m/s and
   more in wind; flow-aided LOITER holds to roughly 5-10 cm (1 sigma).

Parameters (flow + downward rangefinder, e.g. MicoAir MTF-01, which is one
unit with an 8 m ToF -- prefer it to a VL53L0X-based board, whose ~2 m range
caps the altitude EKF3 can use flow at):

| param | value | why |
|---|---|---|
| `FLOW_TYPE` | per sensor (MAVLink for MTF-01) | |
| `RNGFND1_TYPE` | per sensor | flow needs height above ground |
| `EK3_SRC1_VELXY` | **5 (OpticalFlow)** | the velocity source |
| `EK3_SRC1_POSXY` | 0 (None) | EKF3 integrates flow velocity into a relative position |
| `EK3_SRC1_POSZ` | 1 (Baro) or 2 (RangeFinder) | |

Known weak spots, from ArduPilot issues: flow needs light (>60 lux on the
MTF-01) and texture -- grass and leaves moving in prop wash are the forest's
version of a blank floor; above rangefinder range flow position is dropped;
position-controller scaling can let flow LOITER drift in wind (#28242). Test
over real forest litter before trusting it.

## 10. A second, independent avoidance path: OBSTACLE_DISTANCE

`--voxel` now computes, from EVERY depth frame, 72 horizontal distances
clockwise from the nose (`obstacleDistanceFromFrame`: a level band of the
depth image, the 12th-nearest return per 5 deg sector so a lone speckle is
not an obstacle, unknown where nothing returned). It needs only attitude, so
unlike the per-stop map it stays valid WHILE MOVING. `FcLink` forwards it as
`OBSTACLE_DISTANCE` at <= 10 Hz while fresh and never repeats a stale set, so
ArduPilot's own avoidance runs underneath our planner with different code and
different failure modes -- the same shape ArduPilot's own RealSense script
sends.

| param | value | why |
|---|---|---|
| `PRX1_TYPE` | 2 (MAVLink) | proximity from the companion |
| `AVOID_ENABLE` | 7 | fence + proximity |
| `AVOID_MARGIN` | 1.0-1.5 m | keep-out distance from the proximity boundary |
| `AVOID_BEHAVE` | 1 (Stop) to start | slide only once trusted |

Where ArduPilot applies it (its simple-avoidance docs): LOITER shortens the
commanded velocity (stop or slide), ALT_HOLD adds a lean-away angle, GUIDED
applies it through the velocity controller. Whether it constrains
`SET_ATTITUDE_TARGET` in GUIDED_NOGPS is NOT verified here -- check it on the
bench before relying on it with the attitude uplink. The RC-override uplink in
ALT_HOLD or LOITER is covered by the documented behaviour.

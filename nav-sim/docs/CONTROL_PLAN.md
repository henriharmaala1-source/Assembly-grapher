# Control module — plan

Not built. This is the design, written down so the decisions are visible before
any code exists.

## What is there now, and why it is worse than nothing

There is no controller. What exists is

```cpp
const float tau = 0.35f;
float k = std::min(1.f, dt / tau);
vx += (dx * gr.speed - vx) * k;      // and vy, vz
px += vx * dt;
```

copy-pasted into **four** places: `voxel_sim.cpp`, `voxel_gui.cpp`,
`test/gui_preview.cpp`, and the rollout in `voxel_traj.cpp`.

That is not control, it is an *assumption* — that the vehicle reaches commanded
velocity with a 0.35 s time constant. Nothing achieves that for free; a
controller is what makes it approximately true.

**The circularity is the real problem.** `voxel_traj.cpp` uses `tau` to
**predict** motion when it rolls out primitives, and `voxel_sim.cpp` uses the
same `tau` to **produce** motion. The planner is validated against a plant
defined to match its own model. Every "the primitive is flyable by
construction" claim in this repository rests on that, and it is the one place
where the sim's optimism is structural rather than a parameter you can sweep.

## Interface

One header, no dependencies beyond `<cmath>`. No map, no planner, no I/O, no
threads — velocity setpoint in, sticks out. It cannot be broken by anything
upstream of it, and it can be unit-tested with step inputs headlessly.

```cpp
struct VelCmd   { float vE, vN, vU, yawRateDps; };     // world frame
struct VehState { float vE, vN, vU, rollDeg, pitchDeg, yawDeg;
                  float pDps, qDps, rDps; };           // rates for the inner loop
struct CtlOut   { float roll, pitch, yaw, throttle; }; // normalised sticks

class VelocityController {
public:
    CtlOut step(const VelCmd&, const VehState&, float dt);
};
```

## Two backends, one split

The controller splits into an outer velocity loop and an inner attitude loop.
That split is what lets both flight modes share code rather than existing as
two separate controllers that drift apart.

```
              outer (velocity)                 inner (attitude)
VelCmd ---> accel = PI(v_err), clamped ---> tilt = atan(a/g) ---> rate = P(att_err)
                                             |                      |
                                        ANGLE backend          ACRO backend
                                        stops here             continues
```

**ANGLE backend.** Outer loop only. Roll/pitch sticks are tilt setpoints; the
FC's own attitude controller closes the inner loop. Simplest, and it keeps
self-levelling underneath as a floor — if the companion stops sending, the
aircraft levels rather than tumbling.

**ACRO backend.** Both loops. The companion owns attitude and sends body-rate
setpoints; the FC runs only the rate loop. More agile, because ANGLE mode's
tilt limit is also an acceleration limit, and because there is no cascade being
fought. Also strictly more dangerous, and it needs a real failsafe story.

## Rate requirement — this is the constraint that bites

The planner runs at **10 Hz**. A controller cannot.

* rate loop (gyro → motors): 500 Hz+, stays on the FC in **both** backends
* attitude loop (angle error → rate cmd): 50–100 Hz — on the FC in ANGLE, on
  the **companion** in ACRO
* velocity loop: 20–50 Hz
* planner: 10 Hz

So the controller must run faster than the planner and hold or interpolate the
velocity setpoint between planner updates. Designing it as "one call per
planner step" would work in the sim and fail in the air, which is exactly the
class of mistake this document exists to prevent.

ACRO therefore requires a 50–100 Hz control task on the Pi with bounded
latency, separate from the 10 Hz perception task. That is a scheduling
requirement, not a tuning one.

## The plant model is not optional

Adding a controller only helps if the sim also gains a **plant distinct from
it** — thrust limit, tilt limit, actuator delay, second-order response.
Otherwise one circularity is swapped for another and every number stays as
optimistic as it is today.

The honest structure is controller and plant as separate objects, with
`TrajParams::tau` **derived from the measured closed-loop step response**
rather than assumed. That is the change that would make `advance = 0.99` mean
something it currently does not.

## INAV specifics to verify before designing around them

Not assumed here, because getting these wrong wastes the whole build:

* Does the target build honour `MSP_SET_RAW_RC`, and does it need the MSP RC
  override feature or `RX_MSP`?
* What is the failsafe behaviour when MSP RC goes quiet mid-flight? This
  decides whether ACRO is acceptable at all.
* What MSP rate does the link sustain at the chosen baud? The 50–100 Hz
  attitude loop above depends on the answer.
* Can ANGLE and ACRO be switched in flight from the companion, or does it need
  a physical mode switch? A pilot-held override switch is the right answer
  regardless.

## Order of work

1. `velocity_control.hpp/cpp` — outer loop, ANGLE backend, pure and testable.
2. A plant model in the sim, distinct from the controller. Re-derive `tau`
   from its step response.
3. Re-run every sweep. Expect the numbers to get worse; that is the point.
4. Inner attitude loop and the ACRO backend, behind a flag, once 1–3 hold.
5. MSP transport last — it is the easiest part and the one most likely to be
   thrown away if the layers above change.

## What must stay true

The controller never reads the map and never talks to the planner. It consumes
a velocity setpoint and nothing else. Any coupling that creeps in here is a
coupling that has to be reasoned about at 3 a.m. in a forest.

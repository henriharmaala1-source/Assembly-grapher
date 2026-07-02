# kestrel — an introduction

**What this is, in one sentence:** kestrel is a cheap companion computer for a
small analog FPV drone — a Raspberry Pi that watches the world through a
camera and helps the aircraft avoid obstacles, fly a route, or hold position,
without replacing or fighting the flight controller that already keeps it
stable in the air.

This document explains the shape of the software: what each piece does, and
— more importantly — *why* it's built that way instead of some other way. It
assumes you know what an FPV drone and an analog video link are, and nothing
else about this project.

---

## The problem this solves

A cheap FPV drone already flies well. Its flight controller (the FC) — a
tiny board running firmware like iNAV — reads the gyro and accelerometer
hundreds of times a second and keeps the aircraft level, stable, and
responsive to the sticks. That loop is fast, proven, and not something you
want a Raspberry Pi anywhere near.

What the FC *can't* do on its own is see. It has no idea whether there's a
tree ahead, whether the operator meant "left" as "left relative to the
drone" or "left relative to the world," or how to hold a GPS position
indoors where there's no GPS. Those are camera problems and reasoning
problems — slow, computationally heavy, and exactly what a Raspberry Pi is
good at.

kestrel's job is to sit next to the FC, watch the camera, think about what
it sees, and *hand the FC ordinary stick inputs* — the same kind of signal
the pilot's radio already sends. The FC never knows the difference between a
human on the sticks and kestrel on the sticks. That's the whole trick: kestrel
doesn't need to understand attitude control, motor mixing, or PID tuning,
because it never touches any of that. It only ever says "more forward," "turn
right a bit," or "do nothing" — and the FC's own, already-trusted control
loop does the rest.

## Why cheap hardware, and what that choice costs

This is the load-bearing decision of the whole project, so it's worth
stating plainly before anything else: **the hardware budget is small on
purpose, and that deliberately caps how autonomous this drone can be.** This
isn't a limitation the project ran into and is working around — it's the
starting constraint everything else was designed against.

The obvious alternative is a Jetson-class board with a real GPU, running a
full ROS stack with LiDAR and stereo cameras. That's what well-funded
autonomous-drone research and companies (Skydio, most companion-computer
research platforms) actually do, and it works — but it costs hundreds of
dollars in compute alone, weighs enough to need a bigger airframe to carry
it, and turns a lightweight FPV quad into a different category of aircraft.

The goal here is different: stay cheap enough that anyone already flying
analog FPV could add this, stay light enough that the airframe doesn't need
to change, **stay an FPV drone** — the pilot still flies it, still watches a
normal video link, and the whole companion-computer add-on (compute plus
every sensor — see [`bom.md`](bom.md) for the actual list and prices) comes
in under €200 on top of a quad that already flies. That number is the
ceiling this project chose to build inside of.

Every "why isn't it smarter yet" question in this document has the same
underlying answer: **that would cost more compute, more sensors, or more
money than the budget allows, and the budget is the point, not an
oversight.** Concretely, this cheap-hardware choice is *why*:

- there's no GPU, so every model runs on the Pi's CPU — which is *why* the
  depth models have to be small and can't run every frame (see below);
- there's no LiDAR and no dedicated stereo rig by default — a single
  camera plus, optionally, a cheap time-of-flight sensor stand in for them,
  which is *why* obstacle sensing is a short-range, camera-shaped estimate
  rather than a dense 3D map;
- there's no onboard SLAM or continuous path-planning yet — building and
  updating a real map costs compute this budget doesn't have to spare,
  which is *why* the drone uses the stop-think-move pattern explained
  further down, instead of thinking and flying at the same time.

None of that is a permanent ceiling — the architecture has clear places to
plug in more capability later (a cheap NPU add-on board, a better sensor, a
faster inference backend) without a redesign, and the project tracks exactly
which upgrades unlock what. But today, on this budget, the honest framing is:
**this is a demonstrator of what's achievable on cheap hardware, not the
most autonomous drone that could be built.** That trade-off — accessible and
FPV-weight versus maximally capable — was chosen deliberately, and it's the
right way to understand every design decision that follows.

## Why no GPU, specifically

The Pi 5's GPU exists for video decode and display, not machine-learning
inference — there's no CUDA-equivalent, so every model the software runs, it
runs on the CPU. That shapes almost every other decision in the project:
which depth models are usable (small ones — MiDaS-small,
DepthAnything-v2-small), how often they can run (not every frame), and why a
lot of engineering effort has gone into making sure a slow camera-side
computation can never be allowed to affect the fast, safety-critical parts
of the system. That last point is the architecture's central idea, covered
next.

## The central idea: two brains, one fast, one slow

Everything in kestrel hangs off one architectural decision: **split the
software into a fast loop and a slow loop, and never let the slow one block
the fast one.**

Think of it like a driver. Your reflexes — braking when something appears in
the road — have to be instant, and they can't wait for you to finish
thinking about the best route to the airport. Route-planning is valuable,
but it must never compete with the reflex for attention at the moment a
reflex is needed.

kestrel is built the same way, as two threads sharing one Raspberry Pi:

```
   camera ──► frame handoff ──────────► THINK  (best-effort, can be slow)
     │                                    heavy perception: depth model,
     │                                    object detection
     ▼                                    │
   FLY  (fast, every frame, guaranteed)   │
     capture → cheap perception →         │
     state estimate → pick a mode →       │
     send a command                       │
     ▲                                    │
     └──────────── shared notebook ───────┘
```

The **fly loop** runs every camera frame: read the flight controller's
telemetry, run only the *cheap* vision (a lock-on tracker, an appearance-
based road detector — both a few milliseconds), update the position
estimate, decide what to do, and send a command. It never waits on anything
slow.

The **think thread** runs the *expensive* vision — the depth model, the
object detector — at whatever pace the Pi's CPU can sustain, which might be
every third or fourth frame, not every frame. If a depth model takes 100
milliseconds and a fresh frame arrives every 33 milliseconds, the fly loop
simply keeps flying on the *last* result the think thread produced, rather
than waiting for a new one.

There's also a third thread, added once real hardware testing surfaced the
need for it: an **I/O thread that owns the connection to the flight
controller** on its own schedule, independent of the camera. If the camera
ever stalls — a USB hiccup, a bad frame — the flight controller still gets
its stick updates on time. Skipping this would risk the FC's own failsafe
triggering over what should have been a harmless camera glitch.

## The shared notebook

The three threads need to share information — the camera-side perception
needs to tell the control side what it saw — without corrupting each
other's data or blocking each other. The answer is a single shared struct,
protected by one lock, that everybody reads from and writes to: the *world
model*. Perception writes what it currently believes ("the open direction
is 15° left, and I'm 80% confident"); the control side reads that belief
and turns it into a command; nothing is ever passed directly from one
thread's internals to another's.

This is a well-known pattern in robotics — sometimes called a *blackboard*
architecture — chosen specifically because it keeps every module honest
about what it does and doesn't know, and because a new module can be added
by writing to the blackboard, without every other module needing to know it
exists.

## How the drone sees

kestrel doesn't use LiDAR or a stereo camera — a single, ordinary camera
feed is the primary sensor, for the same cost-and-weight reason there's no
GPU. From that single feed, a small monocular depth model estimates
roughly how far away everything in the frame is. That depth estimate is
turned into a steering decision using an algorithm called **VFH+** (Vector
Field Histogram Plus): collapse the depth image into a one-dimensional
"how open is each direction" profile, and pick the most open direction that
is still close to where the drone was already heading — the "close to where
it was already heading" part is what stops the steering from flapping back
and forth between two similarly-open gaps.

A cheap time-of-flight distance sensor (a few dollars, effectively free on
the CPU budget) can feed the exact same VFH+ pipeline instead of the depth
model, when one is fitted — the algorithm doesn't care whether "how far is
that" came from a neural network or a laser, only that it gets a distance
grid.

Two other, independent vision modules exist alongside the depth pipeline: an
appearance-based road/line follower (no neural network — it clusters color
in the image, cheap enough to run every frame), and a lock-on tracker that
keeps a bounding box on a designated subject once the operator (or an
automatic detector) points at one.

## Where am I?

Outdoors, GPS answers "where am I." Indoors, or anywhere GPS is degraded,
it doesn't — and this is a common gap in cheap FPV setups, since the flight
controller's own position-hold logic assumes GPS is available.

kestrel runs its own lightweight position filter (a Kalman filter, a
standard way of blending several noisy measurements into one best estimate)
that combines GPS, barometric altitude, and — once a camera-based motion
estimate exists — visual motion, into a single smoothed position. That part
is fairly ordinary. The less ordinary part is what happens next: kestrel
feeds its own estimate *back into the flight controller* as a synthetic GPS
signal, using a message the FC firmware already understands. The FC's own,
already-tuned navigation logic then works exactly as it would with a real
GPS fix — kestrel never has to reimplement position-hold or waypoint-flying
itself; it just gives the FC a position to use when the real GPS can't
provide one.

## What should I do right now?

The drone can be in exactly one "mode" at a time — fly under the pilot's
sticks, hold a hover, follow a road, fly a GPS route the FC already knows
how to fly while kestrel watches for obstacles, or run a fully autonomous
cycle. Each mode is a small, self-contained piece of code with one job:
given everything currently known (the shared notebook), produce a command,
or explicitly hand control back to the pilot or the flight controller.

New modes are added by writing a new one of these and registering it — never
by editing a growing `if / else` chain that every other mode also has to
thread through. Two safety checks sit *above* every mode, regardless of
which one is active and regardless of who wrote it: a low-battery or
operator-abort check that hands the aircraft to the FC's own return-to-home,
and an obstacle check that overrides any mode trying to fly forward into
something close. A mode can only opt out of the obstacle check by proving it
handles obstacles better itself — the default assumes it doesn't.

One mode is worth calling out specifically, because it's the answer to "how
do you trust this before you trust it": a **shadow mode**, where the pilot
flies normally and kestrel computes — but never sends — what it *would* have
done, drawing that intent on the video feed. It's a way to watch the
autonomy's judgment in real flight with zero risk before ever letting it
touch the sticks.

## Moving without a full map yet

Building a real map of the surroundings — the kind a self-driving car or a
research drone builds — is expensive, and the Pi doesn't have the compute
to do it continuously while also flying. So instead of trying to think and
move at the same time, kestrel does what several real robots facing the
same compute limit have done before it (early Mars rovers being the most
famous example): **stop, think, move a short distance, stop again.**

Concretely: the drone hovers in place (the flight controller's own hover-
hold is doing the actual holding — kestrel just refrains from pushing the
sticks), looks at what's ahead, commits to a short leg in a direction that
balances "toward the operator's chosen goal" against "actually open, as far
as the camera can currently tell," flies that leg while continuously
re-checking that the way ahead is still open, and then stops and repeats.
If the way ahead closes off entirely — boxed in on all sides visible from
where it's hovering — it rotates in place to look around before deciding
where to go next, rather than guessing.

This is a real, known trade-off, not a limitation anyone's pretending
doesn't exist: stopping to think is slow, and a drone that can only plan
while stationary will occasionally back itself into a dead end that a
continuously-replanning system wouldn't. That's the same lesson the Mars
rover program learned — and solved, later, once more onboard compute became
available, by moving to continuous replanning instead of stop-and-go. The
same upgrade is the natural next step here too, once it's worth the added
complexity.

## Talking to the flight controller

kestrel talks to iNAV over **MSP**, the same lightweight serial protocol
iNAV's own configurator tool uses — not MAVLink, which is what most
companion-computer tooling (ROS, the PX4 ecosystem) expects. That's a
deliberate choice, not an oversight: iNAV is what a huge share of existing
FPV analog aircraft already run, it doesn't need companion-computer-class
hardware just to fly, and choosing it means the companion computer is a
genuinely optional add-on riding on top of an aircraft that already works
completely on its own — pull the Pi off, and it's just a normal FPV drone
again.

Every control command kestrel computes is **dry-run by default** — printed
and logged, but not actually sent to the flight controller — until
explicitly armed for live control. That's a deliberate piece of engineering
culture reflected directly in the software's default state, not just a
policy written down somewhere: the safest version of a bug is one that never
reaches the motors.

## How this gets tested without a battery in it

Most of the interesting failure modes here — what happens if GPS drops out
mid-flight, what happens if the camera perception stalls, whether the
failsafe actually engages return-to-home — are exactly the kind of thing
you'd rather not discover for the first time in the air. So there's a
simulated flight controller that responds to commands with plausible,
evolving telemetry (it actually "flies," in the sense that commanding
forward pitch moves its simulated GPS position), and a suite of automated
scenarios that fly a simulated goal around simulated obstacles and assert
the drone stays a safe distance away in every one of them, plus a set of
smaller tests exercising the position filter, the mode-safety logic, and
the wire protocol to the flight controller. All of it runs on a laptop, in
about two seconds, with no hardware attached — the same discipline major
open-source flight-controller projects use for their own testing.

## Where to go next

This document is the "why." For the actual hardware and what it costs, see
[`bom.md`](bom.md) — the physical drone platform this software runs on. For
"where is everything and how do I change it," see `AGENTS.md` in the
repository root — written for the level of completeness a developer (or an
AI assistant) making changes needs, rather than for a first read.
`onboard/docs/adding-a-control-mode.md` walks through adding a new mode end
to end. `ROADMAP.md` tracks what's built, what's next, and why it's
sequenced the way it is.

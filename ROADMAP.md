# ROADMAP — hardening fixes + phases P2–P6

Source: full architecture review (2026-07). Two tracks: **Track F** — operational
hardening of what exists (do before more flight testing), then **P2–P6** —
capability phases, continuing the numbering already used in `onboard/README.md`
(P0–P2 core = built; P3 LLM supervisor; P4 ground/telemetry; P5 localization;
P6 mission capability, defined here).

**Recommended execution order:**
`F1–F8  →  P2  →  P4a (recorder/replay)  →  P5a (VIO)  →  P5b (planner)  →  P3  →  P4b  →  P6`

Rationale: safety before capability; the flight recorder (P4a) multiplies the
value of every phase after it, so it jumps the queue; the planner (P5b) already
has a ready-made failing acceptance test in the SITL suite; the LLM supervisor
(P3) is independent and can slot in whenever.

Effort key: S ≈ hours, M ≈ 1–2 days, L ≈ 3days+.

**Status (2026-07):** ✅ **Track F (F1–F9), P2 (P2.1–P2.4), P5b, and the
crash-survivable black box (P4a-bb) are DONE** — all on
`claude/admiring-goldberg-sbcKz` with an in-tree CTest suite (10 tests, all
green: estimator, modes, MSP-over-PTY, config, RC, FcLink, realtime, nav_map,
SITL, black box). The SITL suite now completes the on-path / dead-centre
obstacle cases (P5b grid+planner) that previously stalled. A staged hardware
bring-up plan lives in `onboard/docs/hardware-bringup-checklist.md`. **F10**
(inference runtime swap) and **F11** (core rescheduling) stay deferred until
perception budget is the bottleneck. Recommended next: **P5a** (VIO — also
unblocks a *metric* P5b grid on the mono BOM). Items below are marked ✅ where
complete.

---

## Track F — hardening fixes (pre-flight)

### ✅ F1 — Blackboard staleness + think-tier watchdog (M) — DONE
`corridorValid` (and road/target/detections) are latches, not heartbeats: if the
think thread dies or the depth model stalls, consumers keep acting on frozen
perception forever.
- `WorldState`: per-group monotonic stamps (`corridorStampS`, `roadStampS`,
  `targetStampS`, `detStampS`) + an `age(now)` helper. Writers stamp on write.
- `ModeManager` reflex: stale corridor (> ~0.7 s) counts as **invalid**, with
  `modeReason="perception stale"`.
- `MissionController` THINK/MOVE require a fresh corridor; stale → SETTLE.
- Fly loop checks `Deliberator` liveness (last-tick age > 2 s → warn + treat
  think-tier outputs as stale). Display greys stale overlays.
- **Accept:** unit test with fabricated stale state; new SITL scenario
  "perception dropout mid-leg" → drone settles to hover, never cruises blind.

### ✅ F2 — Decouple FC keep-alive from the camera; camera-loss degrade — DONE
`cap.read()` blocks at the top of the fly loop and `fc->tick()`/`sendControl`
sit below it; a >200 ms camera stall breaks iNAV's ≥5 Hz MSP-RC requirement →
FC failsafe. Camera read failure currently **exits the process** while live.
- Dedicated FC I/O thread (~50 Hz): owns the serial link, repeats the latest
  `ControlCmd` (set atomically by the fly loop), polls telemetry into a
  mutex-guarded snapshot. Fly loop becomes a producer/consumer of that.
- Camera failure: do **not** exit — release control, keep the FC link alive,
  retry open every 2 s, surface `modeReason="camera lost"`.
- **Accept:** stall/unplug the (sim) camera → RC frame cadence stays ≥5 Hz
  (counter in `MspBackend`), process survives and recovers.

### ✅ F3 — Estimator-health gating in the mission — DONE
The EKF stays `estValid` while coasting GPS-denied with `estEphM` growing
unbounded; MOVE legs fly on that position.
- `Mission::Params::maxEphM` (~3 m): `estEphM` above it (or `estGpsDenied`
  persisting) → end leg, SETTLE, `missionPhase="SETTLE(est-degraded)"`.
- **Accept:** SITL scenario — cut GPS mid-leg → drone settles.

### ✅ F4 — Surface "motion mode without obstacle perception" — DONE (with F1)
With no depth model loaded, ASSIST/WAYPOINT get **no** reflex protection,
silently (`control_mode.cpp` reflex requires `corridorValid`).
- If `isMotion() && !ownsObstacleAvoidance()` and corridor invalid/stale →
  `modeReason="no obstacle perception"` (warn-only by default; optional param
  to refuse motion modes entirely).
- **Accept:** unit test + HUD shows the reason.

### ✅ F5 — ATTITUDE poll priority in the MSP backend — DONE
Round-robin polling refreshes attitude at ~5 Hz; `DepthNav::setAttitude()`
de-rotation consumes it — tens of degrees stale during a maneuver.
- `requestNextTelemetry()`: ATTITUDE every cycle, others interleaved
  (A,X,A,Y,A,Z…).
- **Accept:** bench-test reports per-message refresh rates.

### ✅ F6 — Write down the altitude-authority invariant — DONE
`throttle ∈ [0,1] → [1500,2000] µs` means the OS can climb/hold but never
command descent — a deliberate delegation to iNAV ALTHOLD/POSHOLD that is
currently implicit in `thrToUs()`.
- Document in AGENTS §6: *the OS commands lateral/yaw only; altitude is the
  FC's hold mode; throttle is a climb bias with 0 = hold.* (Signed throttle
  rejected for now — leaning on the FC hold is safer.)

### ✅ F7 — Bring the test suite in-tree — DONE
AGENTS §8 references validation tests that lived in discarded scratch dirs;
only `sim_autonomy` survives.
- Port under `onboard/test/` + `BUILD_TESTS`/CTest: estimator (glitch gate,
  re-acquire, ENU round-trip), mode manager (failsafe, reflex, suppression),
  VFH+ hysteresis, mission-cycle unit, MSP framing over a PTY.
- **Accept:** `ctest` green, no camera or hardware needed (CI-able).

### ✅ F8 — Runtime config file — DONE
Gains/params/cadences are compile-time (`Controller::Gains`,
`Mission::Params`, `ModeManager::Params`, scheduler slots) — field tuning
means recompiling.
- Tiny `key=value` parser (no new deps), `--config=kestrel.conf`, overrides
  those structs + scheduler cadences; `--dump-config` prints effective values.
- **Accept:** parser unit test; SITL runs with a modified config.

### ✅ F9 — Real-time scheduling for the fly loop + FcLink — DONE (jitter bench = field step)
The two/three-tier split (fly loop / Deliberator / FcLink, F2) exists so
inference can never stall control — but that guarantee is currently **soft**
(separate threads, default OS fair scheduling), not **hard**. Under real CPU
pressure (a depth model running full-tilt on the Deliberator thread), the
kernel scheduler can still starve the fly loop or FcLink. F2 solved the
camera-blocking case; this closes the general CPU-contention case the same
gap represents.
- `pthread_setschedparam(SCHED_FIFO, elevated)` on the fly-loop and FcLink
  threads so inference threads cannot preempt them.
- `pthread_setaffinity_np` pinning fly-loop + FcLink to one core, Deliberator
  to the rest; document (not code-enforce) an optional `isolcpus=` boot
  parameter to reserve that core from general kernel scheduling too.
- Pi-side prerequisites to verify/document alongside this: official 27W PD
  supply + active cooling (undervoltage/thermal throttling — check via
  `vcgencmd get_throttled` — silently defeats any of this), headless Lite OS.
- **Accept:** unit test asserting the scheduling policy/affinity got set
  (skips gracefully without CAP_SYS_NICE, e.g. in CI); a loaded-Deliberator
  SITL/bench run showing fly-loop tick jitter stays bounded under synthetic
  CPU load on the other threads.

### F10 — CPU inference runtime swap (NCNN / ONNX Runtime+XNNPACK) (L, scoped)
`perception.cpp` pins every model to `cv::dnn::DNN_BACKEND_OPENCV` /
`DNN_TARGET_CPU` — convenient (native `cv::Mat` I/O) but not the fastest CPU
inference path on ARM. `NavigateModule` (~110ms) and `DetectModule` (~90ms)
against a 60ms/tick budget (`PerceptionScheduler::budgetMs_`) is the actual
ceiling on "how much perception can run per tick" — this is the one change
that raises the ceiling rather than approaching it more efficiently.
- NCNN (built for ARM mobile/embedded) or ONNX Runtime + XNNPACK typically
  beat OpenCV DNN 2–3× on Cortex-A76-class hardware for the same model, with
  real INT8 quantization support.
- New inference backend behind the existing `IPerceptionModule` interface
  (no architecture change) + model reconversion for both the depth and
  detect models.
- **Not urgent today** — it becomes the forcing function once perception
  budget is actually the bottleneck, most likely when P5b's occupancy-grid
  module needs to run alongside depth/detect on the same tick budget. Revisit
  then, or sooner if field testing shows the scheduler is starving modules.
- Cheaper interim wins in the same direction, worth doing regardless: verify
  the apt `libopencv-dev` isn't leaving NEON kernels on the table (a
  from-source build with `-DCPU_BASELINE=NEON` is a known win); confirm
  `DepthNav`'s `WORK_W/WORK_H` downsample and the model's own input
  resolution are as small as accuracy tolerates; add `-flto` to the build.
- **Accept:** same-model latency benchmark, OpenCV DNN vs the new backend, on
  real Pi 5 hardware; SITL/bench regression stays green after the swap.

### F11 — Core rescheduling: burst multi-core perception while stationary (M, scoped)
`PerceptionScheduler::tick()` (`scheduler.cpp`) runs every module **sequentially
on the single Deliberator thread** — depth and detect never execute
concurrently, gated by one flat per-tick budget (`budgetMs_ = 60.f`) that
never changes with flight phase. F9's affinity split pins the fly loop +
FcLink to one core and leaves the rest of the Pi 5 to the Deliberator — but
since the Deliberator is one thread, it can only actively drive one of those
cores at a time. During `SETTLE`/`THINK`/`SCAN` (the move-stop-sense cycle's
stationary phases) the fly loop and FcLink are doing almost nothing and 2–3
cores sit idle while depth-then-detect still run one after another. This is
a scheduling problem, not a hardware one: **reschedule cores to the workload
that's actually there**, rather than holding a flight-speed-safe topology
during a phase that doesn't need it.
- A stationary signal already exists (mission phase `SETTLE`/`THINK`/`SCAN`,
  or more generally near-zero commanded + estimated velocity) — surface it
  to the Deliberator as a single bool/flag read each tick, no new estimator
  work required.
- On entering a stationary phase: relax the Deliberator's core pinning and
  dispatch the tick's heavy modules (depth, detect) as short-lived concurrent
  tasks instead of one sequential call each — `WorldModel` is already
  mutex-guarded, so independent writers is not a new hazard, just a new
  caller pattern.
- On leaving it (motion resumes): tear the worker tasks down and revert to
  today's single-thread sequential dispatch and F9's conservative affinity
  layout **before** control-loop timing starts mattering again — the burst
  path must never be live during MOVE.
- **The real risk, and the reason this is scoped, not casual:** any burst
  worker that outlives its stationary window, or lands on the fly-loop/FcLink
  pinned core, reintroduces exactly the jitter F9 was built to eliminate.
  This item does not ship without proving it can't do that.
- **Accept:** unit test asserting parallel dispatch triggers only under the
  stationary signal and sequential dispatch otherwise; a jitter bench on the
  fly-loop/FcLink threads showing **no regression** while burst-mode
  perception is active; a wall-clock benchmark showing depth+detect latency
  improves running concurrently vs. sequentially during a simulated
  SETTLE/THINK/SCAN phase.

---

## P2 — close out the FC / command layer

### ✅ P2.1 — Wire the RTH failsafe for real — DONE (LAND-on-degraded deferred)
`rthTrigger → fc->setMode(RTL)` hit the default no-op; failsafe was release the
sticks and hope. **Done:**
- `MspBackend::setMode(RTL)` drives a configured AUX channel
  (`failsafe.rth_aux` / `rth_aux_us`) high in every RC frame while neutral
  sticks keep RC alive; any other mode releases it. Unconfigured → returns
  false and the caller falls back to the FC's own RC-loss failsafe.
  Covered by `test_msp`.
- **Deferred → P2.1b: estimator-aware LAND.** RTH flown on a degraded position
  estimate is worse than not flying home (field report: hackathon indoor
  "return-to-launch drift"). When `estEphM` is past the gate (or GPS-denied on
  synthetic GPS), failsafe should command **LAND**, not RTH — descending needs
  no position. Needs a second AUX (or a NAV-mode select) + the ModeManager
  failsafe reading `estEphM`. Not yet implemented.

### ✅ P2.2 — RC AUX command source — DONE
Make the drone flyable without a laptop: the world model was designed for
"any command source" — add the radio as one.
- Small `RcCommandSource` in the fly loop reading `FcTelemetry.rc[]`:
  configured channels → mode select (3-pos switch), AUTONOMY goal nudge
  (left/right momentary or a pot), GO/STOP latch — mirroring the keyboard block.
- **Accept:** bench with sim FC (synthetic RC) + real radio props-off.

### ✅ P2.3 — SHADOW mode: advisory autonomy overlay — DONE
Test autonomy in real flight with zero risk: the **operator flies** (mode
always releases control, like FLY), while the full AUTONOMY stack runs
underneath in dry-run and draws what it *would* do on the feed.
- New `ShadowMode` (`IControlMode`): owns a `MissionController` like
  AUTONOMY (same onEnter/onExit, goal steer + GO keys work), calls
  `mission_.update()` every tick but **discards the command** and returns
  `{valid=false}` — release, unconditionally. `isMotion()=false` (no reflex
  needed; nothing moves on its account).
- HUD (already draws the goal arrow + corridor arrow when `missionActive`):
  add an "intended-command" arrow/marker — the yaw/pitch the discarded
  command would have applied — plus the phase tag (`SHADOW:MOVE` etc.), in a
  distinct colour so it can't be mistaken for live control.
- Publish the intended command into `s.control` with `controlActive=false`
  (the plumbing already distinguishes computed-vs-sent), so the recorder
  (P4a) captures "what autonomy wanted vs what the pilot did" for tuning.
- **Accept:** unit test — SHADOW never returns `valid=true` regardless of
  world state (including GO latched, failsafe-adjacent states); SITL run
  shows mission phases advancing while the sim FC receives no commands;
  props-off field pass comparing arrows to pilot judgement.

### P2.4 — Assist-mode field checklist (S)
Bumpless-takeover (baseline latch) validated bench + props-off; written
procedure in `onboard/docs/`.

### P2.5 — MAVLink minimal backend (L, **deferred**)
HEARTBEAT + RC_CHANNELS_OVERRIDE + GPS_INPUT only. Pull the trigger only if an
ArduPilot airframe actually materialises.

---

## P3 — LLM supervisor + guard (advisory only)

Per the original architecture sketch: a `llama.cpp` sidecar that reads the
scene and *suggests*; it can never touch control.
- Transport: kestrel already streams `brief()`; add a command FIFO/UDS.
- **Guard layer (the real work):** whitelist schema —
  `select_mode(name)`, `set_goal_bearing(deg)`, `go/stop`,
  `tune(param, value)` clamped to hard bounds — reject everything else,
  rate-limit, log every accepted/rejected command.
- Watchdog: LLM silence or death never affects flight (it's a reader/suggester,
  not a dependency).
- Scope: §7 boundary applies — the LLM selects among allowed *navigation*
  goals; no target-relative flight commands exist in the schema.
- **Accept:** guard unit tests (malformed / out-of-bounds / flooding inputs);
  kill the sidecar mid-run → zero effect on the fly loop.

### P3.1 — mission language → object search, small quantized LLM (idea, **unscheduled experiment**)
Extends P3 rather than replacing it: parse a compound mission ("look for a
house and find a chair") into an ordered subgoal list (P3's existing
whitelist-schema job), execute each subgoal as **object-goal navigation**
(frontier exploration — nav-sim's `explore-rth` already is this — + the
existing move-stop-sense/occupancy-grid approach, unchanged), and optionally
let the same LLM sidecar bias *which frontier to try first* with a
commonsense prior ("chairs are more likely near a house"), output
grammar-constrained to a fixed label so it's whitelist-compatible like
everything else P3 emits — never a direct command. Written up with concrete
Pi-5 `llama.cpp` quantization numbers and a proposed nav-sim experiment:
`ideas/mission-language-and-object-search.md`. Not designed against the real
interfaces; if pursued, starts in nav-sim, no hardware.

## P4 — flight recorder, replay, ground view

### ✅ P4a-bb — Crash-survivable black box — DONE (replaces the recorder as the bring-up priority)
Ahead of the full recorder/replay tool, the first hardware trials need one
thing above all: **whatever reached disk survives however the process dies**
(brownout, lock-up, crash-on-impact). Built as `BlackBox` (`black_box.*`):
- Append-only, fixed-size (~122 B) records, one per fly-loop tick; per-record
  CRC32; a 2-byte sync word; periodic `fsync` (default 1 s) so data actually
  reaches the medium, not just the page cache. `--blackbox=<path>`.
- Best-effort in the control path: a write error disables the box, never
  throws or blocks the fly loop. Also logged through a camera outage.
- Offline decoder `tools/blackbox_decode` → CSV; resyncs past a torn/garbage
  record instead of giving up, so one bad record never costs the rest.
- **Accept (met):** `test_black_box` — round-trip, N-record file decode,
  truncated-tail recovery (N-1 intact), in-record corruption rejected by CRC,
  and mid-file garbage resynced. 10/10 CTest green.

### P4a — Recorder + replay (M) — *still wanted, now lower priority*
The black box covers crash-survivable capture; the richer replay tool remains:
- `--record=dir`: jsonl `WorldState` @10 Hz + events (mode changes, GO,
  failsafe, lock) + optional frame dump (MJPEG); ring-file rotation.
- `--replay=dir`: `ReplayFcBackend` + replay camera feed the **real pipeline**
  headless from a recording — every field bug becomes desk-reproducible.
  (A converter from the black-box CSV into this replay format is the cheap
  bridge between the two.)
- **Accept:** record a SITL run, replay it, diff the `brief()` streams.

### P4b — Ground view (M)
- Lightweight tail of the jsonl over TCP/websocket → single-page map (ENU
  track), HUD state, corridor/mission phase, est health. No ROS, no heavy deps.

### P4c — Remote/edge-compute simulation over WiFi (idea, **unscheduled experiment**)
Motivated by tactical-5G (Nokia Banshee-class) deployments in Ukraine: a
resilient local network bubble would let a drone offload perception to
ground/edge compute even in a contested-EW environment, matching kestrel's
existing hybrid edge+onboard-autonomy story rather than requiring a new one.
Maps cleanly onto the current architecture with **no new fallback logic** —
the blackboard's staleness gating (F1) already treats any perception source
as "trust it while fresh, ignore it once stale."
- `RemotePerceptionModule` (C++, implements `IPerceptionModule`): ships a
  frame over the network each tick, writes the result into `WorldModel` on
  success. On timeout/failure it simply doesn't write — existing
  staleness/`corridorValid` handling takes over exactly as it does for a
  stalled onboard model (F1), so this needs no bespoke degrade path.
- Small Python edge/ground server (Flask/FastAPI) for the desktop side —
  receives a frame, runs depth/detect, returns the result.
- Testbed: WiFi between the Pi and a desktop stands in for the tactical link.
  Use Linux `tc`/`netem` to inject realistic degradation (latency, jitter,
  packet loss, bandwidth caps) and characterize where the mission degrades
  gracefully vs. where it doesn't.
- New SITL fault-injection scenario, "remote link dropout mid-leg" —
  alongside the existing "perception dropout mid-leg" / "GPS loss mid-leg"
  scenarios (F1/F3): drop the simulated remote link mid-leg, assert the
  mission settles rather than flying on stale remote perception.
- **Accept:** `RemotePerceptionModule` unit test (timeout → no stale write);
  SITL "remote link dropout mid-leg" passes; a `netem`-degraded bench run
  characterizing behavior across latency/loss levels.

## P5 — localization + planning

### P5a — VIO velocity (M/L)
The single biggest GPS-denied win; the EKF hook (`updateVisionVelocity`)
already exists.
- Sparse LK optical flow (fly-loop frames), de-rotated with FC attitude
  (already available, fresher after F5), scaled by height (baro/ToF) →
  body-frame velocity → EKF.
- **De-risk in hardware first:** an FC-side optical-flow module (Matek
  3901-L0X, flow+ToF into iNAV directly) gives indoor position hold with zero
  Pi involvement — field-proven pattern (PixRacer+flow at the UltraHack
  demos). SETTLE's "the FC must actually hover" requirement then holds
  indoors before any Pi VIO exists; Pi-side VIO remains the estimator's win.
- **Accept:** SITL with synthetic flow → GPS cut → `estEphM` stays bounded;
  hand-carry bench test against tape-measure ground truth.

### ✅ P5b — Occupancy grid + global planner — DONE (metric on ToF; nominal on mono)
The SITL suite's two expected-stall scenarios (obstacle on-path / dead-centre)
were the acceptance gate — they now PASS. **Done:**
- `LocalMap` (`nav_map.*`): rolling log-odds grid (80×80 m / 0.5 m cells)
  accumulated from the corridor polar scan; wavefront BFS from a goal cell
  projected along the goal bearing, inflated by a plan berth wider than the
  live safety margin so the routed path flows instead of skimming; gradient
  descent → next bearing. Robust to the drone sitting inside the berth (snaps
  the descent start to the nearest free cell).
- `MissionController` integrates the scan + replans each active tick; the plan
  supplies a smarter GOAL bearing into the existing reactive blend (routes
  around obstacles the FoV forgot) and steers SCAN toward the remembered
  opening. The live corridor still governs speed + the stop reflex, so a
  stale/wrong grid can't drive into something the live sensor sees.
- Perception publishes the scan (`DepthNav::openHist` → `corridorScan`):
  metric on the ToF path, NOMINAL scale on monocular. Config: `nav.use_map`,
  `nav.plan_berth_m`, `nav.grid_size_m/cell_m`.
- **Result:** SITL 5/5 goal-completion (both hard cases reach in ~40 s), 7/7
  standoff safety, 2/2 stop-on-loss. `test_nav_map` covers the grid+planner
  standalone. ctest 9/9.
- **Caveat / remaining:** the grid is geometrically sound only with metric
  depth (ToF/stereo — not in the current mono-only BOM); on monocular it's a
  nominal-scale approximation. "Unknown = free" optimistic replanning means it
  relies on the live reflex for the invisible-obstacle class (nets/wires);
  that structural mitigation (low cruise scaled by openness, stop-to-sense)
  stays. Full P5 SLAM / global localization is still future.

### P5c — Absolute vision fixes (deferred)
Canopy/image-to-map localization → `updateVisionPose()`. Only after P5a/P5b
prove out.

## P6 — mission capability (all within the §7 boundary)

- **P6.1** COCO person/vehicle detector swap (model + labels via config). (S)
  **Validate against the domain it will fly in** — models trained on one
  domain silently fail in another (UltraHack: a fire detector trained on real
  fires didn't recognise artificial demo flames). Runtime open-vocabulary
  models don't fit a Pi 5 CPU; our equivalent is the desktop DINOv2/SAM2
  pipeline as an offline auto-labeler → fine-tune the small onboard YOLO on
  demo-domain footage.
- **P6.2** Detection supervision: person/vehicle proximity during
  WAYPOINT/AUTONOMY → alert + optional auto-HOLD (config). (M)
- **P6.3** FOLLOW_SUBJECT + ORBIT_INSPECT: **standoff-keeping only** — hold a
  range band + bearing on a tracked subject (yaw + gentle lateral), and an
  orbit-scan variant (circle the subject at a FIXED standoff radius, camera
  facing it — the inspect pattern). Approach is *to the standoff radius*,
  never onto the subject; needs a range estimate (box-size heuristic or ToF).
  Explicitly scope-checked against §7. (L)
- **P6.4** Multi-goal missions: ordered ENU/GPS goal list sequenced above
  `missionGoalBearing`; set from ground view (P4b) or RC. (M)
- **P6.5** Geofence + max range/altitude enforced in `ModeManager` as a third
  safety layer (config). (M)
- **P6.6** Context-gated perception + nav profile (idea, **unscheduled**) — a
  cheap always-on classifier names the current setting (indoor/open/forest)
  from object detections, gating both which specialist perception models run
  and which `Mission::Params` profile is active. Written up, not designed
  against the real interfaces yet: `ideas/context-gated-perception.md`. If
  pursued, the low-risk first cut is the `Params`-profile half alone (table
  keyed by setting label, debounced switch, safe-default-on-uncertainty),
  with model-gating as a separate, later step.

### P6.7 — Precise analog-video overlay: MCU + LM1881 keyer (idea, **unscheduled experiment**, M hardware+firmware)
Improve the OSD elements on the operator's analog VTX feed beyond what iNAV can
draw. iNAV's OSD is a MAX7456 **character cell** grid (~30×16): it can only
stamp font glyphs on a coarse grid, and stock firmware has no path to position
an element from external target data. A tracker's bounding box therefore
quantizes to ~11 px steps and jumps cell-to-cell. This item gets **pixel-precise,
smoothly-moving** overlay elements (bounding box, reticle) onto the analog feed,
fed by the Pi tracker's box coordinates.
- **Placement:** the keyer MCU sits **between the FC's video-out and the VTX**
  (`camera → FC [telemetry OSD] → MCU [tracker box] → VTX`), so the FC's own
  OSD and the tracker box stack without conflict.
- **Method — genlock + analog key, no digitizing.** It never decodes/re-encodes
  the video (no framebuffer, no decoder/encoder chips, **near-zero latency**).
  An **LM1881** (8-pin, ~$1.50) separates H/V sync + odd-even field; the MCU
  times exact pixel positions along each scanline and drives a fast analog
  switch/transistor to pull the video line to white/black **only** at the
  box-edge pixels — the original analog passes through untouched everywhere else.
- **MCU: RP2350 (Pico 2).** Its PIO does the cycle-precise, sync-genlocked pixel
  timing this needs (the hard part) almost for free. Pi → MCU over UART/SPI: a
  few bytes of box coords per frame; the MCU redraws the outline each field.
  (STM32G4 is the fewest-chips alternative — an on-chip comparator separates
  sync without the LM1881 — at the cost of more timing firmware than PIO.)
- **Consistency with the tracker:** the box drawn this field is last field's CV
  result **projected forward** — exactly the tracker's existing latency
  compensation (`aimX/aimY`, `latencyFrames`), so algorithm and overlay agree.
- **Caveats (honest):** monochrome outline only — a filled/colour box needs the
  heavier digitize→draw→re-encode seeker (decoder + MCU with DCMI/LTDC +
  encoder, ~1-frame latency), deferred; and robust sync separation is the one
  real engineering risk — the LM1881 makes it a non-issue, a bare-comparator
  approach is where the debugging time goes.
- **Cheap first cut (no added hardware):** a small **iNAV firmware fork** — a
  custom MSP element that positions box glyphs on the existing MAX7456 from
  Pi-fed coords — already gives a *coarse* (grid-snapped, ~10-30 Hz) box using
  the OSD chip that's already in the FC's video path. That's the low-effort
  version; this P6.7 keyer is the precise upgrade for when the coarse box isn't
  enough. Both are pure operator-display situational awareness (within §7).
- **Accept:** bench — feed a known CVBS source + synthetic box coords over UART,
  scope the output showing a stable, pixel-positioned outline genlocked to the
  incoming video with no sync disturbance; then end-to-end with the Pi tracker
  driving a live analog feed through to a VTX/monitor.

---

## Dependency graph (summary)

```
F1..F8 ──► P2 ──► real-flight autonomy testing
   │        │
   │        └──► P6 (needs P2 command paths)
   ├──► P4a ──► P4b, and multiplies P3/P5/P6 debugging
   ├──► P5a ──► P5c
   └──► P5b (uses SITL gate; independent of P5a)
P3: independent after Track F (guard layer reuses F8 config bounds)
```

Keep `AGENTS.md §9` and this file honest as items land.

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

---

## Track F — hardening fixes (pre-flight)

### F1 — Blackboard staleness + think-tier watchdog (M) — *highest priority*
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

### F2 — Decouple FC keep-alive from the camera; camera-loss degrade (M/L)
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

### F3 — Estimator-health gating in the mission (S)
The EKF stays `estValid` while coasting GPS-denied with `estEphM` growing
unbounded; MOVE legs fly on that position.
- `Mission::Params::maxEphM` (~3 m): `estEphM` above it (or `estGpsDenied`
  persisting) → end leg, SETTLE, `missionPhase="SETTLE(est-degraded)"`.
- **Accept:** SITL scenario — cut GPS mid-leg → drone settles.

### F4 — Surface "motion mode without obstacle perception" (S)
With no depth model loaded, ASSIST/WAYPOINT get **no** reflex protection,
silently (`control_mode.cpp` reflex requires `corridorValid`).
- If `isMotion() && !ownsObstacleAvoidance()` and corridor invalid/stale →
  `modeReason="no obstacle perception"` (warn-only by default; optional param
  to refuse motion modes entirely).
- **Accept:** unit test + HUD shows the reason.

### F5 — ATTITUDE poll priority in the MSP backend (S)
Round-robin polling refreshes attitude at ~5 Hz; `DepthNav::setAttitude()`
de-rotation consumes it — tens of degrees stale during a maneuver.
- `requestNextTelemetry()`: ATTITUDE every cycle, others interleaved
  (A,X,A,Y,A,Z…).
- **Accept:** bench-test reports per-message refresh rates.

### F6 — Write down the altitude-authority invariant (S, decision)
`throttle ∈ [0,1] → [1500,2000] µs` means the OS can climb/hold but never
command descent — a deliberate delegation to iNAV ALTHOLD/POSHOLD that is
currently implicit in `thrToUs()`.
- Document in AGENTS §6: *the OS commands lateral/yaw only; altitude is the
  FC's hold mode; throttle is a climb bias with 0 = hold.* (Signed throttle
  rejected for now — leaning on the FC hold is safer.)

### F7 — Bring the test suite in-tree (M)
AGENTS §8 references validation tests that lived in discarded scratch dirs;
only `sim_autonomy` survives.
- Port under `onboard/test/` + `BUILD_TESTS`/CTest: estimator (glitch gate,
  re-acquire, ENU round-trip), mode manager (failsafe, reflex, suppression),
  VFH+ hysteresis, mission-cycle unit, MSP framing over a PTY.
- **Accept:** `ctest` green, no camera or hardware needed (CI-able).

### F8 — Runtime config file (S/M)
Gains/params/cadences are compile-time (`Controller::Gains`,
`Mission::Params`, `ModeManager::Params`, scheduler slots) — field tuning
means recompiling.
- Tiny `key=value` parser (no new deps), `--config=kestrel.conf`, overrides
  those structs + scheduler cadences; `--dump-config` prints effective values.
- **Accept:** parser unit test; SITL runs with a modified config.

---

## P2 — close out the FC / command layer

### P2.1 — Wire the RTH failsafe for real (S) — *the highest-value 30 lines*
`rthTrigger → fc->setMode(RTL)` currently hits the default no-op; failsafe =
release the sticks and hope.
- `MspBackend::setMode(RTL)`: drive a configured AUX channel
  (`rth-aux`, `rth-aux-us` in config) in every RC frame, sticky until cleared;
  matches an iNAV "Nav RTH" mode range set in the configurator.
- **Failsafe policy is estimator-aware:** RTH flown on a degraded position
  estimate is worse than not flying home (field report: hackathon teams hit
  exactly this as indoor "return-to-launch drift"). When `estEphM` is past the
  gate (or GPS-denied on synthetic GPS), failsafe commands **LAND**, not RTH —
  descending needs no position.
- **Accept:** bench vs the iNAV modes tab (mode goes active); SITL asserts the
  failsafe path sets the AUX value, and selects LAND when the estimate is
  degraded.

### P2.2 — RC AUX command source (M)
Make the drone flyable without a laptop: the world model was designed for
"any command source" — add the radio as one.
- Small `RcCommandSource` in the fly loop reading `FcTelemetry.rc[]`:
  configured channels → mode select (3-pos switch), AUTONOMY goal nudge
  (left/right momentary or a pot), GO/STOP latch — mirroring the keyboard block.
- **Accept:** bench with sim FC (synthetic RC) + real radio props-off.

### P2.3 — SHADOW mode: advisory autonomy overlay (S/M)
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

## P4 — flight recorder, replay, ground view

### P4a — Recorder + replay (M) — *do early*
- `--record=dir`: jsonl `WorldState` @10 Hz + events (mode changes, GO,
  failsafe, lock) + optional frame dump (MJPEG); ring-file rotation.
- `--replay=dir`: `ReplayFcBackend` + replay camera feed the **real pipeline**
  headless from a recording — every field bug becomes desk-reproducible.
- **Accept:** record a SITL run, replay it, diff the `brief()` streams.

### P4b — Ground view (M)
- Lightweight tail of the jsonl over TCP/websocket → single-page map (ENU
  track), HUD state, corridor/mission phase, est health. No ROS, no heavy deps.

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

### P5b — Occupancy grid + global planner (L) — *fixes the known stall*
The SITL suite's two expected-stall scenarios (obstacle on-path / dead-centre)
are the acceptance gate — they exist today and fail by design.
- Rolling local grid (~40×40 m, 0.5 m cells, log-odds) accumulated from
  corridor/ToF rays — built during SETTLE (move-stop-sense already buys the
  compute window). Runs as a think-tier `IPerceptionModule` writing
  `planBearing/planValid` into the world model.
- **Unknown ≠ free.** Cells with no return / low-confidence depth carry a
  traversal cost, never "open" — every ranging modality has an invisible
  obstacle class (LiDAR and nets/wires, famously: an UltraHack team flew into
  a net LiDAR couldn't see; monocular depth has the same blind spot). Our
  structural mitigations stay in place: low cruise speed scaled by openness,
  and stopping to sense.
- Wavefront/A* on the grid → next-leg bearing; `MissionController` consumes
  `planBearing` when valid, falls back to the reactive goal+corridor blend
  when not.
- **Accept:** flip both stall scenarios to `expectReach=true` in
  `sim_autonomy.cpp` and pass; standoff safety invariant unchanged (5/5).

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

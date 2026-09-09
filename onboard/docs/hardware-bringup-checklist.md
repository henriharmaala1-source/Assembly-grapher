# Hardware bring-up checklist

Everything in this repo has been validated in software-in-the-loop, not on a
real aircraft. This is the staged plan for closing that gap without risking the
airframe to find a software bug. **One new unknown per stage. Props off until
Stage 5. Dry-run until each mode is proven.**

The guiding rule: never debug two unknowns at once. Each stage adds exactly one
new physical element and does not proceed until that element is trusted.

Turn on the black box for every stage from 1 onward:
`--blackbox=/path/on/persistent/media.kbb`, decode afterwards with
`blackbox_decode`. Every anomaly seen in the field should become a replay/SITL
test case before the next session.

---

## Stage 0 — before touching hardware (desk)

- [ ] Full SITL suite green on the build that will be flashed: `cmake -B build
      -DBUILD_TESTS=ON && cmake --build build && ctest --test-dir build`.
- [ ] `--dump-config` reviewed — every safety/mission/gain knob is what you
      expect, written to a config file under version control (not left to CLI
      memory).
- [ ] A ToF path chosen and buildable: either the MCU sensor-hub firmware
      (streams the `mcu-tof` serial frame — the Pi-side parser is done and
      tested) **or** the vendor VL53 ULD driver ported onto `i2c_hal`. Until one
      exists, the ToF path reports no data and the runtime falls back to nominal
      monocular — know which one you are flying.

## Stage 1 — Pi alone (no FC, no camera)

Goal: prove the compute platform holds up before anything depends on it.

- [ ] Real 27 W USB-PD supply + active cooling fitted. Headless Lite OS.
- [ ] Run the depth model in a loop under load for 30+ min. Watch
      `vcgencmd get_throttled` — **any** non-zero throttle flag silently defeats
      the real-time scheduling guarantees; fix cooling/power before continuing.
- [ ] Confirm `SCHED_FIFO` actually takes (needs `CAP_SYS_NICE` or root) — the
      startup log says whether it fell back to normal priority.
- [ ] Record real per-model inference latency on this silicon and compare to the
      60 ms scheduler budget (`--budget`). The desktop ~110 ms / ~90 ms figures
      are estimates; retune the budget to measured numbers.

## Stage 2 — capture chain (Pi + camera tap, no FC)

Goal: characterise the analog-capture path, which has no proper COTS solution —
this is an experiment with pass/fail criteria, not a purchase.

- [ ] For each candidate USB capture dongle: confirm it enumerates on the Pi 5
      (V4L2) and the ARM kernel has a working driver for its chipset.
- [ ] Measure glass-to-`cv::Mat` latency (film a blinking LED, diff timestamps).
- [ ] Check frame-rate stability and dropped frames over 10+ min.
- [ ] Inspect for interlacing/comb artifacts — CVBS is interlaced and the
      perception models have never seen comb artifacts; note deinterlacing need.
- [ ] Hot-unplug/replug with the fly loop running: the camera-loss degrade path
      must hold (command a hover, retry open) and never exit the process.

## Stage 3 — Pi + FC on the bench (props OFF, dry-run, no battery/motors)

Goal: first contact between our MSP implementation and **real iNAV firmware** —
everything before now was a PTY-simulated FC.

- [ ] `--fc=msp --bench-test`: telemetry decode matches the iNAV Configurator
      (attitude, battery, GPS fix, sats).
- [ ] Dry-run RC map: `MSP_SET_RAW_RC` moves the **right** channels in the
      Configurator receiver tab — **verify AETR order** (throttle=Ch2, yaw=Ch3);
      the classic RPYT assumption swaps throttle and yaw and is dangerous.
- [ ] Synthetic GPS (`--feed-gps`) shows up as a fix in iNAV, subject to its
      glitch/age gating.
- [ ] RTH AUX trigger (`failsafe.rth_aux`) drives the configured channel high and
      iNAV enters NAV RTH.
- [ ] Fault-inject on the bench: yank the camera USB (FcLink must keep RC alive
      with a neutral hover); kill the kestrel process (iNAV's own RC-loss
      failsafe must catch); unplug GPS (estimator must gate, not drift).

## Stage 4 — powered aircraft on the bench (props OFF, on battery)

Goal: surface EMI and power-integrity problems that only appear under real
current.

- [ ] ESC/VTX noise coupling into the capture dongle and I²C/ToF bus — check for
      corrupted frames or dropped ranging under VTX transmit + motor current.
- [ ] Voltage integrity: the Pi must not brown out under motor spin-up load
      (re-check `get_throttled`).
- [ ] GPS/compass: sats acquired and stable with the VTX transmitting.
- [ ] Run the full **assist-mode bench checklist** (`assist-bench-checklist.md`)
      as written — bumpless manual→assisted handoff.

## Stage 5 — flight, SHADOW first, then earn each mode

Goal: watch the autonomy before ever trusting it with control.

- [ ] First flights in `SHADOW`: pilot flies manually, autonomy runs live in
      dry-run and draws its intended command on the feed. Fly the SITL mission
      scenarios; **record everything** and compare what the autonomy wanted to do
      against what a sane pilot did.
- [ ] Only when SHADOW output looks trustworthy across many flights: arm
      `ASSIST`, then `HOLD`, then `AUTONOMY` over an open field — same order as
      the SITL scenario ladder, one mode per session.
- [ ] After every flight: `blackbox_decode` the log, and turn any surprise into
      a new SITL/replay test before the next flight.

---

**Non-negotiables baked into the order above**

- Props off until Stage 5; dry-run (`--allow-control` off) until each mode is
  individually proven.
- SHADOW before authority, always — no mode is armed in flight before its
  intended commands have been watched passively.
- Every field anomaly becomes a regression test. The black box exists so that
  "it did something weird" is always reproducible.

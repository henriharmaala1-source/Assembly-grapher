# Flight-assist bench validation checklist

Procedure for validating **flight-assist mode** (bumpless takeover via the
baseline latch) on the bench, **props removed**, before any flight. Assist
mode trims the OS's control output *relative to the operator's live stick
positions* rather than commanding absolute sticks from neutral, so the
takeover starts from where the pilot's hands are — this checklist confirms
that behaviour, its authority limits, and clean release, without risking a
spin-up.

> **This checklist assumes props are OFF for every step.** No step here
> requires a spinning motor. Do not fit props until assist behaviour has
> passed on the bench and you are moving to a controlled flight test.

---

## What assist mode does (the thing being validated)

- **Total autonomy** (default): `sendControl()` writes absolute stick µs from
  neutral — the OS is the sole stick source.
- **Flight assist** (`--assist`, or `assist` in the config): `sendControl()`
  writes `baseline + delta`, where `baseline` is the operator's RC latched at
  the dry→live transition (`latchBaseline()`), and `delta` is the OS's
  normalized command scaled to µs. The takeover is *bumpless* — at the
  instant of engagement with a zero OS command, the output equals the
  operator's current sticks, so nothing jumps.

The latch fires once, on the dry→live edge, in the fly loop:
`if (allowControl && !wasAllow && assistMode) fcLink.latchBaseline()`. If no
operator RC has been seen (`rcCount < 4`), assist falls back to total
autonomy rather than trimming from a bad baseline.

---

## Pre-conditions

1. **Props removed.** Confirm visually on all four motors.
2. Battery connected, or bench power to the FC + a way to spin motors slowly
   for the authority check (props still off).
3. FC (iNAV) configured and flyable on the operator's radio normally — assist
   is validated on top of a working manual setup, not instead of one.
4. iNAV RC configured so the Pi's `MSP_SET_RAW_RC` can override while the
   operator's channels are still readable via `MSP_RC` (this is what lets the
   baseline reflect live operator input). Verify in the iNAV configurator's
   Receiver tab that operator stick movement is visible **and** that MSP
   override is armed on its AUX switch, before proceeding.
5. kestrel built, FC link up. Confirm with `--bench-test`:
   `./build/kestrel --fc=msp --fc-port=<dev> --fc-baud=<baud> --bench-test`
   → link UP, attitude/GPS/battery updating, RC channels showing operator
   input as the sticks move.

---

## Procedure

Run kestrel with assist enabled but **dry-run** (control not yet live):

```
./build/kestrel --fc=msp --fc-port=<dev> --fc-baud=<baud> --assist --display
```

### 1. Baseline capture (bumpless takeover)

1. Hold the operator sticks at a known, non-centre position (e.g. throttle
   ~40 %, a little right roll) and keep them there.
2. Engage live control (`space`, or `--allow-control` from the start). This is
   the dry→live edge that latches the baseline.
3. In the iNAV configurator (or a second `--bench-test` view of the RC
   channels), confirm the FC's channels **did not jump** at engagement — with
   a zero OS command (FLY/ASSIST releasing, or AUTONOMY hovering), the output
   should equal the sticks you were holding. **A jump here is a failure** —
   stop, the baseline latch is not working.

### 2. Trim direction and authority

1. With control live, command a small OS output (e.g. select AUTONOMY and
   press GO with a clear corridor so it commands a gentle forward pitch, or
   use HOLD which commands centre).
2. Confirm the resulting channel µs = your held baseline **plus** the OS delta
   in the correct direction (forward pitch → pitch channel moves the correct
   way from baseline, not from centre).
3. Confirm the delta is **bounded**: drive a full-scale OS command and verify
   the channel moves by no more than the configured authority (default
   ±500 µs at full stick in the backend's `addDelta`, further clamped by the
   controller's `maxAuthority`). It must never railed to an extreme from a
   mid-baseline. **Unbounded output is a failure.**

### 3. Operator override still present

1. With control live and an OS command active, move the operator sticks.
2. Confirm the output tracks `operator + OS delta` — i.e. the pilot can still
   move the aircraft's setpoint; the OS is trimming, not locking out the
   pilot. Confirm the AUX channels pass the operator's switches through
   unchanged.

### 4. Clean release / disengage

1. Disengage live control (`space` back to dry-run).
2. Confirm the FC's channels return to pure operator RC with no residual
   offset — the OS is no longer contributing.
3. Confirm re-engaging re-latches a fresh baseline at the new stick position
   (repeat step 1 with the sticks somewhere else).

### 5. No-baseline fallback

1. Disconnect / stop the operator RC source so kestrel sees no operator
   channels (`rcCount < 4`).
2. Engage assist. Confirm it falls back to **total autonomy** (absolute sticks
   from neutral) rather than trimming from a stale/zero baseline — the log
   notes the baseline as invalid.

---

## Abort criteria (any of these → stop, do not fly)

- Channels jump at engagement (baseline latch not bumpless).
- OS delta drives a channel to an extreme from a mid-baseline (authority not
  bounded).
- Operator sticks stop affecting the output while live (pilot locked out).
- Residual offset remains after disengaging (release not clean).
- FC enters failsafe during the test (RC cadence too low — check the link and
  the F2/F9 real-time-scheduling notes in `AGENTS.md`).

---

## After the bench passes

Only then move to a controlled flight test: open area, low and slow, finger
on the disarm, assist engaged from a stable hover, and be ready to disengage
(`space` / the configured RC switch) and fly it out manually. The dry-run
default and the `x` abort → RTH path are your backstops; know both before you
arm with props on.

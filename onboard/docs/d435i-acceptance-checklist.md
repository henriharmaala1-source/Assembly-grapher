# D435i — buying a used one: 8-minute checklist

For a live meet with a seller. **RealSense Viewer only** — no tape measure, no
scripts, no calculations. Everything here is a yes/no or a number the Viewer
prints for you.

The goal is not to grade depth quality. It is to rule out the faults that
**cannot be fixed afterwards**. Bias, bow and noise are all measurable at home
and two of the three are recalibrable in 15 seconds; a dead imager, a dead IMU
or a damaged connector are not.

**Take with you:** laptop with RealSense SDK installed and already opened once,
a **known-good USB 3 cable** (see §0), and something flat and textured to point
at — a painted wall or a poster is fine.

---

## 0. Before you go

* **Install the SDK and open the Viewer at home first.** Debugging a driver
  install in a car park is not a negotiation position.
* **Bring your own cable.** A USB 2.0 cable with a C connector plugs in happily
  and negotiates USB 2, and the camera then refuses the higher modes and drops
  frames — which looks exactly like a bad camera. Blue port ≠ USB 3 cable; the
  colour convention is advisory and says nothing about what you plugged in.
* **Straight into the laptop, no hub.** A hub shares bandwidth and adds a
  suspect.

---

## 1. It enumerates, on a real USB 3 link  ·  30 s

Plug in. The device appears in the Viewer's left panel. Open its **info** panel.

| read | want | if not |
|---|---|---|
| USB Type Descriptor | `3.1` / `3.2` | `2.1` → swap the cable and retry. Do not grade anything on a USB 2 link |
| Firmware Version | note it | very old is fine, it is updatable; note it so a later change is reversible |
| Serial Number | note it | should match any photo the seller sent |

**Walk away if:** it does not enumerate at all on a cable you know works.

---

## 2. Both IR imagers are alive  ·  90 s  ← the one people skip

Turn on **Stereo Module → Infrared 1** and **Infrared 2**. Look at the two raw
images side by side. This is the sensor itself; depth is downstream of it, so a
fault here explains everything else and cannot be calibrated away.

* Both present, similar brightness and sharpness?
* Any **fixed** dark blobs, scratches or haze that stay put when you move the
  camera? That is the window, not the scene.
* One noticeably softer than the other → knocked lens.

**Walk away if:** one IR stream is missing, black, or visibly damaged.

---

## 3. The projector works  ·  30 s

With an IR stream still visible, toggle **Emitter Enabled** off and on in the
Stereo Module options. The dot pattern should appear and vanish in the IR view.
It is invisible to the eye, so this is the only way to check it.

**Not a deal-breaker for us** — outdoors a ~1 W pattern against sunlight does
almost nothing past a couple of metres, and our forest numbers assume passive
stereo. But a dead emitter is unfixable and it is a price argument.

---

## 4. The IMU exists and moves  ·  60 s

Look for a **Motion Module** in the device panel. Turn on **Accel** and **Gyro**.
Rotate the camera gently and watch the values change.

**Walk away if there is no Motion Module at all** — that is a D435, not a D435i,
and the IMU is the whole reason for the premium. This is also the most common
way to be sold the wrong model in good faith.

---

## 5. Depth on a flat wall  ·  90 s

Turn on **Depth**. Stand about 1 m from a flat, textured wall, filling the frame.

* The depth image should be smooth and largely uniform, not blotchy, and without
  a large hole in the middle.
* **Hover the cursor over the centre** — the Viewer prints the distance in
  metres. It should be plausible for where you are standing. You are not
  measuring anything; you are checking it does not say 0.3 m or 4 m when you are
  clearly at arm's length plus a bit. That catches gross bias with no tape.
* Check the **FPS readout** on the stream. Near 30 is right. Much lower means
  cable, port or hub — go back to §0.

---

## 6. On-Chip Calibration health score  ·  60 s  ← the only real quality number

Still on the flat wall: **More → On-Chip Calibration**. It runs in about 15 s
and prints a **health score**.

* **Lower is better, closer to zero.** Around 0.25 is quoted as a good result.
  These are guidelines, not thresholds.
* You can read the score **without burning the result to flash** — the Viewer
  asks before writing. On a camera you have not bought yet, read it and decline.

A poor score is *not* by itself a reason to walk: calibration drift is the
recoverable fault and this is the tool that recovers it. But a score that stays
poor after re-running is a sign something is mechanically off, and it is a price
argument either way.

---

## 7. Wiggle the connector  ·  20 s  ← highest value per second

While a stream is running, gently flex the cable **at the camera end** and move
the plug side to side. The stream must not drop or glitch.

**Walk away if it does.** The USB-C connector is not repairable and it is the
part most likely to have been abused.

---

## 8. Eyes on the body  ·  30 s

Front cover cracks, corner scuffs suggesting a drop, bent frame, missing screws,
sticky residue where it was mounted. Ask what it was used on: a desk beats a
robot or a drone, because **shock, vibration and temperature cycling are the
named causes of calibration drift**.

---

## Walk-away list, condensed

1. Does not enumerate on a known-good cable
2. An IR stream missing or damaged
3. No Motion Module (it is a D435, not a D435i)
4. Stream drops when the connector is flexed
5. Visible impact damage to the optical windows

Everything else — bias, bow, noise, firmware, even a dead emitter — is either
fixable, measurable later, or a price argument.

## Afterwards, at home, inside the return window

The live test rules out the unfixable. The numbers we actually need come after:

```
<py|python3> d435i_probe.py --range <tape-measured distance> --preset sweep
```

which gives sigma_d, the flatness (calibration) figure, the emitter-off case and
the honest Z_max. See `onboard/tools/d435i_probe.py`.

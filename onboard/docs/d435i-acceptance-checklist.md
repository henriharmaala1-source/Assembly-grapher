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
* **Bring your own cable, and know it is USB 3 before you go.** A USB 2.0 cable
  with a C connector plugs in happily and negotiates USB 2; the camera then
  refuses the higher modes and drops frames, which looks exactly like a bad
  camera. Blue port ≠ USB 3 cable — the colour convention is advisory and says
  nothing about what you plugged in. How to check a cable with no camera:

  **If it has a USB-A end: count the contacts.** This is definitive, free and
  instant. Look into the rectangular plug at the gold contacts on the tongue:

  | contacts | verdict |
  |---|---|
  | 4, in a row at the front edge | **USB 2.0.** Will not do |
  | 9 — the same 4, plus 5 more set further back | **USB 3.** Good |

  The extra five are the SuperSpeed pairs; a USB 2 cable physically does not
  have the wires. The tongue is usually blue as well, but count rather than
  trust the colour.

  **If it is C-to-C there is no visual test** — both ends look identical either
  way. You need either a printed marking (`SS`, the SuperSpeed trident, "5Gbps",
  "USB 3.2 Gen 1") or a live test with a device you already own: plug a USB 3
  flash drive or external SSD through the candidate cable and copy a large file
  **off** it. A plateau around 35–40 MB/s is a USB 2 link; USB 3 goes well past
  100 MB/s. Windows also pops "this device can perform faster" on a USB 2 link.

  **The trap worth naming: charging wattage tells you nothing about data rate.**
  A 100 W USB-C power-delivery cable is very often USB 2.0 for data. High
  wattage and high bandwidth are separate conductors and separate claims.

  **Prefer C-to-A into a blue port over C-to-C**, precisely because you can
  physically verify the A end by eye.

* **Keep it to 1 m.** Long or cheap USB 3 cables cause frame drops on this
  camera; the boxed unit shipped with a 1 m cable for that reason. If in doubt,
  a cable explicitly rated "USB 3.2 Gen 1 / 5 Gbps" costs about EUR 10 and
  removes a variable from an acceptance test where that variable would
  masquerade as a bad camera.
* **Straight into the laptop, no hub.** A hub shares bandwidth and adds a
  suspect.

### No USB 3 cable in time? Go anyway.

**Every walk-away criterion in this checklist works on a USB 2 link.** They are
all unfixable-hardware checks, and none of them needs bandwidth: both IR streams,
the emitter toggle, the Motion Module, depth on a wall, the connector wiggle. USB
2 costs you RESOLUTION and FRAME RATE, and those feed the quality grading that is
deferred to home regardless.

The one thing that matters is knowing which link you are on. Read the USB Type
Descriptor first and expect `2.1`; then **skip the FPS check in §5 and do not
judge image quality**, because a degraded link looks exactly like a failing
camera and that confusion is the entire trap.

If the seller has a cable too, try both — if theirs reads `3.2` you have learned
the camera negotiates USB 3 properly, which is worth knowing.

### What is NOT evidence of a USB 3 cable

* **Mouse polling rate.** 8000 Hz is the USB 2.0 **High Speed** ceiling —
  125 µs microframes, so 1/125 µs = 8 kHz exactly. That is where 8K mice come
  from; Full Speed's 1 ms frames cap at 1 kHz, which is why 1K was the old
  standard. An 8K mouse proves High Speed, i.e. USB **2.0**. If anything it is
  evidence against: mouse cables are deliberately thin and flexible, and the
  SuperSpeed pairs are extra shielded conductors nobody puts in a mouse cable.
* **Charging wattage.** See above — separate conductors, separate claims.
* **Port or connector colour.** Advisory, and about the port, not the cable.
* **A USB-C connector.** USB-C says nothing at all about data rate.

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

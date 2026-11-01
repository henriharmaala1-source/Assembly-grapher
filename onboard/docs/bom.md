# Bill of materials — the physical drone platform

This is the actual hardware kestrel runs on, sourced from the build's own
parts list ([`AI-companion-project`](https://github.com/henriharmaala1-source/AI-companion-project)).
Read [`introduction.md`](introduction.md) first for the reasoning; this
document is the concrete parts and prices that reasoning produced.

Prices are approximate (EUR, 2026), vary by vendor, and are for orientation,
not a purchase order.

---

## The whole aircraft

kestrel bolts onto an ordinary analog FPV freestyle/long-range quad — this
is that quad's actual parts list, not a hypothetical one:

| Component | Model | ~€ |
|---|---|---|
| Frame | GEPRC GEP-MK5 7" | 40 |
| Flight controller | Matek H743-SLIM V4 | 40 |
| ESC (4-in-1) | SpeedyBee BLS 55A | 35 |
| Motors (×4) | BrotherHobby R5 2207 1750KV | 60 |
| Propellers | HQProp 7×4×3 V1S | 10 |
| Video transmitter | SpeedyBee TX500 | 20 |
| FPV camera | Caddx Ratel 2 Micro | 20 |
| Receiver | RadioMaster RP2 (ELRS 2.4 GHz) | 9 |
| GPS | Matek M10-L-4.11 | 25 |
| Battery (×2) | CNHL 6S 3000 mAh 60C | 90 |
| VTX antenna | Foxeer Lollipop 4 (RHCP) | 8 |
| Capacitor | 50 V 1000 µF low-ESR | 3 |
| BEC | Matek UBEC DUO | 15 |
| Misc. | XT60, wiring, standoffs | 10 |
| **Airframe subtotal** | | **~385** |

This is a normal 7" analog freestyle/long-range build — nothing about it is
special-cased for kestrel. That's deliberate: see `introduction.md`'s
"Talking to the flight controller" section for why the whole design targets
an aircraft that already flies completely on its own.

**Why no magnetometer on the FC is worth knowing:** the H743-SLIM V4 has no
onboard compass, so GPS-denied heading would drift on gyro integration alone
without the external GPS module supplying one — which is exactly why GPS
here is a full GPS+compass unit on a mast, not just a GPS chip.

## The companion-computer add-on

This is the part that's actually kestrel-specific — everything above would
exist on this quad with or without a Raspberry Pi on it.

| Component | Role | ~€ |
|---|---|---|
| Raspberry Pi 5 (4 GB) | Runs kestrel: perception, state estimation, the mode arbiter | 80 |
| USB CVBS capture dongle | Digitizes the analog video for the Pi to process | 8 |
| USB-C breakout + 5.1 kΩ CC resistors | Lets the Pi accept power from the flight battery's regulated rail without a proper USB-PD source | 3 |
| **Companion-computer subtotal** | | **~91** |

**Total, excluding transmitter, chargers, and goggles: ~€476.**

### Forward ToF sensor (committed — the obstacle sensor)

| Component | Role | Connects to | ~€ |
|---|---|---|---|
| Forward time-of-flight sensor (VL53L5CX 8×8 / VL53L9 higher-res, via `ITofSource`) | Metric forward depth → the obstacle-avoidance corridor and the P5b occupancy grid | **Pi** I²C | ~18–45 |

This is now part of the build, not a hypothetical upgrade. It matters for two
reasons the monocular camera can't cover:

1. **It's metric.** Monocular depth has no absolute scale, so the occupancy
   grid built from it is only approximate. The ToF sensor measures true
   distances, which makes the grid geometrically sound.
2. **It's forward-fixed, so camera tilt doesn't affect it.** The FPV camera is
   mounted at an up-tilt (≥20°) so the horizon centres during fast forward
   flight — which means at a hover it points *up* at the sky, exactly when the
   move-stop-sense cycle wants to sense forward obstacles. A ToF module bolted
   to the airframe pointing forward is immune to that: it sees the forward
   obstacle field whether the drone is hovering or moving.

The software already has the plumbing (`ITofSource` → `TofNavigateModule` →
the same VFH+ corridor and the P5b grid); adding the sensor is wiring, not a
redesign. The monocular depth path remains as a supplementary/fallback source,
with a camera-tilt mitigation (`camera.mount_tilt_deg` + grid-scan
pitch-gating) for when it's used alone.

All time-of-flight sensors lose range in bright sunlight — verify the real
daylight range in your environment; for outdoor flight a stereo module
(OAK-D Lite, USB) is the sun-proof alternative through the same interface.

### The one detail that matters most: there's no dedicated camera

The Pi doesn't have its own camera. The "USB CVBS capture dongle" line above
is a small adapter that reads the same **analog composite video signal**
already running from the FPV camera to the video transmitter, and turns it
into a USB video device the Pi can open like any webcam. The pilot's own
signal path — camera → VTX → air → goggles — is untouched and stays purely
analog; the Pi is a passive second listener on that signal, not something
sitting inline in it, so the AI side adds zero latency to what the pilot
sees flying.

This is the concrete answer to "how do you use an analog FPV feed for
autonomy": you don't digitize the whole aircraft, you tap the one signal
that already exists with an €8 part.

### Power is the tight margin, and it's worth flagging

The build notes call this out directly, and it's worth repeating rather
than glossing over: the Matek UBEC DUO regulator's maximum input is 26 V,
and a fully charged 6S battery sits at 25.2 V — a genuinely tight margin,
not a comfortable one. Anyone replicating this build should treat that
number as a real constraint, not a rounding error.

---

## Optional additional sensor

- **Matek 3901-L0X** (PMW3901 optical flow + short-range ToF, ~€28, into the
  FC) — gives iNAV GPS-denied *position hold* natively and a vertical-scale
  anchor. Not committed, but a low-effort add through the FC (not the Pi); it
  complements the forward ToF, which handles obstacle avoidance, not hold.

## What's not in this BOM at all

- **No LiDAR** — cost; a usable one is multiples of this entire build. The
  forward ToF sensor covers short-range obstacle depth far more cheaply.
- **No stereo camera pair** (indoor) — the forward ToF is the metric depth
  source; stereo (OAK-D Lite) is the sun-proof option for outdoor flight,
  through the same `ITofSource` interface.
- **No dedicated Pi-side IMU** — the FC's own IMU is read over MSP telemetry
  instead. The planned path to better GPS-denied velocity (`ROADMAP.md`,
  P5a) is camera-based optical flow using the existing video tap and the
  FC's attitude, not new IMU hardware.

Every one of these is the same trade-off stated in `introduction.md`: cheap
and FPV-weight, chosen deliberately, over maximally capable.

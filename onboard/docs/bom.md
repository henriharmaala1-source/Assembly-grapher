# Bill of materials — the physical drone platform

This is the hardware kestrel actually runs on: what's assumed to already
exist, what the companion-computer add-on stack costs, and — just as
importantly — **what this list deliberately does not include.** Read
[`introduction.md`](introduction.md) first for the reasoning; this document
is the concrete list that reasoning produced.

Prices are approximate (EUR, 2026), vary by vendor, and are for orientation,
not a purchase order.

---

## Scope: this is an add-on, not a full drone kit

kestrel is designed to bolt onto an analog FPV quad that **already flies
fine on its own** — that's the whole point of talking to the flight
controller over MSP rather than replacing it (see the introduction's
"Talking to the flight controller" section). So this BOM has two parts:

1. **The assumed platform** — the flight-controller-and-below stack this
   project made specific choices around, because those choices affect what
   the software can do (e.g. which sensors it can read, how it talks to the
   FC).
2. **The companion-computer stack kestrel actually adds** — the compute and
   sensors bought specifically for this project.

**What's deliberately *not* specified here: the airframe, motors,
propellers, VTX, FPV camera, and battery.** Those are ordinary FPV build
choices (frame size, freestyle vs. cinewhoop, 4S vs. 6S, analog vs. digital
video) that this project intentionally stays agnostic to — kestrel doesn't
care what frame it's bolted to, only that iNAV is flying it. If you're
building from scratch, pick those the same way you would for any FPV build
that doesn't have a companion computer at all.

---

## 1. The assumed platform (flight-controller side)

These aren't "recommendations" in the sense of "you should buy these" — they're
what the project's iNAV integration, telemetry parsing, and sensor-routing
decisions were actually made against. Swapping them is possible but means
re-verifying the assumptions documented in `AGENTS.md`.

| Component | Part | Role | Notes |
|---|---|---|---|
| Flight controller | **Matek H743-SLIM V4** | Runs iNAV; the thing kestrel talks to over MSP | STM32H743 @ 480 MHz, dual ICM-42688-P IMUs, Infineon DPS368 barometer, **no onboard magnetometer**, 7 UART / 2 I²C / 1 CAN — plenty of headroom to add the Pi + GPS + sensors below without contention |
| ESC | **SpeedyBee BLS 60A (WSCU)** | Motor control + battery current sensing | BLHeli_S — **no digital ESC telemetry** (no RPM/combined data stream). Has a built-in *analog* current sensor pad (`Scale=400, Offset=0` in iNAV's power config); battery voltage comes separately, straight off the FC's own VBAT sense pin. Both are effectively free — no extra sensor needed for battery telemetry, just correct iNAV configuration. |

**Why no magnetometer is a real gap, not a nitpick:** without a compass, GPS-denied
heading drifts (gyro-only integration has no correction). That's the reason a
GPS+compass combo, not just a GPS, is in the add-on stack below.

## 2. The companion-computer stack

### Compute

| Component | Role | ~€ |
|---|---|---|
| **Raspberry Pi 5 (8 GB)** | Runs kestrel: perception, state estimation, the mode arbiter | ~85 |
| Active cooler (official or equivalent) | Not optional — the Pi 5 thermal-throttles under sustained CPU inference load within minutes on passive cooling | ~5–10 |
| Official 27 W USB-C PD supply | An undersized supply causes silent undervoltage throttling, worst-case exactly when inference load spikes | ~10 |

The Pi is CPU-only by design (see `introduction.md`) — no GPU acceleration
for any of the vision models below.

### Position + heading

| Component | Chip | Connects to | ~€ |
|---|---|---|---|
| **GEPRC M10** GPS+compass combo | M10 GNSS + QMC5883L compass | **FC** I²C, mast-mounted | ~25 |

Mounted on a mast, away from motors and power wires — motor interference is
the single most common cause of bad heading on a cheap compass, and matters
more than which specific compass chip is used. This module feeds the FC's
own AHRS; kestrel reads the *result* (fused heading, GPS fix) back over MSP
telemetry rather than talking to the compass itself — see the introduction's
"Where am I?" section for why kestrel doesn't reimplement this.

### Forward depth (feeds kestrel's obstacle-avoidance corridor)

All time-of-flight sensors lose range in direct sunlight — that's physics,
not a product gap — so the right choice depends on where you fly.

**Indoor / near-field:**

| Component | Output | Connects to | ~€ |
|---|---|---|---|
| **VL53L5CX** | 8×8 metric depth grid, 4 m range | **Pi** I²C | ~18 |

Feeds the exact same VFH+ steering pipeline the monocular depth model does
— the algorithm doesn't care whether "how far is that" came from a camera
model or a laser, only that it gets a distance grid. Near-zero CPU cost,
unlike the depth model.

*(A higher-resolution part, the VL53L9CX — 54×42 zones, 9 m range — was
evaluated and would be a strict upgrade, but was out of stock everywhere as
a new part at the time of writing and has no confirmed open-source driver.
The VL53L5CX is the proven fallback with a working, tested backend in this
codebase; swap to the L9 later if it becomes available and worth the driver
risk.)*

**Outdoor / sunlight:**

| Component | Output | Connects to | ~€ |
|---|---|---|---|
| **Luxonis OAK-D Lite** | Stereo depth, on-board VPU | **Pi** USB | ~90 |

Works in direct sunlight, longer range, effectively zero Pi CPU cost (the
VPU does the stereo matching), and can run detection models on its own
compute too. Trade-off: it costs roughly the entire sensor budget by
itself, so an outdoor build skips the flow/AGL module below for now.

### Position hold + downward sensing (feeds the FC directly)

| Component | Sensor | Connects to | ~€ |
|---|---|---|---|
| **Matek 3901-L0X** | PMW3901 optical flow + VL53L0X ToF (~2 m) | **FC** UART/I²C | ~28 |
| *(optional)* Benewake TFmini-S or TF-Luna | Longer-range downward ToF (8–12 m) | **FC** UART | ~22–40 |

The 3901-L0X gives iNAV **GPS-denied position hold natively** — this is the
FC's own nav mode working with a flow+range sensor, not something kestrel
implements. It's also the vertical-scale anchor alongside the barometer. The
optional longer-range module only matters if you fly above the 3901's
~2 m ceiling and want altitude-hold there too; below that height it's
redundant with the 3901's built-in ToF.

---

## Budget summary

| Tier | Contents | Total (sensors only, Pi extra) |
|---|---|---|
| **Tightest** | VL53L5CX + Matek 3901-L0X — forward depth and FC-side position hold, nothing else | ~€45 |
| **Recommended, indoor/near-field** | + TFmini-S/TF-Luna for altitude above 2 m | ~€90–95 |
| **Outdoor** | OAK-D Lite alone (skip the flow module for now) | ~€90 |

Add the Pi 5 + cooling + power supply (~€100–105) on top of any tier — that
cost is fixed regardless of which sensor tier is chosen, since it's the
compute the software actually runs on, not a sensor.

**Total companion-computer add-on cost: roughly €150–200**, on top of
whatever the underlying FPV airframe already cost. That number is the
concrete form of the "cheap, on purpose" constraint described in
`introduction.md`.

---

## What's deliberately excluded, and why

- **No LiDAR.** Cost — a usable spinning or solid-state LiDAR is multiples
  of this entire budget.
- **No purpose-built stereo camera pair.** The OAK-D Lite is the one stereo
  option in this BOM, and only in the outdoor tier; two ordinary cameras plus
  hand-rolled stereo matching would burn CPU budget this project doesn't
  have (see `introduction.md`'s "no GPU" section).
- **No dedicated Pi-side IMU.** The FC's dual ICM-42688-P is a genuinely
  excellent chip, but the Pi can only reach it through MSP telemetry at
  ~30–50 Hz fused, not raw and time-synced to the camera — which is what
  tight visual-inertial odometry actually needs. A Pi-local IMU would be the
  fix, but isn't bought here; it's a documented future option if the
  project moves to real VIO (see `ROADMAP.md`, P5a).
- **No airframe, motors, propellers, VTX, FPV camera, or battery.** Not an
  oversight — see "Scope" above. This is intentionally the smallest add-on
  that makes an already-flying FPV drone sense its surroundings, not a
  from-scratch drone kit.

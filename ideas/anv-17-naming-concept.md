# ANV-17 — naming concept (shelved)

*Saved for later, not part of the active project documentation. This was
drafted for `PROJECT.md` and pulled back out — the project is a concept and
a working demonstrator right now, not a product, and a naming story like
this is premature for what exists today. Keeping it here rather than
discarding it, in case it's worth revisiting once (if) that changes.*

---

## Prologue

Early February 2026, a reconnaissance variant of the Russian Geran/Shahed
strike-drone line was recovered and taken apart. Open-source technical
breakdowns of the wreckage found a **Raspberry Pi 5** on board, running video
processing, alongside a Windows mini-PC handling other functions. Not a
custom board built to a military spec. A single-board computer anyone can
order for a few hundred euros, doing onboard video AI on a live combat
airframe.

That's not where this project's hardware bet came from — the hypothesis this
project was already built on, that cheap, CPU-only, general-purpose compute
is enough for real onboard perception rather than a compromise to be
replaced once better hardware is affordable, predates that find. What the
Geran wreckage did was **confirm** it: independent, real-world evidence,
from an active combat example, that the exact hardware-scope bet this
project holds to is one a live war has separately converged on too.

It isn't isolated. On the other side of the same conflict, Ukraine's
**Hornet** — a fixed-wing strike drone built by the American firm Perennial
Autonomy, in service since March 2026 and used to cut Russian logistics 50–150
km behind the front — navigates primarily by **optical flow**: a downward
camera and a processor tracking terrain motion beneath it, GPS reduced to an
occasional cross-check rather than the primary reference, specifically
because satellite navigation is the first thing a contested electromagnetic
environment takes away. It costs roughly **$5,000** a unit. And on the
Russian side, the production Geran-2 MS variant now carries an **Nvidia
Jetson Orin** module for onboard AI object recognition, correcting its own
terminal-phase trajectory against a moving target from its own video feed —
manufactured at a reported 5,000+ units a month.

A fourth data point: since 2025, a commercial category of **AI lock-on
add-on modules for FPV airframes** has gone from prototype to mass-fielded —
**TFL-1**, from the firm The Fourth Law, chief among them, reportedly around
**$442** a unit. Strapped onto an otherwise ordinary FPV, it takes over
final-approach guidance: detecting, tracking, and closing on a target
independent of the radio link. One Ukrainian brigade using it reported hit
rate going from 20% to 80%. This isn't a research paper or a demo — it's a
live commercial market, at a price point in the same neighborhood as this
project's own BOM.

Four data points, not three, and not confined to one side of the war: when
the radio link and the satellite fix are the first casualties of contact,
the airframe has to be able to see and decide for itself, on hardware cheap
enough to be attritable — and, increasingly, cheap enough to be sold as a
module rather than built once per program. That isn't a future requirement
this project is planning for — it's the ground this project already stood
on, and the Geran wreckage, Hornet, and the TFL-1 category are each
confirmation of it from a real, active-combat or active-market example, not
the source of it. This project runs the same underlying bet at a fraction of
even Hornet's budget, with a scope boundary this document states plainly and
holds to (see *ANV-17* and *Design decisions*, below).

---

## ANV-17

*Product name for the system this document describes end to end; internally, the
engineering codename used throughout is* kestrel*.*

### The acronym: Assess — Navigate — Vanquish

- **Assess** — perception. Resolve what's actually present from a single
  analog composite feed — openness, range, a tracked subject — before any
  decision is made about it. *(See Perception, below.)*
- **Navigate** — the deliberation layer. A mission controller and a rolling
  occupancy-grid planner turn what's been assessed into a committed goal
  direction, with live sensor data still able to override it in real time.
  *(See Autonomous motion, below.)*
- **Vanquish** — the target line, and the direction the name commits to.
  Assess finds it, Navigate closes on it, Vanquish is what comes next: not
  passive observation, but something that acts decisively on what it finds.
  What that action becomes is future scope — the name points at a
  destination without specifying, claiming, or ruling out the shape of it
  here. That's deliberate.

### Why 17

Rune 17 of the Kalevala is the canto where the hero Väinämöinen — unable to
finish a boat without three lost words no living being holds — finds them
inside Antero Vipunen: an earth-giant so old the world has stopped
recognizing him as anything but a hillside, trees rooted in his shoulders.
Väinämöinen forces him awake, is swallowed whole, and from inside the dark
hammers at him until the giant has no choice left but to yield. He is not
killed. He is found, closed on, and vanquished — forced into submission,
made to surrender everything he holds — and from that flood, Väinämöinen
takes the exact three words he came for, and finishes the boat.

**ANV** is that name folded to its own initials — **AN**tero **V**ipunen —
the same operation this system performs on a video signal: reduce an
overwhelming, continuous stream to the exact few facts a mission needs.
The giant is the target. He is assessed, navigated to, and vanquished — not
destroyed, but made to yield — which is the oldest possible description of
what this product's name points toward.

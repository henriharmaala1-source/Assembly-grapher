# Project CV — what is claimable, and what it takes to claim it

The other documents describe the work. This one describes the *position*: what
role was actually played, what can be said about it honestly, and the specific
missing artifacts that turn an arguable claim into an unassailable one.

| file | is |
|---|---|
| `THESIS.md` | what the project claims technically, and what would prove it |
| `NOTES.md` | the lab notebook — tried, measured, rejected |
| `PROJECT.md` | the public technical write-up |
| **this** | the professional artifact: role, claims, and the gaps in the record |

Written 2026-08-12. Motivation is stated plainly in `THESIS.md` §1.2: this is a
portfolio piece and a plausible route to funding, in the way an expensive
demonstration is for a funded lab.

---

## 1. The role, named accurately

**Systems integration + technical architecture + research direction.**

Not project management — no schedule, budget, team or stakeholders to
coordinate. Not software implementation of the perception and planning code.

Reviewing work you did not write, and finding defects in it, is **engineering,
not management**. A principal engineer who catches a rendering bug in someone
else's code has done engineering. That is a large fraction of what this role has
consisted of.

## 2. Claimable

* **Thesis, scope and the constraint framing.** Autonomy in a GNSS-denied and
  video-denied zone, on cheap CPU-only compute — three chosen constraints, two of
  them from a single operational scenario, and none of them budget accidents. See
  `THESIS.md` §1.0. The GNSS-denial choice is the one that makes the architecture
  coherent rather than a set of workarounds, and it is also the capability the
  market actually wants.
* **The vehicle.** Part selection, airframe build, and the integration that makes
  it one system — including LiPo → Pi 5 over USB-C PD, which is not a lesser
  layer: the Pi 5 will not grant full performance without a source that actually
  *negotiates* 5 V/5 A, and from 6S that means ~22 V down to 5 V at 25 W on a bus
  shared with ESCs switching tens of amps. Inrush, ripple, ground loops, pack sag
  browning out the companion computer on a throttle punch, the D435i's own bursty
  draw, vibration isolation, and thermal on a board that throttles. **Integration
  is the definitional core of systems engineering, and this is where most hobby
  autonomy attempts die.**
* **Architecture decisions and trade studies.** Sensor vs achievable speed,
  memory vs drift, IMU vs visual odometry, awareness vs permission.
* **Defect discovery by artifact review.** Repeatedly the mechanism that caught
  real faults — the ladder rendering, a degenerate first-person path projection
  that had survived every previous look, an overclaim about the IR emitter.
* **The epistemic standard.** Negative results kept with their numbers, instruments
  checked before the experiments that use them, plans written for components
  deliberately not built. `THESIS.md` §1.1 argues this is the more defensible
  differentiator of the two.

## 3. Not claimable — say so first, before anyone asks

* **Implementation of the perception and planning code.** AI-assisted. Git history
  makes this trivially checkable, so honesty is strictly the better play; in 2026
  AI-assisted is not a weak claim, and being caught pretending otherwise is fatal.
* **Novelty of the ideas.** Scan matching, submapping, local SLAM, VFH, canopy gap
  fraction — all predate this project. The claim that is *true and stronger*:
  **independently derived from constraints, then located in the literature.**
  Verifiable from the notes' timestamps, and more credible to a knowledgeable
  reader than a novelty claim, which in this area is usually false and always
  checked.

## 4. TODO — the four systems-engineering artifacts

A weekend of **writing**, not building. Each is cheap, and each is what an
interviewer or a technical reviewer will probe. Someone who says "systems
engineering" and produces a power budget and a verification matrix is instantly
credible; someone who cannot, is not.

- [ ] **Requirements baseline.** There was no definition of done until
      `THESIS.md` §4, and there is still no requirement *set*: speed, endurance,
      environment, lighting, failure tolerance, recovery behaviour. The
      characteristic SE artifact is requirement → design → test traceability.
- [ ] **Budgets, with allocations and reserve held at system level.** Mass, power,
      thermal, latency, compute. **The power engineering is done; the budget is
      not written.** Latency is the one with a live technical consequence —
      `reactS = 0.25 s` is currently assumed exactly like σ_d, and the true chain
      is exposure → USB → the D435i's own depth ASIC → us → MAVLink → FC loop.
- [ ] **Interface control.** `grep VoxelMap onboard/` returns nothing: `nav-sim`
      and `onboard` share zero code. That is an interface failure, and preventing
      exactly that is the job. It is also `THESIS.md` P3, a flight blocker.
- [ ] **The GNSS-exclusion verification**, as a standing pre-flight item:
      `EK3_SRC*` logged per flight, plus the connected-vs-unplugged controlled
      pair. A silently-fusing EKF voids the central claim while looking like
      success — see `THESIS.md` §1.0.
- [ ] **V&V matrix.** Good ad-hoc tests exist (8 ctest targets, paired A/Bs,
      golden-frame MAVLink checks). What is missing is the mapping from each
      requirement to how it will be demonstrated.

## 5. TODO — the hardware record

- [ ] **`docs/HARDWARE.md`.** Parts list, the power chain and *why that chain*,
      what failed first, measured rail behaviour under throttle, thermal
      behaviour, vibration mounting.

**Right now the hardest integration work in this project is invisible in the
artifact.** By this project's own rule — measured or it did not happen —
undocumented work does not count, and for a portfolio or funding conversation the
bring-up story is compelling and currently missing entirely.

- [ ] **Run the `THESIS.md` P1 benchmark under flight power and flight thermals**,
      not on a desk with a wall adapter. If the board throttles in an airstream or
      the rail sags on a punch-out, the bench number is fiction — and the bench
      number is the headline experiment of the whole thesis.

## 6. TODO — demo assets

The deliverable is **the craft flying**, not a repository. But note the trap:

> A video of a drone not hitting trees is weak evidence on its own. Every viewer
> has seen DJI do it since 2016, a twenty-second clip cannot distinguish autonomy
> from a good pilot, and **the thing that makes this one interesting — €80 of
> compute, no GPU — is completely invisible in the footage.**

So the demo has to show the *constraint*, not just the flight.

- [ ] **Four-pane overlay as picture-in-picture** beside the flight footage:
      depth, voxel ladder, chase view with the rollout fan, command arrow. With
      **`Raspberry Pi 5 · N ms/frame · no GPU · GPS: 14 sats, not used for nav`**
      burned into the corner — both chosen constraints, both invisible in footage
      unless you put them there, and the second is the operationally interesting
      one. Note the wording: *having* a fix and declining to use it is a far
      stronger claim than not having one, which could just mean flying under
      canopy. For a v2
      fly-through-blind run, add the video-link state. The
      visualisation work was built for debugging and it is the demo asset —
      nobody else's video has it, because nobody else's constraint is interesting.
- [ ] **Onboard telemetry doubles as overlay data.** `THESIS.md` §4 already
      requires per-frame integrate/plan/total and valid-pixel fraction for
      verification; it is the same stream.
- [ ] **Keep the raw `.kdr` of every demo run.** Video gets attention; the repo
      survives the technical scrutiny that follows it. Both are needed and the
      harder one already exists.

## 7. TODO — personal

- [ ] **Read this repository closely — the derivations are in it.** The real
      exposure is the gap between "I can direct and evaluate this system" and "I
      can build it," and someone will probe it: why `carveSigK` exists, derive
      `Z_max`, what `recentre` does at the boundary. The mitigation is unusually
      available here, because the code comments and `NOTES.md` were written to be
      read. Unglamorous work that converts exposure into competence.

Two things are true at once and both should be said out loud: this is a capable
systems-integration and architecture role, and the software implementation layer
is directed rather than written. The second is the more fixable of the two.

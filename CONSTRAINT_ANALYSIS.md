# Decisions derived from constraint analysis

A provenance register. `ARCHITECTURE.md` §6 lists *what* was decided; this lists
**which decisions began as your own reading of a constraint**, and what the
constraint was.

The distinction matters because these are not preferences. Each one starts from a
physical, computational or epistemic limit that was identified first, and the
design follows from the limit rather than from taste. Where a decision was later
confirmed by measurement, the number is given — but the number came second.

**Method, visible across every entry below.** Look at the actual output → name
the constraint that explains it → derive the decision → *then* measure. Six map
and instrument defects in this project were found this way. **None was found by a
test.** That is the register's most important single fact, and it is why the
"picture is not a test" rule now cuts both ways: tests do not catch these, and
pictures do not confirm them either.

---

## 1. Sensor physics

### 1.1 Ray casting is right near and wrong far

> *"Is ray casting even the right method? The heat map clearly shows a wall — a
> CV model could see it better than the cast."*

**Constraint identified.** A ray cast spends its budget on range, and range is
exactly what stereo loses first. The heat map showed a wall the voxel cast was
failing to represent, which meant the *information was present in the sensor* and
being discarded by the representation, not by the physics.

**The constraint, quantified afterwards.** Stereo error is anisotropic and the
anisotropy grows with range: along the ray `δZ = Z²σ/(fB)`, across it `Z/f`,
ratio exactly `Z·σ/B` — 10:1 at 2 m, 50:1 at 10 m, **100:1 at 20 m**. A cube must
be sized for the worse of the two, so a voxel honest in range at 20 m is 8 m wide
and throws away the 4.5 cm of lateral detail the sensor still resolves.

**Decision.** Retire the coarse voxel rung. Represent the far field in **bearing
space**. Measured: eleven independent bearings across 87° from the 1.0 m voxel
rung, against 848 from the depth image, for less time.

### 1.2 The near/far split, stated as a requirement

> *"Yes — we want voxels near, and for the far a rough representation of the
> depth image."*

**Constraint identified.** The two fields contain different *kinds* of
information, so they need different representations — not one representation
tuned differently.

**Decision.** The architecture's central split. Near: 3-state voxels, 0.25 m,
honest to `Z_max = 3.54 m`, carrying free space and volume. Far: a bearing field,
360 × 48 at 1° bins, carrying nearest-return-per-bearing and nothing else. This
is now the top-level structure of the whole perception stack.

### 1.3 "Can the 3.5 m go further?" — and the answer that mattered

> *"Could the 3.5 m range be made further? With two voxel levels?"*
>
> *"Seeing further away: it might not help currently with the path planning we
> have. But gathering more information, if we can manage it, WILL be useful —
> when we tune for it."*

**Constraint identified, and this one was a correction of me.** I had collapsed
two separate decisions into one and concluded "more map makes it worse". You
separated them: **gathering information and spending it are different
decisions**, made at different times, with different costs.

**Decision.** Extended range is pursued as *awareness*, decoupled from planner
authority. The far field may inform a goal; it may never grant permission. This
became the load-bearing safety property of the entire far-field design.

### 1.4 Which gaps are real

> *"No — the far-field white gaps are OK and supposed to be there! I mean the
> gaps just near us in the voxels."*

**Constraint identified.** Two visually identical artefacts with different
causes: far-field gaps are the honest absence of measurement; near-field gaps are
the frustum. One is correct behaviour and one is geometry.

**Decision.** Do not "fix" the far field. The near blind disc (radius ≈ 2 ×
altitude — 4.7 m at 2.5 m AGL) is logged as frustum geometry to be solved with
memory or a downward rangefinder, not with map changes.

---

## 2. Representation and authority

### 2.1 The nothing-signal must be separable from the something-signal

> *"In your screenshots even the empty sky shows up. Can we get the nothing
> signal filtered out? So silhouettes of found depth?"*

**Constraint identified.** A count of valid returns cannot distinguish "four good
pixels out of four" from "four good pixels out of four hundred". Empty sky is
precisely the second case — thousands of no-returns plus a handful of spurious
matches on cloud edge — so a bin claiming a surface from it grows a **ceiling
that is not there**.

**Decision.** `minFillFrac` — a bin must also be *backed*: of the pixels pointing
into it, a fraction must have returned. Sky is a few per cent and fails; foliage
is a third and passes; a wall is nearly all of it. This turned the pane from a
drawing of the map into a **silhouette of found depth**, which is what was asked
for. The fill value was then kept rather than discarded, because a bin at 0.9 is
a wall and one at 0.3 is an edge — and that boundary is measured, not drawn.

### 2.2 No-return regions are coherent, not random

> *"But I don't see random borders — it's always depth segments, even if vague."*

**Constraint identified, and this corrected me directly.** I had claimed depth
cannot distinguish silhouettes. You had looked and seen that dropout regions are
*structured*: the matcher loses the interior of something whose rim it resolved.
A hole surrounded by agreeing neighbours is therefore a hole **in a surface**,
not an edge.

**Decision.** Neighbour consensus, in **bearing space only**. And the reason it
is safe there is the asymmetry you had already established in 1.3: filling a
pixel in the depth image creates an OCCUPIED cell, and `sphereClear` hard-rejects
OCCUPIED anywhere in the robot ball, so every invented cell deletes a manoeuvre.
In bearing space the layer has no authority, so an invention costs a score and
nothing else.

> **Inventing awareness is cheap; inventing permission is not.**

That sentence is the project's clearest single design rule and it is a direct
consequence of this pair of observations.

### 2.3 Is the far field good enough to plan on?

> *"The non-voxel seems to work decently close / semi-close too. Is it viable for
> path planning?"*

**Constraint identified.** Whether a representation may be trusted is a separate
question from whether it looks good — and it has to be answered before it is
wired to anything.

**Decision.** Measured rather than asserted: the bearing field **can veto but
cannot grant**. It has no free space and no volume, so it cannot answer "is this
robot-sized tube clear", and it cannot represent two surfaces along one bearing.
Result: it feeds goal selection and `OBSTACLE_DISTANCE`, never `sphereClear`.

---

## 3. Compute budget

### 3.1 Is the visualisation on the critical path?

> *"I'm worried about the performance on Pi. Can it be improved? No visualization
> improves?"*

**Constraint identified.** The right first question about a compute budget is not
"how do we make it faster" but **"what is actually being counted"**.

**Decision / finding.** Visualisation was already excluded and always had been:
`view render 34.45 ms/frame, desktop only` against `ONBOARD TOTAL 19.46 ms/frame`.
The render costs nearly twice the flight budget and the aircraft never pays it.
The question closed the largest apparent lever by showing it was not a lever —
which is worth more than a speed-up, because it stopped work that would have
bought nothing.

**Downstream, from the same prompt.** Dropping the 0.10 m near rung is −37 % of
frame time **and improves** near coverage (100 % vs 76.3 % at 2 m standoff). A
pure win, found only because the budget was questioned.

---

## 4. Method and epistemics

These are the constraints on *how the work is allowed to proceed*, and they have
changed the project's output quality more than any single algorithm.

### 4.1 A new representation is a comparable, not a proposal

> *"Do it as a comparable."*

**Constraint identified.** A new idea evaluated on its own terms will always look
good. The only honest test holds everything else fixed.

**Decision.** The bearing field was built as an instrument first — same
projection, same height colour key, same haze as `VoxelMap::fpvImageWH`, banded
identically — so that the *only* difference between the two panes is the
representation. This rule then caught three of my own instrument faults, because
a flattering comparison is detectable when the harness is symmetric. It is now
the standing rule for any far-field replacement, including learned ones.

### 4.2 Do not fix what works

> *"OK, you told me already — no need to do anything. Just don't fix the far
> field because it works!"*

**Constraint identified.** Explanation and action are separate. Understanding why
something behaves as it does is not a licence to change it, and a working
subsystem is an asset to be protected from improvement.

**Decision.** Scope discipline. The far field has been untouched since, through
four subsequent rounds of analysis.

### 4.3 Source is not behaviour

> *"The depth model is close field only, by my tests on mobile."*
>
> *"The repo isn't up to date in everything."*

**Constraint identified, and this was the sharpest correction in the session.** I
had read `onboard/DepthNav` and reported that a monocular far field works. It
does not — you had tested it — and the code I cited was in the stale half of the
repo besides.

**Decision, two of them.**

1. **A freshness map is now maintained** (`nav-sim/` fresh, `desktop/tracker/`
   fresh, `onboard/` stale), and any claim must say whether it is *(code)* or
   *(measured)*. Reading a file establishes what someone wrote, never what
   worked.
2. **The learned far field is parked.** Your result has a mechanism behind it,
   found afterwards: MiDaS and Depth Anything emit **inverse** depth, so
   resolution per metre falls as 1/Z² and the whole far field occupies a few per
   cent of the output range; then both implementations **min-max normalise per
   frame**, so any near object rescales the image and crushes what survived.
   Your one-line field report was worth more than the entire code review that
   preceded it.

**And it invalidated a proposal of mine on the same day.** The "anchor monocular
scale to the metric near map" idea fails on the same Z²: `Z = 1/(a·d+b)` gives
`dZ/db = −Z²`, so an anchor fitted at 3 m and extrapolated to 20 m amplifies its
own fit error as the square of the range. **The near map cannot lend precision it
does not have at that distance.**

---

## 5. Summary

| # | constraint identified | decision that followed |
|---|---|---|
| 1.1 | stereo anisotropy grows as `Z·σ/B` — far cubes are dishonest | far field in bearing space; coarse voxel rung retired |
| 1.2 | near and far carry different kinds of information | voxels near / bearings far — the top-level architecture |
| 1.3 | gathering information ≠ spending it | far field advises, never grants permission |
| 1.4 | far gaps are honest absence; near gaps are the frustum | leave the far field alone; blind disc is a geometry problem |
| 2.1 | a count cannot separate "4 of 4" from "4 of 400" | fill-fraction gate — silhouettes of found depth |
| 2.2 | dropout regions are coherent segments, not noise | neighbour consensus, in bearing space only |
| 2.3 | looking good ≠ being trustworthy | measured: can veto, cannot grant |
| 3.1 | ask what is counted before asking what is slow | visualisation already excluded; near rung retired, −37 % |
| 4.1 | an idea judged on its own terms always wins | new representations ship as comparables |
| 4.2 | explaining is not a licence to change | working subsystems are protected |
| 4.3 | source records what was written, not what worked | freshness map; learned far field parked |

**Eleven decisions.** Four came from correcting a conclusion I had already
written down and defended. The load-bearing ones — the near/far split, the
authority asymmetry, and the comparable-not-proposal rule — are all in that
group.

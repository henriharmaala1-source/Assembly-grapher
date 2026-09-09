# Mission language → object search, with a small quantized LLM

*(idea + experiment plan, not built, not scheduled)*

*From a design conversation extending `ideas/context-gated-perception.md` and
`ideas/learned-decision-making.md`. Written down as a concrete experiment
plan, not a design proposal — nothing here is designed against the real
interfaces (`IPerceptionModule`, `MissionController`, the P3 sidecar
transport) yet. If it graduates, it lands as an extension of `P3` in
`ROADMAP.md`.*

---

## The mission: "look for a house and find a chair"

This decomposes into two unrelated problems that need different tools —
conflating them is the mistake to avoid:

1. **Parse the instruction into an ordered subgoal list** — `[FIND(house),
   THEN FIND(chair)]`. This is language understanding.
2. **Execute each subgoal** — actually search for and reach something called
   "house," then "chair." This is *not* language at all; it's a named,
   well-studied robotics problem.

```
"look for a house and find a chair"
        │
        ▼  bounded grammar → simple slot parser
        ▼  open-ended phrasing → the P3 LLM sidecar, whitelisted schema
[ FIND(house), THEN FIND(chair) ]
        │
        ▼  plain FSM: subgoal index, advance on detection (no ML)
SEARCH(house) → detected → SEARCH(chair) → detected → ARRIVE
        │                                       │
   frontier explore                    object detector
   (nav-sim's explore-rth,             (P6.1 — COCO person/
    already exists)                     vehicle detector, extend
        │                                classes as needed)
        ▼
   existing move-stop-sense + occupancy-grid approach, unchanged,
   once the target bearing is known
```

**Part 1 (parsing)** only needs an LLM if the phrasing is genuinely
open-ended; a fixed grammar (`look for <object>`, `find <object>`, chainable
with `then`/`and`) is a slot-filling parser, no learning required — the same
technique voice-assistant skills used for a decade before LLMs. This is
exactly what `P3` (the LLM supervisor, `ROADMAP.md`) already is.

**Part 2 (execution)** is the established robotics problem **Object-Goal
Navigation (ObjectNav)**: recognize the target when visible (perception), do
**frontier exploration** when it isn't (push toward the boundary between
mapped/unmapped space — nav-sim's `explore-rth` mode already *is* this),
approach once found (the existing move-stop-sense/occupancy-grid stack,
unchanged), and advance to the next subgoal on detection (a plain FSM
transition, same shape as adding `STUCK`). Modern ObjectNav systems (VLFM,
ESC) are exactly frontier exploration plus one addition: weighting which
frontier to try first by semantic/commonsense likelihood.

---

## Where an LLM's commonsense reasoning helps — and where it explicitly doesn't

**ESC** (Exploration with Soft Commonsense Constraints, ICML 2023) is the
concrete precedent for the specific idea proposed here: it uses a pretrained
LLM *only* to answer "how likely is this object near this kind of place,"
converts that into a soft weighting on which unexplored frontier to try
first — **never to control the robot directly** — and reports a **288%
relative success-rate improvement** over a non-LLM-guided baseline (CoW) on
the MP3D benchmark. Same propose/dispose shape as everywhere else this
pattern has come up in this project's design discussions (P3's guard layer,
the geometry-always-owns-safety rule in `context-gated-perception.md`): the
LLM proposes a *prior*, a classical layer (frontier scoring, then the
existing navigation stack) still does the actual searching and moving.

---

## Experiment: can a small quantized LLM do this on the Pi 5?

Checked against real benchmarks, not assumed:

| Model size | Quantization | Speed on Pi 5 (`llama.cpp`) |
|---|---|---|
| ~1B | Q4_0 | ~14 tok/s |
| 1–1.5B | Q4 | 10–18 tok/s |
| 3B | Q4 | 4–7 tok/s |
| 7B | Q4 | 0.7–3 tok/s — too slow to be useful here |

The 1–3B tier is comfortably usable, and throughput matters far less than it
looks: this is a `think`-tier query issued once per search decision, not per
control tick.

**Quantization has a real cliff, worth respecting:** 4-bit is the safe floor
(near-lossless on most benchmarks, what production deployments actually
ship); 3-bit shows measurable degradation (>7% on harder reasoning
benchmarks); 2-bit is risky with standard post-training quantization
(sometimes collapsing toward near-random) *unless* paired with an advanced
quantization method, or a model **natively trained** at low precision —
BitNet-class ternary (~1.58-bit) models are trained from scratch to work at
that precision and report performance on par with full-precision models, with
real CPU speedups (2–6×) and large energy reductions. If size ever needs to
grow past what plain 4-bit gives, that's the technique to reach for, not
harder quantization of a normal model.

**Why "where's a chair likely" specifically is a favorable case, not a hard
one:** it's a narrow, templated, high-frequency associative query ("chairs
near {room type}: likely/unlikely?") with a small constrained answer space —
much closer to *recall* than *open reasoning*. Published work backs this up
directly: `LiteVLA-Edge` reports a 4-bit-quantized small vision-language
model preserving enough semantic reasoning for real robot action guidance,
and separate 2025 work specifically extracts this exact kind of
"where-would-object-X-likely-be" commonsense from LLMs for robotic agents.

**Reliability lever, and it doubles as a guardrail:** constrain the output to
a fixed small vocabulary (`high`/`medium`/`low`, or a class list) via
grammar-constrained decoding, instead of free text to parse. This both
improves a small model's reliability on this exact task *and* produces an
output that's trivially validated/whitelisted — the same property P3's guard
layer already requires of anything the LLM emits.

---

## Proposed experiment plan, if pursued

1. **Testbed: nav-sim first**, headless, no hardware — matches this
   project's test-first engineering discipline (`PROJECT.md` →
   *Engineering process*). A synthetic arena with a labeled semantic layout
   (rooms tagged by type, objects tagged by class) would be needed; doesn't
   exist yet.
2. **Model candidates:** Llama 3.2 1B or 3B at Q4 via `llama.cpp` (the engine
   P3 already assumes); a small BitNet-class model as a stretch option if
   plain 4-bit turns out insufficient.
3. **Query shape:** a single templated prompt per frontier candidate — "given
   the current frontier is near {context}, how likely is {target class}
   present?" — answered through grammar-constrained decoding into
   `high`/`medium`/`low` only.
4. **What to measure:**
   - Does the LLM-weighted frontier choice actually reduce search steps /
     time-to-find versus blind/geometric-only frontier exploration, A/B in
     nav-sim?
   - Per-query latency, confirming it fits the `think`-tier cadence
     (once-per-search-decision) rather than stalling anything control-rate.
   - Whether the constrained-output form ever produces something outside the
     allowed vocabulary (it shouldn't, by construction, but worth asserting).
5. **Guardrail, non-negotiable if this is ever wired to the real
   `MissionController`:** the LLM's output is *only* a soft weight on frontier
   choice. It can never itself emit a bearing, a command, or override the
   corridor/safety reflex — identical discipline to P3's whitelist schema and
   to `context-gated-perception.md`'s "geometry owns collision safety,
   unconditionally" rule.

---

## Status

Idea + experiment plan only. No arena, no grammar-constrained-decoding
harness, no model pulled, nothing scheduled. If pursued, the natural home is
an extension of `P3` in `ROADMAP.md` — the sidecar transport and guard-layer
work P3 already requires apply directly here, just with a narrower prompt and
a semantic-frontier-weighting consumer instead of direct mode/goal commands.

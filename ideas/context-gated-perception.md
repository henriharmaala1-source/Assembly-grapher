# Context-gated perception (idea, not built, not scheduled)

*Drafted from a design conversation, not from a backlog item. Written down here
— not in `PROJECT.md`/`ROADMAP.md` — because it hasn't been designed against
the real interfaces (`IPerceptionModule`, the Deliberator scheduler,
`Mission::Params`) yet, and nothing below should be read as built or planned
for a specific phase. If it graduates, it lands as a `P6.x` item.*

---

## The idea

A cheap, always-on detector classifies the *setting* the aircraft is in; that
label gates which heavier perception models are actually running, and selects
a navigation parameter profile for the existing move-stop-sense controller.
Restated in the shape it was proposed in:

> sense environment → cheap bootstrap detector sees "chair, door" → classifies
> **inside** → gates in a lightweight specialist (people, stairs) + a depth
> model → geometry (depth) handles collision safety, semantics (specialist)
> flags hazards → the move-stop-sense controller runs an indoor-tuned profile

```
video ─► cheap bootstrap detector (always on, low rate)
              │
              ▼
        setting classifier ── debounce + confidence hold ──► "indoor" / "open field" / "forest" / unknown
              │                                                        │
              ▼                                                        ▼
   gate: which specialist perception          select: Mission::Params profile
   models are resident/hot                    (stepM, cruise, minOpenToMove,
   (indoor: small people+stairs YOLO           scan aggressiveness, stuckSweeps)
    + depth; outdoor: road-follow, etc.)
              │                                                        │
              └──────────────► WorldModel (blackboard) ◄───────────────┘
                                        │
                              geometry (depth/corridor) → collision safety, ALWAYS on
                              semantics (specialist)     → hazard flags (person, stairs)
                                        │
                                        ▼
                              move-stop-sense controller (SETTLE→THINK→SCAN→MOVE→ARRIVE→STUCK)
```

Two independent gates come out of one classification, which is the useful
part: the setting doesn't just retune the controller, it also changes *what
perception even bothers to run* — you don't pay for a stairs-detector outdoors
or a road-follow model indoors.

### Guardrails that came out of talking it through

- **Debounce the label.** A classifier flickers at boundaries (a doorway is
  the adversarial case). Require a sustained, confidence-thresholded read
  before switching profiles or swapping models — the same discipline already
  applied to `STUCK` not un-latching on a single phantom-open corridor tick
  (`move_stop_sense.cpp`).
- **Safe default under uncertainty.** Unknown/low-confidence setting → fall
  back to the most conservative profile (slow, cautious, scans more), never
  the most aggressive one. Misclassification should cost speed, not safety.
- **Geometry owns collision safety, unconditionally.** The setting label may
  retune *style* (`stepM`, `cruise`, scan aggressiveness) but must never gate
  the depth/corridor stop reflex. If the classifier says "open field" and
  there's a wall, the reflex fires regardless — semantics tunes behavior
  inside fixed safety bounds; it can never turn avoidance off.
- **Keep both specialist models resident, don't thrash them.** Load/unload
  latency makes swapping models at every doorway a bad idea on a Pi 5; with
  4–8 GB RAM, keep the indoor and outdoor specialists both loaded and switch
  which one is *ticked*, gated by the debounced label.
- **Fuse depth + semantics for the hazards that need both.** A downward depth
  discontinuity *and* a "stairs" detection together is the strong signal
  (depth alone can't tell stairs from an open doorway at a distance; semantics
  alone can't tell you range). Same principle the project already applies
  (monocular depth has no scale → paired with a metric ToF sensor as ground
  truth; see `PROJECT.md` → *State estimation* / *Autonomous motion*).
- **The always-on bootstrap detector must stay cheap.** Something has to run
  continuously to produce the setting label in the first place; it can't be
  the heavy model it's gating. Matches the project's existing cheap/heavy
  perception-tier split (`IPerceptionModule`, Deliberator budget).

---

## Is this a known pattern, or made up?

Asked and checked against prior art (2026-07) rather than answered from
priors. Short answer: **the pieces are all standard, separately load-bearing
patterns; the specific combination — scene-classify → gate both model
selection and a navigation parameter profile, on a CPU-only aircraft — doesn't
show up pre-packaged in the literature, but it's a reasonable synthesis of
things that are each independently well used, not an invented mechanism.**

- **Cascade / coarse-to-fine classifiers — the "cheap gate" half.** This is
  the oldest and most established piece: the Viola–Jones face-detector cascade
  (2001) is exactly "a lightweight detector rejects/gates first, an expensive
  one runs only on what survives," and it's still the reference pattern for
  cheap-first inference on constrained hardware. Modern edge-AI writeups
  describe the identical shape as a *recurring production pattern*: "cheap
  gate, confidence signal, routing to an expensive consumer," with
  MobileNet/ShuffleNet-class nets as the gate and the heavier model held back
  until warranted. IDK-cascade and coarse-to-fine DNN-cascade work extends the
  same idea to modern edge object detection specifically.
  ([Cascading IDK Classifiers](https://userweb.cs.txstate.edu/~k_y47/webpage/pubs/compsac25.pdf);
  [Coarse-to-Fine hierarchical DNN inference for edge computing](https://www.sciencedirect.com/science/article/pii/S0167739X24000736);
  [Optimizing Edge AI: data/model/system strategies survey](https://arxiv.org/html/2501.03265v1/))

- **Context/scene classification driving a navigation-strategy switch — the
  "decide the setting, then behave differently" half.** This is an active
  robotics research line under exactly this name. Socially-aware navigation
  work runs "a context classification pipeline [that] allows a robot to
  change its navigation strategy based on the observed social scenario,"
  feeding a selected objective into the local planner — structurally the same
  shape as *classify the setting → pick a `Mission::Params` profile* proposed
  here.
  ([A Deep Learning Approach To Multi-Context Socially-Aware Navigation](https://arxiv.org/pdf/2104.10197);
  [Towards a Unified Planner For Socially-Aware Navigation](https://arxiv.org/pdf/1810.00966))
  Indoor/outdoor scene classification specifically for mobile-robot
  navigation is its own established sub-literature (classical SVM/kNN/decision-tree
  classifiers over color/texture features, and shallow CNNs, both reported
  in the 90%+ accuracy range), which is close kin to "YOLO detections → a
  decision tree → most-likely setting" as proposed here.
  ([Indoor vs. Outdoor Scene Classification for Mobile Robots](https://www.researchgate.net/publication/344623135_Indoor_vs_Outdoor_Scene_Classification_for_Mobile_Robots);
  [Scene perception based visual navigation of a mobile robot in indoor environment](https://pmc.ncbi.nlm.nih.gov/articles/PMC7550175/))
  The self-driving world runs the same idea one level up as **Operational
  Design Domain (ODD)**: a car recognizing its current operating context
  (highway vs. surface street, weather, etc.) and changing which features/
  behaviors are active accordingly.
  ([Operational design domain — Wikipedia](https://en.wikipedia.org/wiki/Operational_design_domain);
  [What Are Operational Design Domains? (Aptiv)](https://www.aptiv.com/en/insights/article/what-are-operational-design-domains))

- **Routing to a domain-specialized model — the "swap in a smaller, better
  model for this context" half.** Current (2026) work on domain-specialized
  object detection via model-level mixtures of experts does exactly this:
  route to a smaller expert model matched to the current domain rather than
  running one large generalist. Mixture-of-Experts more broadly is the general
  form of "a gating mechanism picks which specialized sub-model runs," well
  established across robotics multi-task and multi-modal perception.
  ([Domain-Specialized Object Detection via Model-Level Mixtures of Experts](https://arxiv.org/pdf/2604.18256);
  [NVIDIA: What Is Mixture of Experts?](https://www.nvidia.com/en-us/glossary/mixture-of-experts/))

**What's genuinely novel here, if anything, is narrow:** using the classified
setting as a *single label that fans out into two independent decisions at
once* (which perception models are hot, *and* which controller parameter
profile is active) on a CPU-only companion computer riding an analog FPV
video tap. That specific combination and hardware target isn't something the
search surfaced as a named, published system — but every piece it's built
from is a standard, independently-proven pattern. Verdict: **sound applied
engineering, correctly recombining established ideas — not invented from
nothing, and not naive either.** It would be overclaiming to call it a novel
architecture; it's accurate to call it a reasonable, literature-consistent
design.

---

## Status

Idea only. Not designed against `IPerceptionModule`/the Deliberator scheduler,
not costed against the Pi 5 compute budget, no acceptance test, no `Params`
profile table written. Depends on perception hardware/models that don't exist
in the current BOM yet (see `PROJECT.md` → *Current status*). If pursued, the
low-risk first cut is exactly the `Mission::Params` profile-table half (no new
model-gating machinery) — see the note left in `ROADMAP.md` → P6.

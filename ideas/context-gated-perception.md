# Context-gated perception (idea, not built, not scheduled)

*Drafted from a design conversation, not from a backlog item. Written down here
— not in `PROJECT.md`/`ROADMAP.md` — because it hasn't been designed against
the real interfaces (`IPerceptionModule`, the Deliberator scheduler,
`Mission::Params`) yet, and nothing below should be read as built or planned
for a specific phase. If it graduates, it lands as a `P6.x` item. A follow-on
research note — whether AI could ever make the decisions here rather than
just the perception, what hardware/training that would take, and how far
that is from this project's Pi-5 constraint — lives in
`ideas/learned-decision-making.md`.*

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

## Related architectures — the wider landscape this sits in

Scene-classify → adapt behavior is one point in a much older space of "give a
robot information on what to do based on its environment." Surveyed
2026-07-07 so this isn't presented as more novel than it is:

| Family | Environment → decision, in one line | Where in this project |
|---|---|---|
| **Reactive / behavior-based** (Brooks' subsumption, 1986) | Sensing wired straight to acting, no world model; layered behaviors, higher ones suppress lower ones | The VFH+ corridor steering — sensor reading → steering command, no deliberation |
| **Deliberative (sense-plan-act)** | Build a world model, then a symbolic planner (STRIPS/PDDL) searches an action sequence to a goal | Not used directly, but the wavefront planner over the occupancy grid is this family in miniature |
| **Hybrid three-layer** | Reactive layer (fast/safe) + deliberative layer (goals/maps) + a sequencer arbitrating between them | **This project's own architecture**: Deliberator + fly loop + `MissionController` as the sequencer |
| **FSM / hierarchical state machines** | Environment readings gate transitions between named modes, each with its own policy | `MoveStopSense` / `MissionController` (SETTLE→THINK→SCAN→MOVE→ARRIVE→STUCK) |
| **Behavior trees** | Same job as an FSM, composed as prioritized/reactive tree nodes instead of a transition table | Considered, not adopted — see prior design discussion; the FSM already covers this project's scope |
| **Blackboard architecture** (Hearsay-II, 1970s) | No central controller; independent modules read/write shared state, triggering on what's new | `WorldState` |
| **Fuzzy-logic / rule-based controllers** | Sensor readings through fuzzy membership rules straight to actuator commands | Not used; a smoothed cousin of the reactive family |
| **BDI / cognitive architectures** (Bratman) | Environment updates *beliefs*; the agent picks among competing *desires* and commits to an *intention*, replanning on failure | Not used — heavier machinery than this project's scope needs |
| **MDP / POMDP policy control** | Formalizes "state → action" as a value/policy function, explicitly modeling *uncertainty* | Relevant but not solved this way: the occupancy grid's "unknown = free" optimism is a classic POMDP problem (acting under partial observability) sidestepped with a heuristic, not a probabilistic policy |
| **Semantic / topological place classification** | Robot classifies *which place type* it's in (kitchen, corridor, office) from detected objects/features, adapts navigation per place | **The closest published relative to the idea in this file** — an active robotics sub-field, not invented here (see citations above) |
| **End-to-end learned sensorimotor policies** | A trained net maps raw sensor input straight to actuator output, no explicit decision layer | Not used — see *Where AI fits*, below, for why |
| **Vision-Language-Action (VLA) models** | A foundation model takes image + language instruction, outputs action directly, fusing perception and decision into one learned step | Not used — needs GPU-class compute this project's Pi-5/CPU-only constraint rules out, and trades away the inspectability this project is built around |
| **Context-aware computing** (Dey & Abowd, 1990s ubiquitous computing) | The general, non-robotics parent of all of the above: sense context → adapt system behavior | Every row above is a robotics specialization of this older, more general pattern |

This project is a **hybrid three-layer / FSM / blackboard** design. The
context-gated perception idea above doesn't replace any of that — it's a
**semantic place-classification** layer added on top, feeding a new input
(the setting label) into the existing sequencer and scheduler, the same slot
the socially-aware-navigation literature uses a "context classification
pipeline" for.

## Where AI fits in this design (and where it deliberately doesn't)

Worth being precise about, since "AI" gets used to mean two different things:

- **Historically, "AI" meant symbolic search and planning** — A*, STRIPS/PDDL
  planners, rule-based expert systems, decision trees. Under that definition,
  this project already has AI throughout its decision layer: the wavefront
  planner, the FSM's phase logic, even a hand-authored decision tree would
  count.
- **Colloquially today, "AI" means learned/statistical models** — neural
  nets, ML classifiers. Under *that* definition, here's where it actually
  sits in a real perception→decision→action pipeline, checked against what's
  deployed today rather than assumed:

| Layer | Is it (modern-sense) AI, in practice? | Why |
|---|---|---|
| **Perception** — object detection, depth estimation, scene/place classification | **Yes, dominant and undisputed.** | This is what ML is genuinely best at: turning raw pixels into structured facts. This project's depth model and object detector are exactly this; the setting classifier proposed in this file would be too (a learned decision tree/small CNN over detections). |
| **State estimation** — fusing sensors into a position/velocity estimate | **No, classical.** | Kalman filtering is 1960s estimation theory, not machine learning — worth naming explicitly because it's easy to assume "estimator = AI." This project's EKF is a concrete counter-example already in the codebase. |
| **High-level decision/task logic** — FSM, behavior tree, blackboard, symbolic planning | **No, classical (in the modern sense) — and deliberately so.** | This is precisely the layer that needs to be inspectable, testable without a camera, and certifiable — properties hand-authored logic has and a learned policy generally doesn't. This project's `MissionController`, the wavefront planner, and the mode arbiter are all classical by design, not by omission. |
| **Low-level continuous control** (motor/gait control, not task decisions) | **Increasingly yes, and now in production** — reinforcement-learned locomotion policies run on real deployed legged robots (ANYmal, Unitree-class quadrupeds), not just in simulation. | Notable because it shows AI *can* live inside a real control loop today — but even on those robots, the *high-level* task/mission decision layer above the learned gait controller is still classical. AI has entered low-level control before it's entered high-level decision-making. |
| **Advisory task-level suggestions, kept out of the control loop** | **Yes, and this is the emerging safe pattern for it.** | This project's own P3 (LLM supervisor, `ROADMAP.md`) is exactly this: an LLM reads the world state and *suggests*, through a whitelisted command schema, and can never touch control directly. Published work (RoboGuard, 2026) formalizes the same principle for LLM-planned robot tasks: a safety/guardrail module — not the LLM — has final say over whether a proposed plan executes, cutting unsafe-plan execution from 92% to under 2.5% in their evaluation. The recurring shape, in both cases, is: **AI proposes, a separate deterministic/verifiable layer disposes.** |

**Net for this project:** AI (modern sense) belongs in perception — including
the setting-classifier idea in this file — and, cautiously and only as an
advisory suggester, at the very top of the decision stack (P3). It does not
belong in the FSM/planner/estimator layer, not because that's old-fashioned,
but because that layer's job is to be safety-provable on a Pi 5 with no
camera attached, which is exactly what a learned policy gives up.

---

## Status

Idea only. Not designed against `IPerceptionModule`/the Deliberator scheduler,
not costed against the Pi 5 compute budget, no acceptance test, no `Params`
profile table written. Depends on perception hardware/models that don't exist
in the current BOM yet (see `PROJECT.md` → *Current status*). If pursued, the
low-risk first cut is exactly the `Mission::Params` profile-table half (no new
model-gating machinery) — see the note left in `ROADMAP.md` → P6.

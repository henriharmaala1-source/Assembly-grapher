# Learned decision-making for robots — research notes (not built, not scheduled)

*Follow-on from `ideas/context-gated-perception.md`, from a broader design
conversation asking: does AI actually "think" for a robot anywhere today, what
hardware would that need, how do those models decide and get taught, and is
any of it reachable on hardware like this project's (CPU-only Pi 5)? Written
down as reference/background, not a proposal — nothing here is scheduled or
designed against the real interfaces. Some claims below are well-established
field knowledge rather than freshly re-verified sources; noted where relevant.*

---

## 1. Does AI "think" for machines anywhere today?

Split by what "AI" means (this distinction matters — see
`ideas/context-gated-perception.md` → *Where AI fits*):

- **Perception** (object/gate detection, depth, VIO, scene classification):
  AI in the modern sense, dominant and undisputed.
- **State estimation, high-level task/mode logic** (Kalman filters, FSMs,
  planners): classical, not AI — and, in this project, deliberately so, for
  inspectability and certifiability.
- **The genuinely new territory — AI making real-time tactical decisions, not
  just perceiving:**
  - **Swift** (ETH Zürich/Intel, *Nature* 2023) — a deep-RL policy beat human
    world-champion pilots at real, physical FPV drone racing. Not a hand-coded
    maneuver library: a learned policy making the actual flight decisions.
    ([Champion-level drone racing using deep reinforcement learning](https://www.nature.com/articles/s41586-023-06419-4))
  - **World models** (Dreamer → DreamerV3/DreamerV4, Genie, DayDreamer) — the
    machine learns its own internal simulator of the world and **plans by
    imagining outcomes inside it before acting**, the most literal analogue to
    deliberation in this list. DayDreamer already runs this on a real physical
    robot with no external simulator or human demonstrations.
    ([Introducing Dreamer](https://research.google/blog/introducing-dreamer-scalable-reinforcement-learning-using-world-models/);
    [DayDreamer: World Models for Physical Robot Learning](https://arxiv.org/pdf/2206.14176))
  - **Neurosymbolic LLM task planning** — an LLM proposes a plan, a *formal
    verifier* (temporal logic / PDDL) checks it's actually safe before
    execution, e.g. the Verifiable Iterative Refinement Framework (VIRF) and
    similar 2025–2026 work. This is the leading pattern for letting an LLM's
    flexible reasoning drive a robot without giving up a safety guarantee.

- **Why none of this is in a flight-critical system yet — a certification
  problem, not a capability problem.** DO-178C, the standard that certifies
  commercial aerospace software, requires every behavior to trace to an
  explicit requirement in explicit code — and, quoted from current research on
  the gap: *"For neural networks, the DO-178 process breaks down when
  requirements cannot be simply flowed down to explicit lines of code."* EASA
  has worked on this since 2019 (the CoDANN program with Daedalean) and is
  still only at early, incremental concepts for letting a neural net make a
  safety-relevant aviation decision. **The capability is ahead of the ability
  to certify it.** The published guardrail pattern for using an LLM safely
  anyway — RoboGuard — puts a separate, non-learned safety module between the
  LLM's proposed plan and execution, cutting unsafe-plan execution from 92% to
  under 2.5% in evaluation. The recurring shape: **AI proposes, a separate
  deterministic/verifiable layer disposes.** This project's own P3 (LLM
  supervisor, advisory-only, whitelisted schema, `ROADMAP.md`) already follows
  the identical principle.

---

## 2. What hardware does real "AI thinking" require?

Checked directly rather than assumed — none of the systems above run on
anything like a Pi 5:

| System | Onboard compute | Notes |
|---|---|---|
| **Swift** (drone racing) | NVIDIA **Jetson Orin NX** | 966 g airframe; monocular wide-FOV camera (155°×115°) + IMU; perception = stereo VIO (Intel RealSense) + DNN gate detector + learned residual observation model; all compute onboard, no external infrastructure |
| **ANYmal-D** (legged) | Intel Core i7-8850H (control policy) + a separate onboard Jetson (elevation mapping) | Policy inference ~50 Hz |
| **Unitree A1 / later** | Jetson TX2 → Jetson Orin NX | Policy ~50 Hz, PD loop ~200 Hz |

| Board | AI compute | Power | Role |
|---|---|---|---|
| **Raspberry Pi 5** (this project) | **None dedicated** — Cortex-A76 CPU only | ~3 W idle / ~11.6 W full load | General compute; DNN inference is CPU-only, no tensor cores |
| **Hailo-8** (add-on accelerator, already considered for this project) | ~26 TOPS | ~2.5 W | Cheapest real step up |
| **Jetson Orin Nano Super** | ~67 TOPS | 13 W (active cooling) | Entry Jetson tier |
| **Jetson Orin NX** (Swift's board) | up to ~157 TOPS | 10–40 W | Concurrent perception + RL control policy, at flight rate |

The Pi 5 alone sits at zero in the column that matters. This is the concrete,
physical reason `ROADMAP.md`'s F10 item exists — perception already saturates
a 60 ms/tick budget running one or two small models *sequentially*, where
Jetson-class hardware runs several concurrently at 50–100+ Hz. Any move toward
onboard learned tactical decision-making is a **board-level hardware
decision** (Hailo-8 add-on at minimum, a Jetson swap for Swift-class
capability) with real cost/mass/power consequences against this project's
explicit "cheap, CPU-only, minimal added electrical load" design constraint
(`PROJECT.md`) — not a software change.

---

## 3. How do these models actually decide, and how are they taught?

### Decision mechanisms — two different things, not one

- **RL policies (Swift, legged controllers) — a small learned function, not
  reasoning.** Swift's actual policy network: **4 fully-connected layers, 24
  inputs → 6 outputs**, actor and critic combined. No symbols, no internal
  narration — a mathematical function shaped by training that maps a state
  vector (relative position/velocity to the next gate, attitude, etc.)
  straight to a control command, evaluated in a fraction of a millisecond.
- **World models — closer to actual deliberation.** Learn a dynamics model,
  then plan by imagining several action sequences inside it and picking the
  one with the best predicted outcome, *before* touching real actuators.
- **VLA / LLM-based planners — next-token prediction.** A transformer ingests
  image + language + robot-state tokens and predicts the next
  action/plan-step the same way an LLM predicts the next word: continuing a
  sequence, not evaluating explicit logic.

### Teaching methods — three different paradigms

- **Reinforcement learning (trial-and-reward, no labeled "correct" action
  ever given).** Swift's reward: progress toward the next gate's center, plus
  a term for keeping the gate in the camera's field of view. The policy tried
  actions in simulation, got scored, and reinforcement favored what scored
  well — repeated at a scale and speed no real drone could survive. Sim-to-
  real was closed with **learned residual models**: extra correction terms
  trained specifically to capture how real perception and dynamics differ
  from the simulator, layered on top of the simulated training. Legged-robot
  RL controllers use the same recipe (reward for progress + stability,
  trained across thousands of simulated robot instances in parallel, with
  randomized physics so the policy tolerates the sim not matching reality
  exactly) — this is well-established, widely used technique in the field
  (not freshly re-checked this session; flagged for that reason).
- **Imitation / behavior cloning (supervised, from demonstrations).** Show
  many examples of correct behavior; train the model to reproduce the
  mapping. Many VLA models are trained this way on large multi-robot
  demonstration datasets, sometimes RL-fine-tuned afterward.
- **Self-supervised pretraining (predict the next token, no reward or
  demonstration needed).** How the LLM inside a VLA/neurosymbolic planner got
  its general knowledge in the first place, before any robotics-specific
  adaptation.

### What kind of decisions — a granularity spectrum

| Granularity | Example | Update rate |
|---|---|---|
| Reflex / continuous control | Swift's thrust + body-rate output; joint torques | Every control tick (50–200 Hz) |
| Maneuver / short trajectory | "Take this gate on this line"; a gait pattern | A few times per second |
| Task / plan step | "Pick up the cup, then go to the table" | Seconds to minutes apart |

None of the systems surveyed decide "the overall mission" in a human sense —
even the most capable ones operate within a fairly narrow, trained scope. The
task-level tier is the one still leaning on an LLM/planner rather than pure
RL, being closer to language-shaped reasoning than continuous control.

---

## 4. Lighter-weight options — the useful reframe

Swift's policy — **4 layers, 24 in, 6 out** — is tiny: almost certainly
runnable on a plain CPU in well under a millisecond, no accelerator required.
The Jetson Orin NX isn't there for the "thinking" net; it's there for the
**perception front end** (stereo VIO + a gate-detection CNN running every
video frame). That's the expensive part. The decision net riding on top of it
is cheap.

**This separates two questions that are easy to conflate: "can the drone
think" and "can the drone see" are different problems, and the compute-heavy
one is seeing, not thinking.** This project's own `WorldState`/`MssInput` is
already a compact feature vector (corridor openness, offset, plan bearing,
etc.) — structurally the same shape as the 24-dimensional input Swift's
policy consumes. If a small RL-trained policy ever experimentally replaced
part of the hand-coded FSM logic, the *policy* itself likely wouldn't need
new hardware — the open questions would be training it (needs a trustworthy
simulator) and certifying it (see §1), not running it.

Established techniques for shrinking things further (general field knowledge,
not freshly re-verified this session):
- **Policy distillation** — train a large/slow "teacher" policy any way you
  like, then train a small student net (or even a decision tree, for full
  interpretability) to imitate the teacher's outputs, trading a little
  performance for speed and inspectability.
- **Quantization/pruning** of the perception side specifically — this
  project's own `ROADMAP.md` F10 item (CPU inference runtime swap) is exactly
  this lever, and it's the one that actually matters here, since perception —
  not decision-making — is what strains a Pi 5.

---

## 5. Where this is headed

Directional, not freshly re-verified this session: the field is visibly
moving toward **smaller, distilled versions of large robot-foundation
models** for edge deployment (mirroring how LLMs got dramatically cheaper
after the first giant ones proved the idea), continued growth of
**neurosymbolic verified planning** (an LLM proposing more while a formal
checker still gates execution), and **cheaper accelerators creeping toward
Jetson-class capability** at Pi-5-adjacent power budgets — Hailo-8-class chips
are the current leading edge of that. The gap between "CPU-only" and "runs
real learned tactical decisions" is likely to narrow over the next few years,
not vanish — certification stays the slower-moving blocker underneath all of
it.

---

## Status

Research notes only. No design proposal, no `Params`/interface changes
implied, nothing scheduled. Relevant to `ROADMAP.md` → P6.6 (context-gated
perception) as background on what "add a specialist model" could someday grow
into, and to F10 (inference runtime swap) as the one lever that's actually
load-bearing for this project's real compute constraint.

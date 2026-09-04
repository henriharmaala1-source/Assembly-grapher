# RL path-policy harness — plan

Target hardware: **i5-13600K** (6 P-cores / 12 threads + 8 E-cores = 20 threads),
**RTX 4070** 12 GB, **32 GB** RAM.

The goal is not "add RL". It is to produce a learned planner that can be put in
the same table as `--traj` and `--histogram`, on the same worlds and the same
seeds, and be beaten or beat them on numbers this repo already trusts.

---

## 0. The two decisions that matter most

Everything else is plumbing. These two are the design.

### 0.1 Learn the RANKING, not the control

The policy does **not** emit roll/pitch/yaw/throttle. It scores the primitives
that `sphereClear` has **already admitted**, and picks one.

- The hard veto stays geometric. The network can never grant permission — only
  advise among options the geometry approved. That is this project's existing
  authority asymmetry, obtained for free.
- The action space is a discrete choice over ≤210 options rather than
  continuous control: far easier to learn, and far easier to bound.
- A bad policy picks a *worse admissible* primitive. It cannot pick an
  inadmissible one. The Langostino failure mode — a learned model steering into
  space nothing measured — is structurally unreachable.

### 0.2 The observation must be able to say UNKNOWN

A single scalar per direction cannot distinguish *clear* from *unmeasured*, and
a policy trained where ground truth is complete will never discover the
difference. Every spatial channel is therefore **two channels: a value and an
observability mask.**

This is the same rule as three-state occupancy, one level up. Getting it wrong
is not a bug that shows up in training; it is a bug that shows up on hardware.

---

## 1. Observation

Per-primitive features (the thing being ranked) plus a global context, so the
network is a **shared per-item encoder + softmax over primitives**. Small,
permutation-consistent, and cheap enough for the Pi at inference.

**Per primitive `i` (210 × ~8 floats):**

| feature | why |
|---|---|
| `freeM` along the rollout | how far it is confirmed clear |
| min clearance over the swept ball | margin |
| unknown fraction of the rollout | the observability channel |
| endpoint az/el relative to the goal bearing | progress |
| endpoint far-field openness | what the bearing field advises |
| commanded speed, yaw rate, climb | the action itself |
| endpoint visit count (see §2) | memory |

**Global context (~24 floats):** goal bearing/elevation/range, current speed,
attitude, fraction of primitives admitted, mean/max `freeM`, stopped-steps
counter, and a coarse (36 az × 4 el) bearing-field summary with its own mask.

---

## 2. Memory, because the maze is a POMDP

The measured failure in `--world maze` is not tuning: the reactive planner has
**no record of which branches it rejected**, so it re-derives the same local
preference every time it returns to a junction. No stateless score function can
fix that.

Two mechanisms, cheapest first:

1. **Visit-count grid.** A coarse 2-D ego-centric grid (say 1 m cells, 32×32)
   counting how often each cell has been occupied by the vehicle, decayed. Fed
   as a channel, and sampled per primitive endpoint. This alone should enable
   "do not go back down that corridor" and costs almost nothing.
2. **Recurrence** (`RecurrentPPO` from sb3-contrib) if (1) is not enough.

Start with (1). It is interpretable, and if it works the policy stays feed-forward
and trivially cheap on the Pi.

---

## 3. Reward

The user's requirement is longer runs with distance rewarded. **Raw distance
travelled is reward-hackable** — a policy that orbits forever scores well — so
the distance term is split:

```
r =  w_prog * Δ(range-to-goal)          progress, the main dense term
   + w_cov  * (newly visited cells)     coverage, rewards exploring not orbiting
   - w_time * dt                        finish sooner
   - w_stop * (1 if speed < 0.1 else 0) stopping is the measured failure
   - w_clear* max(0, d_min_target - minClearance)
   + R_goal on arrival
   - R_coll on collision (terminal)
```

`w_cov` on **newly visited** cells is what makes long runs pay without paying
for circles. `w_stop` targets the specific pathology the sweep already measures.

---

## 4. Where the environment lives

The sim is C++ and must stay that way — it is the flight code. So:

- **pybind11 module** (`voxelenv`) wrapping a `VoxelEnv` class: `reset(world,
  seed)`, `step(primitive_index)`, returning observation / reward / done / an
  info dict carrying the scorecard fields.
- **Gymnasium** wrapper over it, then SB3 **`SubprocVecEnv`**.

No IPC per step, no serialising a depth image to Python — the observation is
already a few thousand floats by the time it crosses the boundary.

---

## 5. Making use of the hardware

### Thread budget (20 threads)

| workers | assignment |
|---|---|
| 16 env workers | 8 E-cores + 8 P-threads |
| 2–4 threads | learner, data collation, OS |

E-cores are ~60–70 % of P-core IPC but env stepping is a **throughput** problem,
not a latency one, so they are worth using. Pinning the learner to a P-core with
`taskset` is an easy 10–20 % and worth doing once the rest works.

### Throughput, measured from this repo's own numbers

| mode | ms/step | 16 workers | 10 M steps |
|---|---|---|---|
| `--voxelonly` (no camera) | 3.5 | ~4,500 /s | **~37 min** |
| `--truth` depth | ~15 | ~1,050 /s | **~2.6 h** |
| stereo depth (real sensor model) | 45–130 | ~150–350 /s | **8–18 h** |

So a full stereo run is an overnight job, and the cheap modes are minutes. That
sets the curriculum in §6.

### The GPU is nearly idle, and that is correct

A `[64,128,64]`-class MLP at these batch sizes is **faster on CPU** than on the
4070 — the PCIe round trip costs more than the arithmetic. Benchmark
`device="cpu"` against `device="cuda"` before assuming otherwise; expect CPU to
win until the per-item encoder gets much larger.

The 4070 earns its place on other work in this project — learned stereo
(fine-tuning HITNet/RAFT-Stereo, which *is* GPU work and is the only thing that
moves `Z_max`), and any segmentation model. Do not size the RL plan around it.

### RAM

~80 MB per env (fine + mid + far maps, plus the world) × 16 ≈ 1.3 GB, plus
rollout buffers. 32 GB is not a constraint.

---

## 6. Curriculum

Staged because the expensive stage is 20–40× the cheap one, not because the task
needs shaping.

| stage | depth | worlds | steps | wall |
|---|---|---|---|---|
| 0 | `--voxelonly` | forest, maze | 1 M | minutes |
| 1 | `--truth` | forest, lanes, city, road | 5 M | ~1.5 h |
| 2 | **stereo** | same four | 10 M | 8–18 h |

**Stage 0 is for plumbing and reward debugging only.** With the map complete
from step 1 it trains a *known-map* planner, and partial observability is the
entire problem — a policy that has never met an unknown cell will not survive
one. Never report a stage-0 number as a result.

Stage 2 must be stereo. The sensor model — dropout, texture dependence, `Z_max`,
occlusion shadow — is what makes the policy transfer, and it is precisely the
part Langostino's PyBullet training omitted.

---

## 7. Evaluation, and this is the point of the exercise

**Reuse `sweep.sh`'s scorecard verbatim.** The comparison is worthless if the
learned planner is scored on new metrics:

```
world  seed  outcome  travel  end-dist  minClr  falseFree
```

plus collisions, goals reached, stopped steps, and ms/step.

- **Held out: `maze` and `culdesac`.** Train on forest / lanes / city / road only.
  If it only works where it trained, it learned four worlds rather than
  navigation — and the maze is the case that motivated memory in the first place.
- **Held-out seeds** within the training worlds as well.
- **Paired** against `--traj` and `--histogram` on identical seeds, reported with
  |t| exactly as the existing sweeps do. Single-seed results are anecdotes; this
  harness has been burned by that twice already.

### The trap to instrument for

The policy sees the **map**; collisions are scored against **`VoxelWorld`**. That
separation is what makes reward hacking *detectable* — a policy exploiting a
systematic map bias will show rising reward and flat-or-worse true clearance.
Log both curves and watch them diverge.

---

## 8. Build order

1. ~~Build with OMPL and re-fly the maze.~~ **DONE — and it produced the
   baseline this whole comparison needs.** See below.
2. pybind11 `voxelenv` + a **random-action** agent. Proves the loop, the reward
   plumbing and the vectorisation. These projects die here, not in the learning.
3. Stage 0 run; confirm reward goes up and the eval script emits a sweep table.
4. Stages 1–2.
5. Paired comparison; write the result into `NOTES.md` whichever way it falls.


---

## 9. The non-RL baseline, measured

`find_package(ompl QUIET)` was missing a standard Ubuntu install (Debian puts
`omplConfig.cmake` in `/usr/share/ompl/cmake`, off CMake's default path), so
every maze result before this was the straight-line fallback. Fixed with HINTS;
the tell was 0.02 ms per replan against RRTConnect's 1.0 ms.

**Maze, 1500 steps, horizon 0.6 s, path travelled in metres:**

| seed | OMPL RRTConnect + reactive | reactive only |
|---|---|---|
| 1 | 13.1 | 0.8 |
| 2 | 18.5 | 14.6 |
| 3 | 10.4 | 0.8 |
| 4 | **20.5** | 0.8 |
| mean | **15.6** | 4.3 |

The global planner wins **4 of 4**, and reactive-only is wedged at the spawn on
three of them. So the maze does need a global planner, and it is not a tuning
problem.

**But the global planner plateaus.** Seed 4 at 4000 steps travels the same
20.5 m as at 1500 — closing 31.1 m to 17.1 m, about two thirds of the way — and
is **stopped on 3837 of 4000 steps (96 %)**. More budget buys nothing.

### Why this is the right baseline to put RL against

RRTConnect plans through the **map**, and in a maze the map is mostly UNKNOWN
because the corridors have not been looked down yet. Unknown is not free, so
there is nothing to plan through, and the planner cannot route to a goal it has
no observed corridor to. The maze is therefore an **exploration** problem
wearing a path-planning problem's clothes.

That is a clean, measured ceiling with a known cause, which makes it exactly the
comparison the RL policy has to win:

- beat **15.6 m mean** on held-out maze seeds, and
- break the **20.5 m plateau**, which requires remembering which branches were
  already refused — the thing §2's visit-count grid exists for.

If a learned policy cannot beat a classical global planner that is provably
blocked by partial observability, it has not earned its place.


---

## 10. Forest baseline, measured

Same protocol as the maze (§9): 1500 steps, path travelled in metres.

| seed | OMPL RRTConnect + reactive | reactive only |
|---|---|---|
| 1 | 215.9 | 10.8 |
| 2 | 242.5 | 20.7 |
| 3 | 245.3 | 23.2 |
| 4 | 112.4 | 46.2 |
| mean | **204.0** | **25.2** |

**8×**, against the maze's 3.7×, and the global planner wins 4 of 4 again. In
open forest it is doing nearly all of the work.

Together the two worlds bracket the problem: the maze is where the classical
stack hits a hard exploration ceiling (§9), the forest is where it runs freely
and any learned policy has a high bar to clear.

---

## 11. On CUDA — the earlier answer was too glib

"The GPU will be idle and that is correct" is right about the POLICY and wrong
as a general claim, so it is worth separating.

**Why most RL uses GPUs**, and whether it applies here:

| reason | applies? |
|---|---|
| Vision policies (CNN over pixels) | **No** — the observation is 1914 floats, not an image |
| Large policies (transformers) | **No** — a 256×256 MLP is microseconds |
| **GPU-resident physics** (Isaac Gym, Brax, MJX, Madrona) | **Yes, in principle** |

The third is the real one, and it is where the 100–1000× numbers in the
literature come from. And this environment is genuinely GPU-shaped: a 160×120
depth render is 19,200 independent DDA raycasts, and 210 primitives × ~20 points
× a 5³ sphere check is ~500,000 independent voxel lookups per step. Both are
embarrassingly parallel.

### But a full CUDA port is the wrong trade, for a specific reason

`frame_source.hpp` states the discipline: the sim runs **the same code that
flies**, so a bug seen on real data is a bug in the code that would fly. A CUDA
`VoxelMap` and a CUDA `sphereClear` are a *second implementation* of the flight
code. They would drift, and then the policy would be trained against a world
model that is not the one deployed. That is Langostino's failure with the serial
numbers filed off.

### The version that is worth doing

**Port only the depth renderer.** It is the right piece for three reasons:

1. **It is the dominant cost.** `voxel_sim` reports it separately — 46.8 ms/step
   in forest against ~14 ms of everything else, so roughly 75 % of a step.
2. **It is purely functional**: world in, depth image out. No state, no
   scattered writes, no atomics.
3. **It is not flight code.** The tool labels it `[sim-only]` in its own output,
   because the real aircraft gets depth from a D435i. A CUDA version forks
   nothing that flies.

Removing ~75 % of the step is roughly a 4× speedup, which takes a 10 M-step
stereo run from 8–18 h to **2–5 h** on the same 16 workers — without touching a
line of the code under test.

### And measure the policy device rather than assuming

`train.py --device cuda` exists so the claim can be checked. Expect CPU to win
at a 256×256 MLP; if the per-primitive encoder is widened substantially, that
flips, and the answer should come from a stopwatch either way.


---

## 12. State, and what has actually been run

Verified end to end on a 4-core container. Everything below was executed, not
just written.

| piece | state |
|---|---|
| `VoxelEnv` (C++) | **tested** — `rl_env_check`, 18/18 suite |
| `voxelenv` pybind11 module | **tested** — loads, 210 prims, 1914-float obs |
| `voxel_gym.py` | **tested** — resets, steps, masks |
| `rl_bench` (baselines exe) | **tested** — 4 policies, real numbers |
| `train.py` | **RUN** — 4096 steps, 4 workers, model saved |
| `evaluate.py` | **RUN** — loads that model, emits the scorecard |
| CUDA kernel | **never compiled.** `cuda_depth_check` is the gate |

First actual training run, maze, truth depth:

```
4096 steps in 24 s, 163 steps/s on 4 workers (~41 /s/worker)
ep_rew_mean -41.2   explained_variance 0.468   approx_kl 0.056
```

Gradients flow and the value function is learning something. That is all a
4096-step run can show; it is a plumbing result, not a capability one.

Round-tripping that model through `evaluate.py`: 8.2 m and 2.4 m travelled on
two held-out maze seeds, no collisions, stopped on ~90 % of steps. **Worse than
random** (25.6 m in `rl_bench`), which is exactly what an undertrained policy
should look like — it has learned to stop rather than to move.

### Extrapolating to a 13600K

41 steps/s/worker on truth depth here. With 16 workers that is ~650 steps/s, so
**10 M steps in ~4 h** on truth depth. Stereo is roughly 3× the render cost, so
8–18 h as estimated in §5 — and the CUDA renderer, if its gate passes, should
take that to 2–5 h.

### What is not done

- **No policy has been trained to convergence.** Nothing here says a learned
  planner beats the classical one; the harness exists to answer that, and the
  question is still open.
- The CUDA kernel is unverified.
- `rl_bench`'s `score` baseline is a re-derivation of the classical weighting on
  normalised features, not a call into `TrajectoryPlanner`'s own argmax. It is
  close but it is not identical, and a headline claim against "the classical
  planner" should use `voxel_sim --traj` numbers rather than this one.

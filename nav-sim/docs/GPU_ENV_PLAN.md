# Moving the environment to the GPU

A plan, not a proposal to start today. It exists so the decision can be made
on numbers, and so the first phase is the one that decides whether the rest is
worth building.

## What we are trying to beat

Measured on this machine, forest, truth depth, one environment:

| camera  | ms/step |
|---------|---------|
| 40x30   |  7.25   |
| 80x60   |  9.42   |
| 160x120 | 15.19   |
| 320x240 | 37.88   |

That fits **~6.8 ms fixed + ~4.05 us per pixel**. At the 160x120 training
default roughly half the step scales with pixels -- depth render plus
`map.integrate` walking rays into three levels -- and half does not.

16 workers on a 20-thread desktop therefore give **on the order of 1000
env-steps/s**, which is 10 M steps in about three hours.

**What the fixed half actually is remains unmeasured**, and one probe came
back backwards: holding the camera at 40x30 and raising the rollout horizon
from 0.3 s to 4.0 s made a step FASTER (5.88 -> 2.07 ms), when marching 210
primitives further should cost more. Something other than march length
dominates -- plausibly work proportional to the number of primitives that
survive, since a longer rollout rejects more of them early. **Phase 0 exists
to answer this**, because a rewrite aimed at the wrong half is wasted.

## The constraint that decides the design: memory, not compute

Per environment, at today's grid sizes -- 6.87 M voxels across three levels:

| storage          | MB/env | envs in 10 GB |
|------------------|--------|---------------|
| fp32 (today)     |  27.5  |   363         |
| fp16             |  13.7  |   727         |
| uint8 quantised  |   6.9  |  1454         |

A 12 GB card holds a few hundred environments at today's precision, which is
not the thousands that make the rewrite worth doing.

Shrinking the fine grid to 160x160x64 (40 x 40 x 16 m at 0.25 m, still far
larger than the ~3.5 m honest marking range and the 11 m carve range) gives
2.23 M cells:

| storage | MB/env | envs in 10 GB |
|---------|--------|---------------|
| fp16    |   4.5  |  2243         |
| uint8   |   2.2  |  4487         |

**So the design is: smaller fine grid, quantised log-odds, map recentred as it
already is.** Precision is the thing being traded, and the equivalence gate
below is what decides whether the trade is acceptable.

## The rule this must not break

`CMakeLists.txt` says only the renderer may leave the CPU, because the map,
the planner and the veto are the code that flies and a second implementation
would drift from what is deployed. That rule is right about the risk and wrong
as a prohibition -- **the project already contains the answer to it**:
`cuda_depth_check` renders the same scene on both paths and fails if they
disagree by more than one voxel.

A GPU environment gets the same treatment, and it is the spine of this plan
rather than a test bolted on at the end:

- `gpu_env_check` steps the CPU and GPU environments through the SAME world,
  seed and action sequence, and asserts:
  - **the action mask is identical, bit for bit, at every step.** This is the
    one that matters most: the mask IS the veto boundary, and a policy trained
    against a different boundary than the one that will veto it in flight has
    learned the wrong interface.
  - the observation agrees within a stated per-channel tolerance, with
    `why == 2` (stopped on UNKNOWN rather than on a surface) treated as exact,
    because unknown-is-not-free is the distinction the architecture rests on.
  - reward and termination agree exactly.
- It runs in ctest and gates the GPU path exactly as `cuda_depth_check` gates
  the renderer. Divergence is a build failure, not a footnote.

## Phases, each with a go/no-go

**Phase 0 -- profile properly (half a day).** Instrument the C++ step into
render / integrate / plan / observe / recentre and print the split. Cheap, and
it settles the horizon anomaly above. *Go/no-go: if the fixed half turns out to
be dominated by something small and fixable on the CPU, fix that and stop.*

**Phase 1 -- one kernel, measured (2-3 days).** Verify the existing
`depth_cuda.cu` with `cuda_depth_check`, then measure its throughput in rays
per second on a real card. Extrapolate the whole-environment cost from that
measured rate rather than from a guess. *Go/no-go: if the projected end-to-end
speedup is under about 5x, stop -- it does not pay for a second implementation
of the flying code.*

**Phase 2 -- the batched environment (2-3 weeks).** N environments in
lockstep, one kernel per stage: render, integrate, march the 210 primitives,
build the observation. Every stage is parallel over (env x pixel) or
(env x primitive x march step), which is the shape a GPU wants. Voxel storage
as above. CUDA in-tree, or NVIDIA Warp if its Python integration proves
simpler -- decided in Phase 1 by which gets the verification harness working
faster, not by preference.

**Phase 3 -- the SB3 seam (2-3 days).** Replace `SubprocVecEnv` with a
vectorised env that steps all N at once and hands SB3 a torch tensor already on
the device, so observations never cross the bus. MaskablePPO takes a custom
VecEnv; nothing about the algorithm changes.

**Phase 4 -- prove it, then trust it (1 week).** `gpu_env_check` green. Then
the result that actually matters: train the same policy on both paths for the
same number of steps and compare on the SAME scorecard the classical planners
are measured in. *If a GPU-trained policy scores worse on the CPU environment
it will fly in, the speedup bought nothing.*

## What stays on the CPU regardless

The deployed aircraft. This is a training accelerator; a Pi 5 has no CUDA and
`frame_source.hpp` is the code that flies. The GPU path exists to produce
weights, and the weights are the only thing that crosses over.

## Honest expectation

An order-of-magnitude estimate, stated as such: roughly 50 k ray-and-march
operations per env-step, times ~1000 envs, against a memory-bound access
pattern, suggests **10-20x** end to end -- 10 M steps in minutes rather than
hours. It could easily be 3x if the voxel access pattern scatters badly, which
is exactly why Phase 1 measures a real kernel before Phase 2 is written.

## Why this is not the next thing to build

The binding constraint today is not throughput. In one session the reward
scaling, the episode length, the world set and the journey randomisation all
changed, and each invalidated the numbers before it. Making runs 20x faster
does not help while the thing being measured is still moving. When the task
stops changing and the question becomes "what does 100 M steps do", this plan
is ready and its first phase costs half a day.

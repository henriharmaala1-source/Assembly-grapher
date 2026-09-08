# First pilot: does the learned planner beat the classical ones?

Not yet. This records the first end-to-end run of the harness, because a
harness with no numbers in it is a claim rather than a result.

## What was run

    kestrel train --workers 3 --steps 150000        # 30 min, 82 steps/s
    kestrel bench --worlds W --seeds 101 104 --steps 600
    kestrel evaluate --model final.zip --worlds W --seeds 101 102 103 104 \
                     --max-steps 600

150 000 steps is **1.5 % of the 10 M the trainer is sized for**, on 4 cores,
at PERFECT DEPTH rather than the honest stereo setting. Everything below is a
pilot: it establishes that the loop runs and that reward moves, and it does
not establish what a converged policy can do.

Seeds 101-104 are held out from training. Both sides use the same environment,
the same seeds, the same step budget and the same depth model, because a
comparison on different settings is worth nothing.

## Training did learn

    ep_rew_mean   -43.2  ->  +33.2      still climbing at the end
    ep_len_mean     253  ->  ~1250      it stopped dying early

That is the first evidence in this project that the RL side learns anything.

## The scorecard did not follow

Mean travel in metres, collisions out of 4 runs.

| planner            | maze         | forest        |
|--------------------|--------------|---------------|
| random             | 37.8   0/4   | 25.2   1/4    |
| freeM              | 43.2   2/4   | **56.8  0/4** |
| goal               | 33.3   0/4   | 23.2   0/4    |
| score (hand-tuned) | 26.2   4/4   | 33.2   1/4    |
| **RL, 150 k**      | 38.1   0/4   | 27.3   **3/4**|

**Maze: a tie with random, not a win.** 38.1 m against 37.8 m over four seeds
is nothing. It is collision-free, which random also is here.

**Forest: clearly worse.** Three collisions in four runs, against freeM's zero
collisions and more than twice the distance. On seed 103 it collides after
0.1 m -- it drives into something essentially at spawn, reproducibly, which is
a specific failure worth chasing rather than a general "needs more steps".

**Nobody reaches the goal. Not one planner, learned or classical, in any run.**
That was already written down as a known open problem and this confirms it is
not an artefact of the classical planners.

## A bug the pilot found, which invalidates the forest column above

Chasing "forest seed 103 collides after 0.1 m" turned out not to be a policy
problem at all. On that seed the RANDOM baseline also travels 0.1 m in 600
steps, and the hand-tuned one collides at 0.1 m. The aircraft was starting
inside a tree.

The forest spawn was hardcoded at (15, 10, 6) regardless of where the trees
landed. Across seeds 101-120, **four of twenty started with less clearance
than the robot's own radius** -- 0.19, 0.38, 0.43 and 0.17 m against a 0.6 m
radius. Seed 103's 0.63 m is why it behaved differently for every planner:
it sat one voxel from the threshold. The maze never had this, because
genMaze returns a corridor start; the forest is now nudged outward until it
has room, and all twenty seeds spawn clear.

This matters more for TRAINING than for the table. About a fifth of forest
episodes handed the policy -50 for a state no action could avoid, from step
zero -- teaching that the opening position is catastrophic rather than
teaching anything about flying. **The pilot policy was trained with that
defect present**, so its forest behaviour cannot be cleanly attributed to
too-few-steps until it is retrained without it.

Re-evaluated after the fix, the same checkpoint still collides in 3 of 4
forest runs (24.8 m). So the spawn bug was not the whole story -- but the
forest numbers above were measured on a broken start and should not be
quoted until a policy is retrained.

## A second pilot, on the fixed spawn: higher reward, worse flying

Same budget, same seeds, same everything except the spawn fix.

| checkpoint                     | final ep_rew_mean | maze         | forest       |
|--------------------------------|-------------------|--------------|--------------|
| pilot 1, spawn bug present     | 33.2              | 38.1   0/4   | 24.8   3/4   |
| pilot 2, spawn fixed           | **36.4**          | 26.3   3/4   | 22.3   3/4   |

The retrain scored HIGHER on the objective it was optimising and flew WORSE
on every column of the scorecard, in both worlds. It went from no maze
collisions to three out of four.

That is the clearest evidence yet that the gap is **reward against metric**,
not training budget: more reward bought fewer metres and more crashes. Adding
steps optimises harder for the thing that is already pointing the wrong way.

Read it with the obvious caveat: one training run per condition, four seeds
each. RL run-to-run variance is large and this is not a controlled experiment.
What it does not support is the comfortable reading -- that fixing the spawn
would improve the policy. It did not.

Also worth knowing when reading maze numbers: every maze seed shares the same
spawn corridor and goal corner, so the opening view is byte-identical across
seeds and a run that dies early cannot distinguish them. The mazes do diverge
-- a fixed policy ends 24.1 / 27.6 / 22.3 / 25.0 m from goal on 101-104 -- but
four maze seeds are less independent than four numbers suggest.

## The forest baselines moved too

The spawn bug was penalising the classical planners, not just the policy:

| forest planner | before fix   | after fix      |
|----------------|--------------|----------------|
| random         | 25.2   1/4   | 34.3   1/4     |
| freeM          | 56.8   0/4   | 57.2   0/4     |
| goal           | 23.2   0/4   | 24.8   0/4     |
| score          | 33.2   1/4   | **46.4   0/4** |

The hand-tuned planner gains 40 % of its distance and loses its collision.
freeM is unchanged because it was already escaping the bad start. So the
earlier reading of this table -- "the hand-tuned planner is worst and hits
something every run" -- was mostly an artefact of spawning inside trees.

Against the corrected baselines the learned policy is clearly behind in the
forest: 22.3 m with three collisions against score's 46.4 m with none.

## Where the reward actually goes

EnvStep now carries the episode total of each reward term, because a scalar
return says a policy improved and cannot say which term it improved. Pilot 2's
checkpoint, evaluator's own settings:

| world  | seed | outcome      | total | progress | coverage | clear | stop  | terminal |
|--------|------|--------------|-------|----------|----------|-------|-------|----------|
| forest | 101  | COLLIDED     |  +5.6 |   47.3   |   10.2   | -1.2  |   0   |  -50     |
| forest | 102  | COLLIDED     |  -3.1 |   40.3   |    8.9   | -1.8  |   0   |  -50     |
| forest | 103  | COLLIDED     | -34.9 |   13.1   |    3.6   | -1.4  |   0   |  -50     |
| forest | 104  | out of steps |  +9.0 |   25.2   |    6.0   | -2.6  | -19.0 |    0     |
| maze   | 101  | out of steps | +14.9 |   15.1   |    4.8   | -4.3  |   0   |    0     |
| maze   | 103  | COLLIDED     | -48.7 |    1.4   |    1.3   | -1.3  |   0   |  -50     |

**Forest 101 crashed and still scored +5.6. Forest 104 survived the whole
episode and scored +9.0.** Crashing is worth almost as much as flying.

The reward already does what it should in shape: reward closing distance,
punish collisions hard. The fault is SCALE, and it is not scale-invariant
across worlds.

- Progress telescopes to (start distance - end distance), so it is banked
  permanently as the aircraft approaches and is never given back.
- The forest goal is 175 m from the spawn. Closing 50 m therefore pays +50,
  which exactly cancels rCollide. **Beyond 50 m of progress, crashing is
  strictly profitable.**
- The maze goal is ~35 m away, so progress can never exceed ~35 and the -50
  genuinely dominates. Maze collisions score -48.7; forest collisions score
  +5.6.

The same weights mean opposite things in the two worlds, which is why the
policy behaves so differently in them. -50 is not "punish hard"; it is
"punish hard in the maze and barely at all in the forest".

Forest 104 shows the other half: -19.0 of stop penalty, so it spent roughly
380 of 600 steps either standing still or selecting masked actions.

### The fix this implies

Make progress scale-free -- pay a fraction of the START distance rather than
raw metres, so the most an episode can earn from progress is fixed and the
collision penalty dominates in every world by construction. That is a change
to the objective itself, so it is written down here rather than made
unilaterally: every number in this document is measured against the current
reward and would need re-measuring.

## What this says

Reward went up by 76 points and the scorecard did not move. Either the reward
is not measuring what the scorecard measures -- progress and coverage against
travel, collisions and arrival -- or 1.5 % of the training budget is simply
too early to ask. Both are worth separating before spending 18 hours on a
full run, and the cheapest test is a longer pilot on the forest alone, since
that is where the policy is losing.

Reproduce with the commands at the top; the checkpoints and the TensorBoard
log are written to whatever --out names, printed as an absolute path.

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

## What this says

Reward went up by 76 points and the scorecard did not move. Either the reward
is not measuring what the scorecard measures -- progress and coverage against
travel, collisions and arrival -- or 1.5 % of the training budget is simply
too early to ask. Both are worth separating before spending 18 hours on a
full run, and the cheapest test is a longer pilot on the forest alone, since
that is where the policy is losing.

Reproduce with the commands at the top; the checkpoints and the TensorBoard
log are written to whatever --out names, printed as an absolute path.

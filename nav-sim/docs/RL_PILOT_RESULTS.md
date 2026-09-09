# RL pilots

# Open worlds were paying more for the same flying

With six world styles in the training mix, the risk is that the big open ones
set the weights and the tight ones become noise. Progress telescopes to
(start - end) distance, so in raw metres a 340 m city journey pays far more
than a 30 m maze for identical quality of flying. Measured, same planner,
same settings, mean episode return per world:

| world    | start dist | return BEFORE | return AFTER |
|----------|-----------:|--------------:|-------------:|
| road     |     145 m  |     **66.3**  |      44.7    |
| city     |     340 m  |       29.2    |      15.8    |
| culdesac |     151 m  |       28.6    |      26.0    |
| forest   |      78 m  |       10.0    |      19.2    |
| maze     |      30 m  |        9.7    |      24.4    |
| corridor |      97 m  |      **8.7**  |       9.5    |

Progress is now paid as a FRACTION OF THE JOURNEY rather than in metres, so
the most an episode can earn from it is the same number in every world.

  spread, biggest/smallest return   7.6x  ->  4.7x
  correlation of world SIZE with return   +0.37  ->  -0.10
  city (the largest world) by return      2nd of 6  ->  5th of 6

The correlation is the number that matters. Before, a bigger world meant a
bigger return, which is the poisoning mechanism: PPO normalises advantages
per batch, but a world contributing systematically larger advantages still
steers the update. After, size does not predict return at all, and the
largest world pays less than the smallest.

The residual 4.7x is DIFFICULTY, not scale -- road is open and the planner
does well there, corridor is tight and it does badly. A world where the
policy genuinely flies better should return more; that is the signal, not the
bias.

The terminals follow the same rule: rCollide and rGoal scale against
(wProgress * progressScaleM), which is now world-independent, so a crash
costs the same relative to the best possible episode wherever it happens.


# Trained from zero with no safety veto

300 k steps, 3000-step episodes, the geometric veto OFF so the policy can
select primitives that fly into things and must learn avoidance from the
-collide penalty. Scored in the same world it trained in (veto off), forest.

Training curves: episode length **69 -> ~1000 steps**. It began by crashing
within seven seconds of flight and ended surviving roughly a hundred. Reward
went -135 -> -319 -> -201: worse before better, which is what learning to
stop dying looks like when dying is the cheap option early on.

Per checkpoint, on TWO seeds:

| checkpoint | collisions | mean travel |
|------------|------------|-------------|
| 49 k       | 2/2        |   8.5 m     |
| 99 k       | 2/2        |   5.5 m     |
| 149 k      | 2/2        |  10.5 m     |
| 199 k      | 2/2        |  11.5 m     |
| 249 k      | 2/2        |  10.4 m     |
| 299 k      | **0/2**    | **184.4 m** |

That last row looked like a phase transition. **On eight seeds it is not.**
The same final checkpoint, eight seeds, everything veto-off:

| planner            | collisions | mean travel |
|--------------------|------------|-------------|
| freeM              | **1/8**    | **280.0 m** |
| policy (from zero) | 6/8        |  56.8 m     |
| score (hand-tuned) | 8/8        |  36.9 m     |
| goal               | 7/8        |  15.1 m     |
| random             | 8/8        |  14.0 m     |

What is real: the policy learned partial avoidance from nothing. It travels
**four times further than random** and crashes less often than the hand-tuned
planner, having started at 8.5 m and 2/2 crashes. Over 300 k steps that is
genuine progress from a standing start.

What is not real: reliable navigation. Six crashes in eight runs is not a
planner anyone would fly, and the 0/2 that suggested otherwise was two lucky
seeds. Two seeds cannot distinguish 0 % from 75 %.

The result that matters most here is freeM's. **A four-line heuristic --
steer where the map has confirmed the most free space -- crashes once in
eight runs with no veto at all, and travels five times further than the
learned policy.** Whatever the learned planner is eventually worth, it has
not yet earned its place against that.


# The learned policy does not survive a long run

Scored at 3000 steps instead of 600, forest, same checkpoint (pilot 3, which
was TRAINED on 1500-step episodes):

| planner | mean travel | collisions | best closest approach |
|---------|-------------|------------|-----------------------|
| freeM   | **285.3 m** | **0/2**    | **9.7 m**             |
| score   | 146.4 m     | 0/2        | 79.9 m                |
| policy  | 139.0 m     | **2/2**    | -                     |
| random  | 111.4 m     | 0/2        | 144.6 m               |
| goal    |  88.6 m     | 1/2        | 98.0 m                |

At 600 steps this policy was the best collision-free planner in the forest.
At 3000 it collides in **every** run and is the least reliable thing on the
list -- worse than random, which never hits anything.

That is the answer to "we need long reliable navigation": it is currently
neither trained for nor, until now, tested for. The policy was trained on
1500-step episodes and evaluated on 600, and it is safe over roughly the
horizon it saw and not beyond it. Nothing about 600-step numbers predicts
3000-step behaviour, and the earlier table in this document should be read as
short-range results.

freeM is the long-range planner to beat: 285 m, no collisions, and the only
one that has come within 10 m of a goal.

# Why nothing has ever reached a goal: the episode ended first

The forest goal is 175 m from the spawn. Training and evaluation both used
1500-step episodes. Measured with the best classical planner, at truth depth:

| planner | max steps | travel  | end-dist | closest | at step |
|---------|-----------|---------|----------|---------|---------|
| freeM   | 1500      | 143.0 m |  74.5 m  | 74.5 m  | 1500    |
| freeM   | 4000      | 380.7 m |  52.4 m  |  9.7 m  | 2464    |
| score   | 4000      | 187.6 m | 141.2 m  | 141.2 m |  439    |
| goal    | 4000      |  79.1 m |  98.0 m  |  98.0 m | 1833    |

At 1500 steps freeM is **still closing when the episode is cut off**. It needs
about 2500 steps to get within 10 m. Every training episode was therefore
truncated before arrival was even possible: the goal bonus was unreachable
dead code, no policy has ever experienced arriving, and "nobody reaches the
goal" was a property of the episode budget rather than of the planners.

Defaults are now 3000 steps for both training and evaluation.

Two things this also exposed:

**Closest approach is now reported.** freeM reaches 9.7 m at step 2464 and
ends 52.4 m away -- it arrives and then wanders off. The final distance alone
calls that a navigation failure when it is a termination failure, and those
want opposite fixes. `evaluate` and `bench` both print closest and the step it
happened.

**Goal tolerance is 3.0 m and the best planner only manages 9.7 m.** Nothing
has yet demonstrated it can close the last ten metres, so arrival is still
unproven -- just no longer impossible by construction.

A hypothesis this killed, recorded because it was wrong and expensive:
the voxel map is 60 m across and centred on the aircraft, so a 175 m journey
looked like it must run off the edge of the map. It does not. rl_env calls
VoxelMap::recentre every step, the grid scrolls with the vehicle, and a 380 m
run completed with zero collisions.


**Everything below the "revamped reward" section is measured against the OLD
reward and is historical.** It is kept because the reasoning that led to the
change is the useful part.

# Pilot 3: the revamped reward fixes the forest and breaks the maze

wProgress 1.0 -> 2.0, and terminals scaled to (start distance x wProgress)
so a crash cannot be bought with metres. 150 k steps, same budget as before.

| world  | policy              | mean travel | collisions |
|--------|---------------------|-------------|------------|
| forest | RL, old reward      | 22.3 m      | 3/4        |
| forest | **RL, new reward**  | **49.9 m**  | **0/4**    |
| forest | score (hand-tuned)  | 46.4 m      | 0/4        |
| forest | freeM               | 57.2 m      | 0/4        |
| forest | random              | 34.3 m      | 1/4        |
| maze   | RL, old reward      | 26.3 m      | 3/4        |
| maze   | **RL, new reward**  | **25.0 m**  | **4/4**    |
| maze   | random              | 37.8 m      | 0/4        |
| maze   | score (hand-tuned)  | 26.2 m      | 4/4        |

**The forest is fixed.** Zero collisions, more than double the distance, and
it now edges the hand-tuned planner (49.9 m against 46.4 m) with only freeM
further ahead. That is the first time the learned planner has beaten a
classical one on anything.

**The maze got worse**, from three collisions to four. The mechanism is
visible in the decomposition: on maze 102 and 103 it reaches 10-11 m from the
goal, having banked 42 of progress, and then clips a wall. Doubling wProgress
doubled the pull toward the goal; in open forest that is free distance, in a
4 m corridor it is aggression with nothing opposing it.

Nothing opposes it because the clearance term is now tiny by comparison: over
a whole maze episode it totals -1.6 to -3.3, against progress paying 40+.
wClear was tuned against a progress weight half this size and was not scaled
with it.

Training curve, for the record: ep_rew_mean -71.8 -> -54.9, ep_len 831 ->
1230. The absolute reward is lower than the old pilots' simply because a
crash now costs 10x more; comparing reward across different reward functions
is meaningless, which is why the table above compares the scorecard instead.

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

---

# The 15 M-step run: the first policy that arrives, and the first that regresses

Everything above was written when no run in this project had ever reached a
goal. That is no longer the state of the world, and this section supersedes
the "nobody arrives" conclusion rather than amending it.

A 15 M-step run with `--max-steps 3000` over six worlds reached goals at
scale. Per-world rolling goal rate over the last ~600 k steps:

| world    | goal_rate   | collision_rate | journey |
|----------|-------------|----------------|---------|
| road     | 0.98 - 1.00 | low            | 180 m   |
| maze     | 0.75 - 0.93 |                | ~40 m   |
| corridor | 0.82 - 0.90 |                | ~89 m   |
| forest   | 0.58 - 0.67 |                | 175 m   |
| culdesac | 0.45 - 0.63 |                | 155 m   |
| city     | 0.10 - 0.17 | 0.317 -> 0.483 | 368 m   |

Overall 0.62 - 0.70.

## It went backwards over the last 600 k steps

| metric           | 14.34 M | 14.97 M |
|------------------|---------|---------|
| collision_rate   | 0.169   | 0.233   |
| goal_rate        | 0.700   | 0.622   |
| ep_rew_mean      | ~180    | 108-139 |
| ep_len_mean      | 2.2e3   | 1.45e3  |

Shorter episodes with more collisions and fewer goals is one story, not
three: it is dying earlier.

**Raising the collision penalty was considered and rejected on the
arithmetic.** It is already `max(50, 1.5 * wProgress * progressScaleM)` = 300
against a best-possible episode of +200 progress and +100 for arriving, so
crashing already costs one and a half times everything an episode can earn and
cannot be bought with metres in any world. And the terminals do not account
for the loss: 0.064 more collisions at 300 is -19, 0.078 fewer goals at 100 is
-8, so -27 of a -60 drop. The rest is progress and coverage, which matches the
episode length falling by a third. The policy is travelling less, not merely
crashing more. A larger terminal would also raise the variance of an advantage
estimate that a -300 cliff already dominates.

## What was wrong, and what changed

**1. Nothing annealed.** `learning_rate` was a hard 3e-4 and `ent_coef` a hard
0.01 for the whole run, with no `target_kl`. At 15 M steps a fixed entropy
bonus is a standing payment to stay random, and a fixed learning rate lets one
bad batch move the weights as far at 15 M as at 15 k. Both now start higher and
decay to a tenth over `--anneal` steps (`--explore` sets the entropy start,
default 0.02), and `--target-kl` (default 0.02) abandons an update that moves
the policy too far. `--anneal 0` restores the old constant behaviour.

**2. The near-miss penalty was not scale-free.** The collision terminal is a
cliff that arrives once, after the mistake; `wClear` is the gradient, and the
only avoidance signal that exists before contact. Progress was made scale-free
and this was not, so against progress it was ~5x weaker in a tight world than
an open one -- the wrong way round, since tight is where clearance is the whole
problem. Now scaled by the same `progressScaleM / startDist`; verified
directly, forest `r_clear` -0.075 -> -0.043 (x0.571 = 100/175) and maze -0.862
-> -2.220 (x2.576 = 100/38.8). `--raw-clear` restores the old term.

**3. The city goal was beyond what an episode can fly.** Not a shaping
problem. Opposite corners of a 300 m world is a 367.7 m straight line, and
straight line is a lower bound on the path. Measured with the best classical
planner over 3000-step episodes, path length saturates at 260-278 m in every
world:

```
world      travel in 3000 steps (freeM, 6 seeds)
city       96.8 / 126.5 / 260.7 / 268.1 / 278.4
road       153.0 / 247.2 / 253.9 / 258.1 / 259.6 / 261.8
culdesac   259.9 / 262.7 / 264.8 / 264.8 / 267.4 / 268.6
```

So the city goal was out of reach by a wide margin however well anything flew,
and the goal-rate ordering across all six worlds tracks journey length almost
exactly. This is the same failure as the 1500-step episode cap that put the
forest goal out of reach, and it hid the same way -- as "the policy cannot
learn this world" for millions of steps.

The city journey is now 183.8 m, matching road. `--vary-goal` had the same
fault for the same reason: minimum separation was six tenths of the *box*
diagonal, 229 m in the city; it is six tenths of the world's own nominal
journey now.

`train` checks this before every run and prints the journey, the steps it needs
at 0.09 m/step, and TOO FAR where it does not fit. Run against the old
1500-step cap the check independently reproduces the known finding: forest
needs 1944 steps.

## One column to stop reading

`best_closest_m` sat at 2.91-2.93 in every world for the whole log. `goalTolM`
is 3.0, so it saturates the moment goals are reached and carries no information
after that. It was useful when nothing ever arrived. `goal_rate` is the column
now.

## Not measured

Whether any of this produces a better policy. These are changes to the
objective and the task, so every number above is against the old ones and none
of it transfers. The immediate practical step for the run that produced this
log is unchanged: an earlier checkpoint probably scores better than the final
one, and `kestrel evaluate --run <dir> --progress --baselines` says which.

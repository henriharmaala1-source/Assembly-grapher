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

---

# A paired A/B of the horizon change, and a critic fix that failed

Three 150k-step runs on maze, `--seed 7`, identical in everything but the
settings named. The seed fixes initial weights, PPO's sampling and each worker's
stream of worlds, so these are comparisons rather than samples.

| | OLD | NEW | FIX |
|---|---|---|---|
| gamma / lambda | 0.995 / 0.95 | 0.999 / 0.996 | 0.999 / 0.996 |
| value horizon | 200 steps | 1000 | 1000 |
| clearance | absolute | scaled | scaled |
| returns | raw | raw | normalised + clip_vf 0.2 |

## Held out, 18 episodes each (maps 101-106, three repeats, never trained on)

| | OLD | NEW | FIX |
|---|---|---|---|
| arrived | 2 | 0 | 1 |
| collided (blind) | 6 | 0 | 0 |
| orbited | 7 | 14 | 12 |
| still closing | 3 | 4 | 5 |
| mean travel | 46.2 m | 60.3 m | 63.1 m |
| **closing fraction** | **0.51** | **0.49** | **0.49** |
| collision rate | 0.33 | 0.00 | 0.00 |

## What this says

**The horizon change bought safety and sold arrival.** Collisions went 6 -> 0;
arrivals went 2 -> 0; orbiting went 7 -> 14. That is the signature of a policy
that learned "do not die" and never learned "get there" -- which is what you
expect when the collision terminal is immediate and locally learnable while the
goal sits beyond what the critic can estimate.

**Nothing navigated better.** Closing fraction is 0.49-0.51 in all three. The one
column that measures "did it get meaningfully nearer" did not move. Whatever is
limiting this policy at 150k steps, none of these three settings is it.

**The critic fix failed, and the reasoning behind it was wrong.** It was added
because explained_variance fell from +0.705 to +0.286 when the horizon grew.
Normalising the return moved it to +0.250 and made the worst single update go
from -8.75 to -75.60. explained_variance is `1 - Var(y-yhat)/Var(y)` and is
scale-invariant by construction: rescaling targets could never have changed it.
What it fixed was `value_loss`, ~500 -> ~0.05, which was only ever the units.
It is now off by default and kept as `--norm-reward`.

**The comparison that motivated it was also unsound.** explained_variance is not
comparable across gammas -- a 200-step return is dominated by the near future and
is easier to predict than a 1000-step one, so +0.705 partly means "easier
question", not "better critic". Only runs sharing a gamma can be compared on it,
and there (NEW vs FIX) the fix lost.

## Method notes, because two readings were wrong before this one

The training rolling scorecard said OLD won; the three held-out seeds said NEW
won; 18 held-out episodes say they are doing different things and neither
navigates better. The rolling scorecard is measured on TRAINING maps, so it
cannot see generalisation, and n=3 produced a 3/3 collision rate for FIX that
did not replicate at all on 18 episodes (0.00). Neither number was wrong; both
were read for more than they could carry.

---

# The observation was missing the one thing a navigator needs

`o[1]` was a bit-exact copy of `o[0]` -- `e.clear` is `freeLen/(vMax*horizonS)`
and `o[0]` is `freeM/(3*horizonS)` with `vMax` exactly 3. Measured over 300
steps both reported min 0.000, max 0.714, mean 0.406, std 0.180. 210 of 1914
inputs carried nothing.

Worse, the only goal channel the policy had was `o[3] = goalErr`, which is
`|wrap(endAz - goalAz)|/180 + |endEl - goalEl|/90 * 0.5` -- an ABSOLUTE value.
A primitive 30 degrees left of the goal and one 30 degrees right were
numerically identical. There was no signed left/right gradient anywhere in the
observation. And `g[1]`,`g[2]` gave the goal bearing in the WORLD frame while
every action is body-relative, with heading absent entirely.

`o[1]` now carries the signed bearing; `g[1]`,`g[2]` are relative to heading.

## What it did, paired against the identical configuration (--seed 7, 150k)

| | runNEW (old obs) | runOBS (new obs) |
|---|---|---|
| training crash rate | 35.9% | **25.1%** |
| training goals/window | 3.00 | **4.13** |
| held-out orbited (of 18) | 14 | **6** |
| held-out closing fraction | 0.49 | **0.40** |
| held-out collisions | 0 | 4 |

The mechanism worked: orbiting more than halved, which was the specific failure
it targeted. The outcome did not follow -- closing fraction went the wrong way
on held-out maps while improving on training maps. At one seed and 150k steps
that cannot be separated from noise.

The change is kept anyway, on grounds independent of this run: a duplicated
input carries zero information and a world-frame bearing with no heading cannot
be acted on. Both are defects whatever this A/B had said.

# The bar, and we are not over it

18 identical held-out episodes (maze, maps 101-106, three repeats each), the
policy and all four classical planners under the same build:

| planner | arrive | crash | orbit | travel | closest | closed |
|---------|--------|-------|-------|--------|---------|--------|
| policy  | 0 | 4  | 6  | 42.3 m | 22.2 m | 0.40 |
| freeM   | 0 | 0  | 18 | 81.6 m | 20.2 m | 0.48 |
| goal    | 0 | 3  | 3  | 32.4 m | 23.5 m | 0.38 |
| score   | 0 | 18 | 0  | 26.9 m | 19.0 m | 0.52 |
| random  | 0 | 2  | 16 | 51.6 m | 25.7 m | 0.31 |

**The learned policy does not beat freeM.** freeM closes more of the journey and
never collides. At 150k steps the policy is not over the bar this project sets.

**Nothing arrives in 90 episodes.** The maze goal needs 573 steps inside a
1000-step budget, so it is reachable; nothing reaches it on a held-out map.

`score` colliding 18/18 is not a regression: `kestrel bench` measured it at 10
collisions in 10 runs before any of this work, which independently confirms the
`o[1]` -> `o[0]` repointing preserved that baseline exactly.

## What five configurations could not move

| run | gamma | observation | closed |
|-----|-------|-------------|--------|
| OLD | 0.995 | old | 0.51 |
| NEW | 0.999 | old | 0.49 |
| FIX | 0.999 + normalised returns | old | 0.49 |
| OBS | 0.999 | new | 0.40 |
| freeM | classical | - | 0.48 |

Closing fraction sits between 0.40 and 0.52 for everything tried, the classical
planners included. At 150k steps on four cores that looks like the ceiling of
the experiment rather than a property any one setting controls -- the 15M-step
run needed two orders of magnitude more experience before maze goal rate reached
0.75-0.93. Nothing here licenses a claim about which discount or which
observation makes a better pilot; what it licenses is that the apparatus now
measures the question, and that the answer is not yet yes.

# Is there a route at all? Asked for the first time, and the answer is yes

The map panel for maze seed 101 showed fifteen held-out episodes flown by five
completely different planners -- openness-seeking, goal-seeking, weighted-score,
uniform random and a learned network -- all running the same corridor, turning
at the same wall and stopping, with the goal ring somewhere none of them went.
Five objectives do not agree on a wrong turn by coincidence, so the obvious
hypothesis was a fourth unreachable goal: this project had already shipped three
(the 1500-step forest cap, the 368 m city goal, the 600-step bench default).

Nothing had ever checked. `journey_fit` compares straight-line distance against
the step budget, and straight-line distance is a lower bound on the path that
says nothing whatever about whether a path exists.

`VoxelEnv::goalPathM` now answers it: Dijkstra on a 1 m lattice over cells with
at least `robotR` of true clearance, from the spawn to the goal.

| world | journey | route | detour | steps needed | budget |
|-------|---------|-------|--------|--------------|--------|
| forest | 175 m | 185 m | 1.06x | 1944 | 3000 |
| maze | 44 m | 46 m | 1.04x | 489 | 3000 |
| corridor | 96 m | 106 m | 1.10x | 1061 | 3000 |
| city | 186 m | 209 m | 1.13x | 2065 | 3000 |
| road | 180 m | 180 m | 1.00x | 2000 | 3000 |
| culdesac | 155 m | 182 m | 1.17x | 1722 | 3000 |

**THE HYPOTHESIS IS REFUTED. Every goal is reachable**, and the detours are
small. The ordering is a check on the checker: road is a straight corridor and
comes out at exactly 1.00x, the cul-de-sac is a trap needing backtracking and is
the worst at 1.17x, the city is a grid of doglegs at 1.13x. It also re-validates
the earlier city fix against a PATH rather than a straight line: 209 m is 2322
steps at cruise, inside the 3000-step budget.

That makes the flat closing fraction harder to explain, not easier. On maze the
route is 1.04x the straight line -- the direct way is essentially open -- and
five planners still converge on a wrong turn and stop 20 m short. Whatever is
wrong is not the task being impossible, and it is not the discount, the critic
scaling, or the observation frame, all of which were tried. The next place to
look is the sensing and the primitive library: whether `sphereClear` admits the
corridor widths these mazes actually have.

Two bugs in the check itself, both caught before it was believed:

  * It first counted lattice MOVES and multiplied by cell size, with
    26-connectivity, so a diagonal costing sqrt(2) or sqrt(3) cells was charged
    1. It reported a 33.0 m path between points 44.1 m apart -- impossible on
    its face, which is the only reason it was visible. Replaced with Dijkstra
    over true Euclidean edge costs.
  * The "lattice too large" guard returned -1, the same value as "no route",
    which would have printed NO ROUTE for the 300 m city: a false alarm of
    exactly the kind this check exists to prevent, raised by the check itself.
    It returns -2, and the printers say "-" rather than "NO ROUTE".

---

# Safe travel: the objective was the problem, not the tuning

The goal in these worlds is scaffolding. What is wanted is an aircraft that
keeps flying, gets away from where it started, covers ground, and does not hit
anything. `--objective range` pays for net displacement from the spawn and for
newly visited cells, and the goal no longer ends an episode.

Verified against the degenerate solution BEFORE training on it: a hoverer that
always picks the hardest-turning admissible primitive banks 21.6 m of path at
76x its displacement across 2 cells, and scores **-33.99** against freeM's
+132.43 and a goal-seeker's +96.60. Circling is not merely unrewarded, it is
negative.

## Training, same --seed 7 and budget as every goal-objective run

| run | objective | travel | crash |
|-----|-----------|--------|-------|
| runOLD | goal, gamma .995 | 53.8 m | 19.0% |
| runNEW | goal, gamma .999 | 48.5 m | 35.9% |
| runOBS | goal + observation fix | 44.1 m | 25.1% |
| **runRANGE** | **safe travel** | **54.8 m** | **12.4%** |

The first cleanly improving curve in the series: travel 33.9 -> 56.4 m and crash
64% -> 12% over 150k steps. Four commits of discount, critic and observation
tuning moved these numbers around inside noise; changing what the policy is paid
for moved both at once, on the first try.

## Held out, 18 episodes, every planner scored under the same objective

| planner | net disp | cells | crash | blind | travel | m/step | loops |
|---------|----------|-------|-------|-------|--------|--------|-------|
| policy | 16.7 m | 36 | 1/18 | 1 | 49.2 m | 0.052 | 2.9x |
| policy cov .45 | 18.5 m | 35 | 11/18 | 11 | 41.1 m | 0.063 | 2.2x |
| freeM | 9.9 m | 64 | 0/18 | 0 | 81.6 m | 0.082 | 8.2x |
| random | 9.8 m | 32 | 2/18 | 2 | 51.6 m | 0.054 | 5.3x |
| goal | 14.9 m | 20 | 3/18 | 3 | 32.4 m | 0.038 | 2.2x |
| score | 18.6 m | 30 | 18/18 | 18 | 26.9 m | 0.081 | 1.4x |

It beats every goal-trained policy on all three columns at once -- net 15.5 ->
16.7 m, cells 26 -> 36, collisions 4 -> 1 -- and against freeM it is a split:
69% further from the spawn and circling a third as much, but 36 cells against
64.

## Tripling the coverage reward: refuted

Collisions went 1/18 -> 11/18 and cells did not move (36 -> 35). New cells are
at the frontier, the frontier is unmapped, so paying more for novelty paid the
policy to fly blind.

## EVERY COLLISION IS A BLIND ONE

The `blind` column equals the `crash` column in every row. Across 108 held-out
episodes and five planners, **not one collision was into a cell the map had
already marked OCCUPIED.** The geometric veto has never failed -- it simply
cannot veto what nothing has seen. Collisions here are a sensing problem, not a
policy-preference one, and without `hit_unknown` splitting the two classes this
would read as "the policy sometimes flies into walls", which is false.

And speed is not the lever. `score` flies 0.081 m/step and collides 18 times in
18; freeM flies 0.082 m/step and collides never. Identical speed, opposite
safety. What separates them is that freeM maximises CONFIRMED-FREE path length:
it goes fast only where it has looked.

The coverage gap decomposes the same way. Cells per metre of path are nearly
equal -- freeM 0.78, policy 0.73 -- so the gap is not routing, it is path
length, and path length is speed. But the policy cannot simply be made faster,
because `score` is exactly that and dies every time.

`--seen` charges for the fraction of the chosen primitive's rollout that was not
confirmed free, which is the one thing the policy is never paid to care about.
A run at 0.30 is training.

## --seen 0.30: right idea, weight 10x too big

| held out, 18 eps | net | cells | crash | travel | steps |
|------------------|-----|-------|-------|--------|-------|
| plain range | 16.7 m | 36 | 1/18 | 49.2 m | 948 |
| seen 0.30 | 17.4 m | 32 | 11/18 | 42.4 m | 738 |
| coverage 0.45 | 18.5 m | 35 | 11/18 | 41.1 m | 653 |
| freeM | 9.9 m | 64 | 0/18 | 81.6 m | 1000 |

The reward decomposition says why:

```
r_progress   +97.81      displacement
r_coverage   +13.76
r_clear     -142.49      clearance + the seen charge
r_terminal  -183.33
```

The seen penalty is LARGER THAN THE ENTIRE DISPLACEMENT REWARD it was meant to
support. At 0.30 per step over ~738 steps, with most rollouts ending in unknown
at a 3.5 m sensing range, it accumulates ~133 points of drag against a maximum
earnable displacement of 200. It stopped being a discriminator between
primitives and became a cost of living -- and a large cost of living makes
ending the episode early relatively cheaper. Steps fell 948 -> 738 and
collisions went 1 -> 11, which is the classic shape of a per-step cost set high
enough that dying looks attractive.

The idea is not refuted; the weight is wrong by about an order of magnitude.

IT ALSO EXPOSED A FLAW IN THE INSTRUMENT. tSeen was folded into the clearance
accumulator, so the one diagnostic built to say WHICH term drove an episode was
conflating two of them -- and it mattered on the first use: -142.49 reads as a
clearance problem until you separate it and find almost all of it is the new
term. rSeen now has its own column in EnvStep, the bindings, the gym info dict
and report.csv.

## --seen at 0.03: still worse than zero, and monotonic in the weight

| held out, 18 eps | net | cells | crash | travel | loops | steps |
|------------------|-----|-------|-------|--------|-------|-------|
| **plain range (seen 0)** | 16.7 m | 36 | **1/18** | 49.2 m | 2.9x | 948 |
| seen 0.03 | 17.7 m | 34 | 5/18 | 48.0 m | 2.7x | 781 |
| seen 0.30 | 17.4 m | 32 | 11/18 | 42.4 m | 2.4x | 738 |
| coverage 0.45 | 18.5 m | 35 | 11/18 | 41.1 m | 2.2x | 653 |
| freeM | 9.9 m | 64 | 0/18 | 81.6 m | 8.2x | 1000 |

Collisions order strictly by the weight: 1 -> 5 -> 11 as --seen goes 0 -> 0.03
-> 0.30. Lowering it recovered most of the damage but never beat ZERO. A merely
mistuned term would show an interior optimum; monotonic degradation says the
term is harmful at every weight tried, which is a different conclusion from the
one the previous section reached, and supersedes it.

The likely reason -- stated as a hypothesis, not a finding -- is that the policy
already receives confirmed-free length as o[0] on all 210 primitives. Paying it
again in the reward adds no information and only tilts the objective away from
displacement. freeM wins on safety by CHOOSING on that signal, not by being
taxed on it.

--seen stays in the code at default 0, with these numbers, the same way
--norm-reward did.

## What is locked in

The winning configuration is the DEFAULT, so `kestrel train` with no flags
reproduces it: objective range, coverage 0.15, seen 0, gamma 0.999, lambda
0.996, explore 0.02, target-kl 0.02, no reward normalisation, 3000-step
episodes.

| | net disp | cells | crash | loops |
|---|---|---|---|---|
| **learned policy** | **16.7 m** | 36 | 1/18 | **2.9x** |
| freeM | 9.9 m | **64** | **0/18** | 8.2x |

69% further from the spawn than the classical planner, circling a third as much,
one collision against none, and less ground covered. On "displacement and ground
covered, hovering is bad" that is a split decision rather than a win -- but it
is the first configuration in this project to beat freeM on anything that
matters.

THREE REWARD-WEIGHT EXPERIMENTS, THREE LOSSES; one objective change, one large
win. The structural choice of what to pay for was worth more than every weight
inside it.

A LAST BUG, found while checking that claim. --gae-lambda defaulted to 0.98 on
the command line while the GUI's credit-horizon stepper defaults to 200 steps
and emits 0.996 on every run. A bare `kestrel train` was therefore training with
a 48-step credit horizon while the RUN button used 200, and every measured run
here passed 0.996 explicitly. The window and the command line are not allowed to
drift; this one had, silently, in the one place where it would have made the
documented results irreproducible from the CLI.

---

# The endurance test: it can fly forever, by not going anywhere

20,000-step episodes -- a 1800 m ceiling, twenty times any episode flown here
before -- across all six worlds, two seeds each, on the safe-travel policy.
Under this objective an episode ends ONLY on a collision or the cap, so a large
cap measures range directly.

| world | seed | steps | travel | net | cells | loops | outcome |
|-------|------|-------|--------|-----|-------|-------|---------|
| forest | 101 | 20000 | 821.7 m | 15.0 m | **19** | **54.8x** | survived |
| maze | 102 | 20000 | 862.6 m | 19.5 m | 25 | 44.2x | survived |
| maze | 101 | 20000 | 863.6 m | 30.6 m | 50 | 28.2x | survived |
| corridor | 101 | 20000 | 900.1 m | 52.1 m | 145 | 17.3x | survived |
| culdesac | 101 | 20000 | 153.1 m | 69.2 m | 136 | 2.2x | survived |
| city | 101 | 20000 | 53.3 m | 16.8 m | 48 | 3.2x | survived |
| city | 102 | 20000 | 47.5 m | 24.5 m | 41 | 1.9x | survived |
| culdesac | 102 | 10189 | 640.1 m | 180.1 m | **516** | 3.6x | BLIND HIT |
| corridor | 102 | 2114 | 106.7 m | 1.9 m | 42 | 55.7x | BLIND HIT |
| road | 102 | 771 | 49.2 m | 21.6 m | 43 | 2.3x | BLIND HIT |
| forest | 102 | 769 | 50.0 m | 28.9 m | 46 | 1.7x | BLIND HIT |
| road | 101 | 198 | 13.1 m | 3.6 m | 14 | 3.6x | BLIND HIT |

**Seven of twelve survived the full 20,000 steps, and they survived by
hovering.** forest/101 flew 821 metres and covered NINETEEN distinct cells.

## When did it stop finding new ground?

Step of the last new cell, against 19,999:

| survivors | last new cell | | deaths | last new cell | died at |
|-----------|---------------|---|--------|---------------|---------|
| forest 101 | 360 | | corridor 102 | 2114 | 2114 |
| maze 102 | 410 | | culdesac 102 | 10180 | 10189 |
| city 102 | 580 | | forest 102 | 760 | 769 |
| city 101 | 690 | | road 102 | 770 | 771 |
| maze 101 | 810 | | road 101 | 198 | 198 |
| culdesac 101 | 2330 | | | | |
| corridor 101 | 19560 | | | | |

**Every one of the five deaths happened within ~10 steps of discovering a new
cell.** All five, all blind. And five of seven survivors stopped exploring
inside the first 800 steps, then orbited for the remaining 96% of the flight.

The policy has exactly two behaviours: explore briefly and die, or stop
exploring and live forever.

## And it is playing optimally

- coverage pays `0.15 * (100/44)` = **+0.34** per new cell in the maze
- collisions run at 5 per 1125 new cells = **0.0044** per cell
- a collision costs **300**

One new cell is worth +0.34 and costs `0.0044 * 300 = 1.33` in expectation. Net
**-0.99 per cell**. Hovering is worth 0. Zero beats negative, so it parks. The
reward asked for this.

This also makes the "mean free path 912 m per collision" figure meaningless:
most of those metres are zero-risk hovering metres. The honest hazard is **one
collision per ~225 new cells**, and it applies only while exploring.

So "how far can it go" has two answers: indefinitely, if it does not go
anywhere; and about 225 new cells of real exploration before something nothing
had seen kills it.

## Two fixes, and they work by opposite mechanisms

Scored on a hard-turning hoverer against freeM over 1500 steps:

| variant | hoverer | freeM | gap |
|---------|---------|-------|-----|
| plain range | -90.5 | 178.1 | 269 |
| **--far 1.0** | -90.5 | **198.7** | **289** |
| --revisit 0.05 | -165.4 | 108.6 | 274 |
| both | -165.4 | 129.2 | 295 |

`--far` widens the gap by REWARDING good behaviour: the hoverer is untouched
because it never gets far enough to earn the bonus, while freeM gains 20.

`--revisit` widens it by PUNISHING EVERYTHING. It costs the hoverer 75 and costs
freeM 70, because freeM loops at 8.2x and therefore revisits constantly. A flat
revisit charge cannot tell pointless circling from necessary backtracking, and
in a maze you often must retrace a corridor to get anywhere. That is a real
hazard, recorded before the run rather than after it.

Both are training at --max-steps 3000, which is the first budget long enough
that the hovering regime is inside training at all -- at 1000 steps exploration
saturates around step 800 and the pathology barely appears.

## CORRECTION: "playing optimally" was wrong, and the reason is a term I omitted

The section above concluded the policy hovers because hovering is optimal play.
That holds for maze and corridor and is false everywhere else, and the error was
mine: the reward decomposition I read left out `r_stop`, which turned out to
dominate. With it:

| world | seed | steps | travel | STOP | total | m/step |
|-------|------|-------|--------|------|-------|--------|
| city | 102 | 20000 | 47.5 m | **-1932** | **-1937** | 0.0024 |
| city | 101 | 20000 | 53.3 m | **-1923** | **-1932** | 0.0027 |
| culdesac | 101 | 20000 | 153.1 m | **-1766** | **-1718** | 0.0077 |
| forest | 101 | 20000 | 821.7 m | 0 | -15.9 | 0.0411 |
| maze | 102 | 20000 | 862.6 m | 0 | **+142** | 0.0431 |
| corridor | 101 | 20000 | 900.1 m | 0 | **+114** | 0.0450 |
| *(5 deaths)* | | | | 0 | -136 to -296 | 0.050-0.066 |

THREE REGIMES, NOT ONE:

- **Stalled** (city, culdesac/101): near-zero speed and masked actions, paying
  wStop's 0.05 + 0.05 for ~19,000 consecutive steps. Total -1932, which is SIX
  TIMES WORSE than simply crashing. A policy that would be better off flying
  into a wall is not optimizing; it is broken. Note this policy trained on maze
  ONLY, so city and cul-de-sac are out of distribution -- this is a
  generalisation collapse, a different thing from the hovering.
- **Orbiting** (maze, corridor, forest/101): flying 820-900 m in loops, stop
  penalty zero, total +114 to +142 against -289 for dying. Here the original
  conclusion stands: orbiting genuinely pays.
- **Dead**: and they fly FASTER than the survivors, 0.050-0.066 m/step against
  0.041-0.045. Speed and death track together.

## Dividing the score by loops

The suggestion was to divide by the loop ratio, which is already computed. It
ranks the behaviours correctly -- culdesac/102's genuine exploration is 3.6x,
maze orbiting is 28.2x, forest/101 is 54.8x -- so the signal is right.

Algebraically, `score / loops = score * net / travel`, and for a score that IS
path flown that reduces to net displacement, which is what --objective range
already pays. Its useful content beyond that is the part it adds: dividing an
ACCUMULATED score by a growing ratio makes hovering destroy value rather than
merely earn none.

As a literal per-step reward it is badly behaved -- non-Markovian (the reward
for an action depends on the whole history, so the policy cannot tell why it
fell) and singular as net -> 0 near the spawn. Its well-behaved linearisation is
"charge for movement that buys no new ground", which is exactly --revisit.

## The sweep: both new terms lost

150k steps, --seed 7, --max-steps 3000, maze, last quarter:

| run | travel | net | loops | cells | crash | crash per metre |
|-----|--------|-----|-------|-------|-------|-----------------|
| **base3k** | 87.9 m | **15.9 m** | 5.6x | **57** | 51.2% | 1 per 172 m |
| far3k | 33.6 m | 10.4 m | **3.3x** | 27 | 96.7% | 1 per **35 m** |
| revisit3k | **89.0 m** | 14.6 m | 6.1x | 40 | 46.8% | 1 per **190 m** |

`--far` collapsed. Its mechanism worked -- loops fell to 3.3x, the best of the
three -- but it bought directness by paying more for distant new ground, which
means paying to fly into unmapped space. Identical to the --coverage 0.45
failure.

`--revisit` did not do what it was designed to do: loops went UP, 5.6 -> 6.1,
and cells fell 57 -> 40.

The baseline wins on both columns the objective names.

METHODOLOGICAL NOTE: crash rate per EPISODE is not comparable across episode
lengths, because a longer episode is more exposure. Per metre flown the ranking
is revisit 190 m, base 172 m, far 35 m -- which does not change the conclusion,
but the raw percentages would have if compared against the 1000-step runs.

FOUR REWARD-TERM EXPERIMENTS, FOUR LOSSES: coverage 0.45, seen 0.30, seen 0.03,
far 1.0, revisit 0.05. One objective change, one large win. Whatever is left is
not in the shaping weights.

## Held out at 3000 steps: episode length was the win, not any reward term

12 episodes each (maze 101-106 x2), every policy scored identically:

| policy | net | cells | travel | loops | crash | metres/crash | stalled |
|--------|-----|-------|--------|-------|-------|--------------|---------|
| **base3k** | 18.0 m | **86** | 158.0 m | 8.8x | 2/12 | **948 m** | 1 |
| runRANGE | **19.1 m** | 43 | 127.6 m | 6.7x | 2/12 | 765 m | 0 |
| revisit3k | 15.5 m | 40 | 60.3 m | **3.9x** | 4/12 | 181 m | **5/12** |
| far3k | 15.3 m | 33 | 63.6 m | 4.2x | **10/12** | 76 m | 0 |

base3k DOUBLES the ground covered, 86 cells against 43, at the same collision
count and better metres-per-crash. The only difference between those two runs is
the training episode budget: 3000 steps instead of 1000. No reward term did
anything comparable.

CAVEAT ON THAT ROW: runRANGE trained at --max-steps 1000 and is scored here at
3000, so g[8] -- the fraction-of-episode-elapsed channel -- ticks three times
slower than anything it saw. Part of the margin is it running off-distribution.
There is no clean way around this: comparing policies trained at different
episode lengths must evaluate at least one of them outside its regime, and
scoring each at its own length would compare different tasks instead. The clean
comparisons are base3k / revisit3k / far3k, which share a seed and a budget.

--revisit HAS A FAILURE MODE NOT PREDICTED HERE: it stalls in 5 of 12 held-out
episodes. Charging for time spent on ground already covered produced a policy
that stops moving, which is the opposite of the intent and the same stall
pattern the endurance run found in city. No confident mechanism for it; recorded
as observed rather than explained.

FIVE REWARD-TERM EXPERIMENTS, FIVE LOSSES -- coverage 0.45, seen 0.30, seen
0.03, far 1.0, revisit 0.05. TWO STRUCTURAL CHANGES, TWO WINS: what the policy
is paid for (goal -> safe travel), and how long it is paid for it (1000 -> 3000
steps). The shaping weights are not where the remaining gap lives, and that is
now five measurements deep rather than a hunch.

## The bar, re-measured at 3000 steps -- and the policy clears it

freeM's 81.6 m was measured at 1000-step episodes. Every current number is at
3000, so the bar had to be re-measured rather than quoted. 12 identical
episodes each:

| planner | net | cells | travel | loops | crash | m/crash |
|---------|-----|-------|--------|-------|-------|---------|
| **policy (base3k)** | **18.0 m** | 86 | 158.0 m | **8.8x** | 2/12 | 948 m |
| freeM | 6.8 m | **110** | 201.3 m | **29.5x** | **0/12** | 2415 m |
| random | 12.9 m | 57 | 139.9 m | 10.9x | 2/12 | 840 m |
| goal | 14.9 m | 20 | 77.9 m | 5.2x | 2/12 | 468 m |
| score | 18.6 m | 30 | 26.9 m | 1.4x | 12/12 | 27 m |

**freeM flies 201 m to finish 6.8 m from where it started.** Given three times
the budget its loop ratio went 8.2x -> 29.5x and its displacement FELL, 9.9 ->
6.8 m, while coverage grew sub-linearly (64 -> 110 cells for 2.5x the path).
More time does not make it explore; it makes it circle more.

On the three things the objective names:

- displacement: policy 18.0 m against 6.8 m, **2.6x**
- ground covered: freeM 110 against 86, 1.28x to freeM
- hovering: policy 8.8x against 29.5x, decisively to the policy

On distance x cells, the composite of the two:

| policy | freeM | random | score | goal |
|--------|-------|--------|-------|------|
| **1548** | 748 | 735 | 558 | 298 |

THIS IS THE FIRST TIME A LEARNED POLICY HAS BEATEN freeM ON THIS OBJECTIVE.

> **Superseded -- read "The bar was four goal-seekers" below before quoting
> this.** Every planner in the table above optimises a goal or nothing. Against
> planners aimed at safe travel the learned policy is third on `net x cells`,
> and `net x cells` turns out to be the wrong number regardless: it cannot see
> metres-before-a-crash. The paragraph is left as written because the mistake it
> records -- declaring a win against a bar that was never matched to the
> objective -- is the useful part.
Not a clean sweep -- freeM covers more ground and never collides against the
policy's 2 in 12 -- but on displacement and on not circling it is ahead, and
those are the two things asked for.

A distinction worth keeping: **distance x cells failed as a REWARD term and
works as a SCORING metric.** Paying for it in training (--far 1.0) taught the
policy to fly into unmapped space chasing distant ground, 10 collisions in 12.
Measuring with it afterwards correctly ranks behaviour that is already safe. A
good metric is not automatically a good reward.

## The bar was four goal-seekers, and it was holding the result up

Every number above compares the policy against random / freeM / goal / score.
Two of those four maximise `-goalErr`, a quantity nothing has been paid for
since the objective became safe travel; one is a coin flip and one collides in
every episode. "The learned policy beat the baselines" was measured against a
set that was never matched to what is scored.

Five planners that are. All read only the observation and the mask, so they run
through the identical harness a policy does, and all five are under a dozen
lines -- `freeG` is `freeM` plus two terms:

    freeG     freeM projected onto the GROUND and charged for turning
    novelG    freeG, avoiding ground already flown over
    cover     frontier-seeking, gated on the path there being confirmed free
    frontRaw  the same seeker with the gate REMOVED -- the control
    circler   turn as hard as geometry allows: the degenerate solution

### First, the harness agrees with itself

`bench` is C++ and the documented table came from python's `report`. On maze
101-106, `bench` reproduces it to three significant figures -- freeM 201.3 m /
6.8 m / 110 cells, goal 77.9 / 14.9 / 20, score 26.9 / 18.7 / 30 -- so the two
harnesses are one harness, and the old protocol is pinned as maze seeds 101-106.
`random` is the one row that differs (5.1 m net here against 12.9 m there)
because its RNG stream is seeded differently in the two programs; every other
planner is deterministic given the world.

### Ten planners, maze 101-106 x 2, 3000 steps, ONE command

| planner | travel | net | cells | loops | crash | m/crash | net x cells |
|---------|--------|-----|-------|-------|-------|---------|-------------|
| novelG | 137.5 m | 17.8 m | **124** | 7.7x | 6/12 | 275 m | **2197** |
| freeG | 128.2 m | **20.9 m** | 102 | **6.1x** | 6/12 | 256 m | 2138 |
| policy (base3k) | 158.0 m | 18.0 m | 86 | 8.8x | **2/12** | **948 m** | 1551 |
| cover | 145.9 m | 19.9 m | 77 | 7.3x | **2/12** | 875 m | 1541 |
| freeM | **201.3 m** | 6.8 m | 110 | 29.5x | **0/12** | **never** | 752 |
| random | 139.9 m | 12.9 m | 57 | 10.9x | 2/12 | 840 m | 737 |
| frontRaw | 112.1 m | 14.5 m | 46 | 7.7x | 4/12 | 336 m | 667 |
| score | 26.9 m | 18.6 m | 30 | 1.4x | 12/12 | 27 m | 559 |
| goal | 77.9 m | 14.9 m | 20 | 5.2x | 2/12 | 468 m | 294 |
| circler | 16.3 m | 8.6 m | 16 | 1.9x | 12/12 | 16 m | 142 |

    kestrel report --run RUN --baselines --worlds maze \
                   --seeds 101 102 103 104 105 106 --repeats 2 --max-steps 3000

Every row above comes out of that one command, so there is no cross-harness
line anywhere in the table. It reproduces the documented policy row exactly --
158.0 m, 18.0 m, 86 cells, 8.8x, 2/12 -- and the nine classical rows agree with
`bench` to a tenth of a metre.

**THE COMPOSITE CLAIM IS NOT REFUTED -- IT IS UNRESOLVED, AND SO WAS THE
ORIGINAL.** `net x cells` was the number that said "the first time a learned
policy has beaten freeM on this objective", 1551 against 752. Two planners of a
dozen lines each score 2197 and 2138 on it, and the learned policy is third on
the point estimates. Paired episode by episode, those leads are +620 +/- 828 and
+481 +/- 705. **Neither clears its own standard error.** See "What twelve
episodes can and cannot settle" below; the ranking above is the point estimate
and very little of it is resolvable.

**AND THE COMPOSITE IS THE WRONG NUMBER**, which is the more useful half. It
multiplies the two columns the objective names and silently drops the third:
CLAUDE.md calls metres-before-a-collision the headline, and `net x cells` cannot
see it. On that column the order is unchanged -- freeM never crashes, base3k
goes 948 m, `cover` 875 m, and nothing else clears 470 m. A metric that ranks a
planner crashing every 256 m above one that has never crashed is not measuring
safe travel.

`cover` is the row that matters. It is level with the learned policy on every
column at once -- the SAME 2 collisions in 12, 19.9 m of displacement against
18.0, 875 m per crash against 948, 1541 against 1551, and paired by episode a
difference of +11 +/- 521 -- having never been trained. That is the one place a
null result is the interesting one: 150k steps of PPO is indistinguishable from
a dozen lines of frontier-seeking, and the measurement is easily powerful enough
to have caught a large difference if there were one.

`random` is worth a glance too: 737, within 3% of freeM's 752, because freeM
spends its enormous path length going nowhere.

### freeM's circling is load-bearing

`freeG` is `freeM` with the free length projected onto the ground and a 0.20
charge on yaw rate. That is enough to take the loop ratio from 29.5x to 6.1x and
triple the displacement, 6.8 m to 20.9 m -- so the circling is not something
freeM cannot help, it is something nothing was charging it for.

It also takes the collisions from 0/6 to 3/6. Turning was how freeM stayed
alive: a hard turn is short and stays inside mapped air, and the policy that
will not turn commits further into space its map has not confirmed. The two
findings are one finding, and it argues that "orbits too much" and "never
crashes" were never separable properties of that planner.

### A prediction that failed: the safety gate

`cover` gates its frontier bonus on most of the commanded path being confirmed
free; `frontRaw` removes the gate and nothing else. The stated expectation was
that `frontRaw` would collide much more, since every collision measured in this
tree has been into unmapped space, and that this would show the gate was what
kept `cover` alive.

The direction was right and the size was not. `frontRaw` collides 4 times in 12
against `cover`'s 2 -- twice as often, on twelve episodes, which this sample
cannot resolve from chance. **The prediction is neither confirmed nor refuted,
and it was stated as though one run of twelve could settle it.** What can be
said is that the gate is not worth the weight the comment on it claimed.

The clear difference is reach, not safety: 77 cells against 46 and 19.9 m
against 14.5 m. On metres-before-a-crash, which folds both effects together,
`cover` goes 875 m and `frontRaw` 336 m.

A mechanism for why an ungated frontier seeker is not obviously suicidal is
visible in the travel column. `o[7]` is set when a rollout STOPPED on unknown,
which happens near the fog boundary, so the primitives `frontRaw` selects are
the short ones -- it creeps, 112 m and 46 cells, the least of the
ground-seekers. It may be surviving because it barely goes anywhere rather than
because steering at fog is safe. That is a hypothesis the travel column is
consistent with, not a result: separating them needs more episodes than were
flown.

### circler did not manage to game the metric, and that is not reassuring

`circler` is in the set as an adversary: the degenerate solution CLAUDE.md names
is to bank distance turning in a safe clearing forever. It scores 142, last, and
collides in 12 of 12.

That is not evidence the metric is robust. **It is evidence the test world is
wrong for the question.** A maze has no clearings, so hard turns end in walls --
circler crashes for want of room, not for want of exploit. Whether the metric can
be gamed has to be asked in forest, road or culdesac, and has not been.

### Wider held-out set, maze 101-112

The ranking is stable; every planner is a little worse on the six maps nothing
has ever been tuned against, and the composite order is unchanged.

| planner | travel | net | cells | loops | crash | m/crash | net x cells |
|---------|--------|-----|-------|-------|-------|---------|-------------|
| novelG | 136.2 m | 15.3 m | 119 | 8.9x | 7/12 | 233 m | 1812 |
| freeG | 135.8 m | 17.3 m | 94 | 7.8x | 6/12 | 272 m | 1638 |
| cover | 136.4 m | 16.7 m | 72 | 8.2x | 4/12 | 409 m | 1199 |
| freeM | 200.1 m | 8.6 m | 100 | 23.2x | 0/12 | never | 860 |
| frontRaw | 108.3 m | 14.6 m | 40 | 7.4x | 3/12 | 433 m | 591 |
| score | 24.5 m | 19.0 m | 28 | 1.3x | 12/12 | 25 m | 527 |
| goal | 67.0 m | 15.1 m | 20 | 4.4x | 5/12 | 161 m | 297 |
| random | 93.1 m | 7.4 m | 30 | 12.6x | 5/12 | 223 m | 221 |
| circler | 44.2 m | 9.2 m | 21 | 4.8x | 11/12 | 48 m | 193 |

Reproduce either table with:

    kestrel bench --worlds maze --seeds 101 106 --steps 3000
    kestrel bench --worlds maze --seeds 101 112 --steps 3000

Every weight in the five new planners is hand-set and none were swept. They are
quoted as written; tuning them against an untuned policy would be the same
dishonesty in the other direction.

## What twelve episodes can and cannot settle

The table above is 6 maps x 2 repeats. Eight of the ten planners are
deterministic given the world, so for them it is **6 independent samples, not
12** -- the repeats are bit-identical and add nothing. This section is what the
numbers support once that is taken seriously, and the short version is that most
of the ranking is not resolvable and a smaller set of findings is solid.

### Episodes are not the same length, and the means mix two things

| planner | survived 3000 | mean steps | died at step |
|---------|---------------|-----------|--------------|
| freeM | 12/12 | 3000 | -- |
| policy | 10/12 | 2703 | 123, 2317 |
| cover | 10/12 | 2547 | 280, 280 |
| random | 10/12 | 2784 | 436, 2967 |
| goal | 10/12 | 2512 | 70, 70 |
| frontRaw | 8/12 | 2139 | 296, 296, 537, 537 |
| freeG | 6/12 | 1997 | 735 ... 1907 |
| novelG | 6/12 | 1934 | 608 ... 1902 |
| score | 0/12 | 333 | 213 ... 500 |
| circler | 0/12 | 178 | 66 ... 300 |

A planner that dies at step 700 banked 700 steps of travel, coverage and
displacement, not 3000. Per-episode means therefore confound HOW FAST a planner
covers ground with HOW LONG it survived, and they do it in the direction that
flatters the safe planners on totals and punishes them on rates. Two views fix
it -- rates per metre actually flown, and survivors only.

### Rates per 100 m flown, with bootstrap intervals

| planner | cells / 100 m | net m / 100 m |
|---------|---------------|---------------|
| novelG | **89.8** [75.4, 103.6] | 12.9 [10.5, 16.5] |
| freeG | 79.7 [63.0, 97.1] | 16.3 [11.1, 22.5] |
| freeM | 54.7 [40.0, 65.9] | **3.4** [1.7, 5.0] |
| policy | 54.5 [46.5, 62.3] | 11.4 [8.3, 14.7] |
| cover | 53.0 [42.4, 63.6] | 13.7 [11.2, 15.8] |
| random | 40.9 [23.3, 60.0] | 9.2 [6.6, 12.5] |
| frontRaw | 40.9 [22.8, 61.7] | 13.0 [7.8, 20.6] |
| goal | 25.2 [19.1, 40.4] | 19.2 [13.2, 33.9] |

**THE POLICY COVERS GROUND AT EXACTLY freeM'S RATE.** 54.5 [46.5, 62.3] against
54.7 [40.0, 65.9] -- the same number. Its 86 cells against freeM's 110 is not a
coverage deficit at all; it is the same coverage per metre over a shorter path,
because the policy flies 158 m where freeM flies 201 m. Every statement anywhere
above about the policy "covering less ground than freeM" is really a statement
about path length. The planners that genuinely cover faster are novelG and
freeG, and novelG's interval clears the policy's.

`score` and `circler` are left out: they die in under 500 steps, and the first
metres of an episode are always novel and always directed, so their rates are
inflated by the same truncation the rates were meant to remove.

### Survivors only: the 3000-step episodes, all the same length

| planner | n | travel | net | cells | loops | net x cells |
|---------|---|--------|-----|-------|-------|-------------|
| novelG | 6 | 202.1 m | **25.7 m** | **166** | 7.9x | **4257** |
| freeG | 6 | 170.3 m | 20.7 m | 108 | 8.2x | 2227 |
| cover | 10 | 170.3 m | 23.5 m | 88 | **7.2x** | 2062 |
| policy | 10 | 174.6 m | 19.7 m | 94 | 8.9x | 1840 |
| freeM | 12 | **201.3 m** | 6.8 m | 110 | 29.5x | 752 |
| random | 10 | 146.3 m | 12.5 m | 52 | 11.7x | 650 |
| frontRaw | 8 | 149.2 m | 12.5 m | 49 | 12.0x | 611 |
| goal | 10 | 92.7 m | 17.1 m | 22 | 5.4x | 384 |

This is the most flattering view of the new planners and it is also the most
selected one: **novelG's six survivors are the maps novelG happens to survive.**
Restricting to episodes every ground-seeker survived leaves 4 episodes on 2
maps. That is not a comparison, it is an anecdote, and it is reported here only
so nobody reconstructs it and believes it:

| planner | travel | net | cells | loops |
|---------|--------|-----|-------|-------|
| novelG | 202.8 m | 27.9 m | 176 | 7.3x |
| freeG | 178.2 m | 30.1 m | 134 | 5.9x |
| cover | 178.6 m | 28.9 m | 107 | 6.2x |
| policy | 147.6 m | 12.0 m | 71 | 12.3x |
| freeM | 220.3 m | 2.7 m | 139 | 81.1x |

### Paired against the policy, episode by episode

`net x cells`, mean difference over the same 12 episodes, +/- one standard error:

    novelG  +620 +/- 828    NOT resolved
    freeG   +481 +/- 705    NOT resolved
    cover    +11 +/- 521    NOT resolved
    freeM   -892 +/- 307    resolved

**Only the freeM comparison clears its own error bar**, and it clears it in the
direction the original write-up claimed. The learned policy beating freeM on
this composite is a real result. Two ten-line planners beating the learned
policy is not -- it is a point estimate with an interval twice its size.

### Crash rate, with exact Poisson intervals on the exposure

Exposure is metres actually flown, which is the right denominator: a planner
that dies early gets less opportunity to die again.

| planner | m flown | crashes | m per crash | 95% interval |
|---------|---------|---------|-------------|--------------|
| freeM | 2415 | 0 | never | 655 m - inf |
| policy | 1896 | 2 | 948 m | 340 - 3065 m |
| cover | 1750 | 2 | 875 m | 314 - 2829 m |
| random | 1679 | 2 | 840 m | 301 - 2714 m |
| goal | 935 | 2 | 468 m | 168 - 1512 m |
| frontRaw | 1345 | 4 | 336 m | 153 - 828 m |
| novelG | 1650 | 6 | 275 m | 141 - 586 m |
| freeG | 1538 | 6 | 256 m | 132 - 547 m |

The policy's interval and freeG's overlap between 340 and 547 m. The 3.7x gap
in the point estimates is **not established** by this run either. Two crashes is
two crashes; an interval nine times wide is what two events buys.

### The policy is sampled, and the spread is the largest effect here

Net displacement, the same policy on the same map, two draws:

    seed 101   39.3 vs 30.9 m        seed 104    1.8 vs 24.5 m
    seed 102   25.3 vs 12.5 m        seed 105   14.5 vs  7.2 m
    seed 103    7.1 vs 19.2 m        seed 106   12.3 vs 21.6 m

One map differs by **13.7x between two runs of the same weights**, and seed 103
gave 8.8 m of travel on one draw and 189.7 m on the other. The policy's mean of
18.0 m has a standard error of 3.1 m, a 95% interval of 11.9 to 24.2 m -- wide
enough to contain cover, freeG and novelG. Scoring a SAMPLED policy on 12
episodes cannot separate it from anything except freeM.

### What IS established

1. **freeM does not go anywhere.** 3.4 net metres per 100 m flown, interval
   [1.7, 5.0], against 9.2 or better for every other planner, non-overlapping
   with all of them. 29.5x looping is not a sampling accident.
2. **The learned policy beats freeM on net x cells**, paired, -892 +/- 307.
3. **The policy's coverage rate is freeM's**, 54.5 against 54.7 cells per 100 m.
4. **novelG covers ground faster than the policy**, 89.8 [75.4, 103.6] against
   54.5 [46.5, 62.3] -- intervals do not overlap. It is the one new planner with
   a resolved advantage on any column, and it is not the safety column.
5. **cover is indistinguishable from the policy**, +11 +/- 521, on the same 2
   collisions in 12. A tie against something never trained.
6. **48 collisions out of 48 were into UNKNOWN space; none into mapped space.**
   The geometric veto has still never failed.
7. **`min_clear_m` is a tautology, not a margin.** Crashed episodes: max 0.60 m.
   Survived: min 0.60 m. `robotR` is 0.60. The column records contact, not how
   close anything came to it, and cannot be used as a safety score.
8. **The old bar was mismatched**, and **`net x cells` cannot see safety.** Both
   are facts about the design, not measurements, and neither needs statistics.

### What the next run has to do differently

Not more repeats of a deterministic planner -- those are free and worthless.
**More maps.** Six is the sample size, and everything unresolved above is
unresolved for that reason. Seeds 101-140 at 3000 steps would put the paired
standard errors near 300 and settle novelG, freeG and cover against the policy
in one run. The other gap is worlds: every number here is maze, and `circler`
failed to game the metric only because a maze has no clearings to circle in.

## novelG tops the composite and loses on the reward

`novelG` leads `net x cells` at 2197. Scored by the reward the environment
actually pays, over the same 12 episodes:

| planner | total | range | coverage | clear | terminal |
|---------|-------|-------|----------|-------|----------|
| **policy** | **+73.5** | 107.5 | 39.6 | -5.2 | -50.0 |
| freeM | +70.9 | 44.2 | 43.8 | -14.1 | 0.0 |
| cover | +59.3 | 108.9 | 30.8 | -27.8 | -50.0 |
| novelG | **-17.0** | 99.7 | 50.2 | -15.0 | **-150.0** |

novelG earns the MOST coverage of the four and finishes last by 90 points,
because a collision costs 300 and it collides 6 times in 12. The learned policy
is first. Two scorings of one set of episodes put the same two planners in
opposite orders, and the disagreement is entirely the collision terminal --
which is the concrete form of the point made above, that `net x cells` cannot
see safety. The reward can. Prefer it.

### The composite lead is one map

Leave-one-map-out on `net x cells`:

| dropped | novelG | freeG | policy | cover | freeM |
|---------|--------|-------|--------|-------|-------|
| none | 2197 | 2138 | 1551 | 1541 | 752 |
| 101 | 2035 | 2001 | 1107 | 1202 | 413 |
| 102 | 2361 | 2103 | 1470 | 1673 | 951 |
| 103 | 2879 | 2758 | 1816 | 2062 | 984 |
| 104 | 2026 | 2044 | 1657 | 1382 | 928 |
| **105** | **1513** | 1612 | **1770** | 1200 | 671 |
| 106 | 2439 | 2363 | 1541 | 1813 | 624 |

Drop seed 105 and novelG falls behind the policy. One map in six flips the
ordering -- which is what +620 +/- 828 means in a form that can be checked by
looking. novelG scored 261 m of travel and 279 cells on 105, roughly double its
own average, and that single episode carries its lead.

### The policy hovers, and nothing else does

| planner | stopped steps per episode | of steps flown |
|---------|---------------------------|----------------|
| policy | **156.2** | 2703 |
| novelG | 0.0 | 1934 |
| cover | 0.0 | 2547 |
| freeG | 0.0 | 1997 |
| freeM | 0.0 | 3000 |

Every classical planner commands speed on every step of every episode, because
none of them has a reason not to. The learned policy spends about 6% of its
episode commanding stop and is the only thing in the table that does. `r_stop`
is charging it -15.6 a run and it does it anyway -- so the stop penalty is
priced too low, or stopping is buying something the decomposition does not
separate. This is a defect that needs no more episodes to see, and it is the one
behaviour the objective names outright as unwanted.

### The two policy collisions

    seed 103 rep 0   died at step 123 after 8.8 m      into UNKNOWN
    seed 102 rep 1   died at step 2317 after 141.7 m   into UNKNOWN

The first is the same draw that makes seed 103 the 13.7x variance case: the
other repeat on that map flew 189.7 m and survived. The policy's worst failure
and its median behaviour are the same weights on the same world.

## The veto let unknown space through, and closing it removes the collisions

Every collision this project has ever measured has been into UNKNOWN space --
48 of 48 in the run above, 0 into anything the map had marked occupied. That
was read for a long time as a sensing limit: the camera cannot see far enough,
so the fix is range. It is not. `sphereClear` rejects a cell whose log-odds are
above `occThresh` and lets everything else pass, so a primitive may sweep
through air NOTHING HAS MEASURED and the veto approves it. The sensor never
lied. The veto permitted the motion.

`coreFrac` is the switch that closes it -- the fraction of the body's own
radius that must be CONFIRMED free rather than merely not-known-occupied -- and
it has been 0 since it was written, on the strength of one sweep that could not
have detected its effect.

### The old sweep had no discriminating power

Forest, 400 steps, 4 seeds, scored on `travel / endDist / stopped`:

| trunkTex | coreFrac | coll/4 | travel | endDist | stopped |
|----------|----------|--------|--------|---------|---------|
| 0.70 | 0.00 | 0/4 | 68.3 | 107.9 | 169 |
| 0.70 | 0.65 | 0/4 | 53.9 | 122.4 | 217 |
| 0.25 | 0.00 | 0/4 | 56.1 | 120.6 | 210 |
| 0.25 | 0.65 | 1/4 | 36.9 | 139.5 | 180 |
| 0.15 | 0.00 | 4/4 | 6.9 | 169.1 | 0 |
| 0.15 | 0.65 | 4/4 | 57.9 | 118.9 | 1 |

It concluded "never safer, usually slower". **Two of the three rows are 0/4
against 0/4 and 4/4 against 4/4** -- no difference of any size could have shown
there -- and the third turns on one collision. "Slower" came from `endDist` and
`stopped`, which score a goal nothing is scored on any more.

### Re-swept in metres per collision

Maze, seeds 101-110, 3000 steps, truth depth, `--objective range`. Exposure is
metres actually flown, because a planner that dies early gets less opportunity
to die again.

| planner | coreFrac | travel | cells | crash | m/crash | 95% interval | minClr |
|---------|----------|--------|-------|-------|---------|--------------|--------|
| cover | 0.00 | 137.9 m | 71 | 3/10 | 460 m | 191 - 1265 m | 0.57 |
| cover | 0.45 | 166.6 m | **127** | **0/10** | never | 451 - inf | 0.61 |
| cover | 0.62 | 152.7 m | 71 | **0/10** | never | 414 - inf | 0.63 |
| cover | 0.80 | 152.0 m | 86 | 1/10 | 1520 m | 412 - 6277 m | 0.60 |
| freeG | 0.00 | 147.1 m | 102 | 4/10 | 368 m | 168 - 906 m | 0.58 |
| freeG | 0.45 | **185.9 m** | 118 | **0/10** | never | 504 - inf | 0.63 |
| novelG | 0.00 | 150.7 m | 129 | 5/10 | 301 m | 147 - 684 m | 0.58 |
| novelG | 0.45 | 160.0 m | 122 | 1/10 | 1600 m | 434 - 6606 m | 0.60 |
| novelG | 0.80 | 155.7 m | 92 | **0/10** | never | 422 - inf | 0.60 |
| freeM | 0.00 | 199.0 m | 95 | 0/10 | never | 540 - inf | 0.64 |
| freeM | 0.45 | 198.3 m | 104 | 0/10 | never | 538 - inf | 0.64 |

**Pooled over the three planners that collide at all: 12 collisions in 4357 m
becomes 1 in 5125 m.** At the old rate 14.1 were expected; P(<= 1) = **1.1e-5**.
Unlike almost everything else measured in this tree, this one is not close.

`freeM` is the control and behaves like one -- it never collided at any
setting, because it never goes anywhere near anything.

### It is not faster, and the travel column says otherwise

The travel numbers rise by 20-26%, and that is **survivorship**. A collision
ends an episode, so removing collisions leaves the full budget to fly.
Comparing only the episodes that survived:

| planner | survivor travel at 0.00 | at 0.45 |
|---------|-------------------------|---------|
| cover | 160.9 m (7/10) | 166.6 m (10/10) |
| freeG | 183.2 m (6/10) | 185.9 m (10/10) |
| novelG | 192.8 m (5/10) | 172.1 m (9/10) |
| freeM | 199.0 m (10/10) | 198.3 m (10/10) |

Flat, and slightly down for novelG. **The safety is free in travel terms. It is
not a speed gain, and reporting the first table alone would have claimed one.**

### The parameter is quantised, and most of its range does nothing

`sphereClear` walks INTEGER cell offsets, so the squared distances it can test
are `k * cell^2`. The core condition fires at `d2 <= (robotR * coreFrac)^2`, so
behaviour only changes as the parameter crosses `sqrt(k) * cell / robotR` --
0.417, 0.589, 0.722, 0.833, 0.932 at 0.25 m cells and a 0.6 m body.

Below 0.417 only the centre cell is tested, and an admissible rollout's own
cell is already free: **`--corefrac 0.30` is bit-identical to 0 across all
forty episodes.** 0.65, the only value the old sweep tried, is the 19-cell
regime. 0.45 is the 7-cell one and is enough.

At **1.00 the vehicle never leaves the spawn** -- 0.2 m flown. The whole ball
must be confirmed free, the map starts empty, so nothing is ever admissible.
The header's own warning about deadlock was right; this is where the bound is.

### It is still 0 by default

For one reason: base3k and every number in this document were produced at 0,
and the action mask is what the policy learned against. Adopting 0.45 retrains
the policy and re-measures the tree. The evidence says it should be 0.45.

    kestrel bench --worlds maze --seeds 101 110 --steps 3000 \
                  --policies cover freeM freeG novelG --corefrac 0.45

Raw output of all six sweep points is in `docs/corefrac_sweep_maze_3000.txt`.

## Re-measured on a working collision check: the result reverses

`sphereClear` was wrong. It walked INTEGER cell offsets from the query's own
cell and compared centre-to-centre distance against the body radius, so it
missed 60 voxels that intersect the body -- worst overlap 0.185 m -- and tested
that body as if it sat at its cell's centre, up to 0.217 m from where it was.
Two copies of it, in `voxel_traj.cpp` and `voxel_planner.cpp`, with the same two
defects. **Every number above this section was measured through it.**

Corrected, and the whole comparison re-run on the same episodes -- maze seeds
101-106 x 2, 3000 steps, one command.

| planner | travel | net | cells | loops | crash | net x cells |
|---------|--------|-----|-------|-------|-------|-------------|
| **freeM** | **232.7 m** | **29.3 m** | **156** | 8.0x | **0/12** | **4549** |
| freeG | 185.3 m | 18.6 m | 120 | 10.0x | 0/12 | 2232 |
| policy (base3k) | 150.8 m | 18.8 m | 72 | 8.0x | 0/12 | 1365 |
| novelG | 168.2 m | 19.4 m | 70 | 8.7x | 0/12 | 1360 |
| cover | 152.5 m | 18.7 m | 56 | 8.1x | 0/12 | 1049 |
| random | 135.7 m | 15.8 m | 62 | 8.6x | 0/12 | 982 |
| frontRaw | 138.5 m | 12.5 m | 54 | 11.1x | 2/12 | 674 |
| score | 27.2 m | 18.4 m | 30 | 1.5x | 10/12 | 555 |
| goal | 80.2 m | 16.9 m | 22 | 4.8x | 4/12 | 371 |
| circler | 112.4 m | 9.5 m | 17 | 11.9x | 2/12 | 164 |

### The safety effect is enormous and it was a bug, not a policy

Pooled over all ten planners: **48 collisions in 13.7 km became 18 in 16.6 km**.
58.1 were expected at the old rate, so P(<= 18) = **7.7e-10**. Six of the ten
now collide zero times in twelve episodes, where before only freeM did.

For scale: the `coreFrac` sweep, the best deliberate safety result in this
document, moved 12 collisions in 4357 m to 1 in 5125 m. Fixing the collision
check did more, across every planner at once, and it was not an improvement --
it was the removal of a defect.

### The headline claim reverses, with significance both times

Paired on `net x cells`, against the policy, on the fixed veto:

    freeM    +3382 +/- 1059   RESOLVED
    freeG     +660 +/-  403   not resolved
    novelG    -148 +/-  350   not resolved
    cover     -299 +/-  666   not resolved
    random    -150 +/-  659   not resolved

The earlier table had the policy beating freeM by **-892 +/- 307**, and that was
the basis of "THE FIRST TIME A LEARNED POLICY HAS BEATEN freeM ON THIS
OBJECTIVE". It is now **+3382 +/- 1059 the other way**. The finding did not
weaken into noise; it changed sign, and was resolved in both directions.

**The policy is not distinguishable from random.** -150 +/- 659 against a
uniform draw over the admissible primitives. At this sample size, on this
objective, 150k steps of PPO cannot be told apart from a coin flip.

### What moved, and why freeM moved most

| planner | composite broken -> fixed | cells |
|---------|---------------------------|-------|
| freeM | 752 -> **4549** | 110 -> 156 |
| freeG | 2138 -> 2232 | 102 -> 120 |
| policy | 1551 -> 1365 | 86 -> 72 |
| novelG | 2197 -> 1360 | 124 -> 70 |
| cover | 1541 -> 1049 | 77 -> 56 |
| random | 737 -> 982 | 57 -> 62 |

freeM gained sixfold. Its old behaviour -- 201 m flown to finish 6.8 m from the
spawn, looping at 29.5x -- was the broken veto waving through primitives that
grazed obstacles, walking it into pockets it then had to turn out of. With a
correct check it goes 232.7 m to finish 29.3 m out, at 8.0x, covering more
ground than anything else here. Every sentence this document wrote about
freeM's pathological circling, and about that circling being what kept it
alive, was a description of a bug.

**novelG's advantage was an artefact too.** It was the one classical planner
with a resolved edge -- 89.8 cells per 100 m against the policy's 54.5,
intervals disjoint -- and on the corrected veto it drops from 124 cells to 70
and lands level with the policy.

### What this does and does not say about learning

It does not say learning cannot work here. It says **nothing measured so far is
evidence that it has**, because the bar was crippled in a way that flattered
the policy, and because base3k was also TRAINED against the broken veto -- its
action mask was wrong for every one of its 150k steps.

The first honest experiment is a retrain on the corrected veto. Until that
exists, the correct description of the learned policy on this objective is
"not distinguishable from random".

Raw output: `docs/bar10_maze_3000_fixedveto.csv`.

## The retrain: 150k steps does not learn this task, and never did

The corrected veto made every earlier measurement suspect, so base3k's training
was repeated on it. One run collapsed -- 3.1 m of net displacement, 14 cells,
23x looping, 4 collisions in 12, a composite of 43 against random's 982. That
is worse than a coin flip by a factor of 23.

One run settles nothing, and this document has had to relearn that repeatedly.
So it was run four times, identical but for the seed:

| training seed | travel | net | cells | loops | stopped | crash | net x cells |
|---------------|--------|-----|-------|-------|---------|-------|-------------|
| 7 | 71.3 m | 3.1 m | 14 | 23.0x | 519 | 4/12 | **43** |
| 10 | 85.8 m | 8.7 m | 28 | 9.9x | 58 | 6/12 | 245 |
| 8 | 146.0 m | 15.7 m | 66 | 9.3x | 0 | 2/12 | 1038 |
| 9 | 151.4 m | 22.5 m | 75 | 6.7x | 0 | 1/12 | **1696** |

**A 39x spread between seeds.** Mean 756, standard error 380.

### What that answers

- **The collapse was the seed, not the veto.** Seed 7 was an unlucky draw from
  a distribution that also contains 1696.
- **base3k was a lucky draw.** Its 1365 sits inside this range, +1.6 standard
  errors from the mean of runs trained on the corrected veto. It is not a
  different kind of policy; it is the same process, sampled once, at the
  favourable end.
- **The mean is below random.** 756 against 982, which is +0.6 se -- so
  "indistinguishable from random" was right, and generous.
- **None of them approaches freeM.** 4549 is +10.0 standard errors above the
  mean of the learned runs. The best of four seeds is still 2.7x below it.
- **They are also less SAFE than the classical planners.** 1, 2, 4 and 6
  collisions in 12, where freeM, freeG, cover, novelG and random all collide
  zero times on the same episodes.

### The methodological consequence, which is the larger one

**Every 150k-step A/B in this document compared single draws from a
distribution with a 39x spread.** The five reward-term experiments -- coverage
0.45, seen 0.30, seen 0.03, far 1.0, revisit 0.05, recorded here as "FIVE
REWARD-TERM EXPERIMENTS, FIVE LOSSES" -- were one run each. So were the
base3k / far3k / revisit3k sweep and the objective and episode-length
comparisons that were called "TWO STRUCTURAL CHANGES, TWO WINS".

None of those comparisons had the power to detect anything smaller than the
noise, and the noise is larger than any effect they claimed. They are not
evidence that the shaping weights do not matter; they are not evidence of
anything. The two "structural wins" may well be real -- goal to range is a
change of what is being optimised, not a tuning nudge -- but they were not
demonstrated by those runs.

### What would settle it

More steps, or more seeds, and the repository has been doing neither. 150k is
0.75% of the 20 M where the reference run was said to peak. A comparison at
this budget needs at least four seeds per arm to say anything at all, which at
11 minutes a run is affordable -- it simply was never done.

Raw: `docs/retrain_seeds_maze_3000.csv`.

## 3M steps, a flat curve, and an evaluation that was never like-for-like

150k was 0.75% of where the reference run was said to peak, so the obvious
objection to everything above is budget. Seed 7 -- the seed that collapsed
worst at 150k, chosen deliberately -- was rerun at **3,000,000 steps**, 20x the
budget, 5.56 hours, checkpoints every 250k.

### The collapse modes went away. The gap did not.

| steps | travel | net | cells | loops | stopped | crash | net x cells |
|-------|--------|-----|-------|-------|---------|-------|-------------|
| 150k | 71.3 m | 3.1 m | 14 | 23.0x | 519 | 4/12 | 43 |
| 250k | 111.2 m | 17.5 m | 59 | 6.3x | 582 | 2/12 | 1034 |
| 750k | 43.0 m | 8.2 m | 15 | 5.2x | 1076 | 5/12 | **124** |
| 1.5M | 171.9 m | 19.4 m | 69 | 8.9x | 0 | 0/12 | 1337 |
| 2.25M | 143.5 m | 20.6 m | 37 | 7.0x | 0 | 0/12 | 769 |
| 3M | 154.6 m | 22.0 m | 66 | 7.0x | 209 | 1/12 | **1453** |
| freeM | 232.7 m | 29.3 m | 156 | 8.0x | 0 | 0/12 | **4571** |

The orbiting and hovering are gone -- 154 m flown instead of 71, 7.0x looping
instead of 23.0x, 209 stopped steps instead of 519. That is a real change and
budget bought it.

**IT IS NOT A LEARNING CURVE.** From 250k on, mean 943, standard error 237:
**+0.2 se from random**, and 15.3 se below freeM. 3M is not distinguishable
from 250k. And the swing WITHIN this single run -- 124 at 750k against 1453 at
3M, a factor of 12 -- is the same order as the 39x swing across seeds at 150k.
The instability is not a property of short runs. It survives every budget
measured.

### The comparison was never like-for-like

Every classical planner here is DETERMINISTIC by construction: freeM takes the
argmax of o[0], every time, forever. Every learned-policy number this project
has ever quoted was SAMPLED from the action distribution. A deployed aircraft
would fly the argmax.

| policy | sampled | deterministic |
|--------|---------|---------------|
| base3k | 1365 (0/12 crashes) | **434** (2/6 crashes) |
| 3M seed 7 | 1453 (1/12) | **514** (0/6) |

**Sampling is worth roughly 2x**, and it is worth it by supplying exploration:
coverage halves when it is removed, 66 cells to 35 and 72 to 34. The entropy in
the action distribution is doing work the learned preferences are not.

**Deterministically, both policies score BELOW RANDOM** -- 434 and 514 against
980. The greedy action of the trained network is worse than a uniform draw over
the primitives geometry admits.

### So the original claim had three independent problems

"The first time a learned policy has beaten freeM on this objective" rested on:

1. a collision check that missed 60 voxels intersecting the body, which
   flattered the policy and crippled freeM;
2. one lucky seed out of a distribution spanning 39x; and
3. an evaluation that gave the policy stochastic exploration its competitor did
   not get and a deployment would not use.

Each alone would have been enough to void it.

### Limits of this section

One seed at 3M, not four. Deterministic evaluation is n=6, because repeats of a
deterministic policy are identical. Maze only. And `random` has no argmax, so
"below random" compares a deterministic policy against an inherently stochastic
one -- which is the right comparison for deployment, and worth naming anyway.

Raw: `docs/long3m_maze_3000.csv`.

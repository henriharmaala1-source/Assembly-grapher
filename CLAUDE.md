# Working notes for this repository

## Every command must be reachable from the window

`kestrel` has a GUI (`nav-sim/kestrel_gui.cpp`) and a CLI, and they are not
allowed to drift. **When a subcommand is added or gains an option, wire it into
the window in the same change** — a mode button and a panel for a new command, a
control for a new flag. A feature that only exists on the command line is only
half delivered here: the window is what a reviewer double-clicks, and anything
missing from it is invisible to them.

This is cheap to honour because of how the GUI is built: it does not implement
anything. It collects settings, prints the exact `kestrel ...` line it is about
to run along the bottom, and calls the same function the CLI calls. So wiring a
command in means adding a panel that builds an argument vector — never a second
implementation that can disagree with the first.

Checklist for a new subcommand:

- `kestrel_gui.hpp` — add it to `kgui::Actions`
- `kestrel_gui.cpp` — extend `Mode`/`MODE_NAME`, add a `panelX`, add its button
  ids, and handle it in `buildArgs`, `blocker` and `apply`
- `kestrel.cpp` — bind the action in `gui()`, and add the subcommand to `main`
  and to `--help`
- `RUN_ME_windows.txt` — document it
- `kestrel gui --check` must still report 0 violations; it walks every mode, so
  a new panel is covered automatically

## WHAT THE POLICY IS FOR

**The objective is safe travel, as far as possible. It is not goal-finding.**

The simulator has a goal in it, and for a long time everything here was scored
on reaching one -- goal rate, closing fraction, "still closing when cut off".
That was the wrong target. The goal is scaffolding: something to give the
aircraft a direction. What is actually wanted is an aircraft that keeps flying,
covers ground, and does not hit anything.

So the columns that matter are:

- **metres travelled before a collision ends it** -- the headline number
- **collision rate** -- a crash is the failure, not a missed goal
- **cells visited / net displacement** -- against the degenerate solution, which
  is to circle in a safe clearing forever and bank distance for free

The bar is `freeM`, the classical openness-seeking planner, because it is a
greedy version of half this objective: it picks the primitive with the longest
confirmed-free path. It is also, measured properly, the best planner in this
tree on every column named above -- it flies furthest, ends furthest from its
spawn, covers the most ground and does not collide.

THAT IS A CORRECTION. This file used to say freeM "buys that safety by
circling", looping at 29.5x its own displacement and flying 201 m to finish
6.8 m from where it started, with coverage growing sub-linearly while
displacement FELL. All of that was a description of a broken collision check
(see below): with `sphereClear` corrected, freeM loops at 8.0x and finishes
29.3 m out. The circling was the veto waving through primitives that grazed
obstacles and walked it into pockets it then had to turn out of.

So quote it honestly. Held-out maze, seeds 101-106 x 2, 3000-step episodes,
re-measured 2026-09-21 after `sphereClear` was corrected:

| planner | travel | net | cells | loops | crashes | net x cells |
|---------|--------|-----|-------|-------|---------|-------------|
| **freeM** | **232.7 m** | **29.3 m** | **156** | 8.0x | **0/12** | **4549** |
| freeG | 185.3 m | 18.6 m | 120 | 10.0x | 0/12 | 2232 |
| learned (base3k) | 150.8 m | 18.8 m | 72 | 8.0x | 0/12 | 1365 |
| novelG | 168.2 m | 19.4 m | 70 | 8.7x | 0/12 | 1360 |
| cover | 152.5 m | 18.7 m | 56 | 8.1x | 0/12 | 1049 |
| random | 135.7 m | 15.8 m | 62 | 8.6x | 0/12 | 982 |
| frontRaw | 138.5 m | 12.5 m | 54 | 11.1x | 2/12 | 674 |
| score | 27.2 m | 18.4 m | 30 | 1.5x | 10/12 | 555 |
| goal | 80.2 m | 16.9 m | 22 | 4.8x | 4/12 | 371 |
| circler | 112.4 m | 9.5 m | 17 | 11.9x | 2/12 | 164 |

### THE LEARNED POLICY DOES NOT BEAT THE BAR. IT DID, AGAINST A BROKEN VETO.

`sphereClear` walked integer cell offsets from the query's own cell and
compared centre-to-centre distance, so it missed 60 voxels that intersect the
body and tested that body as if it sat at its cell's centre, up to 0.217 m from
where it was. Every number this file has ever carried was measured through it.
Corrected, and everything re-run on the same episodes:

- **Collisions across all ten planners: 48 in 13.7 km became 18 in 16.6 km.**
  58.1 were expected at the old rate; P(<= 18) = 7.7e-10. This is the largest
  safety effect ever measured here, and it was a bug in the collision check.
- **freeM now beats the learned policy by +3382 +/- 1059 on `net x cells`,
  paired, RESOLVED.** The old table had the policy beating freeM by -892 +/-
  307. The result did not weaken; it reversed, with significance both times.
- **The policy is indistinguishable from novelG (-148 +/- 350), cover
  (-299 +/- 666) and RANDOM (-150 +/- 659).** On this objective, at this
  sample size, 150k steps of PPO cannot be told apart from a coin flip over
  the admissible primitives.
- **novelG's coverage advantage is gone**: 124 cells to 70, composite 2197 to
  1360. It was the one classical planner with a resolved edge, and the edge was
  an artefact.
- The policy itself got slightly worse, 1551 to 1365, and its safety margin
  vanished because everything is safe now: six of the ten planners collide zero
  times in twelve episodes.

freeM wins on every column that matters. It flies furthest, ends furthest from
its spawn, covers the most ground, never collides, and its looping -- the thing
this file spent months holding against it -- is 8.0x, not 29.5x. The circling
was the broken veto letting it graze obstacles and walk into pockets it then
had to turn out of.

WHAT THIS MEANS FOR THE POLICY. It is not that learning cannot work here. It is
that nothing measured so far is evidence that it has, because the bar it was
measured against was crippled in a way that flattered it. base3k was also
TRAINED against the broken veto, so its action mask was wrong throughout
training; a retrain on the corrected one is the first honest experiment, and
until it exists the right description of the learned policy is "not
distinguishable from random".

Raw: docs/bar10_maze_3000_fixedveto.csv.

A GOAL-SHAPED REWARD WILL NOT PRODUCE THIS. Progress-to-goal pays for closing
distance to one point and stops paying when the aircraft is there; it says
nothing about staying alive or covering ground, and it actively punishes the
detour that avoids a tree.

## Things that are load-bearing

- **Unknown is not free.** Pale/grey is UNKNOWN everywhere in this tree and is
  rendered as fog, never as air. A view that makes unknown look empty is a bug.
- **The window is checkable without a display.** `gui --shot PREFIX` renders
  every panel to PNG and `gui --check` asserts the layout; `watch --shot FILE`
  does the same for the live view. Keep that property — it is the only way any
  of this gets reviewed over ssh or in CI.
- **Name the interpreter, never "python".** On a machine with several,
  `pip install` into the wrong one succeeds and the import still fails. Every
  install line this program prints quotes an absolute `sys.executable`.
- **Don't put logic inside `#ifdef _WIN32`.** A block the dev machine never
  compiles is a block that is never checked; gate behaviour at runtime and keep
  the code compiling everywhere. One-line platform API calls are the exception.

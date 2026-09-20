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
confirmed-free path. But it buys that safety by circling, and the longer you let
it fly the worse that gets -- at 1000-step episodes it loops at 8.2x its own
displacement, at 3000 steps it loops at **29.5x**, flying 201 m to finish 6.8 m
from where it started. Its coverage grows sub-linearly with path length while
its displacement actually FALLS.

So quote it honestly. Held-out maze, 3000-step episodes:

| planner | net disp | cells | loops | collisions | m before a crash |
|---------|----------|-------|-------|------------|------------------|
| novelG | 17.8 m | **124** | 7.7x | 6/12 | 275 m |
| freeG | **20.9 m** | 102 | **6.1x** | 6/12 | 256 m |
| learned (base3k) | 18.0 m | 86 | 8.8x | 2/12 | **948 m** |
| cover | 19.9 m | 77 | 7.3x | 2/12 | 875 m |
| freeM | 6.8 m | 110 | 29.5x | **0/12** | **never** |

AND QUOTE IT AGAINST A BAR THAT IS MATCHED TO IT. The four planners this was
first measured against were written when reaching a goal was the score, and two
of them optimise a goal nothing pays for. Against that set the learned policy
led on `net x cells`, 1551 to freeM's 752, and that was read as the first
learned win on this objective. Against planners aimed at what IS scored it is
third on the point estimates: novelG 2197, freeG 2138, base3k 1551, cover 1541.

**SIX MAPS CANNOT RANK THOSE FOUR, AND SAYING OTHERWISE IS THE STANDING ERROR
HERE.** Paired episode by episode, novelG leads the policy by 620 +/- 828 and
freeG by 481 +/- 705 -- neither clears its own error bar. The policy's own net
displacement varies 13.7x between two draws on the SAME map. What IS resolved:
the policy beats freeM (-892 +/- 307), novelG covers ground faster than the
policy (89.8 vs 54.5 cells per 100 m, intervals disjoint), and `cover` --
frontier-seeking with a gate, a dozen lines, never trained -- is
indistinguishable from 150k steps of PPO at +11 +/- 521 on the same 2 collisions
in 12. Before quoting any other ordering, run more MAPS; repeats of a
deterministic planner are bit-identical and add nothing.

`net x cells` IS ALSO THE WRONG NUMBER, and that is the more useful half. It
multiplies two of the three columns named above and cannot see the third, so it
ranks a planner that crashes every 256 m over one that has never crashed. On
metres-before-a-crash the point estimates are freeM never, base3k 948 m, cover
875 m, and nothing else over 470 m -- but two crashes buys a 95% interval of 340
to 3065 m, so that column is not resolved either. Use the columns, not their
product, and quote the interval with the column.

TWO MORE TRAPS IN THE TABLE. **Per-episode means mix rate with survival**: a
planner that dies at step 700 banked 700 steps of coverage, not 3000, so compare
per metre flown. Do that and **the policy covers ground at exactly freeM's rate**
-- 54.5 against 54.7 cells per 100 m -- meaning its 86 cells against freeM's 110
is a shorter path, not worse coverage. And **`min_clear_m` is a tautology**:
crashed episodes max out at 0.60 m, survivors start at 0.60 m, `robotR` is 0.60.
It records contact, never margin, and is not a safety score.

The other thing that came out of that measurement: **freeM's circling is what
keeps it alive.** freeG is freeM plus a charge on yaw rate, and that single term
takes the loop ratio 29.5x -> 6.1x and the displacement 6.8 -> 20.9 m -- and the
collisions 0/12 -> 6/12. A hard turn is short and stays inside mapped air. "Orbits
too much" and "never crashes" are one property of that planner, not two.

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

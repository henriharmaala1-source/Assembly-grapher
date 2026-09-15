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

So quote it honestly. Over 12 held-out maze episodes at 3000 steps:

| planner | net disp | cells | loops | collisions |
|---------|----------|-------|-------|------------|
| learned (base3k) | **18.0 m** | 86 | **8.8x** | 2/12 |
| freeM | 6.8 m | **110** | 29.5x | **0/12** |

freeM still wins on ground covered and on never crashing. The learned policy
wins 2.6x on displacement and circles a third as much, and on distance x cells
-- the composite of the two things asked for -- it is 1548 against 748.

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

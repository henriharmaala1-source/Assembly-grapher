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

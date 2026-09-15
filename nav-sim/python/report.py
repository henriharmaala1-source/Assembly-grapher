#!/usr/bin/env python3
"""Fly episodes and write the data a failure report is drawn from.

THIS FILE DRAWS NOTHING. It runs the policy and writes two CSVs; kestrel's C++
side reads them and draws the panels, and `kestrel report --plot DIR` redraws
from the same files without flying anything again. That split is the point:
flying is the expensive half, so changing a plot must never mean re-running a
twenty-minute job -- and the drawing then needs no python at all, which is what
lets `report --check` run in CI beside `gui --check`.

Why not matplotlib: it is not in requirements.txt, and kestrel's preflight only
probes gymnasium, stable_baselines3 and sb3_contrib. A matplotlib report would
sail through the check the whole program is built around and then die on an
ImportError -- exactly the failure kestrel works hardest to prevent.

  kestrel report                                  the newest run, all six worlds
  kestrel report --repeats 4                      four runs per world instance
  kestrel report --progress                       every checkpoint, not just the newest
  kestrel report --baselines                      the classical planners too
  kestrel report --plot DIR                       redraw, no flying (C++ only)
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [p for p in (os.environ.get("KESTREL_MODULE_DIR"),
                            os.path.join(_here, "..", "build"),
                            os.path.join(_here, ".."),
                            _here) if p]

import numpy as np

import voxelenv
from voxel_gym import (CRUISE_M_PER_STEP, TRAIN_WORLDS, VoxelNavEnv,
                       all_checkpoints, journey_fit, newest_checkpoint,
                       newest_run_dir, run_root)

# One trace row per this many steps. A 3000-step episode times six worlds times
# eight seeds times four repeats is 576k rows at stride 1, which is a 30 MB CSV
# to answer a question about the SHAPE of a curve. The step number is written
# per row, so the stride never has to be guessed by the reader.
TRACE_STRIDE = 10

COLUMNS = ["id", "world", "seed", "repeat", "checkpoint_steps", "planner",
           "steps", "max_steps", "travel_m", "start_dist_m", "dist_to_goal_m",
           "min_dist_to_goal_m", "min_dist_step", "net_disp_m", "cells_visited",
           "min_clear_m", "stopped_steps", "collisions", "hit_unknown",
           "reached_goal", "r_progress", "r_coverage", "r_time", "r_stop",
           "r_clear", "r_terminal", "goal_tol_m", "robot_r",
           "goal_x", "goal_y"]

BASELINES = {"random": voxelenv.Baseline.random, "freeM": voxelenv.Baseline.freeM,
             "goal": voxelenv.Baseline.goal, "score": voxelenv.Baseline.score}


def fly(env, model, baseline, rng, world, seed, max_steps, deterministic):
    """One episode. Returns (info, trace) where trace is [(step, x, y, dist)]."""
    obs, _ = env.reset(seed=seed, options={"world": world, "seed": seed})
    trace, info = [], {}
    for k in range(max_steps):
        mask = env.action_masks()
        if baseline is not None:
            a, fly.rng = voxelenv.choose_baseline(baseline, obs, mask,
                                                  len(mask), fly.rng)
            a = int(a)
        elif model is None:
            legal = np.flatnonzero(mask)
            a = int(rng.choice(legal)) if len(legal) else 0
        else:
            a, _st = model.predict(obs, action_masks=mask,
                                   deterministic=deterministic)
            a = int(a)
        obs, _r, done, trunc, info = env.step(a)
        if k % TRACE_STRIDE == 0 or done or trunc:
            x, y = env.position_xy()
            trace.append((k, x, y, info["dist_to_goal_m"]))
        if done or trunc:
            break
    return info, trace


fly.rng = 12345


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="", help="run directory; default newest")
    ap.add_argument("--out", default="", help="where the report goes; "
                                              "default <run>/report")
    ap.add_argument("--model", default=None, help="a specific checkpoint")
    ap.add_argument("--worlds", nargs="+", default=list(TRAIN_WORLDS))
    ap.add_argument("--seeds", type=int, nargs="+",
                    default=list(range(101, 105)))
    ap.add_argument("--max-steps", type=int, default=3000)
    ap.add_argument("--repeats", type=int, default=1,
                    help="runs per world instance. A DETERMINISTIC policy gives "
                         "one trail per instance however many times you ask, so "
                         "anything above 1 samples instead -- which is what "
                         "makes the map panel show a distribution rather than a "
                         "line.")
    ap.add_argument("--deterministic", action="store_true",
                    help="argmax. Off by default for the same reason --repeats "
                         "exists: one trail per world says nothing about where "
                         "failures cluster.")
    ap.add_argument("--progress", action="store_true",
                    help="every checkpoint in the run, not just the newest -- "
                         "this is what fills the across-checkpoints panel")
    ap.add_argument("--baselines", action="store_true",
                    help="the four classical planners on the same episodes")
    ap.add_argument("--random", action="store_true",
                    help="the floor: uniform over admissible primitives")
    ap.add_argument("--stereo", action="store_true")
    ap.add_argument("--no-veto", action="store_true")
    ap.add_argument("--vary-goal", action="store_true")
    ap.add_argument("--seen", type=float, default=0.0,
                    help="match the run's --seen; see train.py --seen")
    ap.add_argument("--coverage", type=float, default=0.15,
                    help="match the run's --coverage so the reward terms in the "
                         "CSV mean what they did in training")
    ap.add_argument("--objective", default="range", choices=["range", "goal"],
                    help="score under the same objective the policy was "
                         "trained on. Under range the goal does not end an "
                         "episode, so scoring a range policy under goal would "
                         "cut it off at exactly the thing it was paid for.")
    args = ap.parse_args()

    if not args.run:
        args.run = newest_run_dir(run_root()) or run_root()
    out = args.out or os.path.join(args.run, "report")
    os.makedirs(out, exist_ok=True)
    print(f"[report] run  {os.path.abspath(args.run)}", flush=True)
    print(f"[report] out  {os.path.abspath(out)}", flush=True)

    # SAY WHAT CANNOT BE WON BEFORE SPENDING TWENTY MINUTES ON IT. The report's
    # whole job is explaining failure, and "the goal was further away than the
    # budget can fly" is a failure of the SETUP that would otherwise be drawn as
    # a failure of the policy.
    try:
        fit = journey_fit(args.worlds, args.max_steps,
                          truth_depth=not args.stereo, vary_goal=args.vary_goal)
        print(f"\n  {'world':10} {'journey':>9} {'route':>9} "
              f"{'needs':>8} {'budget':>8}")
        for w, _mean, far, need, over, path in fit:
            route = ("NO ROUTE" if path == -1 else
                     "  -" if path < 0 else f"{path:.0f}m")
            print(f"  {w:10} {far:>8.0f}m {route:>9} {need:>8} "
                  f"{args.max_steps:>8}"
                  + ("   TOO FAR" if over else "")
                  + ("   UNREACHABLE" if path == -1 else ""))
        print(flush=True)
    except Exception as exc:
        print(f"[report] journey-fit check skipped ({exc})", flush=True)

    # Which policies to fly. Each is (label, model, baseline, checkpoint_steps).
    jobs = []
    if not args.random:
        if args.model:
            from sb3_contrib import MaskablePPO
            jobs.append(("policy", MaskablePPO.load(args.model, device="cpu"),
                         None, 0))
        elif args.progress:
            cps = all_checkpoints(args.run)
            if not cps:
                return (f"[report] no checkpoints in {os.path.abspath(args.run)}")
            from sb3_contrib import MaskablePPO
            for path, trained in cps:
                jobs.append((f"policy@{trained}",
                             MaskablePPO.load(path, device="cpu"), None,
                             trained))
            print(f"[report] --progress: {len(cps)} checkpoints", flush=True)
        else:
            path, trained = newest_checkpoint(args.run)
            if not path:
                return (f"[report] no checkpoint in {os.path.abspath(args.run)}."
                        "\n          Train one first, name one with --model, or"
                        " pass --random.")
            from sb3_contrib import MaskablePPO
            print(f"[report] policy {os.path.basename(path)} "
                  f"({trained} trained steps)", flush=True)
            jobs.append(("policy", MaskablePPO.load(path, device="cpu"), None,
                         trained))
    else:
        jobs.append(("random", None, None, 0))
    if args.baselines:
        for name, pol in BASELINES.items():
            jobs.append((name, None, pol, 0))

    rows, traces = [], []
    rng = np.random.default_rng(0)
    eid = 0
    t0 = time.time()
    total = len(jobs) * len(args.worlds) * len(args.seeds) * args.repeats
    for label, model, baseline, trained in jobs:
        for w in args.worlds:
            env = VoxelNavEnv(worlds=(w,), seeds=list(args.seeds),
                              max_steps=args.max_steps,
                              truth_depth=not args.stereo,
                              mask_unsafe=not args.no_veto,
                              vary_goal=args.vary_goal,
                              objective=args.objective,
                              coverage=args.coverage, seen=args.seen)
            for sd in args.seeds:
                for rep in range(args.repeats):
                    fly.rng = 12345 + rep     # same stream for every planner
                    info, trace = fly(env, model, baseline, rng, w, sd,
                                      args.max_steps,
                                      args.deterministic and args.repeats == 1)
                    gx, gy = env.goal_xy()
                    rows.append({
                        "id": eid, "world": w, "seed": sd, "repeat": rep,
                        "checkpoint_steps": trained, "planner": label,
                        "steps": info["steps"], "max_steps": args.max_steps,
                        "travel_m": info["travel_m"],
                        "start_dist_m": info["start_dist_m"],
                        "dist_to_goal_m": info["dist_to_goal_m"],
                        "min_dist_to_goal_m": info["min_dist_to_goal_m"],
                        "min_dist_step": info["min_dist_step"],
                        "net_disp_m": info["net_disp_m"],
                        "cells_visited": info["cells_visited"],
                        "min_clear_m": info["min_clear_m"],
                        "stopped_steps": info["stopped_steps"],
                        "collisions": info["collisions"],
                        "hit_unknown": int(bool(info["hit_unknown"])),
                        "reached_goal": int(bool(info["reached_goal"])),
                        "r_progress": info["r_progress"],
                        "r_coverage": info["r_coverage"],
                        "r_time": info["r_time"], "r_stop": info["r_stop"],
                        "r_clear": info["r_clear"],
                        "r_terminal": info["r_terminal"],
                        "goal_tol_m": 3.0, "robot_r": 0.6,
                        "goal_x": gx, "goal_y": gy,
                    })
                    for st, x, y, d in trace:
                        traces.append((eid, st, x, y, d))
                    eid += 1
                    if eid % 10 == 0 or eid == total:
                        el = time.time() - t0
                        print(f"  {eid}/{total} episodes  {el:.0f}s elapsed"
                              + (f", ~{el / eid * (total - eid):.0f}s left"
                                 if eid < total else ""), flush=True)

    with open(os.path.join(out, "report.csv"), "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=COLUMNS, extrasaction="ignore")
        wr.writeheader()
        wr.writerows(rows)
    with open(os.path.join(out, "trace.csv"), "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["id", "step", "x", "y", "dist"])
        wr.writerows(traces)
    print(f"\n[report] {len(rows)} episodes -> {os.path.join(out, 'report.csv')}"
          f"\n[report] {len(traces)} trace rows (every {TRACE_STRIDE} steps)",
          flush=True)

    # HAND THE DRAWING BACK TO C++. One implementation, and it is the one that
    # `report --plot`, `--shot` and `--check` also use, so what CI checks is
    # what a run produces.
    n = voxelenv.draw_report(out, os.path.join(out, "report"))
    print(f"[report] {n} panel(s) written to {os.path.abspath(out)}\n"
          f"[report] redraw any time without flying again:\n"
          f"           kestrel report --plot {out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

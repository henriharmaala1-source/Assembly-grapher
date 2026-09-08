#!/usr/bin/env python3
"""Score a policy in sweep.sh's columns, so it lands in the same table.

  python3 evaluate.py --model runs/ppo_voxel/final.zip --worlds forest maze

A comparison against --traj and --histogram on NEW metrics would be worthless,
so this emits exactly what sweep.sh does: world, seed, outcome, travel,
end-dist, minClr -- plus collisions, stopped steps and goals reached.

--random gives the floor. A policy that cannot beat uniform sampling over the
ADMISSIBLE set has learned nothing, and that is a cheaper thing to discover here
than after a training run.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

# WHERE voxelenv IS depends on how you got here, and guessing "../build" was
# a build-tree assumption that silently fails everywhere else. In the release
# package python/ sits beside the exe and the module sits beside it too, so
# "../build" resolves to a directory that does not exist -- and the failure
# reads as "No module named voxelenv", which looks like a missing dependency
# rather than a wrong path.
#
# kestrel sets KESTREL_MODULE_DIR to the directory it found the module in, so
# when it launches this script the two cannot disagree. The rest are for
# running this file by hand.
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [p for p in (os.environ.get("KESTREL_MODULE_DIR"),
                            os.path.join(_here, "..", "build"),
                            os.path.join(_here, ".."),
                            _here) if p]

import voxelenv
from voxel_gym import VoxelNavEnv, newest_checkpoint


def run_episode(env, model, rng, world, seed, baseline=None):
    env.worlds = (world,)
    env.seeds = [seed]
    obs, _ = env.reset(seed=seed)
    info = {}
    while True:
        mask = env.action_masks()
        if baseline is not None:
            # Through the extension module, so this is the same code `kestrel
            # bench` runs rather than a python re-implementation of it.
            a, run_episode.rng = voxelenv.choose_baseline(
                baseline, obs, mask, len(mask), run_episode.rng)
            a = int(a)
        elif model is None:
            legal = np.flatnonzero(mask)
            a = int(rng.choice(legal)) if len(legal) else 0
        else:
            a, _ = model.predict(obs, action_masks=mask, deterministic=True)
            a = int(a)
        obs, _, done, trunc, info = env.step(a)
        if done or trunc:
            break
    outcome = ("reached goal" if info["reached_goal"]
               else "COLLIDED" if info["collisions"]
               else "ran out of steps")
    return outcome, info


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None,
                    help="a specific checkpoint. Without it the newest one in "
                         "--run is used.")
    ap.add_argument("--run", default="runs/ppo_voxel",
                    help="where to look for a checkpoint when --model is not given")
    ap.add_argument("--random", action="store_true", help="the floor: uniform over admissible")
    ap.add_argument("--worlds", nargs="+", default=["forest", "maze"])
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(101, 109)),
                    help="HELD OUT from training by default")
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--stereo", action="store_true")
    ap.add_argument("--baselines", action="store_true",
                    help="score the four classical planners on the SAME seeds "
                         "and print one comparison table. This is the whole "
                         "point of the harness; running bench and evaluate "
                         "separately gives two tables in two formats that have "
                         "to be lined up by eye.")
    ap.add_argument("--reward", action="store_true",
                    help="also print where the reward went, per term. Produced "
                         "by THIS code path rather than a side script, because "
                         "an ad-hoc harness that sets the per-world rollout "
                         "horizon differently silently measures a different "
                         "aircraft.")
    args = ap.parse_args()

    # RESOLVE THE POLICY, OR SAY SO. With no --model this used to leave model
    # as None and quietly score the RANDOM FLOOR while the caller believed it
    # was scoring a trained policy -- the two print identical tables. Anything
    # driving this from a menu would have reported the floor as the result.
    model = None
    if not args.random:
        path = args.model
        if not path:
            path, trained = newest_checkpoint(args.run)
            if not path:
                return (f"[evaluate] no checkpoint in {os.path.abspath(args.run)}.\n"
                        f"           Train one first, name one with --model, or\n"
                        f"           pass --random to score the floor deliberately.")
            print(f"[evaluate] policy: {path}"
                  + (f"  ({trained} trained steps)" if trained else ""), flush=True)
        else:
            print(f"[evaluate] policy: {path}", flush=True)
        from sb3_contrib import MaskablePPO
        model = MaskablePPO.load(path, device="cpu")
    else:
        print("[evaluate] RANDOM over admissible primitives -- the floor, not a "
              "policy", flush=True)

    env = VoxelNavEnv(worlds=tuple(args.worlds), seeds=args.seeds,
                      max_steps=args.max_steps, truth_depth=not args.stereo)
    rng = np.random.default_rng(0)

    hdr = (f"{'planner':<8} {'world':<8} {'seed':<5} {'outcome':<16} {'travel':>9} "
           f"{'end-dist':>9} {'minClr':>9} {'stopped':>8}")
    if args.reward:
        hdr += (f" {'total':>8} {'progress':>9} {'coverage':>9} {'clear':>7} "
                f"{'stop':>7} {'terminal':>9}")
    print(hdr)
    # One scoring pass, reused for the policy and for each classical planner,
    # so every row in the comparison is produced by identical code on identical
    # seeds. That equality is the only thing that makes the table mean anything.
    def score(label, mdl, baseline):
        tot = hits = reach = 0
        travels = []
        for w in args.worlds:
            for s in args.seeds:
                run_episode.rng = 12345          # same stream for every planner
                outcome, i = run_episode(env, mdl, rng, w, s, baseline)
                row = (f"{label:<8} {w:<8} {s:<5} {outcome:<16} "
                       f"{i['travel_m']:>9.1f} {i['dist_to_goal_m']:>9.1f} "
                       f"{i['min_clear_m']:>9.2f} {i['stopped_steps']:>8}")
                if args.reward:
                    # NOT named tot: that is the run counter in this scope, and
                    # shadowing it once printed "runs -91.7" in the summary.
                    rtot = (i["r_progress"] + i["r_coverage"] + i["r_time"]
                            + i["r_stop"] + i["r_clear"] + i["r_terminal"])
                    row += (f" {rtot:>8.1f} {i['r_progress']:>9.1f} "
                            f"{i['r_coverage']:>9.1f} {i['r_clear']:>7.1f} "
                            f"{i['r_stop']:>7.1f} {i['r_terminal']:>9.1f}")
                print(row, flush=True)
                tot += 1
                hits += 1 if i["collisions"] else 0
                reach += 1 if i["reached_goal"] else 0
                travels.append(i["travel_m"])
        return label, tot, hits, reach, float(np.mean(travels)) if travels else 0.0

    rows = [score("policy" if model else "random", model, None)]
    if args.baselines:
        for b in (voxelenv.Baseline.random, voxelenv.Baseline.freeM,
                  voxelenv.Baseline.goal, voxelenv.Baseline.score):
            rows.append(score(str(b).split(".")[-1], None, b))

    print("---")
    print(f"{'planner':<8} {'runs':>5} {'collisions':>11} {'goals':>6} {'mean travel':>12}")
    for label, tot, hits, reach, mt in rows:
        print(f"{label:<8} {tot:>5} {hits:>11} {reach:>6} {mt:>11.1f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

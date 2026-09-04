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

sys.path[:0] = [os.path.join(os.path.dirname(__file__), "..", "build"),
                os.path.dirname(__file__)]

from voxel_gym import VoxelNavEnv


def run_episode(env, model, rng, world, seed):
    env.worlds = (world,)
    env.seeds = [seed]
    obs, _ = env.reset(seed=seed)
    info = {}
    while True:
        mask = env.action_masks()
        if model is None:
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
    ap.add_argument("--model", default=None)
    ap.add_argument("--random", action="store_true", help="the floor: uniform over admissible")
    ap.add_argument("--worlds", nargs="+", default=["forest", "maze"])
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(101, 109)),
                    help="HELD OUT from training by default")
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--stereo", action="store_true")
    args = ap.parse_args()

    model = None
    if args.model and not args.random:
        from sb3_contrib import MaskablePPO
        model = MaskablePPO.load(args.model, device="cpu")

    env = VoxelNavEnv(worlds=tuple(args.worlds), seeds=args.seeds,
                      max_steps=args.max_steps, truth_depth=not args.stereo)
    rng = np.random.default_rng(0)

    print(f"{'world':<8} {'seed':<5} {'outcome':<16} {'travel':>9} "
          f"{'end-dist':>9} {'minClr':>9} {'stopped':>8}")
    tot = hits = reach = 0
    travels = []
    for w in args.worlds:
        for s in args.seeds:
            outcome, i = run_episode(env, model, rng, w, s)
            print(f"{w:<8} {s:<5} {outcome:<16} {i['travel_m']:>9.1f} "
                  f"{i['dist_to_goal_m']:>9.1f} {i['min_clear_m']:>9.2f} "
                  f"{i['stopped_steps']:>8}")
            tot += 1
            hits += 1 if i["collisions"] else 0
            reach += 1 if i["reached_goal"] else 0
            travels.append(i["travel_m"])
    print("---")
    print(f"runs {tot}   collisions {hits}   goals reached {reach}")
    print(f"mean travel {np.mean(travels):.1f} m   median {np.median(travels):.1f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

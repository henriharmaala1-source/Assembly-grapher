#!/usr/bin/env python3
"""Watch the policy fly while it is still training.

WHY THIS EXISTS. A training curve tells you the reward went up; it does not
tell you what the aircraft is doing. A policy that has learnt to spin on the
spot and one that has learnt to follow a corridor can sit at the same reward
for a long time, and only one of them is progress. This opens a grid of live
panes -- one episode per pane, several worlds and seeds at once -- so the
behaviour is visible while the run is still going.

IT DOES NOT TOUCH THE TRAINING RUN. It reads the checkpoints the trainer is
already writing (CheckpointCallback, every ~50k steps) and reloads the newest
one whenever it changes, so training keeps all its workers and this is a
separate, idle-priority spectator. Nothing here writes to the run directory.

RENDERING IS CPU, and so is training: the bottleneck in both is environment
steps, which are C++. Running this alongside costs roughly one worker.
"""
import argparse
import os
import sys
import time

import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [p for p in (os.environ.get("KESTREL_MODULE_DIR"),
                            os.path.join(_here, "..", "build"),
                            os.path.join(_here, ".."),
                            _here) if p]

import voxelenv  # noqa: E402,F401
from voxel_gym import newest_checkpoint  # noqa: E402

try:
    import cv2
except ImportError:
    # sys.executable, not "python". kestrel launched this script with the ONE
    # interpreter that can load voxelenv, which on a machine with several is
    # rarely the one "python" means -- and pip installing into the wrong one
    # succeeds, leaving the package present and the import still failing.
    sys.exit("watch needs opencv-python. Install it into THIS interpreter:\n"
             f'    "{sys.executable}" -m pip install opencv-python')


def load(path):
    from sb3_contrib import MaskablePPO
    return MaskablePPO.load(path, device="cpu")


def label(img, text, colour=(235, 235, 240)):
    cv2.rectangle(img, (0, 0), (img.shape[1], 18), (24, 24, 28), -1)
    cv2.putText(img, text, (5, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                colour, 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="runs/ppo_voxel",
                    help="the trainer's --out directory")
    ap.add_argument("--worlds", nargs="+", default=["forest", "maze"])
    ap.add_argument("--panes", type=int, default=4)
    ap.add_argument("--px", type=int, default=320, help="pane width")
    ap.add_argument("--layout", default="all",
                    choices=["fpv", "top", "depth", "both", "all"],
                    help="all (default): fpv with a plan inset and a DEPTH "
                         "strip below. The fpv alone is mostly white in an open "
                         "world and correctly so -- unknown is fog and at "
                         "0.25 m voxels the map only marks obstacles to about "
                         "3.5 m -- so depth is what tells you the sensor is "
                         "seeing anything at all. fpv / top / depth show one "
                         "each; both is fpv with the plan inset only.")
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--stereo", action="store_true")
    ap.add_argument("--deterministic", action="store_true",
                    help="argmax the policy instead of sampling it. OFF by "
                         "default, and that matters early: an untrained network "
                         "argmaxed picks the SAME primitive every step whatever "
                         "it sees, so every pane flies one identical slow arc "
                         "and the view looks frozen. Measured at 512 steps of "
                         "training: 20.6 m of travel in forest, corridor, road "
                         "and culdesac -- the same number in four different "
                         "worlds. Sampling shows what training is actually "
                         "doing; use --deterministic to judge a finished "
                         "policy.")
    ap.add_argument("--no-veto", action="store_true",
                    help="watch a run with the safety mask off, to see the "
                         "collisions a --no-veto policy is learning from")
    ap.add_argument("--shot", default="", metavar="FILE",
                    help="fly --shot-steps steps, write one grid PNG, exit. "
                         "No display needed, so the layout is checkable over "
                         "ssh and in CI -- the same reason `gui --shot` exists.")
    ap.add_argument("--shot-steps", type=int, default=120)
    args = ap.parse_args()

    cfg = voxelenv.EnvConfig()
    cfg.max_steps = args.max_steps
    cfg.truth_depth = not args.stereo
    cfg.mask_unsafe = not args.no_veto

    envs = [voxelenv.VoxelEnv(cfg) for _ in range(args.panes)]
    worlds = [args.worlds[i % len(args.worlds)] for i in range(args.panes)]
    seeds = [9000 + i for i in range(args.panes)]      # held out from training
    for e, w, s in zip(envs, worlds, seeds):
        e.reset(w, s)
    steps = [0] * args.panes
    outcome = [""] * args.panes
    # How long the finished-episode banner stays up. Without it the outcome is
    # gone on the next frame and a run that ended is indistinguishable from one
    # that never started.
    flash = [0] * args.panes
    eps = [0] * args.panes
    hits = [0] * args.panes
    goals = [0] * args.panes

    policy, at_steps, ckpt = None, -1, None
    cols = int(np.ceil(np.sqrt(args.panes)))
    rows = int(np.ceil(args.panes / cols))
    win = "kestrel watch - policy while training"
    if not args.shot:
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    period = 1.0 / max(1e-3, args.fps)
    last_poll = 0.0

    while True:
        # Pick up a newer checkpoint. Polled rather than watched so this works
        # the same on every platform and over a network share.
        if time.time() - last_poll > 5.0:
            last_poll = time.time()
            path, n = newest_checkpoint(args.run)
            if path and path != ckpt:
                try:
                    policy, ckpt, at_steps = load(path), path, n
                    print(f"[watch] loaded {os.path.basename(path)}", flush=True)
                except Exception as exc:                  # a half-written zip
                    print(f"[watch] {os.path.basename(path)}: {exc}", flush=True)

        tiles = []
        for i, e in enumerate(envs):
            mask = e.action_mask()
            legal = np.flatnonzero(mask)
            if policy is None:
                # No checkpoint yet. Random among the LEGAL primitives, which is
                # the same baseline `kestrel bench` reports, so the first pane
                # you see is a fair "before" rather than a frozen screen.
                a = int(np.random.choice(legal)) if len(legal) else 0
            else:
                a, _ = policy.predict(e.observation(), action_masks=mask,
                                      deterministic=args.deterministic)
                a = int(a)
            st = e.step(a)
            steps[i] += 1

            w = args.px
            h = int(w * 3 / 4)
            if args.layout == "top":
                img = np.ascontiguousarray(e.render_frame(w, h, True))
            elif args.layout == "depth":
                img = np.ascontiguousarray(e.render_depth(w, h))
            else:
                img = np.ascontiguousarray(e.render_frame(w, h, False))
                if args.layout in ("both", "all"):
                    # A small plan view in the corner. The FPV alone cannot say
                    # whether the aircraft is making ground -- a policy hovering
                    # in a clearing and one crossing it look identical through
                    # fog -- and the inset answers that at a glance.
                    k = max(56, w // 4)
                    ins = np.ascontiguousarray(e.render_frame(k, k, True))
                    img[h - ins.shape[0]:h, w - ins.shape[1]:w] = ins
                    cv2.rectangle(img, (w - ins.shape[1], h - ins.shape[0]),
                                  (w - 1, h - 1), (90, 90, 100), 1)
                if args.layout == "all":
                    # WHAT THE CAMERA RETURNED, under what the map believes.
                    # Grey is NO RETURN, not far away -- the two panes disagreeing
                    # is the interesting case: sensor sees it, map has not marked
                    # it yet.
                    dep = np.ascontiguousarray(e.render_depth(w, h // 2))
                    cv2.putText(dep, "DEPTH", (6, 14), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (240, 240, 240), 1, cv2.LINE_AA)
                    img = np.vstack([img, dep])
            if st.done or st.truncated:
                outcome[i] = ("reached goal" if st.reached_goal
                              else "COLLIDED" if st.collisions else "out of steps")
                flash[i] = 30
                eps[i] += 1
                hits[i] += 1 if st.collisions else 0
                goals[i] += 1 if st.reached_goal else 0
            elif flash[i] > 0:
                flash[i] -= 1
                if flash[i] == 0:
                    outcome[i] = ""      # STALE LABELS WERE THE BUG: this was
                                         # never cleared, so one collision left
                                         # every later frame reading COLLIDED
                                         # while a new episode flew underneath.
            label(img, f"{worlds[i]} s{seeds[i]}  {st.travel_m:5.1f}m  "
                       f"d{st.dist_to_goal_m:5.1f}m  ep{eps[i]} "
                       f"x{hits[i]} g{goals[i]}  {outcome[i]}",
                  (90, 230, 90) if st.reached_goal else
                  (90, 90, 240) if st.collisions else (235, 235, 240))
            tiles.append(img)

            if st.done or st.truncated:
                seeds[i] += args.panes
                e.reset(worlds[i], seeds[i])
                steps[i] = 0

        blank = np.zeros_like(tiles[0])
        while len(tiles) < rows * cols:
            tiles.append(blank)
        grid = np.vstack([np.hstack(tiles[r*cols:(r+1)*cols]) for r in range(rows)])
        bar = np.full((26, grid.shape[1], 3), 24, np.uint8)
        cv2.putText(bar, f"checkpoint: {os.path.basename(ckpt) if ckpt else 'none yet (random legal)'}"
                         f"   trained steps: {at_steps}   q to quit",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 210), 1, cv2.LINE_AA)
        composed = np.vstack([bar, grid])
        if args.shot:
            if min(steps) >= args.shot_steps or max(steps) >= args.shot_steps:
                cv2.imwrite(args.shot, composed)
                print(f"[watch] {args.shot}", flush=True)
                return
            continue
        cv2.imshow(win, composed)
        if cv2.waitKey(max(1, int(period * 1000))) in (ord('q'), 27):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

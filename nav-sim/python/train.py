#!/usr/bin/env python3
"""Train a maskable PPO policy to RANK trajectory primitives.

  python3 train.py --steps 10000000 --workers 16 --stereo

Sized for an i5-13600K (6 P-cores / 12 threads + 8 E-cores = 20): 16 workers,
the rest left for the learner and the OS. E-cores are ~60-70 % of P-core IPC but
env stepping is a THROUGHPUT problem, not a latency one, so they earn their keep.

ON THE GPU. It will sit nearly idle and that is the correct outcome, not a
misconfiguration -- see docs/RL_HARNESS_PLAN.md. The policy is small and the
bottleneck is environment steps, which are C++ on the CPU. `--device cuda` is
offered so the assumption can be MEASURED rather than argued about; expect CPU
to win until the per-primitive encoder gets much wider.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

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

from sb3_contrib import MaskablePPO
from sb3_contrib.common.maskable.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

from voxel_gym import TRAIN_WORLDS, make_env, newest_checkpoint


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=10_000_000)
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--worlds", nargs="+", default=list(TRAIN_WORLDS))
    ap.add_argument("--max-steps", type=int, default=3000,
                    help="steps per episode. THE GOAL MUST BE REACHABLE INSIDE "
                         "THIS. At 1500 the forest goal was not: the best "
                         "classical planner is still closing when the episode "
                         "is cut off (74.5 m out at 1500) and needs ~2500 steps "
                         "to get within 10 m. Every training episode was "
                         "therefore truncated before arrival was possible, "
                         "which made the goal bonus unreachable dead code and "
                         "is why no run in this project has ever reached a goal.")
    ap.add_argument("--stereo", action="store_true",
                    help="real sensor model. SLOWER and the only honest setting "
                         "for a final run -- dropout, Z_max and occlusion are "
                         "what make a policy transfer.")
    ap.add_argument("--cam", type=int, nargs=2, default=(160, 120))
    ap.add_argument("--device", default="cpu", choices=["cpu", "cuda"])
    ap.add_argument("--out", default="runs/ppo_voxel")
    ap.add_argument("--n-steps", type=int, default=256, help="rollout per worker")
    ap.add_argument("--vary-goal", action="store_true",
                    help="sample start and goal every episode instead of "
                         "flying one fixed journey. Every episode used to have "
                         "the SAME geometry -- forest 175 m at 36.9 deg, maze "
                         "31.8 m at 45 deg, on every seed, with only the trees "
                         "and walls moving. A policy can score well on that by "
                         "learning a compass heading and never reading the goal "
                         "channels. Harder, and not comparable with "
                         "fixed-geometry numbers.")
    ap.add_argument("--no-veto", action="store_true",
                    help="TRAIN FROM ZERO WITH NO SAFETY MASK. Normally the "
                         "geometric veto hides primitives that fly into "
                         "something, so the policy picks among options already "
                         "approved and cannot collide by choosing. With this it "
                         "can pick anything, hit walls, take the -50, and has to "
                         "learn avoidance itself. A measurement of what the veto "
                         "is worth -- not a deployment mode.")
    ap.add_argument("--resume", nargs="?", const="auto", default="",
                    metavar="CHECKPOINT",
                    help="continue from a checkpoint instead of starting over. "
                         "Bare --resume takes the furthest-along one in --out. "
                         "Without this a run ALWAYS starts from scratch, which "
                         "is what you want for a clean comparison and not what "
                         "you want after an interrupted overnight run.")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    # SAY WHERE THE POLICY IS GOING, as an absolute path. --out is relative to
    # the working directory, which for a double-clicked exe is the folder it
    # sits in and for a shell is wherever you happened to be -- so "runs/..."
    # alone does not answer "where did my weekend of training go".
    out_abs = os.path.abspath(args.out)
    print(f"[train] checkpoints and final policy -> {out_abs}", flush=True)
    kw = dict(worlds=tuple(args.worlds), max_steps=args.max_steps,
              truth_depth=not args.stereo, cam=tuple(args.cam),
              mask_unsafe=not args.no_veto, vary_goal=args.vary_goal)
    if args.no_veto:
        print("[train] NO VETO: every primitive is selectable, including ones "
              "that fly into things.\n"
              "        Expect early collisions and a slower start -- that is "
              "the experiment.", flush=True)
    venv = VecMonitor(SubprocVecEnv([make_env(i, **kw) for i in range(args.workers)]))

    # RESUMING, OR NOT, IS AN EXPLICIT CHOICE. It used to be neither: a fresh
    # model was constructed every run and the checkpoints were written but
    # never read, so an interrupted 10 M-step run could only be started again
    # from zero. The weights were on disk the whole time with nothing able to
    # load them.
    resume_from, resume_at = None, 0
    if args.resume:
        if args.resume == "auto":
            resume_from, resume_at = newest_checkpoint(args.out)
            if resume_from is None:
                print(f"[train] --resume: nothing in {out_abs} to resume from, "
                      f"starting fresh", flush=True)
        else:
            resume_from = args.resume
            if not os.path.exists(resume_from):
                return f"[train] --resume: no such checkpoint: {resume_from}"

    if resume_from:
        model = MaskablePPO.load(resume_from, env=venv, device=args.device,
                                 tensorboard_log=os.path.join(args.out, "tb"))
        # final.zip carries no step count in its name, so say so rather than
        # printing "at 0 trained steps", which reads as "it lost everything".
        where = (f"at {resume_at} trained steps" if resume_at
                 else "(final.zip does not record its step count)")
        print(f"[train] resumed {os.path.basename(resume_from)} {where}",
              flush=True)
    else:
        model = MaskablePPO(
            "MlpPolicy", venv, device=args.device, verbose=1,
            n_steps=args.n_steps, batch_size=args.workers * args.n_steps // 4,
            learning_rate=3e-4, ent_coef=0.01, gamma=0.995, gae_lambda=0.95,
            policy_kwargs=dict(net_arch=[256, 256]),
            tensorboard_log=os.path.join(args.out, "tb"))

    # Checkpoint OFTEN. A crash at hour 40 with nothing on disk is the classic
    # way to lose a weekend, and this is an unattended run by design.
    ckpt = CheckpointCallback(save_freq=max(1, 50_000 // args.workers),
                              save_path=args.out, name_prefix="ppo")

    t0 = time.time()
    # reset_num_timesteps=False on a resume, so the step counter and the
    # TensorBoard curves continue the old run instead of restarting the x axis
    # and making a continued run look like a new one that learned instantly.
    model.learn(total_timesteps=args.steps, callback=ckpt, progress_bar=True,
                reset_num_timesteps=not resume_from)
    model.save(os.path.join(args.out, "final"))
    print(f"[train] final policy -> {os.path.join(out_abs, 'final.zip')}", flush=True)
    dt = time.time() - t0
    print(f"trained {args.steps} steps in {dt/3600:.2f} h "
          f"({args.steps/max(1e-9, dt):.0f} steps/s, {args.workers} workers)")
    venv.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

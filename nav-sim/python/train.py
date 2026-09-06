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

from voxel_gym import TRAIN_WORLDS, make_env


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=10_000_000)
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--worlds", nargs="+", default=list(TRAIN_WORLDS))
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--stereo", action="store_true",
                    help="real sensor model. SLOWER and the only honest setting "
                         "for a final run -- dropout, Z_max and occlusion are "
                         "what make a policy transfer.")
    ap.add_argument("--cam", type=int, nargs=2, default=(160, 120))
    ap.add_argument("--device", default="cpu", choices=["cpu", "cuda"])
    ap.add_argument("--out", default="runs/ppo_voxel")
    ap.add_argument("--n-steps", type=int, default=256, help="rollout per worker")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    kw = dict(worlds=tuple(args.worlds), max_steps=args.max_steps,
              truth_depth=not args.stereo, cam=tuple(args.cam))
    venv = VecMonitor(SubprocVecEnv([make_env(i, **kw) for i in range(args.workers)]))

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
    model.learn(total_timesteps=args.steps, callback=ckpt, progress_bar=True)
    model.save(os.path.join(args.out, "final"))
    dt = time.time() - t0
    print(f"trained {args.steps} steps in {dt/3600:.2f} h "
          f"({args.steps/max(1e-9, dt):.0f} steps/s, {args.workers} workers)")
    venv.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

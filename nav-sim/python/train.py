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
from collections import deque

from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

import numpy as np

from voxel_gym import (TRAIN_WORLDS, VoxelNavEnv, make_env,
                       newest_checkpoint, newest_run_dir, run_root)


class Scorecard(BaseCallback):
    """Live per-world scorecard, printed each rollout and logged to TensorBoard.

    SB3 already prints ep_rew_mean and ep_len_mean. Neither answers the
    questions this project is judged on -- how far does it get, does it crash,
    does it arrive -- and reward has already disagreed with those once here: a
    retrain scored higher on its objective and flew worse on every column. So
    the training log now carries the scorecard alongside the reward, headless,
    with no window and no separate evaluation run.

    Per world, because the average over a 30 m maze and a 340 m city describes
    neither. Over a rolling window rather than the whole run, so what is shown
    is what the policy does NOW and not what it did an hour ago.
    """

    def __init__(self, window=60):
        super().__init__()
        self.window = window
        self.ep = {}                     # world -> deque of finished episodes

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            # Monitor puts "episode" in the info of the step that ended one.
            if "episode" not in info or "world" not in info:
                continue
            d = self.ep.setdefault(info["world"], deque(maxlen=self.window))
            d.append((info.get("travel_m", 0.0),
                      info.get("min_dist_to_goal_m", 0.0),
                      1 if info.get("collisions") else 0,
                      1 if info.get("reached_goal") else 0))
        return True

    def _on_rollout_end(self) -> None:
        if not self.ep:
            return
        tot = [0, 0, 0]
        print(f"\n  {'world':10} {'eps':>4} {'travel':>9} {'closest':>9} "
              f"{'crash':>7} {'goals':>7}", flush=True)
        for w in sorted(self.ep):
            e = self.ep[w]
            if not e:
                continue
            trav = sum(x[0] for x in e) / len(e)
            clos = min(x[1] for x in e)
            crash = sum(x[2] for x in e)
            goals = sum(x[3] for x in e)
            tot[0] += len(e); tot[1] += crash; tot[2] += goals
            print(f"  {w:10} {len(e):>4} {trav:>8.1f}m {clos:>8.1f}m "
                  f"{100.0*crash/len(e):>6.0f}% {goals:>7}", flush=True)
            # Per world in TensorBoard too, so the curves can be compared
            # rather than averaged into one uninformative line.
            self.logger.record(f"score/{w}/travel_m", trav)
            self.logger.record(f"score/{w}/collision_rate", crash / len(e))
            self.logger.record(f"score/{w}/goal_rate", goals / len(e))
            self.logger.record(f"score/{w}/best_closest_m", clos)
        if tot[0]:
            print(f"  {'ALL':10} {tot[0]:>4} {'':>9} {'':>9} "
                  f"{100.0*tot[1]/tot[0]:>6.0f}% {tot[2]:>7}", flush=True)
            self.logger.record("score/collision_rate", tot[1] / tot[0])
            self.logger.record("score/goal_rate", tot[2] / tot[0])


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
    ap.add_argument("--out", default="",
                    help="where the run goes. Default: a dated folder inside "
                         "kestrel-runs on your Desktop, so a run is somewhere "
                         "you can find it and two runs never mix their "
                         "checkpoints.")
    ap.add_argument("--name", default="",
                    help="name this run's folder instead of dating it")
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

    # ASKING FOR CUDA AND SILENTLY GETTING CPU is the failure this catches.
    # requirements.txt installs plain stable-baselines3, which pulls the DEFAULT
    # torch wheel -- and on Windows that wheel is CPU-only. SB3 then falls back
    # without stopping, so a run started with --device cuda trains on the CPU
    # and the only clue is a buried warning. Say it plainly instead.
    if args.device == "cuda":
        import torch
        if not torch.cuda.is_available():
            # TWO DIFFERENT FAULTS, and they need opposite fixes. A CPU-only
            # wheel reports torch.version.cuda as None; a CUDA wheel that
            # cannot see a device reports a version string and still fails,
            # which is a driver or a machine problem, not a pip problem.
            if torch.version.cuda is None:
                # ONE LINE, no continuation character. This printed a
                # trailing backslash to wrap it, which is bash syntax -- pasted
                # into PowerShell, where the continuation is a backtick, the
                # command breaks in half and fails. The audience for this
                # message is on Windows by definition.
                why = ("this is the CPU-ONLY torch wheel, which is what plain "
                       "`pip install torch` gives on Windows.\n"
                       "        Install the CUDA build into THIS interpreter "
                       "(one line):\n"
                       f'          "{sys.executable}" -m pip install '
                       "--force-reinstall --index-url "
                       "https://download.pytorch.org/whl/cu124 torch")
            else:
                why = (f"this torch IS a CUDA build (cuda {torch.version.cuda}) "
                       "but no device is visible.\n"
                       "        That is a driver or a hardware problem, not a "
                       "pip one -- check nvidia-smi.")
            return (f"[train] --device cuda, but torch {torch.__version__} "
                    "reports cuda.is_available() = False:\n"
                    f"        {why}\n"
                    "        Or run with --device cpu, which is what the "
                    "bottleneck wants anyway: environment\n"
                    "        steps are C++ on the CPU and the policy is small.")
        print(f"[train] cuda: {torch.cuda.get_device_name(0)}", flush=True)

    # A DATED FOLDER PER RUN. One shared directory would let a short run's
    # ppo_50000 sit beside a long run's ppo_9000000, and --resume picks by step
    # count -- so the short run would silently inherit the long one's weights.
    if not args.out:
        root = run_root()
        # Resuming without naming a directory means "carry on with what I was
        # doing", which is the newest run rather than a fresh empty folder.
        if args.resume == "auto" and newest_run_dir(root):
            args.out = newest_run_dir(root)
        else:
            args.out = os.path.join(
                root, args.name or time.strftime("run-%Y%m%d-%H%M%S"))

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
    model.learn(total_timesteps=args.steps, callback=[ckpt, Scorecard()], progress_bar=True,
                reset_num_timesteps=not resume_from)
    model.save(os.path.join(args.out, "final"))
    print(f"[train] final policy -> {os.path.join(out_abs, 'final.zip')}", flush=True)

    # WHAT THE RUN ACTUALLY PRODUCED, in the columns the scorecard uses.
    # A training log ends on a reward number, and reward has already disagreed
    # with the scorecard once in this project -- a retrain scored higher on its
    # objective and flew worse on every column. So the run reports what it can
    # be judged on: per world, does it arrive, does it crash, how far does it
    # get. Held-out seeds, and the same episode budget it trained with.
    print("\n[train] scoring the final policy on held-out seeds "
          f"(max {args.max_steps} steps)...", flush=True)
    try:
        from voxel_gym import EVAL_WORLDS
        rows = []
        for w in args.worlds if args.worlds else EVAL_WORLDS:
            env = VoxelNavEnv(worlds=(w,), seeds=[901, 902, 903],
                              max_steps=args.max_steps,
                              truth_depth=not args.stereo,
                              mask_unsafe=not args.no_veto,
                              vary_goal=args.vary_goal)
            trav, coll, reach, closest = [], 0, 0, []
            for sd in (901, 902, 903):
                obs, _ = env.reset(seed=sd)
                info = {}
                for _ in range(args.max_steps):
                    m = env.action_masks()
                    a, _st = model.predict(obs, action_masks=m, deterministic=True)
                    obs, _r, done, trunc, info = env.step(int(a))
                    if done or trunc:
                        break
                trav.append(info["travel_m"])
                closest.append(info["min_dist_to_goal_m"])
                coll += 1 if info["collisions"] else 0
                reach += 1 if info["reached_goal"] else 0
            rows.append((w, float(np.mean(trav)), coll, reach, float(np.min(closest))))

        print(f"\n{'world':10} {'mean travel':>12} {'collisions':>11} "
              f"{'goals':>6} {'best closest':>13}")
        for w, mt, c, g, cl in rows:
            print(f"{w:10} {mt:>11.1f} m {c:>8}/3 {g:>5}/3 {cl:>11.1f} m")
        print(f"{'TOTAL':10} {np.mean([r[1] for r in rows]):>11.1f} m "
              f"{sum(r[2] for r in rows):>8}/{3*len(rows)} "
              f"{sum(r[3] for r in rows):>5}/{3*len(rows)}")
        print("\nCompare against the classical planners on the same seeds:\n"
              f"  kestrel evaluate --baselines --run {args.out} "
              f"--max-steps {args.max_steps}"
              + (" --vary-goal" if args.vary_goal else "")
              + (" --stereo" if args.stereo else ""), flush=True)
    except Exception as exc:                      # never lose a finished run
        print(f"[train] end-of-run scoring failed ({exc}); the policy is saved "
              "and `kestrel evaluate` will score it.", flush=True)
    dt = time.time() - t0
    print(f"trained {args.steps} steps in {dt/3600:.2f} h "
          f"({args.steps/max(1e-9, dt):.0f} steps/s, {args.workers} workers)")
    venv.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

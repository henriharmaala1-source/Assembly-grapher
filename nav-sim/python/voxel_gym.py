"""Gymnasium wrapper over the C++ VoxelEnv.

The action is an INDEX into the trajectory library, and only primitives
sphereClear has already admitted are legal. `action_masks()` exposes that to
MaskablePPO, so the policy never has to learn admissibility -- it learns
PREFERENCE among options the geometry approved. The hard veto stays where it
belongs, and the failure mode where a learned model steers into space nothing
measured is unreachable rather than merely unlikely.
"""
from __future__ import annotations

import glob
import os
import re

import gymnasium as gym
import numpy as np
from gymnasium import spaces

import voxelenv


# Held out from training on purpose. If a policy only works where it trained it
# has learned four worlds rather than navigation, and the maze is the case that
# motivated memory in the first place.
# FIVE STYLES, SO WHAT IS LEARNED IS AVOIDANCE AND NOT A WORLD.
# Training on forest and maze alone lets a policy learn two modes and pick
# between them from the first frame. The obstacle geometry differs completely
# across these -- trunks, corridor walls, building blocks, thin poles and
# overhead wires, and a trap with a single way out -- so a policy that handles
# all five is doing something more general than one that handles two.
TRAIN_WORLDS = ("forest", "maze", "corridor", "city", "road", "culdesac")
EVAL_WORLDS = TRAIN_WORLDS


class VoxelNavEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, worlds=TRAIN_WORLDS, seeds=range(1, 65), max_steps=1500,
                 truth_depth=False, cam=(160, 120), horizons=None,
                 mask_unsafe=True, vary_goal=False, scale_clear=True):
        super().__init__()
        self.worlds = tuple(worlds)
        self.seeds = list(seeds)
        # The rollout horizon is per-world: the maze wants ~0.6 s and the forest
        # ~2.0 s, because the optimum tracks the size of the space rather than
        # the scene type. Training both at one value handicaps one of them.
        # The rollout horizon tracks the SIZE OF THE SPACE, not the scene type:
        # a maze wants ~0.6 s and open forest ~2.0 s, and training everything at
        # one value handicaps whichever end it is not tuned for.
        self.horizons = horizons or {"maze": 0.6, "corridor": 0.9, "city": 1.2,
                                     "culdesac": 1.6, "forest": 2.0, "road": 2.0}
        self._cfg = voxelenv.EnvConfig()
        self._cfg.max_steps = max_steps
        self._cfg.truth_depth = truth_depth
        self._cfg.cam_w, self._cfg.cam_h = cam
        self._cfg.world = self.worlds[0]
        self._cfg.horizon_s = self.horizons[self.worlds[0]]
        # False = the policy may pick primitives the geometry rejected, so it
        # can fly into things and must learn avoidance from the consequence.
        self._cfg.mask_unsafe = mask_unsafe
        # Sample start and goal per episode; see EnvConfig::varyGoal.
        self._cfg.vary_goal = vary_goal
        # The near-miss penalty scaled like progress, so avoidance pressure is
        # the same relative to progress in a 35 m maze as in a 340 m city.
        self._cfg.scale_clear = scale_clear
        self._env = voxelenv.VoxelEnv(self._cfg)

        n = self._env.n_prims
        self.action_space = spaces.Discrete(n)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._env.obs_size,), dtype=np.float32)
        self._rng = np.random.default_rng(0)
        self._last = None

    # MaskablePPO looks for this by name.
    def action_masks(self) -> np.ndarray:
        m = self._env.action_mask()
        # A step with nothing admissible would leave the policy no legal action
        # and break the sampler. Allow index 0 (a hold) so the episode can
        # continue and the reward -- not an exception -- reports the situation.
        if not m.any():
            m = m.copy()
            m[0] = True
        return m

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        world = self.worlds[int(self._rng.integers(len(self.worlds)))]
        s = int(self._rng.choice(self.seeds))
        # Kept so every STEP's info can name its world. A training callback
        # cannot bucket a result by world it cannot see, and "mean travel" over
        # a 30 m maze and a 340 m city mixed together says nothing about either.
        self._world, self._seed = world, s
        self._cfg.horizon_s = self.horizons[world]
        # horizon_s is read at construction, so a world with a different horizon
        # needs a fresh env rather than a reset.
        self._env = voxelenv.VoxelEnv(self._cfg)
        self._env.reset(world, s)
        self._last = None
        return (self._env.observation(),
                {"world": world, "seed": s,
                 "start_dist_m": self._env.start_dist_m})


    def step(self, action):
        st = self._env.step(int(action))
        self._last = st
        info = {
            "travel_m": st.travel_m, "dist_to_goal_m": st.dist_to_goal_m,
            "min_clear_m": st.min_clear_m,
            "min_dist_to_goal_m": st.min_dist_to_goal_m,
            "min_dist_step": st.min_dist_step, "collisions": st.collisions,
            "stopped_steps": st.stopped_steps, "steps": st.steps,
            "reached_goal": st.reached_goal,
            "world": getattr(self, "_world", self.worlds[0]),
            "seed": getattr(self, "_seed", 0),
            # Episode totals per reward term. A scalar return says a policy
            # improved and cannot say WHICH term it improved, which is the only
            # question worth asking when reward rises while the scorecard falls.
            "r_progress": st.r_progress, "r_coverage": st.r_coverage,
            "r_time": st.r_time, "r_stop": st.r_stop,
            "r_clear": st.r_clear, "r_terminal": st.r_terminal,
        }
        return self._env.observation(), float(st.reward), bool(st.done), bool(st.truncated), info


# WHAT ONE EPISODE CAN ACTUALLY COVER. dt is 0.1 s and vMax 3 m/s, so the
# ceiling is 0.3 m per step -- but nothing flies at vMax: speed is one of the
# three primitive dimensions and a planner spends most of it turning. Measured
# over 3000-step episodes with the best classical planner, path length came out
# at 260-278 m in every one of the six worlds. It saturates, and the number is
# the same everywhere, so it is a property of the vehicle rather than of a map:
#
#   world      travel in 3000 steps (freeM, 6 seeds)
#   city       96.8 / 126.5 / 260.7 / 268.1 / 278.4
#   road       153.0 / 247.2 / 253.9 / 258.1 / 259.6 / 261.8
#   culdesac   259.9 / 262.7 / 264.8 / 264.8 / 267.4 / 268.6
#
# 0.09 m/step is the low end of that, used deliberately: a warning that fires
# on a journey a policy could just about make is worth less than one that only
# fires on a journey nothing can.
CRUISE_M_PER_STEP = 0.09


def journey_fit(worlds, max_steps, seeds=(1, 2, 3), **kw):
    """Per world: the nominal journey, and the steps it needs at cruise.

    A GOAL BEYOND THE EPISODE BUDGET IS INVISIBLE AND EXPENSIVE. It has
    happened twice in this project -- the 1500-step cap that put the forest
    goal out of reach, and a city goal 367.7 m away when an episode covers
    ~270 m -- and both times it read as "the policy cannot learn this world"
    for millions of steps rather than as "this episode cannot contain this
    journey". It costs one reset per world to ask.
    """
    rows = []
    for w in worlds:
        d = []
        for sd in seeds:
            env = VoxelNavEnv(worlds=(w,), seeds=[sd], max_steps=max_steps, **kw)
            _obs, info = env.reset(seed=sd)
            d.append(info["start_dist_m"])
        far = max(d)
        rows.append((w, sum(d) / len(d), far,
                     int(far / CRUISE_M_PER_STEP), far / CRUISE_M_PER_STEP > max_steps))
    return rows


def make_env(rank: int, worlds=TRAIN_WORLDS, **kw):
    def _init():
        env = VoxelNavEnv(worlds=worlds, **kw)
        env.reset(seed=1000 + rank)
        return env
    return _init


def newest_checkpoint(run_dir):
    """(path, trained_steps) of the furthest-along checkpoint, or (None, 0).

    BY STEP COUNT IN THE NAME, not by mtime. The trainer writes
    ppo_<n>_steps.zip as it goes and final.zip at the end, so picking the most
    recently modified file would happily resume a 10 M-step run from a
    final.zip that a short earlier run left in the directory -- silently
    throwing the long run away. The step count is the only ordering that means
    what it says.

    Shared by train.py (to resume) and watch.py (to follow along) so the two
    cannot disagree about which checkpoint is current.
    """
    best, best_n = None, -1
    for f in glob.glob(os.path.join(run_dir, "ppo_*_steps.zip")):
        m = re.search(r"_(\d+)_steps\.zip$", f)
        if m and int(m.group(1)) > best_n:
            best, best_n = f, int(m.group(1))
    if best is None:
        f = os.path.join(run_dir, "final.zip")
        if os.path.exists(f):
            return f, 0
    return best, max(best_n, 0)


def all_checkpoints(run_dir):
    """Every ppo_<n>_steps.zip in training order, as (steps, path).

    Ordered by the step count in the NAME, like newest_checkpoint: a
    progression plotted in mtime order would reorder itself the moment a
    directory is copied.
    """
    out = []
    for f in glob.glob(os.path.join(run_dir, "ppo_*_steps.zip")):
        m = re.search(r"_(\d+)_steps\.zip$", f)
        if m:
            out.append((int(m.group(1)), f))
    return sorted(out)


def run_root():
    """Where runs go by default: a `kestrel-runs` folder on the Desktop.

    OneDrive is checked because a Windows Desktop is very often redirected
    into it, and writing to the literal %USERPROFILE%\\Desktop then puts the
    run somewhere the user does not see. Falls back to ./runs when there is no
    Desktop at all, which is the normal case on a server.
    """
    home = os.path.expanduser("~")
    for cand in (os.path.join(home, "OneDrive", "Desktop"),
                 os.path.join(home, "Desktop")):
        if os.path.isdir(cand):
            return os.path.join(cand, "kestrel-runs")
    return os.path.abspath("runs")


def newest_run_dir(root):
    """The most recently written run folder under root, or None.

    By mtime here, unlike checkpoints within a run: these are separate runs
    rather than steps of one, so "the one I was just working on" is what is
    meant and there is no step count to order by.
    """
    if not os.path.isdir(root):
        return None
    subs = [os.path.join(root, d) for d in os.listdir(root)
            if os.path.isdir(os.path.join(root, d))]
    return max(subs, key=os.path.getmtime) if subs else None

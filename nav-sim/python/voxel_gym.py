"""Gymnasium wrapper over the C++ VoxelEnv.

The action is an INDEX into the trajectory library, and only primitives
sphereClear has already admitted are legal. `action_masks()` exposes that to
MaskablePPO, so the policy never has to learn admissibility -- it learns
PREFERENCE among options the geometry approved. The hard veto stays where it
belongs, and the failure mode where a learned model steers into space nothing
measured is unreachable rather than merely unlikely.
"""
from __future__ import annotations

import gymnasium as gym
import numpy as np
from gymnasium import spaces

import voxelenv


# Held out from training on purpose. If a policy only works where it trained it
# has learned four worlds rather than navigation, and the maze is the case that
# motivated memory in the first place.
TRAIN_WORLDS = ("forest", "maze")
EVAL_WORLDS = ("forest", "maze")


class VoxelNavEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, worlds=TRAIN_WORLDS, seeds=range(1, 65), max_steps=1500,
                 truth_depth=False, cam=(160, 120), horizons=None):
        super().__init__()
        self.worlds = tuple(worlds)
        self.seeds = list(seeds)
        # The rollout horizon is per-world: the maze wants ~0.6 s and the forest
        # ~2.0 s, because the optimum tracks the size of the space rather than
        # the scene type. Training both at one value handicaps one of them.
        self.horizons = horizons or {"maze": 0.6, "forest": 2.0}
        self._cfg = voxelenv.EnvConfig()
        self._cfg.max_steps = max_steps
        self._cfg.truth_depth = truth_depth
        self._cfg.cam_w, self._cfg.cam_h = cam
        self._cfg.world = self.worlds[0]
        self._cfg.horizon_s = self.horizons[self.worlds[0]]
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
        self._cfg.horizon_s = self.horizons[world]
        # horizon_s is read at construction, so a world with a different horizon
        # needs a fresh env rather than a reset.
        self._env = voxelenv.VoxelEnv(self._cfg)
        self._env.reset(world, s)
        self._last = None
        return self._env.observation(), {"world": world, "seed": s}

    def step(self, action):
        st = self._env.step(int(action))
        self._last = st
        info = {
            "travel_m": st.travel_m, "dist_to_goal_m": st.dist_to_goal_m,
            "min_clear_m": st.min_clear_m, "collisions": st.collisions,
            "stopped_steps": st.stopped_steps, "steps": st.steps,
            "reached_goal": st.reached_goal,
        }
        return self._env.observation(), float(st.reward), bool(st.done), bool(st.truncated), info


def make_env(rank: int, worlds=TRAIN_WORLDS, **kw):
    def _init():
        env = VoxelNavEnv(worlds=worlds, **kw)
        env.reset(seed=1000 + rank)
        return env
    return _init

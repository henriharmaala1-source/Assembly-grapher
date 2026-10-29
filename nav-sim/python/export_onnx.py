"""Export a trained checkpoint to ONNX, so `kestrel demo` can fly it.

WHY THIS EXISTS. The demo is one binary and its whole claim is that there is
nothing to install on the machine you are showing it on. A MaskablePPO .zip
needs PyTorch, stable-baselines3 and sb3-contrib, which is precisely the
dependency kestrel.cpp goes to such lengths to keep out of every command except
training. An ONNX graph needs OpenCV's dnn, which is already linked in.

WHAT IS EXPORTED, and what is NOT. Only the ACTOR: observation in, action
logits out. The value head is not in the graph because the demo never asks what
a state is worth, and the action MASK is not in the graph either -- masking is
done where the mask lives, in the environment, by the same rule the trainer
uses (masked entries to -inf before the argmax). Baking the mask in would mean
the exported network took two inputs and the caller could still get it wrong;
leaving it out means the caller CANNOT forget, because a raw argmax over 210
logits visibly flies into things.

NOTHING IS NORMALISED ON THE WAY IN. train.py builds VecNormalize with
norm_obs=False -- it normalises the RETURN, not the observation -- so the graph
takes the environment's raw 1914 floats. If that ever changes, this script must
bake the running mean and variance in as constants, and the check below will be
the thing that notices: a normaliser missing from one side moves every action.

THE CHECK IS THE POINT. An exported graph that is subtly wrong produces a demo
that flies badly and looks like a bad policy, which is unfalsifiable from the
outside -- so this does not just write the file. It replays real observations
from a real VoxelEnv through BOTH the PyTorch policy and the exported graph, as
loaded by the same cv2.dnn the C++ demo uses, and refuses to leave a file behind
unless every masked argmax agrees. That is the cuda_depth_check pattern: two
implementations of one thing, compared on real input, with the build failing
rather than the difference being discovered later.

    kestrel demo --model policy.onnx     what this produces
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

# THE SAME BOOTSTRAP train.py USES, and it has to be: kestrel sets
# KESTREL_MODULE_DIR to the directory it found voxelenv in, so when this is
# launched through `kestrel train --script export_onnx.py` the two cannot
# disagree. The rest are for running the file by hand.
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [p for p in (os.environ.get("KESTREL_MODULE_DIR"),
                            os.path.join(_here, "..", "build"),
                            os.path.join(_here, ".."),
                            _here) if p]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # OPTIONAL, like report.py's and evaluate.py's. Naming nothing means the
    # newest checkpoint in the newest run, which is what you want right after
    # training and is what lets the window offer this as one button.
    ap.add_argument("checkpoint", nargs="?", default="",
                    help="a ppo_*.zip or final.zip; default is the newest "
                         "checkpoint in the newest run")
    ap.add_argument("--out", default="", help="default: beside the checkpoint")
    ap.add_argument("--world", default="forest",
                    help="which world the agreement check flies in")
    ap.add_argument("--steps", type=int, default=200,
                    help="observations to compare. These are REAL ones: a graph "
                         "that agrees on random noise and disagrees on the "
                         "manifold the policy actually sees has been checked "
                         "against nothing.")
    ap.add_argument("--opset", type=int, default=12)
    args = ap.parse_args()

    import torch
    from sb3_contrib import MaskablePPO

    # NAME THE INTERPRETER, never "python". On a machine with several, pip
    # installing into the wrong one succeeds and the import still fails, which
    # is the most confusing state available -- so the line printed here is the
    # one that will work, by absolute path.
    try:
        import onnx  # noqa: F401  (torch.onnx.export needs it present)
    except ImportError:
        print("[export] torch.onnx.export needs the onnx package, which is not\n"
              "         installed in this interpreter. Install it with:\n\n"
              f'           "{sys.executable}" -m pip install onnx\n')
        return 1

    ckpt = args.checkpoint
    if not ckpt:
        from voxel_gym import newest_checkpoint, newest_run_dir, run_root
        run = newest_run_dir(run_root())
        ckpt = newest_checkpoint(run)[0] if run else None
        if not ckpt:
            print("[export] no checkpoint named and none found. Train one "
                  "first, or name a .zip.")
            return 1
        print(f"[export] newest checkpoint: {ckpt}")
    out = args.out or os.path.splitext(ckpt)[0] + ".onnx"
    model = MaskablePPO.load(ckpt, device="cpu")
    policy = model.policy.eval()
    nobs = int(np.prod(model.observation_space.shape))
    nact = int(model.action_space.n)
    print(f"[export] {ckpt}: {nobs} inputs -> {nact} action logits")

    class Actor(torch.nn.Module):
        """The actor, flattened to one graph: features, policy trunk, logits."""

        def __init__(self, p):
            super().__init__()
            self.p = p

        def forward(self, obs):
            feats = self.p.extract_features(obs)
            # SB3 splits the extractor in two when the policy and value trunks
            # do not share weights; extract_features then returns a pair and
            # the actor wants the first of them.
            if isinstance(feats, tuple):
                feats = feats[0]
            return self.p.action_net(self.p.mlp_extractor.forward_actor(feats))

    dummy = torch.zeros(1, nobs, dtype=torch.float32)
    torch.onnx.export(
        Actor(policy), dummy, out,
        input_names=["obs"], output_names=["logits"],
        opset_version=args.opset,
        dynamo=False,
        # NO DYNAMIC BATCH. The demo asks one question at a time, and a fixed
        # shape is the shape cv2.dnn optimises for.
        dynamic_axes=None)
    print(f"[export] wrote {out} ({os.path.getsize(out) / 1024:.0f} KiB)")

    # ---- the agreement check ------------------------------------------------
    import cv2
    import voxelenv
    from voxel_gym import VoxelNavEnv

    net = cv2.dnn.readNet(out)
    if net.empty():
        print("[export] FAILED: opencv could not read back what was just written")
        os.remove(out)
        return 1

    env = VoxelNavEnv(worlds=(args.world,), seeds=[101], max_steps=args.steps + 10,
                      objective="range")
    obs, _ = env.reset(seed=101, options={"world": args.world, "seed": 101})
    agree = disagree = 0
    worst = 0.0
    for _ in range(args.steps):
        mask = np.asarray(env.action_masks(), dtype=bool)

        with torch.no_grad():
            ref = Actor(policy)(torch.as_tensor(obs, dtype=torch.float32)
                                .reshape(1, -1)).numpy().reshape(-1)
        net.setInput(np.asarray(obs, dtype=np.float32).reshape(1, -1))
        got = np.asarray(net.forward()).reshape(-1)

        worst = max(worst, float(np.max(np.abs(ref - got))))
        # THE COMPARISON THAT MATTERS IS THE ARGMAX UNDER THE MASK, not the
        # logits. Two graphs can differ in the last decimal of every logit and
        # still fly identically; they can also agree everywhere except on the
        # one primitive that was going to be chosen, which is the only
        # difference that reaches the aircraft.
        def pick(v):
            z = v.copy()
            z[~mask[:len(z)]] = -np.inf
            return int(np.argmax(z))

        a_ref, a_got = pick(ref), pick(got)
        if a_ref == a_got:
            agree += 1
        else:
            disagree += 1
            if disagree <= 5:
                print(f"[export]   step chose {a_got} but torch chose {a_ref}")
        obs, _, term, trunc, _ = env.step(a_ref)
        if term or trunc:
            obs, _ = env.reset(seed=101, options={"world": args.world, "seed": 101})

    print(f"[export] agreement over {agree + disagree} real observations: "
          f"{agree} identical, {disagree} different")
    print(f"[export] largest logit difference: {worst:.3e}")
    if disagree:
        print("[export] FAILED: the exported graph would fly differently. "
              "Removing it rather than leaving a file that looks usable.")
        os.remove(out)
        return 1
    print(f"[export] OK. kestrel demo --model {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

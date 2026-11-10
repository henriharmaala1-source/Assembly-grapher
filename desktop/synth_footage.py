#!/usr/bin/env python3
"""
Realistic test-footage generator for the lock-on stack.

WHY THIS EXISTS
---------------
`simtrack.py`'s scenarios put a high-contrast blob on a WHITE-NOISE background.
Measured, a random 28px patch of that background matches elsewhere at NCC 0.08;
a 1/f "natural image" background matches at 0.62 (p95 0.81). White noise has no
self-similar structure, so nothing in the scene ever competes with the target —
which is why almost every scenario scores 100% and the suite stops discriminating
between good and bad tracker changes.

This generates clips with the properties that actually break trackers on a real
analog feed:
  * 1/f natural background + hard straight edges (roads, roofs, poles) — the
    structure that produces false NCC peaks
  * small, LOW-CONTRAST targets (a drone at range is a few dim pixels)
  * interlaced CVBS comb on motion + limited horizontal bandwidth + chroma bleed
  * motion blur, sensor noise, auto-gain pumping
  * handheld/airframe shake (high-frequency, not a smooth pan)
  * clutter distractors that genuinely resemble the target

Output is an mp4 + a labels CSV in the format `eval_tracker.py` already reads, so
the SAME code path that will run real dongle clips is exercised here. Drop a real
clip in beside these and the comparison is apples-to-apples.

Usage:
  python3 synth_footage.py --out clips/            # generate the whole battery
  python3 synth_footage.py --out clips/ --only hard_clutter
"""
import argparse
import csv
import os

import cv2
import numpy as np


# --------------------------------------------------------------- scene pieces
def pink_field(h, w, rng, beta=1.5):
    """1/f^beta noise — natural-image spectral statistics."""
    F = np.fft.fftshift(np.fft.fft2(rng.randn(h, w)))
    fy = np.fft.fftshift(np.fft.fftfreq(h))[:, None]
    fx = np.fft.fftshift(np.fft.fftfreq(w))[None, :]
    r = np.sqrt(fy ** 2 + fx ** 2)
    r[r == 0] = 1e-6
    img = np.real(np.fft.ifft2(np.fft.ifftshift(F / r ** beta)))
    img -= img.min()
    img /= (img.max() + 1e-9)
    return img


def build_world(rng, H=520, W=1400):
    """A wide scene we pan across. 1/f base + man-made straight structure."""
    base = pink_field(H, W, rng) * 110 + 45
    # Roads / rooftops / poles: long straight high-contrast edges. These are what
    # a template-matcher false-locks onto, and white noise has none of them.
    for _ in range(14):
        x0, y0 = rng.randint(0, W), rng.randint(0, H)
        L, th = rng.randint(60, 380), rng.randint(3, 16)
        val = float(rng.uniform(25, 205))
        if rng.rand() < 0.5:
            base[y0:y0 + th, x0:x0 + L] = val
        else:
            base[y0:y0 + L, x0:x0 + th] = val
    # Blobby clutter: bushes/vehicles/roof furniture — target-like lumps.
    for _ in range(40):
        cx, cy = rng.randint(0, W), rng.randint(0, H)
        rad = rng.randint(3, 13)
        yy, xx = np.mgrid[max(0, cy - rad):cy + rad, max(0, cx - rad):cx + rad]
        if yy.size == 0:
            continue
        m = (xx - cx) ** 2 + (yy - cy) ** 2 <= rad * rad
        sl = base[max(0, cy - rad):cy + rad, max(0, cx - rad):cx + rad]
        if sl.shape == m.shape:
            sl[m] = float(rng.uniform(35, 200))
    return np.clip(base, 0, 255).astype(np.float32)


def stamp_target(img, cx, cy, rad, contrast, rng, elong=1.0, ang=0.0):
    """Low-contrast target: `contrast` is the delta over LOCAL background."""
    h, w = img.shape
    x0, x1 = int(max(0, cx - rad * 2)), int(min(w, cx + rad * 2))
    y0, y1 = int(max(0, cy - rad * 2)), int(min(h, cy + rad * 2))
    if x1 <= x0 or y1 <= y0:
        return
    yy, xx = np.mgrid[y0:y1, x0:x1]
    a = np.radians(ang)
    px = (xx - cx) * np.cos(a) + (yy - cy) * np.sin(a)
    py = -(xx - cx) * np.sin(a) + (yy - cy) * np.cos(a)
    body = (px / (rad * elong)) ** 2 + (py / rad) ** 2 <= 1.0
    local = float(img[y0:y1, x0:x1].mean())
    img[y0:y1, x0:x1][body] = np.clip(local + contrast, 0, 255)
    nose = body & (px > rad * elong * 0.45)          # small asymmetric marking
    img[y0:y1, x0:x1][nose] = np.clip(local + contrast * 0.35, 0, 255)


# ------------------------------------------------------------- CVBS pipeline
def analog_degrade(y, rng, prev_y=None, interlace=True, blur=0.0,
                   noise=4.0, gain=1.0):
    """Make a clean luma frame look like it came off a CVBS capture dongle."""
    out = y.astype(np.float32)
    if blur > 0:                                   # motion blur along travel
        k = max(3, int(blur) | 1)
        ker = np.zeros((k, k), np.float32); ker[k // 2, :] = 1.0 / k
        out = cv2.filter2D(out, -1, ker)
    # Composite video is horizontally band-limited far more than vertically.
    out = cv2.GaussianBlur(out, (5, 1), 1.1)
    if interlace and prev_y is not None:
        # Odd field is half a frame older -> comb teeth on anything moving.
        out[1::2, :] = prev_y[1::2, :]
    out *= gain                                     # AGC pumping
    out += rng.randn(*out.shape) * noise            # sensor + line noise
    return np.clip(out, 0, 255)


# ------------------------------------------------------------------- clips
def clip_frames(spec, rng):
    """Yield (bgr, cx, cy) per frame for a clip spec."""
    H, W = 240, 320
    world = build_world(rng)
    wh, ww = world.shape
    n = spec["frames"]
    prev = None
    shake_x = shake_y = 0.0
    for i in range(n):
        t = i / max(1, n - 1)
        # camera pan across the world + airframe shake (high-frequency)
        panx = spec["pan"][0] * i
        pany = spec["pan"][1] * i
        shake_x = 0.75 * shake_x + rng.randn() * spec["shake"]
        shake_y = 0.75 * shake_y + rng.randn() * spec["shake"]
        ox = np.clip(120 + panx + shake_x, 0, ww - W - 1)
        oy = np.clip(120 + pany + shake_y, 0, wh - H - 1)
        view = world[int(oy):int(oy) + H, int(ox):int(ox) + W].copy()

        # target path, in view coordinates
        cx, cy = spec["path"](i, t)
        rad = spec["rad"](t)
        stamp_target(view, cx, cy, rad, spec["contrast"](t), rng,
                     elong=spec.get("elong", 1.0), ang=spec.get("ang", lambda t: 0.0)(t))
        # optional clutter distractor that looks like the target
        if spec.get("distractor"):
            dx, dy = spec["distractor"](i, t)
            stamp_target(view, dx, dy, rad, spec["contrast"](t) * 0.95, rng,
                         elong=spec.get("elong", 1.0))
        if spec.get("occ") and spec["occ"](i):
            ox2, oy2, orad = spec["occ"](i)
            view[max(0, int(oy2 - orad)):int(oy2 + orad),
                 max(0, int(ox2 - orad)):int(ox2 + orad)] = 62.0

        gain = 1.0 + spec["agc"] * np.sin(i * 0.21)
        y = analog_degrade(view, rng, prev, interlace=spec["interlace"],
                           blur=spec["blur"], noise=spec["noise"], gain=gain)
        prev = view
        # colour: low chroma bandwidth, target slightly tinted
        bgr = cv2.cvtColor(y.astype(np.uint8), cv2.COLOR_GRAY2BGR).astype(np.float32)
        bgr[..., 0] *= 1.06; bgr[..., 2] *= 0.96
        bgr = cv2.GaussianBlur(bgr, (7, 1), 2.0)          # chroma bleed
        yield np.clip(bgr, 0, 255).astype(np.uint8), cx, cy


def specs():
    """Battery, easy -> brutal. `rad` in px, `contrast` is delta over local bg."""
    S = {}
    S["a_baseline"] = dict(
        frames=90, pan=(0.0, 0.0), shake=0.0, blur=0.0, noise=3.0, agc=0.0,
        interlace=False, rad=lambda t: 11.0, contrast=lambda t: 70.0,
        path=lambda i, t: (70 + i * 2.4, 120.0))
    S["b_analog"] = dict(                       # same, but through the CVBS chain
        frames=90, pan=(0.0, 0.0), shake=0.6, blur=3.0, noise=7.0, agc=0.06,
        interlace=True, rad=lambda t: 11.0, contrast=lambda t: 70.0,
        path=lambda i, t: (70 + i * 2.4, 120.0))
    S["c_lowcontrast"] = dict(                  # dim target, the range case
        frames=90, pan=(0.0, 0.0), shake=0.6, blur=3.0, noise=7.0, agc=0.10,
        interlace=True, rad=lambda t: 7.0, contrast=lambda t: 26.0,
        path=lambda i, t: (70 + i * 2.4, 120.0))
    S["d_pan_shake"] = dict(                    # airframe motion over clutter
        frames=100, pan=(1.6, 0.35), shake=2.2, blur=4.0, noise=8.0, agc=0.10,
        interlace=True, rad=lambda t: 9.0, contrast=lambda t: 64.0,
        path=lambda i, t: (150 + 26 * np.sin(i * 0.13), 120 + 20 * np.cos(i * 0.11)))
    S["e_recede"] = dict(                       # flies away: shrinks AND fades
        frames=110, pan=(0.3, 0.0), shake=1.0, blur=3.0, noise=7.0, agc=0.08,
        interlace=True, rad=lambda t: 14.0 - 9.5 * t, contrast=lambda t: 70.0 - 46.0 * t,
        path=lambda i, t: (150 + i * 0.7, 120.0))
    S["f_maneuver"] = dict(                     # hard turns, blur, clutter
        frames=110, pan=(0.8, 0.0), shake=1.8, blur=3.0, noise=8.0, agc=0.10,
        interlace=True, rad=lambda t: 10.0, contrast=lambda t: 82.0,
        path=lambda i, t: (160 + 62 * np.sin(i * 0.20), 120 + 42 * np.sin(i * 0.31)))
    S["g_occlusion"] = dict(                    # passes behind structure
        frames=110, pan=(0.5, 0.0), shake=1.2, blur=4.0, noise=8.0, agc=0.08,
        interlace=True, rad=lambda t: 10.0, contrast=lambda t: 50.0,
        path=lambda i, t: (60 + i * 2.1, 120.0),
        occ=lambda i: (60 + 38 * 2.1, 120, 30) if 34 <= i <= 52 else None)
    S["h_clutter_distractor"] = dict(           # a second, similar object crosses
        frames=110, pan=(0.6, 0.0), shake=1.5, blur=4.0, noise=8.0, agc=0.10,
        interlace=True, rad=lambda t: 9.0, contrast=lambda t: 45.0,
        path=lambda i, t: (60 + i * 2.0, 118.0),
        distractor=lambda i, t: (105 + i * 1.72, 132.0))
    S["i_worst"] = dict(                        # everything at once
        frames=130, pan=(1.4, 0.4), shake=2.6, blur=4.0, noise=10.0, agc=0.14,
        interlace=True, rad=lambda t: 10.0 - 2.0 * t, contrast=lambda t: 72.0 - 18.0 * t,
        elong=2.2, ang=lambda t: t * 220.0,
        path=lambda i, t: (150 + 58 * np.sin(i * 0.17), 120 + 38 * np.cos(i * 0.23)),
        distractor=lambda i, t: (150 + 58 * np.sin(i * 0.17 + 2.2), 120 + 38 * np.cos(i * 0.23 + 1.7)),
        occ=lambda i: None)
    S["z_below_floor"] = dict(   # DELIBERATELY past the detectability limit:
        # target SNR ~1.0 (differs from local background by ~1 sigma). Kept as a
        # calibration marker -- a low score HERE is expected physics, not a bug.
        frames=90, pan=(0.8, 0.0), shake=1.8, blur=6.0, noise=8.0, agc=0.10,
        interlace=True, rad=lambda t: 9.0, contrast=lambda t: 42.0,
        path=lambda i, t: (160 + 62 * np.sin(i * 0.20), 120 + 42 * np.sin(i * 0.31)))
    return S


def write_clip(name, spec, outdir, seed=1234):
    rng = np.random.RandomState(seed)
    os.makedirs(outdir, exist_ok=True)
    mp4 = os.path.join(outdir, f"{name}.mp4")
    csvp = os.path.join(outdir, f"{name}.csv")
    vw = None
    rows = []
    for i, (bgr, cx, cy) in enumerate(clip_frames(spec, rng)):
        if vw is None:
            vw = cv2.VideoWriter(mp4, cv2.VideoWriter_fourcc(*"mp4v"), 30,
                                 (bgr.shape[1], bgr.shape[0]))
        vw.write(bgr)
        rows.append((i, round(float(cx), 2), round(float(cy), 2),
                     round(float(spec["rad"](i / max(1, spec["frames"] - 1)) * 2.6), 1)))
    vw.release()
    with open(csvp, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["frame", "cx", "cy", "size"]); w.writerows(rows)
    return mp4, csvp, len(rows)


def main():
    ap = argparse.ArgumentParser(description="generate realistic tracker test footage")
    ap.add_argument("--out", default="clips", help="output directory")
    ap.add_argument("--only", help="generate a single named clip")
    ap.add_argument("--seed", type=int, default=1234)
    a = ap.parse_args()
    S = specs()
    names = [a.only] if a.only else sorted(S)
    for n in names:
        if n not in S:
            raise SystemExit(f"unknown clip {n}; have {sorted(S)}")
        mp4, csvp, k = write_clip(n, S[n], a.out, a.seed)
        print(f"{n:<22} {k:>4} frames -> {mp4}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
D435i acceptance probe -- run this on a laptop before the camera goes near the
airframe.

  Linux / macOS:                        Windows (PowerShell):
    python3 -m pip install ...             py -m pip install pyrealsense2 numpy
    python3 d435i_probe.py ...             py d435i_probe.py ...

  ON WINDOWS, `python3` DOES NOT EXIST and is not a typo you can fix by adding
  an alias. Windows ships a stub called python3.exe -- an "App Execution Alias"
  -- whose entire job is to offer to open the Microsoft Store. It reports
  "Python was not found" even when Python is installed and working, which sends
  people off reinstalling a Python they already have. Use `py`, the Python
  launcher, which is what the python.org installer puts on PATH. Turning the
  aliases off (Settings > Apps > Advanced app settings > App execution aliases)
  stops the stub intercepting.

  Use -m pip, never bare `pip`: it guarantees the packages land in the
  interpreter you are about to run rather than some other one on PATH.

    <py|python3> d435i_probe.py --selftest  # NO CAMERA NEEDED. Do this first.
    <py|python3> d435i_probe.py             # full check with the camera
    <py|python3> d435i_probe.py --range 3.0 # you tape-measured 3.0 m to the target
    <py|python3> d435i_probe.py --record 600 --record-every 6 --emitter off
                                            # walk the forest with it on a stick
    <py|python3> d435i_probe.py --replay walk.npz --range 2.5
                                            # analyse it later, numpy only

  Python 3.11 or 3.12. pyrealsense2 wheels lag new Python releases badly, so
  3.13+ tends to give "no matching distribution". On Windows prefer the
  python.org build over the Store one, which sandboxes file access.

WHY THIS EXISTS ALONGSIDE realsense-viewer, RATHER THAN INSTEAD OF IT.

Run the Viewer FIRST. It answers "is the hardware alive" and it answers it
better than any custom code could, because when it fails you know the fault is
not yours. It is the instrument check.

But the Viewer is built to make a depth camera look good, and it succeeds. It
picks a visual preset, it can apply spatial/temporal/hole-filling filters, it
runs the IR projector, and it is usually pointed at a textured indoor wall three
metres away in even light. Every one of those flatters exactly the number this
project depends on. Our voxel map's whole design rests on

    dZ = Z^2 * sigma_d / (f * B)        and       Z_max = sqrt(cell * f * B / sigma_d)

and the sim currently ASSUMES sigma_d = 0.25 px, which is a number we took from
the literature and have never measured on this unit. If the real figure is
0.5 px, the honest mapping range is 30 % shorter than every table in NOTES.md
and the far-map ladder was tuned against a fiction.

DESIGNED TO BE UNDEBUGGABLE IN THE FIELD, because that is the situation:

  * --selftest runs the ENTIRE analysis on synthetic data with a KNOWN sigma_d
    and asserts it is recovered. No camera, no laptop-specific anything. If that
    passes, the maths is not what is wrong. It reports the SDK SEPARATELY and
    diagnoses it: "installed but will not load" and "not installed for this
    interpreter" are the two common failures and they have opposite fixes.
    Exit 0 = all good, 1 = the analysis is wrong, 3 = analysis fine, SDK unusable.
  * Nothing raises. Every device call is guarded; a failure prints what failed
    and the run continues with whatever else it can measure. A probe that dies
    on the one machine you cannot debug on is worse than no probe.
  * The stream config FALLS BACK through resolutions. The USB 2 case is the
    single most likely fault, and 848x480 is exactly what a USB 2 link refuses --
    so the naive version crashes precisely when it has something to tell you.
  * It SAVES THE RAW FRAMES to an .npz next to a text log. If a number looks
    wrong, you do not need to diagnose it there; bring the file back.
  * --record / --replay: the camera does NOT have to be on a flying aircraft to
    produce any of this. Depth frames from a forest are depth frames from a
    forest whether the sensor is bolted to a quad or carried on a stick at
    walking pace. Record a walk, replay it at home. Same numbers, no crash risk.
    The format is a compressed .npz of uint16 device units plus metadata, so it
    reads back with numpy ALONE -- deliberately not librealsense's .bag, whose
    API is version-dependent and could not be tested here.

WHAT TO DO WITH THE OUTPUT: paste the log into NOTES.md. These numbers replace
assumptions that are currently load-bearing.
"""

import argparse
import datetime
import json
import os
import sys
import time
import traceback
import warnings

try:
    import numpy as np
except ImportError:
    sys.exit("missing dependency: numpy\n  pip install numpy")


# ---------------------------------------------------------------------------
# The analysis. A PURE FUNCTION of frames and geometry -- no pyrealsense2, no
# device, no globals. This is the half that can be wrong in an interesting way,
# and keeping it separable is what lets --selftest exercise the exact code that
# runs on real data rather than a re-implementation of it.
# ---------------------------------------------------------------------------
def analyse(roi_m, fx, baseline_m, cell_m=0.25, tape_m=None):
    """roi_m: (frames, h, w) float32 depth in METRES, 0 = invalid."""
    out = {}
    stack = np.asarray(roi_m, dtype=np.float64)
    if stack.ndim != 3 or stack.shape[0] < 2:
        return {"error": "need at least 2 frames of ROI data"}

    finite = stack.copy()
    finite[finite <= 0] = np.nan

    # TEMPORAL spread per pixel, then the median across the ROI.
    #
    # Temporal rather than spatial, on purpose. A real target is not a plane
    # normal to the camera, so the spatial spread of one frame measures the
    # scene's geometry as much as the sensor's noise. Spread at a FIXED pixel
    # over time is the sensor alone. Median rather than mean across the ROI
    # because a couple of pixels on a depth edge flip between foreground and
    # background and their variance is enormous and meaningless.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")           # all-NaN columns are expected
        per_px_std = np.nanstd(finite, axis=0, ddof=1)
        per_px_mean = np.nanmean(finite, axis=0)
        per_px_n = np.sum(np.isfinite(finite), axis=0)

    # A pixel needs enough samples to have a meaningful variance. Below half the
    # frames it is intermittent, and intermittent pixels are a different
    # measurement (validity) that we report separately.
    ok = (per_px_n >= max(2, stack.shape[0] // 2)) & np.isfinite(per_px_std) \
         & np.isfinite(per_px_mean) & (per_px_mean > 0.05)
    out["roi_valid_frac"] = float(ok.sum()) / float(ok.size)
    out["roi_px_total"] = int(ok.size)

    if not ok.any():
        out["error"] = "no ROI pixel returned depth in at least half the frames"
        return out

    # Fixed-pattern error against a best-fit plane, from the TEMPORAL MEAN so
    # random noise is already averaged out.
    mean_img = np.where(ok, per_px_mean, np.nan)
    pr = plane_residual(mean_img, fx, baseline_m)
    if pr:
        out["plane_rms_px"] = pr["rms_px"]
        out["plane_p95_px"] = pr["p95_px"]

    sigma_z = float(np.median(per_px_std[ok]))
    z = float(np.median(per_px_mean[ok]))
    out["sigma_z_m"] = sigma_z
    out["z_m"] = z

    if tape_m:
        out["bias_m"] = z - tape_m
        out["bias_pct"] = 100.0 * (z - tape_m) / tape_m

    # Invert dZ = Z^2 * sigma_d / (f*B). Use the TAPE distance when we have one:
    # a biased camera reports the wrong Z, and squaring that bias would fold a
    # calibration error into the noise figure.
    z_for_model = tape_m if tape_m else z
    if fx > 0 and baseline_m > 0 and z_for_model > 0 and sigma_z > 0:
        sigma_d = sigma_z * fx * baseline_m / (z_for_model ** 2)
        out["sigma_d_px"] = sigma_d
        out["z_max_raw_m"] = float(np.sqrt(cell_m * fx * baseline_m / sigma_d))
        out["z_max_derated_m"] = out["z_max_raw_m"] * 0.75
        out["z_used_for_model_m"] = z_for_model

        # WHERE THE Z^2 MODEL STOPS BEING TRUE, and it is not a footnote on a
        # 50 mm baseline. dZ = Z^2*sigma_d/(f*B) is a LINEARISATION of
        # Z = f*B/d, valid only while the disparity noise is small against the
        # disparity itself. Measured against synthetic data with a known answer
        # (see --selftest), the recovered sigma_d errs by:
        #
        #     sigma_d/d      0.04    0.09    0.19    0.38
        #     error           0 %     3 %    15 %   218 %
        #
        # At f=425 px and B=50 mm, disparity is 21 px at 1 m but only 2.7 px at
        # 8 m. So if this camera's real sigma_d is 0.5 px, the model is already
        # ~15 % optimistic at 8 m -- and every Z_max in NOTES.md is built on it.
        # Measure CLOSE, where the linearisation holds, and extrapolate; do not
        # measure far and trust the number.
        disp = fx * baseline_m / z_for_model
        out["disparity_px"] = float(disp)
        out["noise_to_disparity"] = float(sigma_d / disp) if disp > 0 else float("inf")
        if out["noise_to_disparity"] > 0.10:
            out["model_warning"] = (
                f"disparity is only {disp:.1f} px at {z_for_model:.1f} m and "
                f"sigma_d is {sigma_d:.2f} px ({100 * out['noise_to_disparity']:.0f} % of it). "
                "The Z^2 linearisation is marginal here; sigma_d is over-estimated. "
                "Re-measure at a shorter range.")
    return out



def plane_residual(roi_mean_m, fx, baseline_m):
    """FIXED-PATTERN disparity error against a best-fit plane, in pixels.

    THIS IS THE CALIBRATION MEASUREMENT, and it is a different thing from
    sigma_d. sigma_d is RANDOM noise -- it is set by the imagers, the lens and
    the ASIC's matcher, and it does not drift with age. Calibration drift shows
    up as SHAPE: a flat wall reads as bowed or tilted because the rectification
    between the two imagers has moved. Averaging over frames first removes the
    random part, so what is left is the fixed pattern.

    Done in DISPARITY space on purpose. For a planar surface seen by a pinhole
    camera, depth is NOT linear across the image but disparity IS -- so a plane
    fit to d = fx*B/Z has a residual that is directly comparable, in the same
    pixels, to the temporal sigma_d. That comparison is the whole diagnostic:

        spatial ~ temporal      noise-limited. Calibration is fine.
        spatial >> temporal     the wall is not coming out flat. Run on-chip
                                calibration; this is exactly what it fixes.

    Only meaningful pointed at a genuinely flat surface, which the caller has to
    assert -- nothing here can tell a bowed wall from a bowed calibration."""
    z = np.asarray(roi_mean_m, dtype=np.float64)
    ok = np.isfinite(z) & (z > 0.05)
    if ok.sum() < 30 or fx <= 0 or baseline_m <= 0:
        return None
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    disp = np.zeros_like(z)
    disp[ok] = fx * baseline_m / z[ok]
    A = np.stack([u[ok].ravel(), v[ok].ravel(), np.ones(int(ok.sum()))], axis=1)
    b = disp[ok].ravel()
    try:
        coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    except Exception:
        return None
    resid = b - A @ coef
    return dict(rms_px=float(np.sqrt(np.mean(resid ** 2))),
                p95_px=float(np.percentile(np.abs(resid), 95)),
                tilt_px_per_px=(float(coef[0]), float(coef[1])),
                n=int(ok.sum()))


def synth_roi(frames, h, w, z_true, fx, baseline_m, sigma_d_px, rng,
              invalid_frac=0.0, depth_scale=0.001):
    """Synthetic depth, generated the way a stereo camera actually errs: noise on
    DISPARITY, not on depth. That is the whole reason error grows as Z^2, and a
    self-test that added Gaussian noise to depth directly would confirm the
    inversion against an error model the sensor does not have."""
    d_true = fx * baseline_m / z_true
    d = d_true + rng.normal(0.0, sigma_d_px, size=(frames, h, w))
    d = np.maximum(d, 1e-6)
    z = fx * baseline_m / d
    # Quantise to the uint16 millimetre grid the device actually reports on.
    z = np.round(z / depth_scale) * depth_scale
    if invalid_frac > 0:
        z[rng.random(z.shape) < invalid_frac] = 0.0
    return z.astype(np.float32)


def selftest():
    """Runs the real analysis on synthetic data with a known answer. No camera."""
    print("=== selftest: analysis pipeline, no camera ===")
    rng = np.random.default_rng(7)
    fails = 0

    def check(cond, what, detail=""):
        nonlocal fails
        print(f"  {what:<58s} {'ok' if cond else 'FAIL'}{'  ' + detail if detail else ''}")
        if not cond:
            fails += 1

    fx, B = 425.0, 0.050          # D435i-ish at 848x480

    # 1. Recover sigma_d wherever the Z^2 linearisation is VALID -- that is,
    #    wherever the disparity noise is under about a tenth of the disparity.
    #    Checked over the whole space rather than at one point, because the
    #    probe will be used at whatever range the target happens to be.
    worst, worst_at = 0.0, ""
    for sd_true in (0.10, 0.25, 0.50, 1.00):
        for z_true in (1.0, 2.0, 4.0, 8.0):
            if sd_true / (fx * B / z_true) > 0.10:
                continue                     # outside the model, checked in 1b
            r = analyse(synth_roi(80, 48, 84, z_true, fx, B, sd_true, rng),
                        fx, B, tape_m=z_true)
            rel = abs(r["sigma_d_px"] - sd_true) / sd_true
            if rel > worst:
                worst, worst_at = rel, f"sigma_d {sd_true} at {z_true} m"
    check(worst < 0.05, "recovers sigma_d wherever the Z^2 model holds",
          f"worst {worst * 100:.1f} % ({worst_at})")

    # 1b. AND FAILS LOUDLY WHERE IT DOES NOT. This is the more valuable half.
    #     On a 50 mm baseline the disparity at 8 m is 2.7 px, so a camera with
    #     sigma_d = 1.0 px is being asked to resolve a third of its own noise --
    #     the linearisation collapses and sigma_d reads 3x high. Every Z_max in
    #     NOTES.md rests on that model, so the probe must SAY when it is outside
    #     it rather than print a confident wrong number.
    r = analyse(synth_roi(80, 48, 84, 8.0, fx, B, 1.00, rng), fx, B, tape_m=8.0)
    check("model_warning" in r, "warns when disparity noise swamps the disparity",
          f"reads {r['sigma_d_px']:.2f} px for a true 1.00 px")
    r_ok = analyse(synth_roi(80, 48, 84, 1.0, fx, B, 0.25, rng), fx, B, tape_m=1.0)
    check("model_warning" not in r_ok, "and stays quiet where the model is fine",
          f"disparity {r_ok['disparity_px']:.1f} px")

    # 2. The Z^2 law itself. If sigma_Z does not quadruple when the range
    #    doubles, either the generator or the analysis has the model wrong, and
    #    every range table in NOTES.md is built on it.
    a = analyse(synth_roi(80, 48, 84, 2.0, fx, B, 0.25, rng), fx, B, tape_m=2.0)
    b = analyse(synth_roi(80, 48, 84, 4.0, fx, B, 0.25, rng), fx, B, tape_m=4.0)
    ratio = b["sigma_z_m"] / a["sigma_z_m"]
    check(3.5 < ratio < 4.5, "sigma_Z scales as Z^2 (2 m -> 4 m gives ~4x)",
          f"measured {ratio:.2f}x")

    # 3. Z_max matches the closed form the project uses.
    r = analyse(synth_roi(80, 48, 84, 3.0, fx, B, 0.25, rng), fx, B,
                cell_m=0.25, tape_m=3.0)
    want = np.sqrt(0.25 * fx * B / 0.25)
    check(abs(r["z_max_raw_m"] - want) / want < 0.10,
          "Z_max agrees with sqrt(cell*f*B/sigma_d)",
          f"{r['z_max_raw_m']:.2f} m vs {want:.2f} m")

    # 4. Bias is reported against the tape, not swallowed.
    roi = synth_roi(60, 48, 84, 3.0, fx, B, 0.25, rng) * 1.05      # 5 % long
    r = analyse(roi, fx, B, tape_m=3.0)
    check(abs(r["bias_pct"] - 5.0) < 1.0, "reports a 5 % range bias as 5 %",
          f"{r['bias_pct']:+.1f} %")

    # 5. Holes are counted, not silently averaged away. A camera returning half
    #    a frame must not look like a camera returning all of it.
    roi = synth_roi(60, 48, 84, 3.0, fx, B, 0.25, rng, invalid_frac=0.5)
    r = analyse(roi, fx, B, tape_m=3.0)
    check(r["roi_valid_frac"] < 0.95, "50 % dropout shows up in roi_valid_frac",
          f"{r['roi_valid_frac'] * 100:.0f} % usable")

    # 6. Degenerate inputs return an error rather than raising. This is the
    #    field-robustness property: no traceback on a machine nobody can debug.
    for name, bad in [("all-zero frames", np.zeros((10, 8, 8), np.float32)),
                      ("single frame", np.ones((1, 8, 8), np.float32)),
                      ("empty", np.zeros((0, 8, 8), np.float32))]:
        try:
            r = analyse(bad, fx, B)
            ok = "error" in r
        except Exception as e:
            ok, r = False, str(e)
        check(ok, f"degenerate input ({name}) returns an error, does not raise")

    # 7. THE SDK DIAGNOSIS ITSELF, checked against synthetic failures. The
    #    classifier is what turns "it still complains" into a fix, so it does
    #    not get to be the one untested thing.
    st, _ = classify_sdk(False, None)
    check(st == "missing", "not-installed is classified as a pip/python mismatch")
    st, lines = classify_sdk(True, ImportError(
        "libusb-1.0.so.0: cannot open shared object file: No such file or directory"))
    check(st == "broken" and any("libusb-1.0-0" in l for l in lines),
          "installed-but-libusb-missing names the actual package")
    st, lines = classify_sdk(True, ImportError("libudev.so.1: cannot open shared object file"))
    check(st == "broken" and any("shared library" in l for l in lines),
          "a different missing .so is classified as a link problem")
    st, lines = classify_sdk(True, ImportError("undefined symbol: _ZN2rs7contextC1Ev"))
    check(st == "broken" and any("2.54" in l for l in lines),
          "a symbol mismatch suggests pinning a version")
    st, lines = classify_sdk(False, None, plat="darwin")
    check(any("macOS" in l for l in lines), "macOS gets told wheels are not published")
    st, lines = classify_sdk(False, None, pyver=(3, 14))
    check(any("3.14" in l for l in lines), "a too-new Python is called out")
    check(classify_sdk(True, None)[0] == "ok", "a working SDK classifies as ok")

    # 7b. FLATNESS, which is the CALIBRATION measurement -- a different thing
    #     from sigma_d. A flat wall plus noise must come out noise-limited; a
    #     bowed rectification must come out as fixed-pattern error well above
    #     the noise. If this cannot tell those apart it cannot tell a
    #     drifted camera from a noisy one, which is the whole question.
    fx2, B2 = 425.0, 0.050
    flat = synth_roi(120, 48, 84, 2.0, fx2, B2, 0.25, rng)
    rflat = analyse(flat, fx2, B2, tape_m=2.0)
    ratio_flat = rflat["plane_rms_px"] / rflat["sigma_d_px"]
    check(ratio_flat < 1.0, "a flat wall reads as noise-limited",
          f"fixed-pattern {ratio_flat:.2f}x the random sigma_d")

    #     Now bow it, the way a drifted rectification does: a quadratic error
    #     across the image in DISPARITY, a few tenths of a pixel at the edges.
    h2, w2 = 48, 84
    vv, uu = np.mgrid[0:h2, 0:w2]
    #     Note the arithmetic, because it sets what the threshold MEANS: a
    #     quadratic of amplitude A across the frame has an RMS of 0.298*A once
    #     the plane fit has absorbed its linear part. So 1.5 px of bow against
    #     0.25 px of noise is a ratio of 1.8, not 6 -- a real but mild bow that
    #     deliberately does NOT trip the warning. The 2x threshold means "fixed
    #     pattern DOMINATES noise", not "any bow at all".
    def bowed_stack(amp_px):
        bow = amp_px * (((uu - w2 / 2) / (w2 / 2)) ** 2 - 0.5)
        z_b = fx2 * B2 / (fx2 * B2 / 2.0 + bow)
        return np.broadcast_to(z_b, (120, h2, w2)) + (flat - 2.0)   # same noise

    rbow = analyse(bowed_stack(4.0), fx2, B2, tape_m=2.0)
    ratio_bow = rbow["plane_rms_px"] / rbow["sigma_d_px"]
    check(ratio_bow > 3.0, "a bowed rectification reads as fixed-pattern, not noise",
          f"4 px bow -> fixed-pattern {ratio_bow:.1f}x the random sigma_d")
    rmild = analyse(bowed_stack(1.5), fx2, B2, tape_m=2.0)
    ratio_mild = rmild["plane_rms_px"] / rmild["sigma_d_px"]
    check(1.2 < ratio_mild < 2.0, "a MILD bow is visible but below the warning line",
          f"1.5 px bow -> {ratio_mild:.1f}x (0.298*A/sigma_d, as predicted)")
    check(abs(rbow["sigma_d_px"] - rflat["sigma_d_px"]) / rflat["sigma_d_px"] < 0.25,
          "and the bow does NOT inflate sigma_d -- they are separate faults")

    # 8. The Windows frame-drop trap. Half the affected machines behave
    #    normally, so this cannot be caught by observation -- only by knowing.
    check(windows_frame_drop_warning("win32", 22631, 3296) != [],
          "warns on the AFFECTED revision 22631.3296")
    check(windows_frame_drop_warning("win32", 22631, 6199) == [],
          "stays quiet on 22631.6199 -- past the bad revision")
    check(windows_frame_drop_warning("win32", 22631, None) != [],
          "asks for winver when the revision cannot be read")
    check(windows_frame_drop_warning("win32", 19045, 4046) == [],
          "stays quiet on Windows 10")
    check(windows_frame_drop_warning("linux", 22631, 3296) == [],
          "stays quiet off Windows")

    # 8b. The symptom check, which does not care why frames went missing.
    check(fps_warning(30, 6.0, 60) != [], "flags 6 fps delivered against 30 requested")
    check(fps_warning(30, 29.4, 60) == [], "quiet at 29.4 of 30")
    check(fps_warning(30, 18.5, 60) == [], "quiet at 18.5 of 30 (62 %, within tolerance)")
    check(fps_warning(30, 5.0, 3) == [], "no verdict from 3 frames")

    # 9. THE SDK, reported SEPARATELY from the maths above.
    #
    #    Everything before this point is numpy only. If those passed, the
    #    analysis is sound whatever the SDK is doing, and conflating the two
    #    would say "SELFTEST FAILED" when the only problem is a pip that
    #    installed into a different Python. So the SDK gets its own section, its
    #    own exit code (3), and a real diagnosis rather than a shrug.
    print()
    host_report(print)
    sdk_ok = sdk_report(print)

    print(f"\n  {'ANALYSIS FAILED' if fails else 'analysis checks passed'} "
          f"({fails} failure{'' if fails == 1 else 's'})")
    if fails:
        return 1
    if not sdk_ok:
        print("  -> the maths is fine; you cannot talk to a camera until the SDK is.")
        return 3
    return 0



# ---------------------------------------------------------------------------
# A WINDOWS TRAP THAT LOOKS EXACTLY LIKE A BROKEN CAMERA.
#
# The librealsense v2.58.3 release notes carry this warning verbatim: "There is
# 50% probability of up to 80% frame drops with Windows 11 builds 22621.3296 and
# 22631.3296 (KB5035853)." Windows 10 RS5, or Windows 11 with KB5030219 (build
# 22621.2283), are unaffected.
#
# This matters here more than it looks. Eighty per cent frame drops on a probe
# whose outputs are a valid-pixel fraction and a per-pixel temporal variance
# would read as a dying camera, a bad cable, or a USB 2 link -- all of which we
# check for and none of which would be the cause. Worse, half the machines with
# the bad update are FINE, so it does not reproduce reliably enough to be
# diagnosed on the spot.
#
# We can read the build number but not the update revision (the .3296 part)
# without poking the registry, so this names the affected revisions and tells
# you to check winver rather than pretending to a precision it does not have.
# ---------------------------------------------------------------------------
AFFECTED_WIN_BUILDS = {22621, 22631}
AFFECTED_WIN_UBR = 3296


def windows_frame_drop_warning(plat, build, ubr=None):
    """Pure, so --selftest can check it. build: int major build. ubr: the update
    revision (the .3296 part), or None if it could not be read.

    KNOWING THE REVISION IS THE WHOLE POINT. Build 22631 is Windows 11 23H2 --
    an enormous population, almost all of it fine. Warning on the build alone
    fires on every 23H2 machine, and a check that cries wolf is a check that
    gets ignored on the one day it is right. Only revision .3296 is affected;
    anything later has moved past it."""
    if not str(plat).startswith("win") or build is None:
        return []
    if int(build) not in AFFECTED_WIN_BUILDS:
        return []
    if ubr is None:
        return [
            f"Windows 11 build {build}, revision unknown.",
            "  Revisions 22621.3296 and 22631.3296 (KB5035853) drop up to 80 % of",
            "  frames on about half of machines. Run `winver`: if yours ends",
            "  .3296, the frame counts below measure WINDOWS, not the camera.",
        ]
    if int(ubr) != AFFECTED_WIN_UBR:
        return []          # past it, or before it. Not this bug.
    return [
        f"THIS MACHINE IS ON THE AFFECTED REVISION {build}.{ubr} (KB5035853).",
        "  Up to 80 % of frames are dropped, on about half of machines, and it",
        "  looks identical to a failing cable or a USB 2 link. Update Windows",
        "  before trusting any number below.",
    ]


def fps_warning(requested, measured, frames):
    """Catch the SYMPTOM, whatever the cause. A version check only knows about
    the bugs we happened to read about; delivered frame rate is the thing that
    actually matters and it is free to measure while capturing anyway."""
    if requested <= 0 or frames < 5 or measured <= 0:
        return []
    ratio = measured / float(requested)
    if ratio >= 0.6:
        return []
    return [
        f"DELIVERED {measured:.1f} fps AGAINST {requested} REQUESTED "
        f"({100 * ratio:.0f} %).",
        "  Something is dropping frames. In rough order of likelihood:",
        "    - USB 2 link or a charge-only cable (see the usb descriptor above)",
        "    - a USB hub, or another device sharing the controller's bandwidth",
        "    - the Windows 11 KB5035853 frame-drop bug (revision .3296)",
        "    - a laptop power profile throttling the USB controller",
        "  The statistics below are still valid per frame, but a low frame rate",
        "  on the aircraft means a stale map, which is a safety property.",
    ]


PRESETS = ["default", "high_accuracy", "high_density", "medium_density"]


def apply_preset(rs, sensor, name, emit):
    """Set the depth visual preset on an ACTIVE sensor. Returns the name that
    actually took effect, which is not always the one asked for."""
    if name == "default":
        return "default"
    try:
        if not sensor.supports(rs.option.visual_preset):
            emit(f"  [warn] visual_preset unsupported; '{name}' ignored")
            return "unsupported"
        val = float(getattr(rs.rs400_visual_preset, name))
        sensor.set_option(rs.option.visual_preset, val)
        got = int(sensor.get_option(rs.option.visual_preset))
        if got != int(val):
            emit(f"  [warn] preset '{name}' did not take (reads {got})")
            return f"failed:{got}"
        return name
    except Exception as e:
        emit(f"  [warn] preset '{name}': {str(e)[:60]}")
        return "error"


def host_report(emit):
    """What machine is this, and is it one of the known-bad ones."""
    import platform
    emit("=== host ===")
    emit(f"  {platform.platform()}")
    emit(f"  python {platform.python_version()} ({sys.executable})")
    build = ubr = None
    if sys.platform.startswith("win"):
        try:
            build = int(sys.getwindowsversion().build)
        except Exception:
            try:
                build = int(platform.version().split(".")[2])
            except Exception:
                build = None
        # The update revision lives only in the registry -- sys.getwindowsversion
        # stops at the build. Without it we cannot tell a bad .3296 machine from
        # the millions of fine ones on the same build.
        try:
            import winreg
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                r"SOFTWARE\Microsoft\Windows NT\CurrentVersion") as k:
                ubr = int(winreg.QueryValueEx(k, "UBR")[0])
        except Exception:
            ubr = None
        emit(f"  windows build {build}" + (f".{ubr}" if ubr is not None else
                                           " (revision unreadable)"))
    for ln in windows_frame_drop_warning(sys.platform, build, ubr):
        emit("  !! " + ln if ln else "")
    emit("")


# ---------------------------------------------------------------------------
# SDK DIAGNOSIS.
#
# "I did pip install pyrealsense2 but it still complains" has exactly two
# common causes and they have OPPOSITE fixes, so the first job is to tell them
# apart:
#
#   NOT FOUND for this interpreter -- pip installed into a different Python.
#     Overwhelmingly the most common. `pip` on PATH is often not the `python3`
#     you then run, especially with a venv, pyenv, Homebrew, or the Windows
#     Store Python. Fix: install with the interpreter itself, never bare pip.
#
#   FOUND but fails to load -- the wheel is there and its shared library is
#     not. On Linux that is almost always libusb. Reinstalling pyrealsense2
#     cannot fix it and will look like it did nothing.
#
# find_spec() separates them cleanly: it locates the module without executing
# it, so a module that is found but raises on import is unambiguously a link
# problem rather than a missing package.
# ---------------------------------------------------------------------------
def classify_sdk(spec_found, exc, plat=None, pyver=None):
    """Pure classifier -- no imports, no side effects, so --selftest can check
    it against synthetic failures instead of hoping."""
    plat = plat or sys.platform
    pyver = pyver or sys.version_info[:2]
    if spec_found and exc is None:
        return "ok", []
    if not spec_found:
        lines = [
            "pyrealsense2 is NOT INSTALLED FOR THIS INTERPRETER.",
            "",
            "  This is almost always a pip/python mismatch: the `pip` on your PATH",
            "  belongs to a different Python than the `python3` you just ran.",
            "",
            "  Fix -- install with the interpreter itself, never bare pip:",
            f"      {sys.executable} -m pip install pyrealsense2 numpy",
            ("      (on Windows the launcher spelling is:  py -m pip install "
             "pyrealsense2 numpy)" if plat.startswith("win") else ""),
            "",
            f"  This interpreter: {sys.executable}",
            f"  Version:          {sys.version.split()[0]} on {plat}",
        ]
        if plat == "darwin":
            lines += [
                "",
                "  AND NOTE: you are on macOS, where Intel does not publish",
                "  pyrealsense2 wheels. `pip install pyrealsense2` cannot succeed",
                "  here in general; librealsense has to be built from source, and",
                "  even then macOS support is poor and Apple Silicon worse.",
                "  Use a Windows or Linux machine for the camera work. A VM will",
                "  not do -- USB passthrough for RealSense is unreliable.",
            ]
        if pyver >= (3, 13):
            lines += [
                "",
                f"  AND NOTE: you are on Python {pyver[0]}.{pyver[1]}. Wheels lag new",
                "  Python releases by a long way. If pip reports 'no matching",
                "  distribution', install a 3.11 or 3.12 alongside and use that.",
            ]
        return "missing", lines
    # Found, but import raised.
    msg = str(exc)
    lines = ["pyrealsense2 IS INSTALLED but will not load.",
             "", f"  {type(exc).__name__}: {msg}", ""]
    low = msg.lower()
    if "libusb" in low:
        lines += [
            "  That is the USB library, not the wheel. Reinstalling pyrealsense2",
            "  will not help and will look like it did nothing.",
            "",
            "  Debian/Ubuntu:  sudo apt-get install libusb-1.0-0",
            "  Fedora:         sudo dnf install libusb1",
            "  Arch:           sudo pacman -S libusb",
        ]
    elif ".so" in low or "shared object" in low or "dll" in low:
        lines += [
            "  A shared library the wheel depends on is missing. Install the name",
            "  in the message above from your package manager; on Linux the usual",
            "  culprits are libusb-1.0-0 and libudev.",
        ]
    elif "symbol" in low or "version" in low:
        lines += [
            "  A version mismatch between the wheel and a system library. Try a",
            "  different pyrealsense2 release:",
            "      pip install 'pyrealsense2==2.54.*'",
        ]
    else:
        lines += ["  Unrecognised load failure; the message above is the lead."]
    return "broken", lines


def sdk_report(emit):
    """Import pyrealsense2, check the API surface this script uses, and explain
    any failure properly. Returns True if the SDK is usable."""
    import importlib.util
    emit("")
    emit("=== SDK (pyrealsense2) ===")
    spec = None
    try:
        spec = importlib.util.find_spec("pyrealsense2")
    except Exception:
        spec = None
    exc = None
    rs = None
    if spec is not None:
        try:
            import pyrealsense2 as rs   # noqa: F401
        except Exception as e:
            exc = e
    status, lines = classify_sdk(spec is not None, exc)
    if status != "ok":
        for ln in lines:
            emit("  " + ln if ln else "")
        return False

    # Installed and loading. Now the names this script actually touches -- a
    # wheel built against a different librealsense would otherwise fail with an
    # AttributeError several minutes into a run, in a field.
    missing = []
    for path in ["context", "config", "pipeline",
                 "stream.depth", "stream.infrared", "stream.accel", "stream.gyro",
                 "format.z16", "option.emitter_enabled",
                 "camera_info.name", "camera_info.serial_number",
                 "camera_info.firmware_version",
                 "camera_info.recommended_firmware_version",
                 "camera_info.usb_type_descriptor"]:
        obj = rs
        for part in path.split("."):
            if not hasattr(obj, part):
                missing.append(path)
                break
            obj = getattr(obj, part)
    ver = getattr(rs, "__version__", "?")
    emit(f"  import ok, version {ver}")
    emit(f"  interpreter {sys.executable}")
    if missing:
        emit(f"  !! MISSING API NAMES: {', '.join(missing)}")
        emit("     This wheel does not match what the script expects. Try:")
        emit("         pip install -U pyrealsense2")
        return False
    try:
        rs.context(); rs.config(); rs.pipeline()
    except Exception as e:
        emit(f"  !! context/config/pipeline would not construct: {e}")
        return False
    emit("  API surface present; context/config/pipeline construct")
    return True





# ---------------------------------------------------------------------------
# Device side. Everything here is guarded: a failure reports and continues.
# ---------------------------------------------------------------------------
class Log:
    def __init__(self, path):
        self.lines = []
        self.path = path

    def __call__(self, s=""):
        print(s)
        self.lines.append(s)

    def save(self):
        try:
            with open(self.path, "w") as f:
                f.write("\n".join(self.lines) + "\n")
            print(f"\n[saved] {self.path}")
        except Exception as e:
            print(f"[warn] could not write log: {e}")


def safe(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


def report_arm(log, label, r, args):
    """Print one measurement arm. Split out so --dryrun executes the EXACT lines
    that will be read in the field, rather than a paraphrase of them. Formatting
    a float that turned out to be a missing key is a crash like any other, and
    it would happen after the camera is already packed away."""
    log(f"=== {label} ===")
    if "frames" in r:
        log(f"  frames used                 {r['frames']}")
    if "fps_measured" in r:
        log(f"  delivered frame rate       {r['fps_measured']:5.1f} fps  "
            f"(requested {r['fps_requested']})")
        for ln in fps_warning(r.get("fps_requested", 0), r["fps_measured"],
                              r.get("frames", 0)):
            log("  !! " + ln if ln else "")
    if "frame_valid_mean" in r:
        log(f"  valid pixels, whole frame   {r['frame_valid_mean'] * 100:5.1f} %  "
            f"(worst frame {r['frame_valid_min'] * 100:.1f} %)")
    log(f"  valid pixels, centre ROI    {r.get('roi_valid_frac', 0) * 100:5.1f} %")
    if "error" in r:
        log(f"  !! {r['error']}")
        log("")
        return
    log(f"  ROI depth                   {r['z_m']:.3f} m")
    log(f"  temporal sigma_Z            {r['sigma_z_m'] * 1000:.1f} mm")
    if "bias_pct" in r:
        log(f"  BIAS vs tape measure        {r['bias_m'] * 1000:+.0f} mm "
            f"({r['bias_pct']:+.1f} %)")
    if "sigma_d_px" in r:
        log(f"  disparity at that range     {r['disparity_px']:.1f} px")
        log(f"  -> sigma_d                  {r['sigma_d_px']:.3f} px   "
            f"(nav-sim assumes 0.250)")
        log(f"  -> honest Z_max at {args.cell:.2f} m cells   "
            f"{r['z_max_raw_m']:.2f} m raw, "
            f"{r['z_max_derated_m']:.2f} m with the 25 % derate")
        if "model_warning" in r:
            log(f"  !! {r['model_warning']}")
        if "plane_rms_px" in r:
            ratio = r["plane_rms_px"] / max(1e-9, r["sigma_d_px"])
            log(f"  flatness (ON A FLAT WALL ONLY)  {r['plane_rms_px']:.3f} px rms "
                f"fixed-pattern, {ratio:.1f}x the random sigma_d")
            if ratio > 2.0:
                log("  !! the wall is not coming out flat. That is CALIBRATION, not")
                log("     noise, and it is what on-chip calibration exists to fix:")
                log("     RealSense Viewer > More > On-Chip Calibration (~15 s).")
            elif ratio > 0:
                log("     (noise-limited -- nothing for on-chip calibration to fix)")
    else:
        log("  -> sigma_d not computed (needs intrinsics and a valid ROI)")
    log("")


def save_raw(path, stacks, meta, log):
    """Write the raw ROI frames and the metadata beside them.

    This is the whole contingency plan for "I cannot debug it there": if a
    number looks wrong, nothing has to be diagnosed on site, the data comes
    home. Which means this function failing silently would remove the only
    fallback -- so it reports, and load_raw() reads it back in --dryrun to prove
    the round trip actually works on this numpy."""
    try:
        payload = {k.replace(" ", "_"): v for k, v in stacks.items()}
        payload["meta_json"] = np.array(json.dumps(meta, default=str))
        np.savez_compressed(path, **payload)
        total = sum(v.size for v in stacks.values())
        log(f"[saved] raw ROI frames -> {path}  ({total} depth samples)")
        return True
    except Exception as e:
        log(f"[warn] could not save raw frames: {e}")
        return False


def load_raw(path):
    """Read back what save_raw wrote. Used by --dryrun, and by whoever analyses
    the file after it comes home."""
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    stacks = {k: z[k] for k in z.files if k != "meta_json"}
    return stacks, meta



# ---------------------------------------------------------------------------
# RECORD AND REPLAY.
#
# THE POINT: the camera does not have to be on a flying aircraft to produce
# every number this project needs. Depth frames from a forest are depth frames
# from a forest whether the sensor is bolted to a quad or carried on a stick at
# walking pace. Recording a walk and replaying it offline gets the sigma_d
# figure, the valid-return fraction on real bark, and -- once the map is wired
# to a replay source -- the false-free rate, at zero risk to a 200 EUR camera
# and zero risk of a crash.
#
# NOT librealsense's own .bag. That API is version-dependent (this machine's
# wheel rejects .bag and wants .db3, and the software_video_frame constructor
# differs too), and shipping a path I cannot execute here is exactly the mistake
# that was just called out. This format is a plain compressed .npz of uint16
# device units plus the metadata needed to interpret them, so it reads back with
# numpy ALONE -- no pyrealsense2, no librealsense, on any machine.
#
# If a Viewer-compatible bag is wanted, realsense-viewer records one itself.
# That is the supported route and it does not need this code to be right.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# .kdr -- the recording format nav-sim/voxel_live consumes.
#
# 64-byte header then raw uint16 frames, little-endian. Deliberately the dumbest
# thing that works, so a C++ reader is thirty lines and needs no zlib, no ZIP
# and no librealsense. Spec and the C++ side: nav-sim/depth_record.hpp.
#
# Intrinsics travel WITH the pixels. A depth frame without fx/ppx cannot be
# turned into a ray, so a recording without them is uninterpretable six months
# later -- and the mapper carves along rays, so getting them from the file
# rather than from an assumption is the difference between a map and a guess.
# ---------------------------------------------------------------------------
KDR_MAGIC = b"KDEPTH01"
KDR_HEADER_BYTES = 64
KDR_FLAG_EMITTER_ON = 1


def kdr_header(width, height, frames, depth_scale, fx, fy, ppx, ppy,
               baseline_m, emitter_on):
    import struct
    b = bytearray(KDR_HEADER_BYTES)
    b[0:8] = KDR_MAGIC
    struct.pack_into("<III", b, 8, int(width), int(height), int(frames))
    struct.pack_into("<ffffff", b, 20, float(depth_scale), float(fx), float(fy),
                     float(ppx), float(ppy), float(baseline_m))
    struct.pack_into("<I", b, 44, KDR_FLAG_EMITTER_ON if emitter_on else 0)
    return bytes(b)


def kdr_open_writer(path, width, height, depth_scale, fx, fy, ppx, ppy,
                    baseline_m, emitter_on):
    f = open(path, "wb")
    f.write(kdr_header(width, height, 0, depth_scale, fx, fy, ppx, ppy,
                       baseline_m, emitter_on))
    return f


def kdr_finish(f, frames):
    """Patch the frame count LAST. A capture killed by a pulled cable then has
    frames = 0, and the C++ reader falls back to the file length -- so an
    interrupted recording stays readable instead of becoming a corrupt file."""
    import struct
    f.seek(16)
    f.write(struct.pack("<I", int(frames)))
    f.close()


def kdr_read(path):
    """Returns (frames_uint16, meta). numpy only."""
    import struct
    with open(path, "rb") as f:
        hdr = f.read(KDR_HEADER_BYTES)
        if len(hdr) < KDR_HEADER_BYTES or hdr[0:8] != KDR_MAGIC:
            raise ValueError("not a .kdr recording")
        w, h, n = struct.unpack_from("<III", hdr, 8)
        scale, fx, fy, ppx, ppy, base = struct.unpack_from("<ffffff", hdr, 20)
        flags, = struct.unpack_from("<I", hdr, 44)
        raw = np.frombuffer(f.read(), dtype="<u2")
    per = w * h
    actual = len(raw) // per                       # trust the bytes, not the header
    n = min(n, actual) if n else actual
    frames = raw[:n * per].reshape(n, h, w)
    return frames, dict(width=w, height=h, frames=int(n), depth_scale=scale,
                        fx=fx, fy=fy, ppx=ppx, ppy=ppy, baseline_m=base,
                        emitter_on=bool(flags & KDR_FLAG_EMITTER_ON))


def save_record(path, frames_u16, meta, log):
    """frames_u16: (n, h, w) uint16 in DEVICE units. Stored raw, not converted:
    the depth scale is metadata, and keeping the integers means the file is half
    the size and bit-exact with what the sensor reported."""
    try:
        arr = np.asarray(frames_u16, dtype=np.uint16)
        np.savez_compressed(path, depth_u16=arr,
                            meta_json=np.array(json.dumps(meta, default=str)))
        mb = os.path.getsize(path) / 1e6
        log(f"[saved] {arr.shape[0]} full frames {arr.shape[2]}x{arr.shape[1]} "
            f"-> {path}  ({mb:.1f} MB)")
        return True
    except Exception as e:
        log(f"[warn] could not save recording: {e}")
        return False


def load_record(path):
    z = np.load(path, allow_pickle=False)
    return z["depth_u16"], json.loads(str(z["meta_json"]))


def centre_roi(frames, roi_frac):
    n, h, w = frames.shape
    rh = max(1, int(h * roi_frac / 2))
    rw = max(1, int(w * roi_frac / 2))
    return frames[:, h // 2 - rh:h // 2 + rh, w // 2 - rw:w // 2 + rw]


def replay(args, log):
    """Run the same analysis on a recording. numpy only -- no camera, no SDK."""
    try:
        frames, meta = load_record(args.replay)
    except Exception as e:
        log(f"cannot read {args.replay}: {e}")
        return 2
    scale = float(meta.get("depth_scale", 0.001))
    fx = float(meta.get("fx", 0.0))
    baseline = float(meta.get("baseline_m", 0.0))
    log(f"=== replay: {args.replay} ===")
    log(f"  {frames.shape[0]} frames  {frames.shape[2]}x{frames.shape[1]}   "
        f"fx {fx:.1f} px   baseline {baseline * 1000:.1f} mm   "
        f"scale {scale * 1000:.3f} mm")
    for k in ("recorded", "emitter", "usb_descriptor", "firmware", "serial"):
        if k in meta:
            log(f"  {k:18s} {meta[k]}")
    log("")
    d = frames.astype(np.float32) * scale
    roi = centre_roi(d, args.roi)
    r = analyse(roi, fx, baseline, cell_m=args.cell, tape_m=args.range)
    r["frames"] = int(d.shape[0])
    r["frame_valid_mean"] = float(np.count_nonzero(d)) / d.size
    r["frame_valid_min"] = float(min(np.count_nonzero(f) / f.size for f in d))
    report_arm(log, f"replay, centre {args.roi * 100:.0f} % ROI", r, args)
    if not args.range:
        log("  (no --range given: sigma_d is computed against the camera's own")
        log("   reported depth, so a calibration bias folds into it. Pass the")
        log("   tape-measured distance when the target was a fixed object.)")
    return 0


def dryrun(args, log):
    """Run the whole REPORTING path on synthetic frames. No camera, no
    pyrealsense2. This exists because --selftest checks the maths and the
    no-device path checks the front door, but neither ever executes the lines
    the user actually reads."""
    rng = np.random.default_rng(3)
    fx, B = 425.0, 0.050
    z = args.range or 3.0
    log("=== DRY RUN: synthetic frames, no camera ===")
    log(f"  pretending fx {fx:.0f} px, baseline {B * 1000:.0f} mm, target at {z:.1f} m")
    log("")
    arms = {
        "emitter ON": (0.25, 0.02),      # (true sigma_d px, dropout fraction)
        "emitter OFF": (0.45, 0.35),     # outdoors: noisier and holier
    }
    results = {}
    for label, (sd, drop) in arms.items():
        stack = synth_roi(args.frames, 48, 84, z, fx, B, sd, rng, invalid_frac=drop)
        r = analyse(stack, fx, B, cell_m=args.cell, tape_m=args.range)
        r["frames"] = args.frames
        r["frame_valid_mean"] = float(np.count_nonzero(stack)) / stack.size
        r["frame_valid_min"] = r["frame_valid_mean"] * 0.9
        results[label] = r
        report_arm(log, label, r, args)
    emitter_verdict(log, results.get("emitter ON"), results.get("emitter OFF"))
    # And the degenerate arm, so its formatting is exercised too.
    bad = analyse(np.zeros((args.frames, 12, 12), np.float32), fx, B)
    bad["frames"] = args.frames
    report_arm(log, "degenerate arm (all holes)", bad, args)
    # Exercise the save/load round trip, because it is the contingency plan.
    if not args.no_save:
        stacks = {k: synth_roi(4, 8, 8, z, fx, B, 0.25, rng) for k in arms}
        if save_raw(args.npz, stacks, {"dryrun": True, "z": z}, log):
            try:
                back, meta = load_raw(args.npz)
                ok = (set(back) == {k.replace(" ", "_") for k in arms}
                      and meta.get("dryrun") is True)
                log(f"  round trip re-read: {'ok' if ok else 'MISMATCH'} "
                    f"({', '.join(sorted(back))})")
            except Exception as e:
                log(f"  round trip re-read FAILED: {e}")
    log("  (dry run only -- these numbers are synthetic and mean nothing)")
    return 0


def emitter_verdict(log, a, b):
    if not a or not b or "frame_valid_mean" not in a or "frame_valid_mean" not in b:
        return
    log("=== emitter verdict ===")
    log(f"  valid fraction {a['frame_valid_mean'] * 100:.1f} % on -> "
        f"{b['frame_valid_mean'] * 100:.1f} % off "
        f"({(b['frame_valid_mean'] - a['frame_valid_mean']) * 100:+.1f} pts)")
    log("  Indoors the projector does most of the work. OUTDOORS IT DOES ALMOST")
    log("  NONE: a ~1 W pattern against sunlight, useful to a couple of metres.")
    log("  nav-sim's forest numbers assume PASSIVE stereo, so the emitter-OFF")
    log("  column is the one that predicts flight performance. The real")
    log("  characterisation is outdoors, on bark, at 2/4/6/8 m.")
    log("")


# Ordered widest-first. A USB 2 link will refuse the top entries; falling back
# rather than raising is what turns "the script crashed" into "your cable is
# USB 2", which is the single most likely fault and the one this exists to find.
FALLBACK_MODES = [(848, 480, 30), (640, 480, 30), (640, 360, 30),
                  (480, 270, 30), (424, 240, 30), (424, 240, 15)]


def run_device(args, log):
    import pyrealsense2 as rs

    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        log("no RealSense device found.")
        log("  - is the cable a USB 3 DATA cable? many USB-C cables are charge-only")
        log("  - on Linux, do you have the udev rules? (99-realsense-libusb.rules)")
        log("  - try realsense-viewer first: if it cannot see it either, it is not this script")
        return {}
    dev = devs[0]

    result = {}
    log("=== device ===")
    usb = None
    for label, key in [("name", rs.camera_info.name),
                       ("serial", rs.camera_info.serial_number),
                       ("firmware", rs.camera_info.firmware_version),
                       ("recommended fw", rs.camera_info.recommended_firmware_version),
                       ("usb descriptor", rs.camera_info.usb_type_descriptor)]:
        val = safe(lambda k=key: dev.get_info(k), "?")
        if key == rs.camera_info.usb_type_descriptor:
            usb = val
        log(f"  {label:18s} {val}")
        result[label.replace(" ", "_")] = val

    # THE CABLE TEST. A D435i on a USB 2 link still enumerates and still streams
    # -- it just refuses the higher modes and drops frames under load. That is a
    # worse failure than not working, because it looks like a bad camera.
    if usb and str(usb).startswith("2"):
        log("")
        log("  !! NEGOTIATED USB 2 -- this is a cable or port problem, not the camera.")
        log("     The D435i needs USB 3.1 Gen 1 (5 Gbps). Many USB-C cables are")
        log("     USB 2.0 data only, including most sold with phones.")
        log("     Everything below is measured on a degraded link. Fix this first.")
    elif usb:
        log(f"  -> USB {usb}: full bandwidth available")
    log("")

    # IMU: the 'i' in D435i, and the reason to buy this over a D435.
    log("=== imu ===")
    kinds = set()
    for s in safe(lambda: list(dev.sensors), []) or []:
        for p in safe(lambda s=s: s.get_stream_profiles(), []) or []:
            st = safe(lambda p=p: p.stream_type())
            if st in (rs.stream.accel, rs.stream.gyro):
                kinds.add(str(st).split(".")[-1])
    if kinds:
        log(f"  present: {', '.join(sorted(kinds))}")
    else:
        log("  !! NO IMU STREAMS -- this is a D435, not a D435i, or the IMU is dead.")
    result["imu"] = sorted(kinds)
    log("")

    depth_sensor = safe(lambda: dev.first_depth_sensor())
    depth_scale = safe(lambda: depth_sensor.get_depth_scale(), 0.001) or 0.001

    # --- pick a mode that actually starts -----------------------------------
    modes = [(args.width, args.height, args.fps)] + \
            [m for m in FALLBACK_MODES if m != (args.width, args.height, args.fps)]
    started = None
    for (w, h, fps) in modes:
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        pipe = rs.pipeline()
        try:
            prof = pipe.start(cfg)
            started = (w, h, fps, pipe, prof)
            if (w, h, fps) != (args.width, args.height, args.fps):
                log(f"  [fallback] {args.width}x{args.height}@{args.fps} refused; "
                    f"using {w}x{h}@{fps}")
            break
        except Exception as e:
            safe(lambda: pipe.stop())
            log(f"  [mode] {w}x{h}@{fps} unavailable ({str(e)[:60]})")
    if not started:
        log("  !! no depth mode would start at all. Camera or driver problem.")
        return result
    w, h, fps, pipe, prof = started

    fx = fy = ppx = ppy = 0.0
    baseline = 0.050
    try:
        dprof = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        intr = dprof.get_intrinsics()
        fx, fy = intr.fx, intr.fy
        ppx, ppy = intr.ppx, intr.ppy
        hfov = 2 * np.degrees(np.arctan(intr.width / (2 * intr.fx)))
        try:
            ir2 = prof.get_stream(rs.stream.infrared, 2)
            baseline = abs(dprof.get_extrinsics_to(ir2).translation[0])
            src = "from device"
        except Exception:
            src = "NOMINAL, device did not report it"
        log("=== geometry ===")
        log(f"  {w}x{h}@{fps}   fx {fx:.1f} px   fy {fy:.1f} px   hfov {hfov:.1f} deg")
        log(f"  baseline {baseline * 1000:.1f} mm ({src})   "
            f"depth scale {depth_scale * 1000:.3f} mm/unit")
        log("")
        result.update(dict(width=w, height=h, fps=fps, fx=fx, fy=fy,
                           ppx=ppx, ppy=ppy,
                           baseline_m=baseline, depth_scale=depth_scale))
    except Exception as e:
        log(f"  [warn] could not read intrinsics: {e}")
    safe(lambda: pipe.stop())

    # --- record mode: capture full frames and stop ---------------------------
    if args.record > 0:
        emit = 0 if args.emitter == "off" else 1
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        pipe = rs.pipeline()
        frames = []
        try:
            prof2 = pipe.start(cfg)
            ds = safe(lambda: prof2.get_device().first_depth_sensor())
            if ds is not None and safe(lambda: ds.supports(rs.option.emitter_enabled), False):
                safe(lambda: ds.set_option(rs.option.emitter_enabled, float(emit)))
            if ds is not None:
                pname = "default" if args.preset == "sweep" else args.preset
                log(f"  preset: {apply_preset(rs, ds, pname, log)}")
            for _ in range(fps):
                safe(lambda: pipe.wait_for_frames(5000))     # let AE settle
            log(f"=== recording {args.record} frames "
                f"({'emitter ON' if emit else 'emitter OFF'}) ===")
            t_start = time.monotonic()
            for i in range(args.record):
                fs = safe(lambda: pipe.wait_for_frames(5000))
                if fs is None:
                    log(f"  [warn] timeout after {len(frames)} frames; stopping")
                    break
                f = fs.get_depth_frame()
                if f and (i % max(1, args.record_every) == 0):
                    frames.append(np.asanyarray(f.get_data()).copy())
                if args.record >= 20 and (i + 1) % max(1, args.record // 10) == 0:
                    log(f"  {i + 1}/{args.record}")
        except Exception as e:
            log(f"  [warn] recording stopped: {str(e)[:80]}")
        finally:
            safe(lambda: pipe.stop())
        elapsed = max(1e-6, time.monotonic() - t_start)
        got = len(frames) * max(1, args.record_every)
        for ln in fps_warning(fps, got / elapsed, got):
            log("  !! " + ln if ln else "")
        if len(frames) >= 2:
            meta = dict(result)
            meta.update(dict(recorded=datetime.datetime.now().isoformat(timespec="seconds"),
                             emitter="on" if emit else "off",
                             fx=fx, fy=fy, ppx=ppx, ppy=ppy,
                             baseline_m=baseline, depth_scale=depth_scale,
                             width=w, height=h, fps=fps))
            stack = np.stack(frames)
            save_record(args.npz, stack, meta, log)
            # And the .kdr, which is what nav-sim/voxel_live replays through the
            # REAL map and planner. Same pixels, a format C++ can read without
            # a ZIP library.
            kdr = args.npz.replace(".npz", "") + ".kdr"
            try:
                kf = kdr_open_writer(kdr, w, h, depth_scale, fx, fy, ppx, ppy,
                                     baseline, emit)
                for fr in stack:
                    kf.write(np.ascontiguousarray(fr, dtype="<u2").tobytes())
                kdr_finish(kf, len(stack))
                log(f"[saved] {len(stack)} frames -> {kdr}  "
                    f"({os.path.getsize(kdr) / 1e6:.1f} MB)")
                log(f"  replay through the map and planner:  voxel_live --replay {kdr}")
            except Exception as e:
                log(f"[warn] could not write .kdr: {e}")
            log(f"  analyse the noise with:  <py|python3> d435i_probe.py "
                f"--replay {args.npz} --range <tape distance>")
        else:
            log(f"  !! only {len(frames)} frames captured, nothing saved")
        return result

    # --- emitter x preset arms ----------------------------------------------
    wanted = {"on": [1], "off": [0], "both": [1, 0]}[args.emitter]
    presets = ["default", "high_accuracy", "high_density"] \
        if args.preset == "sweep" else [args.preset]
    saved = {}
    for args._preset_now, emit in [(p_, e_) for p_ in presets for e_ in wanted]:
        label = "emitter ON" if emit else "emitter OFF"
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        pipe = rs.pipeline()
        try:
            prof = pipe.start(cfg)
        except Exception as e:
            log(f"  [warn] {label}: could not start ({str(e)[:60]})")
            continue
        try:
            # Set the emitter on the ACTIVE sensor, after start. Setting it on a
            # handle obtained before streaming is accepted by some firmware and
            # silently ignored by others, which would make both arms identical
            # and the comparison meaningless.
            ds = prof.get_device().first_depth_sensor()
            if safe(lambda: ds.supports(rs.option.emitter_enabled), False):
                safe(lambda: ds.set_option(rs.option.emitter_enabled, float(emit)))
                got = safe(lambda: ds.get_option(rs.option.emitter_enabled))
                if got is not None and int(got) != emit:
                    log(f"  [warn] {label}: emitter would not change (reads {got})")
            else:
                log(f"  [warn] {label}: emitter_enabled not supported; arms are identical")
            if ds is not None:
                got_preset = apply_preset(rs, ds, args._preset_now, log)
                label = f"{label}, preset {got_preset}"

            valid, roi = [], []
            t_start = time.monotonic()
            # Discard the first second: measuring through auto-exposure
            # convergence measures the AE loop, not the camera.
            for _ in range(fps):
                safe(lambda: pipe.wait_for_frames(5000))
            for _ in range(args.frames):
                fs = safe(lambda: pipe.wait_for_frames(5000))
                if fs is None:
                    log(f"  [warn] {label}: frame timeout; stopping this arm early")
                    break
                f = fs.get_depth_frame()
                if not f:
                    continue
                d = np.asanyarray(f.get_data()).astype(np.float32) * depth_scale
                valid.append(float(np.count_nonzero(d)) / d.size)
                fh, fw = d.shape
                rh = max(1, int(fh * args.roi / 2))
                rw = max(1, int(fw * args.roi / 2))
                roi.append(d[fh // 2 - rh:fh // 2 + rh, fw // 2 - rw:fw // 2 + rw])
        finally:
            safe(lambda: pipe.stop())

        elapsed = max(1e-6, time.monotonic() - t_start)
        if len(roi) < 2:
            log(f"  [warn] {label}: only {len(roi)} usable frames, skipping")
            continue

        stack = np.stack(roi)
        r = analyse(stack, fx, baseline, cell_m=args.cell, tape_m=args.range)
        r["frame_valid_mean"] = float(np.mean(valid))
        r["frame_valid_min"] = float(np.min(valid))
        r["frames"] = len(roi)
        r["fps_measured"] = len(roi) / elapsed
        r["fps_requested"] = fps
        r["_base"] = "emitter ON" if emit else "emitter OFF"
        r["_preset"] = args._preset_now
        saved[label] = (stack, r)
        result[label.replace(" ", "_")] = r

        report_arm(log, label, r, args)

    # Pair the emitter arms WITHIN each preset. The labels now carry the preset,
    # so matching on the exact string would silently never fire -- the sort of
    # no-op that looks like "the comparison just did not apply".
    for p_ in {r["_preset"] for _, r in saved.values() if "_preset" in r}:
        on = next((r for _, r in saved.values()
                   if r.get("_preset") == p_ and r.get("_base") == "emitter ON"), None)
        off = next((r for _, r in saved.values()
                    if r.get("_preset") == p_ and r.get("_base") == "emitter OFF"), None)
        if on and off:
            log(f"--- preset {p_} ---")
            emitter_verdict(log, on, off)

    # And the preset comparison, which is the trade that decides our config.
    presets_seen = [(r["_preset"], r) for _, r in saved.values()
                    if r.get("_base") == "emitter OFF" and "sigma_d_px" in r]
    if len(presets_seen) > 1:
        log("=== preset verdict (emitter OFF -- the outdoor case) ===")
        log(f"  {'preset':16s} {'valid':>7s} {'sigma_d':>9s} {'Z_max derated':>15s}")
        for name, r in sorted(presets_seen):
            log(f"  {name:16s} {r['frame_valid_mean'] * 100:6.1f} % "
                f"{r['sigma_d_px']:8.3f} px {r['z_max_derated_m']:12.2f} m")
        log("  high_accuracy returns FEWER pixels and cleaner ones; high_density the")
        log("  reverse. Our map is three-state -- a missing return costs nothing but")
        log("  speed, while a WRONG one carves free space through an obstacle. So the")
        log("  arm to prefer is the one with the lowest sigma_d, not the fullest image.")
        log("")

    # RAW FRAMES TO DISK. The point of this file is that nothing has to be
    # diagnosed on site: if a number looks wrong, the data comes home.
    if saved and not args.no_save:
        save_raw(args.npz, {k: v[0] for k, v in saved.items()}, result, log)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true",
                    help="run the analysis on synthetic data; no camera needed")
    ap.add_argument("--dryrun", action="store_true",
                    help="run the whole REPORT on synthetic data; no camera needed")
    ap.add_argument("--record", type=int, default=0, metavar="N",
                    help="capture N full depth frames to an .npz and stop. "
                         "This is the walk-in-the-forest mode: no aircraft needed.")
    ap.add_argument("--record-every", type=int, default=1, metavar="K",
                    help="keep every Kth frame while recording. A walk at 30 fps "
                         "is ~0.5 MB/frame compressed; K=6 gives 5 Hz, which is "
                         "plenty for range statistics and a tenth of the disk.")
    ap.add_argument("--replay", default=None, metavar="FILE",
                    help="analyse a recording made with --record. numpy only, "
                         "no camera and no SDK needed.")
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--roi", type=float, default=0.10,
                    help="centre ROI as a fraction of the frame")
    ap.add_argument("--range", type=float, default=None,
                    help="TAPE-MEASURED distance to the ROI target, metres")
    ap.add_argument("--cell", type=float, default=0.25,
                    help="voxel size for the Z_max figure")
    ap.add_argument("--emitter", choices=["on", "off", "both"], default="both")
    ap.add_argument("--preset", default="default",
                    choices=["default", "high_accuracy", "high_density",
                             "medium_density", "sweep"],
                    help="depth visual preset. 'sweep' measures all three, which "
                         "is the trade that actually matters: high_accuracy "
                         "returns FEWER but cleaner pixels, high_density the "
                         "reverse. A three-state map wants the former.")
    ap.add_argument("--out", default=None, help="text log path")
    ap.add_argument("--npz", default=None, help="raw frame archive path")
    ap.add_argument("--no-save", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    args.out = args.out or f"d435i_probe_{stamp}.txt"
    args.npz = args.npz or f"d435i_probe_{stamp}.npz"
    log = Log(args.out)

    log(f"d435i_probe  {datetime.datetime.now().isoformat(timespec='seconds')}")
    log(f"argv: {' '.join(sys.argv[1:]) or '(defaults)'}")
    log("")
    host_report(log)

    if args.dryrun:
        rc = dryrun(args, log)
        log.save()
        return rc

    if args.replay:
        rc = replay(args, log)
        log.save()
        return rc

    try:
        import pyrealsense2  # noqa: F401
    except ImportError as e:
        log(f"missing dependency: {e}")
        log("  pip install pyrealsense2 numpy")
        log("  (--selftest still works without it? no -- numpy only. Try that first.)")
        log.save()
        return 2

    # TOP-LEVEL CATCH. On the machine that cannot be debugged, a traceback in the
    # log file that comes home with you beats a traceback on a screen you have
    # already walked away from.
    try:
        run_device(args, log)
    except Exception:
        log("")
        log("!! unhandled error -- everything above still stands:")
        for line in traceback.format_exc().splitlines():
            log("   " + line)
    log.save()
    return 0


if __name__ == "__main__":
    sys.exit(main())

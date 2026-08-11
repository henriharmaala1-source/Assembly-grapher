#!/usr/bin/env python3
"""
D435i acceptance probe -- run this on a laptop before the camera goes near the
airframe.

    python3 -m pip install pyrealsense2 numpy      # -m python3, NOT bare pip
    python3 d435i_probe.py --selftest       # NO CAMERA NEEDED. Do this first.
    python3 d435i_probe.py                  # full check with the camera
    python3 d435i_probe.py --range 3.0      # you tape-measured 3.0 m to the target
    python3 d435i_probe.py --record 600 --record-every 6 --emitter off
                                            # walk the forest with it on a stick
    python3 d435i_probe.py --replay walk.npz --range 2.5
                                            # analyse it later, numpy only

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

    # 8. THE SDK, reported SEPARATELY from the maths above.
    #
    #    Everything before this point is numpy only. If those passed, the
    #    analysis is sound whatever the SDK is doing, and conflating the two
    #    would say "SELFTEST FAILED" when the only problem is a pip that
    #    installed into a different Python. So the SDK gets its own section, its
    #    own exit code (3), and a real diagnosis rather than a shrug.
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

    fx = fy = 0.0
    baseline = 0.050
    try:
        dprof = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        intr = dprof.get_intrinsics()
        fx, fy = intr.fx, intr.fy
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
            for _ in range(fps):
                safe(lambda: pipe.wait_for_frames(5000))     # let AE settle
            log(f"=== recording {args.record} frames "
                f"({'emitter ON' if emit else 'emitter OFF'}) ===")
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
        if len(frames) >= 2:
            meta = dict(result)
            meta.update(dict(recorded=datetime.datetime.now().isoformat(timespec="seconds"),
                             emitter="on" if emit else "off",
                             fx=fx, baseline_m=baseline, depth_scale=depth_scale,
                             width=w, height=h, fps=fps))
            save_record(args.npz, np.stack(frames), meta, log)
            log(f"  analyse it with:  python3 d435i_probe.py --replay {args.npz} "
                f"--range <tape distance>")
        else:
            log(f"  !! only {len(frames)} frames captured, nothing saved")
        return result

    # --- emitter arms --------------------------------------------------------
    wanted = {"on": [1], "off": [0], "both": [1, 0]}[args.emitter]
    saved = {}
    for emit in wanted:
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

            valid, roi = [], []
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

        if len(roi) < 2:
            log(f"  [warn] {label}: only {len(roi)} usable frames, skipping")
            continue

        stack = np.stack(roi)
        r = analyse(stack, fx, baseline, cell_m=args.cell, tape_m=args.range)
        r["frame_valid_mean"] = float(np.mean(valid))
        r["frame_valid_min"] = float(np.min(valid))
        r["frames"] = len(roi)
        saved[label] = (stack, r)
        result[label.replace(" ", "_")] = r

        report_arm(log, label, r, args)

    if "emitter ON" in saved and "emitter OFF" in saved:
        emitter_verdict(log, saved["emitter ON"][1], saved["emitter OFF"][1])

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

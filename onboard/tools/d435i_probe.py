#!/usr/bin/env python3
"""
D435i acceptance probe -- run this on a laptop before the camera goes near the
airframe.

    pip install pyrealsense2 numpy
    python3 d435i_probe.py                 # full check
    python3 d435i_probe.py --range 3.0     # you measured 3.0 m to the target
    python3 d435i_probe.py --emitter off   # the outdoor case

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

So this measures sigma_d directly, with post-processing off, and prints what it
implies. It also checks the two things a used camera can quietly be wrong about:
the USB link it actually negotiated, and whether the IMU is alive.

WHAT TO DO WITH THE OUTPUT: paste it into NOTES.md. The numbers here replace
assumptions that are currently load-bearing.
"""

import argparse
import sys
import time

try:
    import numpy as np
    import pyrealsense2 as rs
except ImportError as e:
    sys.exit(f"missing dependency: {e}\n  pip install pyrealsense2 numpy")


def find_device():
    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        sys.exit(
            "no RealSense device found.\n"
            "  - is the cable a USB 3 DATA cable? many USB-C cables are charge-only\n"
            "  - on Linux, do you have the udev rules? (99-realsense-libusb.rules)\n"
            "  - try realsense-viewer first: if it cannot see it either, it is not this script"
        )
    return devs[0]


def report_device(dev):
    print("=== device ===")
    fields = [
        ("name", rs.camera_info.name),
        ("serial", rs.camera_info.serial_number),
        ("firmware", rs.camera_info.firmware_version),
        ("recommended fw", rs.camera_info.recommended_firmware_version),
        ("usb descriptor", rs.camera_info.usb_type_descriptor),
    ]
    usb = None
    for label, key in fields:
        try:
            val = dev.get_info(key)
        except Exception:
            val = "?"
        if key == rs.camera_info.usb_type_descriptor:
            usb = val
        print(f"  {label:18s} {val}")

    # THE CABLE TEST, and it is the whole reason this runs before anything else.
    # A D435i on a USB 2 link still enumerates and still streams -- it just
    # quietly refuses the higher resolutions and drops frames under load. That
    # is a much worse failure than not working, because it looks like the camera
    # is bad when the cable is.
    if usb and usb.startswith("2"):
        print("\n  !! NEGOTIATED USB 2 -- this is a cable or port problem, not the camera.")
        print("     The D435i needs USB 3.1 Gen 1 (5 Gbps). Many USB-C cables are")
        print("     USB 2.0 only, including most of the ones sold with phones.")
        print("     Every number below is measured on a degraded link. Fix this first.\n")
    elif usb:
        print(f"  -> USB {usb}: full bandwidth available\n")
    return usb


def report_imu(dev):
    # The 'i' in D435i, and the reason to buy this over a D435. VIO is dead
    # without it, and a used unit is exactly where you would find out too late.
    kinds = set()
    for s in dev.sensors:
        for p in s.get_stream_profiles():
            if p.stream_type() in (rs.stream.accel, rs.stream.gyro):
                kinds.add(str(p.stream_type()).split(".")[-1])
    print("=== imu ===")
    if kinds:
        print(f"  present: {', '.join(sorted(kinds))}")
    else:
        print("  !! NO IMU STREAMS -- this is a D435, not a D435i, or the IMU is dead.")
    print()


def stream_stats(pipe, cfg, frames, roi_frac, depth_scale):
    """Collect per-frame valid fraction and a per-pixel time series over a
    centre ROI. The ROI is where the target is; the valid fraction is over the
    whole frame, because a collapsing valid fraction is a health signal in its
    own right and averaging it into the ROI would hide it."""
    prof = pipe.start(cfg)
    try:
        # Let auto-exposure settle. Measuring through the first second of AE
        # convergence is measuring the AE loop, not the camera.
        for _ in range(30):
            pipe.wait_for_frames()

        valid_fracs, roi_stack = [], []
        for _ in range(frames):
            f = pipe.wait_for_frames().get_depth_frame()
            if not f:
                continue
            d = np.asanyarray(f.get_data()).astype(np.float32) * depth_scale
            valid_fracs.append(float(np.count_nonzero(d) / d.size))
            h, w = d.shape
            rh, rw = int(h * roi_frac / 2), int(w * roi_frac / 2)
            roi_stack.append(d[h // 2 - rh:h // 2 + rh, w // 2 - rw:w // 2 + rw])
        return np.array(valid_fracs), np.stack(roi_stack), prof
    finally:
        pipe.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--roi", type=float, default=0.10, help="centre ROI as a fraction of frame")
    ap.add_argument("--range", type=float, default=None,
                    help="TAPE-MEASURED distance to the ROI target, metres")
    ap.add_argument("--cell", type=float, default=0.25, help="voxel size for the Z_max figure")
    ap.add_argument("--emitter", choices=["on", "off", "both"], default="both")
    args = ap.parse_args()

    dev = find_device()
    report_device(dev)
    report_imu(dev)

    depth_sensor = dev.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Baseline and focal length come from the DEVICE, not from a datasheet. A
    # used unit may have been recalibrated, and the whole point of the sigma_d
    # fit below is that it is grounded in this unit's own geometry.
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    pipe = rs.pipeline()
    tmp = pipe.start(cfg)
    dprof = tmp.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = dprof.get_intrinsics()
    try:
        # Baseline as the device reports it: the translation between the depth
        # frame and the right IR imager.
        baseline = abs(dprof.get_extrinsics_to(
            tmp.get_stream(rs.stream.infrared, 2)).translation[0])
    except Exception:
        baseline = 0.050          # D435i nominal, 50 mm
    pipe.stop()

    print("=== geometry (from the device) ===")
    print(f"  {args.width}x{args.height}   fx {intr.fx:.1f} px   fy {intr.fy:.1f} px")
    print(f"  hfov {2 * np.degrees(np.arctan(intr.width / (2 * intr.fx))):.1f} deg")
    print(f"  baseline {baseline * 1000:.1f} mm   depth scale {depth_scale * 1000:.3f} mm/unit")
    print()

    modes = {"on": [True], "off": [False], "both": [True, False]}[args.emitter]
    results = {}
    for emit in modes:
        label = "emitter ON " if emit else "emitter OFF"
        try:
            depth_sensor.set_option(rs.option.emitter_enabled, 1 if emit else 0)
        except Exception:
            print(f"  ({label}: cannot set emitter on this unit, skipping)")
            continue

        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
        valid, roi, _ = stream_stats(rs.pipeline(), cfg, args.frames, args.roi, depth_scale)

        # TEMPORAL std per pixel, then the median across the ROI. Not the spatial
        # std: a real target is not a plane normal to the camera, so spatial
        # spread measures the scene's geometry as much as the sensor's noise.
        # Temporal spread at a fixed pixel is the sensor alone.
        finite = roi.copy()
        finite[finite == 0] = np.nan
        with np.errstate(invalid="ignore"):
            per_px_std = np.nanstd(finite, axis=0)
            per_px_mean = np.nanmean(finite, axis=0)
        ok = np.isfinite(per_px_std) & np.isfinite(per_px_mean) & (per_px_mean > 0.1)
        sigma_z = float(np.median(per_px_std[ok])) if ok.any() else float("nan")
        z_meas = float(np.median(per_px_mean[ok])) if ok.any() else float("nan")
        roi_valid = float(ok.sum() / ok.size)

        results[label] = dict(valid=float(valid.mean()), valid_min=float(valid.min()),
                              sigma_z=sigma_z, z=z_meas, roi_valid=roi_valid)

        print(f"=== {label} ===")
        print(f"  valid pixels, whole frame   {valid.mean() * 100:5.1f} %  "
              f"(worst frame {valid.min() * 100:.1f} %)")
        print(f"  valid pixels, centre ROI    {roi_valid * 100:5.1f} %")
        print(f"  ROI depth                   {z_meas:.3f} m")
        print(f"  temporal sigma_Z at that range  {sigma_z * 1000:.1f} mm")

        if args.range:
            err = z_meas - args.range
            print(f"  BIAS vs tape measure        {err * 1000:+.0f} mm "
                  f"({100 * err / args.range:+.1f} %)")

        # Invert dZ = Z^2 sigma_d / (f B) for the disparity noise. This is the
        # number the sim assumes and has never checked.
        if np.isfinite(sigma_z) and z_meas > 0:
            sigma_d = sigma_z * intr.fx * baseline / (z_meas ** 2)
            z_max = np.sqrt(args.cell * intr.fx * baseline / sigma_d)
            print(f"  -> sigma_d                  {sigma_d:.3f} px   "
                  f"(nav-sim assumes 0.250)")
            print(f"  -> honest Z_max at {args.cell:.2f} m cells   "
                  f"{z_max:.2f} m raw, {z_max * 0.75:.2f} m with the measured 25 % derate")
        print()

    if "emitter ON " in results and "emitter OFF" in results:
        a, b = results["emitter ON "], results["emitter OFF"]
        print("=== emitter verdict ===")
        print(f"  valid fraction {a['valid'] * 100:.1f} % on -> {b['valid'] * 100:.1f} % off "
              f"({(b['valid'] - a['valid']) * 100:+.1f} pts)")
        print("  Indoors the projector does most of the work. OUTDOORS IT DOES ALMOST NONE:")
        print("  it is a ~1 W pattern competing with sunlight, useful to a couple of metres.")
        print("  The forest numbers in nav-sim assume PASSIVE stereo, so the emitter-off")
        print("  column is the one that predicts flight performance. Do the real")
        print("  characterisation outdoors, on bark, at 2/4/6/8 m.")


if __name__ == "__main__":
    main()

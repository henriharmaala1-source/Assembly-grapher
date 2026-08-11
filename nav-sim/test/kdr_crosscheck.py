#!/usr/bin/env python3
"""Cross-language check for the .kdr recording format.

    python3 test/kdr_crosscheck.py [path/to/voxel_live]

A round trip inside one language proves only that the language is
self-consistent with itself. The format exists so that a Python capture on a
Windows laptop can be replayed by a C++ binary through the real map and
planner, so BOTH directions are checked here:

    Python writes -> C++ reads       (run voxel_live --replay on it)
    C++ writes    -> Python reads    (read the file depth_record_check made)

The second direction is the one that catches an endianness or padding mistake,
because the C++ side packs its header field by field and Python unpacks it with
struct: if either got the layout wrong they disagree immediately, whereas two
implementations of the same mistake in one language would agree forever.
"""

import os
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "onboard", "tools"))
from d435i_probe import (kdr_open_writer, kdr_finish, kdr_read,   # noqa: E402
                         KDR_MAGIC, KDR_HEADER_BYTES)

fails = 0


def check(ok, what, detail=""):
    global fails
    print(f"  {what:<58s} {'ok' if ok else 'FAIL'}{'  ' + str(detail) if detail else ''}")
    if not ok:
        fails += 1


def main():
    print("kdr cross-language checks")
    W, H, N = 64, 48, 5
    path = "/tmp/kdr_from_python.kdr"
    fx, fy, ppx, ppy, scale, base = 425.3, 424.9, 31.2, 24.7, 0.001, 0.0499

    # --- Python writes ----------------------------------------------------
    rng = np.random.default_rng(5)
    frames = (rng.integers(500, 6000, size=(N, H, W))).astype("<u2")
    frames[:, 0, 0] = 0                      # an invalid pixel in every frame
    f = kdr_open_writer(path, W, H, scale, fx, fy, ppx, ppy, base, True)
    for fr in frames:
        f.write(np.ascontiguousarray(fr).tobytes())
    kdr_finish(f, N)
    size = os.path.getsize(path)
    check(size == KDR_HEADER_BYTES + N * W * H * 2, "file size is header + frames", size)

    # --- Python reads its own back ----------------------------------------
    back, meta = kdr_read(path)
    check(back.shape == (N, H, W), "shape round-trips", back.shape)
    check(np.array_equal(back, frames), "pixels round-trip BIT-EXACT")
    check(abs(meta["fx"] - fx) < 1e-3 and abs(meta["ppy"] - ppy) < 1e-3,
          "intrinsics round-trip", f"fx={meta['fx']:.2f} ppy={meta['ppy']:.2f}")
    check(meta["emitter_on"] is True, "emitter flag round-trips")

    # --- C++ writes, Python reads (the direction that catches layout bugs) --
    cxx = "/tmp/kdr_test.kdr"
    if os.path.exists(cxx):
        cframes, cmeta = kdr_read(cxx)
        check(cmeta["width"] == 64 and cmeta["height"] == 48,
              "C++-written header parses in Python", f"{cmeta['width']}x{cmeta['height']}")
        check(abs(cmeta["fx"] - 425.3) < 1e-3 and abs(cmeta["fy"] - 424.9) < 1e-3
              and abs(cmeta["ppx"] - 31.2) < 1e-3,
              "C++-written intrinsics parse in Python")
        # depth_record_check writes (k + i*13) % 60000 with pixel 0 zeroed.
        want = np.array([(k + 3 * 13) % 60000 for k in range(64 * 48)],
                        dtype="<u2").reshape(48, 64)
        want[0, 0] = 0
        check(np.array_equal(cframes[3], want),
              "C++-written PIXELS parse in Python bit-exact")
    else:
        check(False, "C++-written file present (run depth_record_check first)", cxx)

    # --- truncation, from the Python side ---------------------------------
    trunc = "/tmp/kdr_from_python_trunc.kdr"
    with open(path, "rb") as a, open(trunc, "wb") as b:
        b.write(a.read(KDR_HEADER_BYTES + int(2.5 * W * H * 2)))
    tf, tm = kdr_read(trunc)
    check(tm["frames"] == 2 and tf.shape[0] == 2,
          "a truncated recording yields only COMPLETE frames", tm["frames"])

    # --- and the app actually replays the Python file ----------------------
    exe = sys.argv[1] if len(sys.argv) > 1 else "/tmp/voxel_live"
    if os.path.exists(exe):
        r = subprocess.run([exe, "--replay", path, "--headless", "--frames", str(N),
                            "--out", "/tmp/kdr_xcheck"],
                           capture_output=True, text=True)
        ok = r.returncode == 0 and "frames from replay" in r.stdout
        check(ok, "voxel_live replays the PYTHON-written recording",
              (r.stdout + r.stderr).strip().splitlines()[-1] if not ok else "")
        # And that it used the intrinsics from the file rather than a default.
        check("fx 425.3" in r.stdout, "and reports the file's own intrinsics")
    else:
        check(False, f"voxel_live present at {exe}", "build it first")

    print(f"{'FAILED' if fails else 'all checks passed'} "
          f"({fails} failure{'' if fails == 1 else 's'})")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

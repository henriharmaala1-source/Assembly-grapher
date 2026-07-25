#!/usr/bin/env python3
"""
Guard against sim/Kotlin tracker DRIFT.

`simtrack.py` is a hand-maintained Python mirror of `LockTracker.kt`, and every
tuning decision in this project is made in the sim and then ported by hand. That
only works while the two agree — and they have silently disagreed before (the
sim was missing the fixed anchor while Kotlin had anchor+adaptive; it was caught
by chance during P0-A, not by any check).

This asserts the shared tuning constants are identical in both files. It is not a
behavioural equivalence proof — it's the cheap 90% that catches the realistic
failure: someone tunes a number on one side only.

Run: python3 test_param_drift.py   (exit 0 = in sync)
"""
import re
import sys
from pathlib import Path

KT = (Path(__file__).parent.parent / "android-tracker/app/src/main/java/"
      "com/kestrel/tracker/track/LockTracker.kt")
PY = Path(__file__).parent / "simtrack.py"

# Constants that MUST match. Kotlin name -> Python name (same unless noted).
SHARED = {
    "CROP": "CROP", "TMPL": "TMPL", "MARGIN": "MARGIN", "SEARCH": "SEARCH",
    "STRIDE": "STRIDE", "LOSS_TIMEOUT": "LOSS_TIMEOUT", "FOV_DELAY": "FOV_DELAY",
    "TMPL_EMA": "TMPL_EMA", "K_KEYFRAMES": "K_KEYFRAMES", "KF_THRESH": "KF_THRESH",
    "KF_ADD_CONF": "KF_ADD_CONF", "OCC_FRAC": "OCC_FRAC", "EGO_CONS": "EGO_CONS",
    "EGO_DEAD": "EGO_DEAD", "EARLY_TERM_PSR": "EARLY_TERM_PSR",
    "HIST_WEIGHT_CAP": "HIST_WEIGHT_CAP",
}


def num(tok):
    return float(tok.rstrip("fF"))


def kotlin_consts(text):
    out = {}
    for m in re.finditer(r"private val (\w+)\s*=\s*(-?[\d.]+f?)\b", text):
        out[m.group(1)] = num(m.group(2))
    return out


def python_consts(text):
    out = {}
    # plain:  NAME = 1.23
    for m in re.finditer(r"^(\w+)\s*=\s*(-?[\d.]+)\s*(?:#.*)?$", text, re.M):
        out[m.group(1)] = num(m.group(2))
    # env-overridable:  NAME = int(os.environ.get('NAME', 123))
    for m in re.finditer(r"^(\w+)\s*=\s*(?:int|float)\(os\.environ\.get\("
                         r"['\"]\w+['\"],\s*(-?[\d.]+)\s*\)\)", text, re.M):
        out[m.group(1)] = num(m.group(2))
    # tuple:  A, B = 1, 2
    for m in re.finditer(r"^(\w+)\s*,\s*(\w+)\s*=\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)", text, re.M):
        out[m.group(1)] = num(m.group(3)); out[m.group(2)] = num(m.group(4))
    return out


def main():
    if not KT.exists():
        print(f"SKIP: {KT} not found"); return 0
    kt = kotlin_consts(KT.read_text())
    py = python_consts(PY.read_text())

    bad = []
    for kname, pname in sorted(SHARED.items()):
        kv, pv = kt.get(kname), py.get(pname)
        if kv is None:
            bad.append(f"  {kname:<16} MISSING in LockTracker.kt")
        elif pv is None:
            bad.append(f"  {pname:<16} MISSING in simtrack.py")
        elif abs(kv - pv) > 1e-6:
            bad.append(f"  {kname:<16} kt={kv!r}  !=  py={pv!r}   <-- DRIFT")

    if bad:
        print("tracker param drift DETECTED:")
        print("\n".join(bad))
        print("\nThe sim no longer mirrors the tracker; sim results are not "
              "evidence about on-device behaviour until these agree.")
        return 1
    print(f"tracker params in sync: {len(SHARED)}/{len(SHARED)} match "
          f"(LockTracker.kt <-> simtrack.py)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

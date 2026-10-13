"""
_debug.py — crash-safe debug logging for the STEP importer pipeline.

Import this module as early as possible (before any OCC imports) so that
Python's faulthandler is armed before any native code runs.  A segfault in
OCC will then write a Python-level traceback to step_debug.log even though
the normal exception mechanism cannot catch SIGSEGV.

Usage:
    from assembly_graph.importers._debug import dbg

    dbg("about to call brepbndlib.Add...")
    brepbndlib.Add(shape, box)
    dbg("brepbndlib.Add returned OK")
"""

from __future__ import annotations

import datetime
import faulthandler
import os
import sys

# ── log file lives in the project root (two levels up from this file) ─────────

_HERE     = os.path.dirname(os.path.abspath(__file__))
_LOG_PATH = os.path.normpath(os.path.join(_HERE, "..", "..", "step_debug.log"))
_FAULT_PATH = _LOG_PATH + ".fault"

# ── arm faulthandler immediately (writes native traceback on SIGSEGV) ─────────
# This must happen BEFORE any OCC imports so segfaults during OCC init are
# captured.  We open the file here; it stays open for the process lifetime.

try:
    _fault_file = open(_FAULT_PATH, "a", encoding="utf-8")
    _fault_file.write(
        f"\n{'='*60}\n"
        f"faulthandler armed at {datetime.datetime.now().isoformat()}\n"
        f"PID={os.getpid()}  Python={sys.version}\n"
        f"{'='*60}\n"
    )
    _fault_file.flush()
    faulthandler.enable(file=_fault_file, all_threads=True)
except Exception:
    pass  # never let debug setup crash the app


def dbg(msg: str) -> None:
    """Write a timestamped line to step_debug.log (flushed immediately)."""
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts} PID={os.getpid()}] {msg}\n"
    try:
        with open(_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)
            f.flush()
    except Exception:
        pass


# Log that this module was imported successfully
dbg(f"_debug.py loaded — faulthandler → {_FAULT_PATH}")

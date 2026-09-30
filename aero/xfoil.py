"""A small XFOIL 6.99 driver for the cross-checks (aero/crosscheck.py).

XFOIL (Drela, GPL) is the panel code with a coupled boundary layer that
NeuralFoil was trained to imitate, so it is the first thing to check NeuralFoil
against: the same physics, run for real on the exact shape.

    apt install xfoil        # Debian/Ubuntu; or set XFOIL=/path/to/xfoil

Ubuntu's build traps floating-point exceptions and dies on the first viscous
point. When that happens and a C compiler is present, this module builds a
one-line shim that turns the trap off (LD_PRELOAD) and carries on.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

PANELS = 279                   # as NeuralFoil's training runs (AeroSandbox's default repanel)
ITER = 250
_ENV = None


def _environment() -> dict:
    """The environment XFOIL runs in; adds the no-trap shim if the build needs it."""
    global _ENV
    if _ENV is not None:
        return _ENV
    exe = os.environ.get("XFOIL", "xfoil")
    if not shutil.which(exe):
        raise RuntimeError("XFOIL not found: install it (apt install xfoil) or set XFOIL=/path/to/xfoil")
    env = dict(os.environ)
    probe = "PLOP\nG F\n\nNACA 0012\nOPER\nVISC 100000\nALFA 1\n\nQUIT\n"
    out = subprocess.run([exe], input=probe, capture_output=True, text=True, timeout=60, env=env)
    if "SIGFPE" in out.stdout + out.stderr or out.returncode < 0:
        cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "kipina"
        cache.mkdir(parents=True, exist_ok=True)
        shim = cache / "xfoil-nofpe.so"
        if not shim.exists():
            src = cache / "xfoil-nofpe.c"
            src.write_text("void _gfortran_set_fpe(int v) { (void)v; }\n")
            subprocess.run(["cc", "-shared", "-fPIC", "-o", str(shim), str(src)], check=True)
        env["LD_PRELOAD"] = str(shim)
        out = subprocess.run([exe], input=probe, capture_output=True, text=True, timeout=60, env=env)
        if "SIGFPE" in out.stdout + out.stderr or out.returncode < 0:
            raise RuntimeError("XFOIL crashes on a floating-point trap even with the shim")
    env["XFOIL_EXE"] = exe
    _ENV = env
    return env


def version() -> str:
    out = subprocess.run([_environment()["XFOIL_EXE"]], input="QUIT\n", capture_output=True, text=True,
                         timeout=30, env=_environment())
    for line in out.stdout.splitlines():
        if "Version" in line:
            return "XFOIL " + line.split("Version")[1].strip()
    return "XFOIL"


def write_dat(path: Path, x, y, name: str = "section") -> None:
    path.write_text(name + "\n" + "\n".join(f"{a:.7f} {b:.7f}" for a, b in zip(x, y)) + "\n")


def _read_polar(path: Path) -> list[dict]:
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 7:
            try:
                v = [float(p) for p in parts[:7]]
            except ValueError:
                continue
            rows.append(dict(zip(("alpha", "cl", "cd", "cdp", "cm", "xtr_top", "xtr_bot"), v)))
    return rows


def _read_cp(path: Path) -> dict | None:
    if not path.exists():
        return None
    xs, cps = [], []
    for line in path.read_text().splitlines():
        parts = line.split()
        try:
            v = [float(p) for p in parts]
        except ValueError:
            continue
        if len(v) >= 2:
            xs.append(v[0])
            cps.append(v[-1])
    return {"x": xs, "cp": cps} if xs else None


def run(dat: Path, re: float, n_crit: float, ops: list[str], cp: bool = False, timeout: int = 180) -> dict:
    """Load the section, set Re and n_crit, then run `ops` (OPER commands) with
    polar accumulation on; returns the converged points (and the Cp of the last)."""
    env = _environment()
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        shutil.copy(dat, tmp / "section.dat")
        keys = ["PLOP", "G F", "", "LOAD section.dat", "PPAR", f"N {PANELS}", "", "",
                "OPER", "VPAR", f"N {n_crit:g}", "", f"VISC {re:.0f}", f"ITER {ITER}"]
        keys += ops
        if cp:
            keys += ["CPWR cp.txt"]
        keys += ["", "QUIT"]
        try:
            subprocess.run([env["XFOIL_EXE"]], input="\n".join(keys) + "\n", capture_output=True, text=True,
                           timeout=timeout, cwd=tmp, env=env)
        except subprocess.TimeoutExpired:
            pass
        return {"points": _read_polar(tmp / "polar.txt"), "cp": _read_cp(tmp / "cp.txt") if cp else None}


def at_cl(dat: Path, re: float, n_crit: float, cl: float, alpha_guess: float, cp: bool = False) -> dict | None:
    """The converged point at a lift coefficient. Walks the angle up (or down)
    from zero first so the boundary layer starts from an easy solution."""
    step = 0.5 if alpha_guess >= 0 else -0.5
    walk = [f"ASEQ 0 {alpha_guess:.2f} {step}"] if abs(alpha_guess) > 0.5 else ["ALFA 0"]
    r = run(dat, re, n_crit, walk + ["PACC", "polar.txt", "", f"CL {cl:.4f}"], cp=cp)
    pts = [p for p in r["points"] if abs(p["cl"] - cl) < 0.005]
    if pts:
        return {**pts[-1], "cp": r["cp"]}
    # fall back: a fine sweep through the target and interpolation
    lo, hi = sorted((0.0, alpha_guess + (2.0 if alpha_guess >= 0 else -2.0)))
    sw = sweep(dat, re, n_crit, np.arange(lo - 1.0, hi + 1.0, 0.25))
    a = sorted(sw, key=lambda p: p["alpha"])
    for p, q in zip(a, a[1:]):
        if (p["cl"] - cl) * (q["cl"] - cl) <= 0 and q["alpha"] - p["alpha"] <= 0.51 and q["cl"] != p["cl"]:
            t = (cl - p["cl"]) / (q["cl"] - p["cl"])
            return {k: p[k] + t * (q[k] - p[k]) for k in p} | {"cp": None, "interpolated": True}
    return None


def sweep(dat: Path, re: float, n_crit: float, alphas) -> list[dict]:
    """An angle sweep, walked outward from zero in both directions."""
    alphas = np.asarray(alphas, float)
    step = float(np.min(np.diff(np.sort(alphas)))) if alphas.size > 1 else 0.5
    out = []
    up, down = alphas[alphas >= 0], alphas[alphas < 0]
    if up.size:
        out += run(dat, re, n_crit, ["PACC", "polar.txt", "", f"ASEQ 0 {up.max():.3f} {step:.3f}"])["points"]
    if down.size:
        out += run(dat, re, n_crit, ["ALFA 0", "PACC", "polar.txt", "", f"ASEQ {-step:.3f} {down.min():.3f} {-step:.3f}"])["points"]
    keep = {round(a, 3) for a in alphas}
    return sorted([p for p in out if round(p["alpha"], 3) in keep], key=lambda p: p["alpha"])

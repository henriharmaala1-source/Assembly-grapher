"""Drag-and-drop / double-click mode for the packaged executable.

Drop one or more logs on bbanalyzer.exe (or run it and drag a log into the window). It asks for
the aircraft details once, remembers them for next time, writes the report next to the log and
opens it in the browser.
"""

from __future__ import annotations

import json
import sys
import traceback
import webbrowser
from pathlib import Path

from .config import Aircraft

SETTINGS = Path.home() / ".bbanalyzer.json"
FIELDS = [  # key, prompt, type
    ("mass_kg", "All-up weight incl. battery [kg]", float),
    ("prop_diameter_in", "Prop diameter [inch]", float),
    ("capacity_mah", "Battery capacity [mAh]", float),
    ("blades", "Blades per prop", int),
    ("motor_range", "Motor output range for CSV logs, LO,HI (blank = from log header)", str),
]


def _load_settings() -> dict:
    try:
        return json.loads(SETTINGS.read_text())
    except (OSError, ValueError):
        return {"blades": 2}


def _save_settings(s: dict) -> None:
    try:
        SETTINGS.write_text(json.dumps(s, indent=2))
    except OSError:
        pass


def _ask(prompt: str, default, typ):
    shown = "" if default in (None, "") else f" [{default}]"
    while True:
        raw = input(f"  {prompt}{shown}: ").strip()
        if raw == "":
            return default
        if raw == "-":
            return None
        try:
            return typ(raw.replace(",", ".") if typ is not str else raw)
        except ValueError:
            print("    not a number - try again, Enter keeps the value, '-' clears it")


def _clean_path(p: str) -> Path:
    return Path(p.strip().strip('"').strip("'"))


def _pick_log_index(path: Path) -> int | None:
    if path.suffix.lower() == ".csv":
        return None
    from .loader import list_logs

    logs = list_logs(path)
    if len(logs) <= 1:
        return None
    print(f"\n  {path.name} contains {len(logs)} logs:")
    for item in logs:
        print(f"    {item['index']}: {item['start']}")
    idx = _ask("Log number (Enter = the largest)", None, int)
    return idx


def run(paths: list[str]) -> int:
    print("bbanalyzer – blackbox log analyser\n")
    files = [_clean_path(p) for p in paths]
    if not files:
        raw = input("Drag a flight log (.TXT / .BBL / .BIN / .log / .CSV) into this window and press Enter:\n> ")
        files = [_clean_path(raw)] if raw.strip() else []
    files = [f for f in files if f.exists()]
    if not files:
        print("No log file found.")
        return 1

    s = _load_settings()
    print("Aircraft details (Enter keeps the remembered value, '-' clears it):")
    for key, prompt, typ in FIELDS:
        s[key] = _ask(prompt, s.get(key), typ)
    _save_settings(s)

    ac = Aircraft(mass_kg=s.get("mass_kg"), prop_diameter_in=s.get("prop_diameter_in"),
                  capacity_mah=s.get("capacity_mah"), blades=s.get("blades") or 2)
    mr = None
    if s.get("motor_range"):
        try:
            mr = tuple(float(x) for x in str(s["motor_range"]).split(","))[:2]
        except ValueError:
            print("  motor range ignored (use the form 1080,2000)")

    from . import analysis, report

    status = 0
    for f in files:
        print(f"\nAnalysing {f.name} ...")
        try:
            idx = _pick_log_index(f)
            log, res, phases = analysis.run(str(f), ac, idx, mr)
            out = f.with_name(f"{f.stem}_report.html")
            out.write_text(report.build(log, res, phases, ac), encoding="utf-8")
            out.with_suffix(".json").write_text(analysis.to_json(res), encoding="utf-8")
            print(f"  report written: {out}")
            for row in res.get("scorecard", []):
                if row["status"] != "good":
                    print(f"    [{row['status']:5}] {row['area']:<22} {row['axis']:<10} {row['value']}")
            webbrowser.open(out.resolve().as_uri())
        except Exception:
            status = 1
            print("  FAILED:")
            traceback.print_exc()
    return status


def main_interactive(paths: list[str]) -> int:
    try:
        code = run(paths)
    except (KeyboardInterrupt, EOFError):
        code = 1
    if getattr(sys, "frozen", False):
        try:
            input("\nPress Enter to close.")
        except EOFError:
            pass
    return code

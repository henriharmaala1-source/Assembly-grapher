"""Command line:

  python -m bbanalyzer analyze LOG.TXT --mass 1.1 --capacity 2200 --prop 7 [--out report.html]
  python -m bbanalyzer list LOG.TXT
  python -m bbanalyzer compare before.json after.json
  python -m bbanalyzer [LOG ...]            window (GUI)
  python -m bbanalyzer --console [LOG ...]  console prompts instead of the window
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .config import Aircraft


def _analyze(a) -> int:
    from . import analysis, report

    ac = Aircraft(mass_kg=a.mass, capacity_mah=a.capacity, prop_diameter_in=a.prop, rotors=a.rotors,
                  blades=a.blades, cells=a.cells, eta_drag=a.eta, air_temp_c=a.air_temp,
                  coast_throttle=a.coast_throttle, motor_poles=a.poles)
    mr = tuple(float(x) for x in a.motor_range.split(",")) if a.motor_range else None
    log, res, phases = analysis.run(a.log, ac, a.log_index, mr)
    src = Path(a.log)
    out = Path(a.out) if a.out else src.with_name(f"{src.stem}_report.html")
    out.write_text(report.build(log, res, phases, ac), encoding="utf-8")
    js = Path(a.json or out.with_suffix(".json"))
    js.write_text(analysis.to_json(res), encoding="utf-8")
    print(f"report : {out}\nmetrics: {js}")
    for row in res.get("scorecard", []):
        if row["status"] != "good":
            print(f"  [{row['status']:5}] {row['area']:<22} {row['axis']:<10} {row['value']}")
    return 0


def _list(a) -> int:
    from .loader import list_logs

    for item in list_logs(a.log):
        print(f"{item['index']:>3}  {item['firmware']}  {item['start']}")
    return 0


def _compare(a) -> int:
    from . import compare

    print(f"| metric | {Path(a.a).stem} | {Path(a.b).stem} | change |\n|---|---|---|---|")
    for label, x, y, ch in compare.rows(compare.load(a.a), compare.load(a.b)):
        fx = f"{x:.4g}" if isinstance(x, (int, float)) else "–"
        fy = f"{y:.4g}" if isinstance(y, (int, float)) else "–"
        print(f"| {label} | {fx} | {fy} | {ch} |")
    return 0


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    # no sub-command: double-click or drag-and-drop -> window (console prompts with --console)
    if not argv or (argv[0] not in ("analyze", "list", "compare", "-h", "--help")):
        if argv[:1] != ["--console"]:
            try:
                from .gui import main as gui_main
                return gui_main(argv)
            except ImportError:          # Python without tkinter
                pass
        from .app import main_interactive
        return main_interactive([a for a in argv if a != "--console"])
    ap = argparse.ArgumentParser(prog="bbanalyzer", description="Blackbox log analyser")
    sub = ap.add_subparsers(dest="cmd", required=True)
    an = sub.add_parser("analyze", help="analyse one log and write an HTML report + JSON metrics")
    an.add_argument("log")
    an.add_argument("--log-index", type=int, default=None, help="log number inside the file (default: largest)")
    an.add_argument("--mass", type=float, help="all-up weight in kg")
    an.add_argument("--capacity", type=float, help="battery capacity mAh")
    an.add_argument("--prop", type=float, help="prop diameter in inches")
    an.add_argument("--rotors", type=int, default=4)
    an.add_argument("--blades", type=int, default=2)
    an.add_argument("--cells", type=int)
    an.add_argument("--poles", type=int, default=14, help="motor magnet poles (for eRPM logs)")
    an.add_argument("--eta", type=float, default=0.55, help="assumed drag-work efficiency for absolute CdA")
    an.add_argument("--air-temp", type=float, default=15.0)
    an.add_argument("--coast-throttle", type=float, default=0.20)
    an.add_argument("--motor-range", help="raw motor output range LO,HI, e.g. 1080,2000 (CSV logs have no header)")
    an.add_argument("--out")
    an.add_argument("--json")
    an.set_defaults(fn=_analyze)
    ls = sub.add_parser("list", help="list logs inside a blackbox file")
    ls.add_argument("log")
    ls.set_defaults(fn=_list)
    cp = sub.add_parser("compare", help="compare two JSON metric files (markdown table)")
    cp.add_argument("a")
    cp.add_argument("b")
    cp.set_defaults(fn=_compare)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())

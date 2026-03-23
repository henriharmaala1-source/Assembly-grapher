"""
Assembly Grapher — GUI entry point with demo data pre-loaded.

Usage:
    python gui_main.py          # open with demo assembly + fasteners
    python gui_main.py --empty  # open empty (load files manually)
"""

import argparse
import sys


def main() -> None:
    parser = argparse.ArgumentParser(description="Assembly Grapher GUI")
    parser.add_argument("--empty", action="store_true",
                        help="Start with empty workspace")
    args = parser.parse_args()

    from gui.app import App
    from dfma.analyzer import analyze

    app = App()

    if not args.empty:
        # Pre-load the demo assembly and fasteners so the GUI shows real data
        from dfma_demo import build_pneumatic_valve, build_fasteners
        assembly  = build_pneumatic_valve()
        fasteners = build_fasteners()
        result    = analyze(assembly, fasteners=fasteners)

        # Push data into panels via the event bus
        app.bus.publish("assembly_loaded",  assembly)
        app.bus.publish("fasteners_loaded", fasteners)
        app.bus.publish("warnings_updated", result)

        # Pre-populate a simple topological sequence for the demo
        from dfma.rules.geometry_scorer import estimate_total_time
        steps = [
            {
                "part_id":   p.id,
                "part_name": p.name,
                "process":   p.process.value,
                "time_s":    estimate_total_time(p.geometry) * p.quantity,
            }
            for p in assembly.all_parts()
        ]
        app.bus.publish("sequence_ready", steps)

        # Set status bar
        e = len(result.errors())
        w = len(result.warnings_only())
        i = len(result.infos())
        app._set_status(
            f"Demo assembly loaded — "
            f"{e} error(s), {w} warning(s), {i} info(s) | "
            f"DFA Index: {result.dfa_index:.1%}"
        )

    app.mainloop()


if __name__ == "__main__":
    main()

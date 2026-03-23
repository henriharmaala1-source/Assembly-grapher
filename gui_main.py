"""
Assembly Grapher — GUI entry point with demo data pre-loaded.

Usage:
    python gui_main.py          # open with demo assembly + fasteners
    python gui_main.py --empty  # open empty (load files manually)
"""

import argparse


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

        # Run the full assembly planner and pre-populate graph + sequence tabs
        try:
            from graph_demo import build_planner
            planner = build_planner()
            plan    = planner.plan(sa_iterations=2000)
            app._plan = plan

            app.bus.publish("plan_ready",     plan)
            app.bus.publish("sequence_ready", {
                "steps":        plan.optimized_steps(),
                "cost_summary": plan.optimized_sequence.summary(),
            })
        except Exception as exc:
            # Fallback: simple BOM-order sequence without full planning
            from dfma.rules.geometry_scorer import estimate_total_time
            steps = [
                {
                    "part_id":        p.id,
                    "part_name":      p.name,
                    "direction":      "—",
                    "tools":          "—",
                    "subassembly_id": "—",
                    "time_s":         estimate_total_time(p.geometry) * p.quantity,
                }
                for p in assembly.all_parts()
            ]
            app.bus.publish("sequence_ready", steps)
            print(f"[gui_main] Planner fallback: {exc}")

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

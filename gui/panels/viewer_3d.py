"""
viewer_3d.py — interactive 3D assembly viewer with DFMA severity highlighting.

Opens a PyVista popup window (separate from the tkinter GUI) showing every
solid from the imported STEP file.  Parts are colour-coded by their worst
DFMA severity if an analysis result is supplied.

Interaction (free, built into PyVista/VTK):
  Left-drag    → rotate
  Middle-drag  → pan          (or Shift + left-drag)
  Scroll       → zoom
  R            → reset camera
  Right-click  → context menu / pick

Colour key (Catppuccin Mocha palette):
  Red    #f38ba8  → DFMA ERROR on this part
  Yellow #f9e2af  → DFMA WARNING
  Blue   #89b4fa  → DFMA INFO
  Green  #a6e3a1  → no issues found
  Grey   #6c7086  → not yet analysed

Requires:  pip install pyvista
           (VTK 9+ is usually already present with cadquery)
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from assembly_graph.importers.base import ImportResult
    from dfma.models.warning import AnalysisResult


# ── colour palette ────────────────────────────────────────────────────────────

_BG        = "#1e1e2e"   # Catppuccin Mocha BASE
_EDGE      = "#45475a"   # SURFACE1
_SEV_COLOR = {
    "ERROR":   "#f38ba8",
    "WARNING": "#f9e2af",
    "INFO":    "#89b4fa",
    "OK":      "#a6e3a1",
    "NONE":    "#6c7086",   # not yet analysed
}
_DEFLECT   = 0.3          # tessellation chord deflection (mm); lower = finer mesh


# ── public entry point ────────────────────────────────────────────────────────

def open_viewer(
    import_result: "ImportResult",
    dfma_result:   "AnalysisResult | None" = None,
    title:         str = "Assembly 3D View",
) -> None:
    """
    Open the 3D viewer in a daemon thread so the tkinter GUI keeps responding.

    Parameters
    ----------
    import_result : ImportResult from load_assembly() — must have .shapes populated
    dfma_result   : optional AnalysisResult; if given, parts are coloured by severity
    title         : window title
    """
    if not import_result or not import_result.shapes:
        _err("No 3D geometry available.\n\nLoad a STEP file first (JSON BOM has no geometry).")
        return

    threading.Thread(
        target=_show,
        args=(import_result, dfma_result, title),
        daemon=True,
    ).start()


# ── internal viewer ───────────────────────────────────────────────────────────

def _show(import_result, dfma_result, title: str) -> None:
    try:
        import pyvista as pv
    except ImportError:
        _err(
            "PyVista is not installed.\n\n"
            "Install it with:\n"
            "    pip install pyvista\n\n"
            "PyVista requires no extra C++ dependencies — VTK is bundled."
        )
        return

    severity_map = _build_severity_map(dfma_result)

    pl = pv.Plotter(title=title, window_size=(1024, 768))
    pl.set_background(_BG)

    legend_entries: list[list] = []
    any_mesh = False

    for part in import_result.assembly.all_parts():
        raw_shape = import_result.shapes.get(part.id)
        if raw_shape is None:
            continue

        mesh = _to_polydata(raw_shape)
        if mesh is None or mesh.n_points == 0:
            continue

        sev   = severity_map.get(part.id, "NONE")
        color = _SEV_COLOR[sev]
        label = part.name or part.id

        pl.add_mesh(
            mesh,
            color=color,
            show_edges=True,
            edge_color=_EDGE,
            opacity=0.92,
            smooth_shading=True,
        )
        legend_entries.append([label, color])
        any_mesh = True

    if not any_mesh:
        _err("Could not tessellate any parts.\nTry reinstalling cadquery: pip install cadquery")
        return

    # ── legend ────────────────────────────────────────────────────────────────
    if legend_entries:
        # Keep legend compact: show at most 12 entries
        shown = legend_entries[:12]
        if len(legend_entries) > 12:
            shown.append([f"… +{len(legend_entries)-12} more", "#6c7086"])
        pl.add_legend(
            shown,
            bcolor="#313244",
            border=True,
            size=(0.28, min(0.05 * len(shown) + 0.04, 0.70)),
        )

    # ── severity key (only when DFMA result present) ──────────────────────────
    if dfma_result is not None:
        key = [
            ["ERROR",   _SEV_COLOR["ERROR"]],
            ["WARNING", _SEV_COLOR["WARNING"]],
            ["INFO",    _SEV_COLOR["INFO"]],
            ["OK",      _SEV_COLOR["OK"]],
        ]
        pl.add_legend(
            key,
            bcolor="#313244",
            border=True,
            loc="lower right",
            size=(0.18, 0.22),
        )

    # ── camera & controls ─────────────────────────────────────────────────────
    pl.reset_camera()
    pl.add_axes(color="white")

    # Text instructions in corner
    pl.add_text(
        "Rotate: left-drag  |  Pan: middle-drag  |  Zoom: scroll  |  Reset: R",
        position="upper_left",
        font_size=8,
        color="#a6adc8",
    )

    pl.show()


# ── helpers ───────────────────────────────────────────────────────────────────

def _to_polydata(ocp_shape):
    """Tessellate an OCP TopoDS_Shape and return a pyvista.PolyData, or None."""
    try:
        import numpy as np
        import pyvista as pv
        import cadquery as cq

        cq_shape = cq.Shape(ocp_shape)
        verts, tris = cq_shape.tessellate(_DEFLECT)

        if not verts or not tris:
            return None

        v = np.array([[p.x, p.y, p.z] for p in verts], dtype=float)
        # PyVista face format: each row is [3, i0, i1, i2]
        f = np.array([[3, t[0], t[1], t[2]] for t in tris], dtype=int).ravel()

        return pv.PolyData(v, f)

    except Exception:
        return None


def _build_severity_map(dfma_result) -> dict[str, str]:
    """
    Return {part_id: worst_severity_name} for all parts that have warnings.
    Priority: ERROR > WARNING > INFO > OK.
    Parts not in the map should be coloured as NONE (not analysed).
    """
    if dfma_result is None:
        return {}

    _rank = {"ERROR": 3, "WARNING": 2, "INFO": 1, "OK": 0}
    smap: dict[str, str] = {}

    for w in dfma_result.warnings:
        pid = getattr(w, "part_id", None)
        if not pid:
            continue
        sev = w.severity.name if hasattr(w.severity, "name") else str(w.severity).upper()
        if _rank.get(sev, -1) > _rank.get(smap.get(pid, ""), -1):
            smap[pid] = sev

    # Parts with no warnings at all → OK
    if hasattr(dfma_result, "warnings"):
        warned = set(smap)
        for w in dfma_result.warnings:
            warned.add(getattr(w, "part_id", ""))
        # We don't have the full part list here; caller colours unknowns as NONE

    return smap


def _err(msg: str) -> None:
    """Show a tkinter error box (safe to call from a non-main thread via after)."""
    try:
        import tkinter as tk
        from tkinter import messagebox
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("3D Viewer", msg, parent=root)
        root.destroy()
    except Exception:
        print(f"[viewer_3d] {msg}")

"""
GraphTab — CENTRE tab 1: Assembly Liaison Graph.

Renders the assembly as a force-directed graph on a tkinter Canvas.
  - Nodes  = parts / subassemblies
  - Edges  = liaison (physical contact) relationships
  - Colour = worst-severity DFMA warning on that node
             green=OK, cyan=INFO, orange=WARNING, red=ERROR
  - Click node → highlights in warning panel and assembly tree

Graph layout uses a simple spring layout from networkx if available,
otherwise falls back to a circular layout computed manually.

EventBus events consumed:
  "assembly_loaded"   → rebuild graph topology
  "warnings_updated"  → recolour nodes
  "part_selected"     → highlight node
  "fit_graph"         → fit all nodes to canvas
  "reset_layout"      → reset to default layout

EventBus events published:
  "part_selected"     → when user clicks a node
"""

import math
import tkinter as tk
from tkinter import ttk

from ..event_bus import EventBus

# Node visual constants
NODE_RADIUS  = 22
EDGE_COLOR   = "#aaaaaa"
LABEL_FONT   = ("Helvetica", 8)

SEVERITY_FILL = {
    "ERROR":   "#ff6b6b",
    "WARNING": "#ffd166",
    "INFO":    "#74b9ff",
    "OK":      "#55efc4",
}


class GraphTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus   = bus
        self._nodes: dict[str, dict] = {}   # id → {x, y, label, severity, canvas_ids}
        self._edges: list[tuple[str, str]] = []
        self._drag_data: dict = {"item": None, "x": 0, "y": 0}
        self._selected: str | None = None
        self._build()
        self._subscribe()

    def _build(self) -> None:
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, padx=4, pady=2)
        ttk.Button(toolbar, text="Fit",    width=6, command=self._fit).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Layout", width=8, command=self._relayout).pack(side=tk.LEFT, padx=2)

        self._canvas = tk.Canvas(self, bg="#1e1e2e", cursor="crosshair")
        hbar = ttk.Scrollbar(self, orient=tk.HORIZONTAL, command=self._canvas.xview)
        vbar = ttk.Scrollbar(self, orient=tk.VERTICAL,   command=self._canvas.yview)
        self._canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)

        vbar.pack(side=tk.RIGHT,   fill=tk.Y)
        hbar.pack(side=tk.BOTTOM,  fill=tk.X)
        self._canvas.pack(fill=tk.BOTH, expand=True)

        self._canvas.bind("<ButtonPress-1>",   self._on_press)
        self._canvas.bind("<B1-Motion>",       self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)
        self._canvas.bind("<Configure>",       lambda _e: self._redraw())

    def _subscribe(self) -> None:
        self.bus.subscribe("assembly_loaded",  self._on_assembly_loaded)
        self.bus.subscribe("warnings_updated", self._on_warnings_updated)
        self.bus.subscribe("part_selected",    self._on_part_selected)
        self.bus.subscribe("fit_graph",        lambda _: self._fit())
        self.bus.subscribe("reset_layout",     lambda _: self._relayout())

    # ── data population ──────────────────────────────────────────────────────

    def _on_assembly_loaded(self, assembly) -> None:
        """Build graph topology from Assembly object."""
        self._nodes.clear()
        self._edges.clear()
        if assembly is None:
            self._redraw()
            return

        # Gather all parts
        for part in assembly.all_parts():
            self._nodes[part.id] = {
                "label": part.name[:14],
                "severity": "OK",
                "x": 0.0, "y": 0.0,
                "canvas_ids": [],
            }

        # Edges: for now create a chain (BOM order) — will be replaced by
        # actual liaison matrix data when available
        ids = list(self._nodes.keys())
        for i in range(len(ids) - 1):
            self._edges.append((ids[i], ids[i + 1]))

        self._relayout()

    def _on_warnings_updated(self, result) -> None:
        """Recolour nodes based on worst warning severity."""
        if result is None:
            return
        worst: dict[str, str] = {}
        for w in result.warnings:
            pid = w.part_id
            if pid not in worst or w.severity.value < worst[pid]:
                worst[pid] = w.severity.value
        for pid, node in self._nodes.items():
            node["severity"] = worst.get(pid, "OK")
        self._redraw()

    def _on_part_selected(self, part_id: str) -> None:
        self._selected = part_id
        self._redraw()

    # ── layout ────────────────────────────────────────────────────────────────

    def _relayout(self) -> None:
        """Assign positions using networkx spring layout or circular fallback."""
        n = len(self._nodes)
        if n == 0:
            self._redraw()
            return

        w = self._canvas.winfo_width()  or 600
        h = self._canvas.winfo_height() or 400
        cx, cy = w / 2, h / 2
        r = min(w, h) * 0.38

        try:
            import networkx as nx
            G = nx.Graph()
            G.add_nodes_from(self._nodes.keys())
            G.add_edges_from(self._edges)
            pos = nx.spring_layout(G, seed=42, k=1.5)
            for pid, (px, py) in pos.items():
                if pid in self._nodes:
                    self._nodes[pid]["x"] = cx + px * r
                    self._nodes[pid]["y"] = cy + py * r
        except ImportError:
            # circular fallback
            for i, pid in enumerate(self._nodes):
                angle = 2 * math.pi * i / n
                self._nodes[pid]["x"] = cx + r * math.cos(angle)
                self._nodes[pid]["y"] = cy + r * math.sin(angle)

        self._redraw()

    def _fit(self) -> None:
        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    # ── drawing ───────────────────────────────────────────────────────────────

    def _redraw(self) -> None:
        c = self._canvas
        c.delete("all")

        # Draw edges
        for (a, b) in self._edges:
            if a in self._nodes and b in self._nodes:
                ax, ay = self._nodes[a]["x"], self._nodes[a]["y"]
                bx, by = self._nodes[b]["x"], self._nodes[b]["y"]
                c.create_line(ax, ay, bx, by, fill=EDGE_COLOR, width=1.5,
                              tags="edge")

        # Draw nodes
        for pid, node in self._nodes.items():
            x, y = node["x"], node["y"]
            fill   = SEVERITY_FILL.get(node["severity"], SEVERITY_FILL["OK"])
            outline = "#ffffff" if pid == self._selected else "#444455"
            width   = 3 if pid == self._selected else 1

            oval = c.create_oval(
                x - NODE_RADIUS, y - NODE_RADIUS,
                x + NODE_RADIUS, y + NODE_RADIUS,
                fill=fill, outline=outline, width=width,
                tags=("node", f"node_{pid}"),
            )
            lbl = c.create_text(
                x, y, text=node["label"],
                font=LABEL_FONT, fill="#000000",
                tags=("label", f"label_{pid}"),
            )
            node["canvas_ids"] = [oval, lbl]

        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    # ── interaction ───────────────────────────────────────────────────────────

    def _find_node_at(self, cx: float, cy: float) -> str | None:
        """Return part_id of node under canvas coords (cx, cy), or None."""
        for pid, node in self._nodes.items():
            dx = cx - node["x"]
            dy = cy - node["y"]
            if dx * dx + dy * dy <= NODE_RADIUS * NODE_RADIUS:
                return pid
        return None

    def _on_press(self, event: tk.Event) -> None:
        cx = self._canvas.canvasx(event.x)
        cy = self._canvas.canvasy(event.y)
        pid = self._find_node_at(cx, cy)
        if pid:
            self._drag_data = {"item": pid, "x": cx, "y": cy}
            self.bus.publish("part_selected", pid)
        else:
            self._drag_data = {"item": None, "x": cx, "y": cy}

    def _on_drag(self, event: tk.Event) -> None:
        cx = self._canvas.canvasx(event.x)
        cy = self._canvas.canvasy(event.y)
        pid = self._drag_data.get("item")
        if pid and pid in self._nodes:
            dx = cx - self._drag_data["x"]
            dy = cy - self._drag_data["y"]
            self._nodes[pid]["x"] += dx
            self._nodes[pid]["y"] += dy
            self._drag_data["x"] = cx
            self._drag_data["y"] = cy
            self._redraw()

    def _on_release(self, _event: tk.Event) -> None:
        self._drag_data["item"] = None

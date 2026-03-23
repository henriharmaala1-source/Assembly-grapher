"""
GraphTab — CENTRE tab 1: Assembly Liaison Graph.

Renders the assembly as a force-directed graph on a tkinter Canvas.
  - Nodes  = parts
  - Edges  = liaison (physical contact) relationships (gray, undirected)
            AND/OR precedence relationships (white arrows, directed)
  - Colour = worst-severity DFMA warning on that node
             green=OK, cyan=INFO, orange=WARNING, red=ERROR
  - Hull   = coloured bounding-box around each detected subassembly group
  - Click node → highlights in warning panel and assembly tree

EventBus events consumed:
  "assembly_loaded"   → rebuild graph topology from Assembly object
  "plan_ready"        → rebuild graph from AssemblyPlan (edges + subassemblies)
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

# ── visual constants ──────────────────────────────────────────────────────────

NODE_RADIUS     = 22
EDGE_LIAISON    = "#6a6a8a"   # undirected liaison edges
EDGE_PREC       = "#ccccdd"   # directed precedence edges
LABEL_FONT      = ("Helvetica", 8)
HULL_PADDING    = 30          # px padding around subassembly bounding box
HULL_DASH       = (6, 3)

SEVERITY_FILL = {
    "ERROR":   "#ff6b6b",
    "WARNING": "#ffd166",
    "INFO":    "#74b9ff",
    "OK":      "#55efc4",
}

# 10 distinct colours for subassembly hull outlines
SUBASM_PALETTE = [
    "#e17055", "#00b894", "#0984e3", "#fdcb6e", "#6c5ce7",
    "#fd79a8", "#55efc4", "#74b9ff", "#a29bfe", "#d63031",
]


class GraphTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus   = bus
        self._nodes: dict[str, dict] = {}   # id → {x, y, label, severity, canvas_ids}
        self._liaison_edges:  list[tuple[str, str]] = []
        self._prec_edges:     list[tuple[str, str]] = []
        # id → {part_ids: set, color: str, label: str}
        self._subassemblies: list[dict] = []
        self._drag_data: dict = {"item": None, "x": 0, "y": 0}
        self._selected: str | None = None
        self._show_prec  = tk.BooleanVar(value=True)
        self._show_liaison = tk.BooleanVar(value=True)
        self._build()
        self._subscribe()

    def _build(self) -> None:
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, padx=4, pady=2)
        ttk.Button(toolbar, text="Fit",    width=6, command=self._fit).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Layout", width=8, command=self._relayout).pack(side=tk.LEFT, padx=2)
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, padx=4, fill=tk.Y)
        ttk.Checkbutton(toolbar, text="Liaison edges",   variable=self._show_liaison,
                        command=self._redraw).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(toolbar, text="Precedence arrows", variable=self._show_prec,
                        command=self._redraw).pack(side=tk.LEFT, padx=2)

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
        self.bus.subscribe("plan_ready",       self._on_plan_ready)
        self.bus.subscribe("warnings_updated", self._on_warnings_updated)
        self.bus.subscribe("part_selected",    self._on_part_selected)
        self.bus.subscribe("fit_graph",        lambda _: self._fit())
        self.bus.subscribe("reset_layout",     lambda _: self._relayout())

    # ── data population ──────────────────────────────────────────────────────

    def _on_assembly_loaded(self, assembly) -> None:
        """Build graph topology from Assembly object (liaison edges unknown → chain)."""
        self._nodes.clear()
        self._liaison_edges.clear()
        self._prec_edges.clear()
        self._subassemblies.clear()
        if assembly is None:
            self._redraw()
            return

        for part in assembly.all_parts():
            self._nodes[part.id] = {
                "label": part.name[:14],
                "severity": "OK",
                "subasm_color": "",
                "x": 0.0, "y": 0.0,
                "canvas_ids": [],
            }

        # Fallback: BOM-order chain until plan_ready provides real edges
        ids = list(self._nodes.keys())
        for i in range(len(ids) - 1):
            self._liaison_edges.append((ids[i], ids[i + 1]))

        self._relayout()

    def _on_plan_ready(self, plan) -> None:
        """
        Populate graph from a full AssemblyPlan.
        Updates nodes, liaison edges, precedence edges, and subassembly hulls.
        """
        if plan is None:
            return

        # ── nodes ─────────────────────────────────────────────────────────
        self._nodes.clear()
        for pid, node in plan.graph.nodes.items():
            self._nodes[pid] = {
                "label":        (node.name or pid)[:14],
                "severity":     "OK",
                "subasm_color": "",
                "x":            0.0,
                "y":            0.0,
                "canvas_ids":   [],
            }

        # ── liaison edges (undirected contacts) ────────────────────────────
        self._liaison_edges.clear()
        seen_pairs: set[frozenset] = set()
        for contact in plan.liaison.all_contacts():
            key = frozenset({contact.part_a, contact.part_b})
            if key not in seen_pairs:
                seen_pairs.add(key)
                self._liaison_edges.append((contact.part_a, contact.part_b))

        # ── precedence edges (directed DAG) ────────────────────────────────
        self._prec_edges.clear()
        for pid, node in plan.graph.nodes.items():
            for succ in plan.graph.successors(pid):
                self._prec_edges.append((pid, succ))

        # ── subassembly groups ─────────────────────────────────────────────
        self._subassemblies.clear()
        for idx, sub in enumerate(plan.subassemblies[:10]):
            color = SUBASM_PALETTE[idx % len(SUBASM_PALETTE)]
            self._subassemblies.append({
                "part_ids": set(sub.part_ids),
                "color":    color,
                "label":    sub.id,
            })
            # Tag each node with its subassembly outline color
            for pid in sub.part_ids:
                if pid in self._nodes:
                    self._nodes[pid]["subasm_color"] = color

        self._relayout()

    def _on_warnings_updated(self, result) -> None:
        """Recolour nodes based on worst warning severity."""
        if result is None:
            return
        worst: dict[str, str] = {}
        for w in result.warnings:
            pid = w.part_id
            sev = w.severity.value
            order = {"ERROR": 0, "WARNING": 1, "INFO": 2}
            if pid not in worst or order.get(sev, 9) < order.get(worst[pid], 9):
                worst[pid] = sev
        for pid, node in self._nodes.items():
            node["severity"] = worst.get(pid, "OK")
        self._redraw()

    def _on_part_selected(self, part_id: str) -> None:
        self._selected = part_id
        self._redraw()

    # ── layout ───────────────────────────────────────────────────────────────

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
            # Use liaison edges for layout (undirected spring)
            edges = self._liaison_edges or self._prec_edges
            G.add_edges_from(edges)
            pos = nx.spring_layout(G, seed=42, k=1.5)
            for pid, (px, py) in pos.items():
                if pid in self._nodes:
                    self._nodes[pid]["x"] = cx + px * r
                    self._nodes[pid]["y"] = cy + py * r
        except ImportError:
            for i, pid in enumerate(self._nodes):
                angle = 2 * math.pi * i / n
                self._nodes[pid]["x"] = cx + r * math.cos(angle)
                self._nodes[pid]["y"] = cy + r * math.sin(angle)

        self._redraw()

    def _fit(self) -> None:
        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    # ── drawing ──────────────────────────────────────────────────────────────

    def _redraw(self) -> None:
        c = self._canvas
        c.delete("all")

        # 1. Subassembly hulls (bounding-box rectangles, drawn first / behind)
        for sub in self._subassemblies:
            xs = [self._nodes[p]["x"] for p in sub["part_ids"] if p in self._nodes]
            ys = [self._nodes[p]["y"] for p in sub["part_ids"] if p in self._nodes]
            if not xs:
                continue
            x0 = min(xs) - HULL_PADDING
            y0 = min(ys) - HULL_PADDING
            x1 = max(xs) + HULL_PADDING
            y1 = max(ys) + HULL_PADDING
            c.create_rectangle(
                x0, y0, x1, y1,
                outline=sub["color"], fill="", width=2, dash=HULL_DASH,
                tags="hull",
            )
            c.create_text(
                x0 + 4, y0 + 4,
                text=sub["label"], anchor=tk.NW,
                font=("Helvetica", 7), fill=sub["color"],
                tags="hull_label",
            )

        # 2. Liaison edges (undirected)
        if self._show_liaison.get():
            for (a, b) in self._liaison_edges:
                if a in self._nodes and b in self._nodes:
                    ax, ay = self._nodes[a]["x"], self._nodes[a]["y"]
                    bx, by = self._nodes[b]["x"], self._nodes[b]["y"]
                    c.create_line(ax, ay, bx, by,
                                  fill=EDGE_LIAISON, width=1.5, tags="edge_liaison")

        # 3. Precedence edges (directed arrows)
        if self._show_prec.get():
            for (a, b) in self._prec_edges:
                if a in self._nodes and b in self._nodes:
                    ax, ay = self._nodes[a]["x"], self._nodes[a]["y"]
                    bx, by = self._nodes[b]["x"], self._nodes[b]["y"]
                    # Shorten line to node border so arrow doesn't overlap circle
                    dx, dy = bx - ax, by - ay
                    dist = math.hypot(dx, dy) or 1.0
                    ex = bx - dx / dist * NODE_RADIUS
                    ey = by - dy / dist * NODE_RADIUS
                    c.create_line(
                        ax, ay, ex, ey,
                        fill=EDGE_PREC, width=1, arrow=tk.LAST,
                        arrowshape=(10, 12, 4),
                        tags="edge_prec",
                    )

        # 4. Nodes
        for pid, node in self._nodes.items():
            x, y = node["x"], node["y"]
            fill    = SEVERITY_FILL.get(node["severity"], SEVERITY_FILL["OK"])
            subasm_col = node.get("subasm_color", "")
            outline = "#ffffff" if pid == self._selected else (subasm_col or "#444455")
            width   = 3 if (pid == self._selected or subasm_col) else 1

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

    # ── interaction ──────────────────────────────────────────────────────────

    def _find_node_at(self, cx: float, cy: float) -> str | None:
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

#!/usr/bin/env python3
"""Plain 2D images of every part, to trace over in CAD.

    python3 frames/mark4-v2-10/trace_images.py

Writes trace/<part>.png next to this file: the part's outline and holes in
black on white, with two overall dimensions (width and height of the part) to
scale the image by. Each image is exactly 10 pixels per mm, so the scale can
also be set from the pixel size.

In SolidWorks: Tools > Sketch Tools > Sketch Picture, then drag the picture's
scale handle (or set its width) so the drawn dimension line matches its number.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import build as b

PX_PER_MM = 10
MARGIN = 22.0          # mm of paper round the part, room for the dimensions
OUT = b.HERE / "trace"


def render(p: b.Part, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # plates lie nose-right (long side across), the arm and camera plate as modelled
    nose_right = p.plane == "xy" and p.key != "arm"
    wires = [p.face.outerWire()] + p.face.innerWires()
    polys = []
    for w in wires:
        for e in w.Edges():
            q = b.edge_points(e, n=120)
            if nose_right:
                q = np.c_[q[:, 1], -q[:, 0]]
            polys.append(q)
    allp = np.vstack(polys)
    lo, hi = allp.min(0), allp.max(0)
    w_mm, h_mm = hi - lo
    W, H = w_mm + 2 * MARGIN, h_mm + 2 * MARGIN
    fig = plt.figure(figsize=(W / 25.4, H / 25.4), dpi=PX_PER_MM * 25.4)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(lo[0] - MARGIN, hi[0] + MARGIN)
    ax.set_ylim(lo[1] - MARGIN, hi[1] + MARGIN)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor("white")
    for q in polys:
        ax.plot(q[:, 0], q[:, 1], c="black", lw=1.0, solid_capstyle="round")

    blue = "#1040c0"

    def dim(a, b_, text, vertical):
        ax.annotate("", xy=a, xytext=b_, arrowprops=dict(arrowstyle="<|-|>", color=blue, lw=0.9,
                                                         mutation_scale=9, shrinkA=0, shrinkB=0))
        mid = ((a[0] + b_[0]) / 2, (a[1] + b_[1]) / 2)
        if vertical:
            ax.text(mid[0] + 2.0, mid[1], text, color=blue, fontsize=9, rotation=90, ha="left", va="center",
                    bbox=dict(fc="white", ec="none", pad=0.5))
        else:
            ax.text(mid[0], mid[1] + 1.8, text, color=blue, fontsize=9, ha="center", va="bottom",
                    bbox=dict(fc="white", ec="none", pad=0.5))

    # extension lines and the two overall dimensions: width along the top, height on the right
    yt = hi[1] + 9
    xr = hi[0] + 9
    # extension lines start at the part's own farthest points, so they visibly touch its ends
    for i in (allp[:, 0].argmin(), allp[:, 0].argmax()):
        x, y = allp[i]
        ax.plot([x, x], [y + 0.8, yt + 2], c=blue, lw=0.5)
    for i in (allp[:, 1].argmin(), allp[:, 1].argmax()):
        x, y = allp[i]
        ax.plot([x + 0.8, xr + 2], [y, y], c=blue, lw=0.5)
    dim((lo[0], yt), (hi[0], yt), f"{w_mm:.2f} mm", False)
    dim((xr, lo[1]), (xr, hi[1]), f"{h_mm:.2f} mm", True)
    ax.text(lo[0] - MARGIN + 3, lo[1] - MARGIN + 3,
            f"{p.title} x{p.qty}\n{b.fg(p.t)} mm thick, {PX_PER_MM} px/mm", fontsize=7, color="#333")
    fig.savefig(path, dpi=PX_PER_MM * 25.4, facecolor="white")
    plt.close(fig)
    return w_mm, h_mm


def main():
    b.extract(b.STEP)
    OUT.mkdir(exist_ok=True)
    for p in b.PARTS:
        w, h = render(p, OUT / f"{p.key}.png")
        print(f"{p.key:12s} {w:7.2f} x {h:7.2f} mm")


if __name__ == "__main__":
    main()

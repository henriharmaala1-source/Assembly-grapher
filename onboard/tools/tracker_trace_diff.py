#!/usr/bin/env python3
"""Compare two tracker_bench traces: per-frame state, box, conf, prediction.

    python3 tools/tracker_trace_diff.py old.bin new.bin

For changes that only move floating-point rounding (they cannot be byte-
identical): the bar is the same state on every frame and sub-pixel deltas.
"""
import struct
import sys

REC = struct.Struct('<iiiiifffff')   # state x y w h conf predX predY aimX aimY


def load(path):
    data = open(path, 'rb').read()
    return [REC.unpack_from(data, i * REC.size) for i in range(len(data) // REC.size)]


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    if len(a) != len(b):
        sys.exit(f"different frame counts: {len(a)} vs {len(b)}")
    states = sum(x[0] != y[0] for x, y in zip(a, b))
    boxes = sum(x[1:5] != y[1:5] for x, y in zip(a, b))
    box_px = max(max(abs(p - q) for p, q in zip(x[1:5], y[1:5])) for x, y in zip(a, b))
    conf = max(abs(x[5] - y[5]) for x, y in zip(a, b))
    pred = max(max(abs(p - q) for p, q in zip(x[6:], y[6:])) for x, y in zip(a, b))
    print(f"{len(a)} frames: state differs on {states}, box on {boxes} (max {box_px} px), "
          f"max conf diff {conf:.2e}, max prediction/aim diff {pred:.4f} px")
    sys.exit(1 if states else 0)


if __name__ == '__main__':
    main()

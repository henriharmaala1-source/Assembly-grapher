#!/usr/bin/env python3
"""Quantify "spinning between gaps" from a voxel_sim --csv log.

The user's complaint was behavioural -- "now it's mostly spinning between
gaps" -- and behavioural complaints need a number before they can be fixed,
otherwise you tune weights until the last run you watched looks nice. Three
numbers, each measuring a different way a run can be bad:

  churn      mean |dyaw| per step, deg. Pure indecision cost.
  reversals  fraction of steps where the heading swung back the other way by
             more than 20 deg. A single big turn is navigation; a turn
             immediately undone is oscillation, and only this metric tells
             them apart.
  advance    net displacement / distance travelled. The bottom line: 1.0 is a
             straight line, 0.0 is a closed loop. This is what "spinning"
             actually costs.
"""
import csv, math, sys


def wrap(d):
    while d > 180: d -= 360
    while d < -180: d += 360
    return d


def analyse(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 3:
        return None
    yaw = [float(r["yaw"]) for r in rows]
    e = [float(r["e"]) for r in rows]
    n = [float(r["n"]) for r in rows]

    d = [wrap(yaw[i + 1] - yaw[i]) for i in range(len(yaw) - 1)]
    churn = sum(abs(x) for x in d) / len(d)
    # A reversal needs both swings to be substantial, else slow drift through
    # zero counts as oscillation and the metric saturates on every run.
    rev = sum(1 for i in range(len(d) - 1)
              if abs(d[i]) > 2 and abs(d[i + 1]) > 2 and d[i] * d[i + 1] < 0
              and abs(d[i]) + abs(d[i + 1]) > 20)
    travelled = sum(math.hypot(e[i + 1] - e[i], n[i + 1] - n[i])
                    for i in range(len(e) - 1))
    net = math.hypot(e[-1] - e[0], n[-1] - n[0])
    return dict(steps=len(rows), churn=churn,
                reversals=rev / max(1, len(d)),
                travelled=travelled, net=net,
                advance=net / max(1e-6, travelled))


if __name__ == "__main__":
    print(f"{'run':<24}{'steps':>6}{'churn/step':>12}{'reversals':>11}"
          f"{'travel':>9}{'net':>8}{'advance':>9}")
    for p in sys.argv[1:]:
        s = analyse(p)
        if not s:
            print(f"{p:<24}  (too short)")
            continue
        print(f"{p.split('/')[-1]:<24}{s['steps']:>6}{s['churn']:>12.2f}"
              f"{s['reversals']:>10.1%}{s['travelled']:>9.1f}"
              f"{s['net']:>8.1f}{s['advance']:>9.2f}")

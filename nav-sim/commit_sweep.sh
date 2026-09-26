#!/usr/bin/env bash
# How long should the reactive planner hold a heading before re-deciding?
#
# Commitment is not free. It removes almost all the yaw churn -- on the forest,
# 28.4 deg/step down to 11.3, and reversals from 5.1% of steps to 0.5% -- but at
# commitSteps = 5 it also costs progress, advance 0.616 -> 0.450, with the
# per-seed ranges not overlapping across four seeds. That is a real trade-off
# rather than noise, and it has an obvious mechanism: a corridor with bends
# needs steering, and a held heading overshoots them.
#
# So sweep the hold length rather than picking a round number. Both metrics are
# reported because optimising either alone gives a useless aircraft: hold
# forever and it flies beautifully straight into a tree, re-decide every frame
# and it spins in place.
set -u
BIN="${BIN:-./build/voxel_sim}"
SEEDS="${SEEDS:-1 2 3 4}"
STEPS="${STEPS:-600}"
HOLDS="${HOLDS:-0 1 2 3 5 8}"
WORLDFLAGS="${WORLDFLAGS:---world forest --cell 0.25 --goal 120 150 8}"
[ -x "$BIN" ] || { echo "no binary at $BIN -- build first"; exit 1; }

printf "%-7s %8s %10s %9s %9s %6s\n" hold churn reversals advance range coll
for h in $HOLDS; do
  csvs=""
  coll=0
  for s in $SEEDS; do
    c="/tmp/cs_${h}_$s.csv"
    $BIN $WORLDFLAGS --steps "$STEPS" --seed "$s" --commit "$h" \
         --csv "$c" --out /tmp/cs 2>&1 | grep -q COLLIDED && coll=$((coll+1))
    csvs="$csvs $c"
  done
  python3 - "$h" "$coll" $csvs <<'PY'
import csv, math, statistics as st, sys
h, coll, paths = sys.argv[1], sys.argv[2], sys.argv[3:]
def wrap(d):
    while d > 180: d -= 360
    while d < -180: d += 360
    return d
ch, rv, ad = [], [], []
for p in paths:
    try: r = list(csv.DictReader(open(p)))
    except OSError: continue
    y = [float(x['yaw']) for x in r if x['yaw']]
    if len(y) < 3: continue
    d = [wrap(y[i+1] - y[i]) for i in range(len(y)-1)]
    ch.append(sum(abs(x) for x in d) / len(d))
    # A reversal needs both swings substantial, else slow drift through zero
    # counts as oscillation and the metric saturates on every run.
    rv.append(sum(1 for i in range(len(d)-1)
                  if abs(d[i]) > 2 and abs(d[i+1]) > 2 and d[i]*d[i+1] < 0
                  and abs(d[i]) + abs(d[i+1]) > 20) / len(d))
    e = [float(x['e']) for x in r]; n = [float(x['n']) for x in r]
    t = sum(math.hypot(e[i+1]-e[i], n[i+1]-n[i]) for i in range(len(e)-1))
    ad.append(math.hypot(e[-1]-e[0], n[-1]-n[0]) / max(1e-6, t))
if ch:
    print(f"{h:<7} {st.mean(ch):8.2f} {st.mean(rv):9.1%} {st.mean(ad):9.3f}"
          f"  [{min(ad):.2f},{max(ad):.2f}] {coll:>5}")
PY
done

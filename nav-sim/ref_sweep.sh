#!/usr/bin/env bash
# Does fixing the REFERENCE bearing fix the "spinning"?
#
# The reactive planner was accused of spinning; its commanded bearing churns
# 20.9 deg/step. The goal bearing handed to it churns 21.5. It is tracking a
# wobbling reference, so three fixes were made upstream of it:
#
#   pursuit  aim at a fixed arclength along the path instead of at "the first
#            waypoint past 3 m", which hopped every few steps
#   reuse    replan when the path stops being usable, not every 25 steps with a
#            randomised planner that returns something different each time
#   filter   low-pass the reference bearing (safe: it is a reference, not a
#            measurement -- speed is still gated on the live map)
#
# Reports GOAL churn as well as CMD churn, because the whole claim is that the
# second follows the first. If goal churn drops and command churn does not, the
# diagnosis was wrong and this script says so.
set -u
BIN="${BIN:-./build/voxel_sim}"
SEEDS="${SEEDS:-1 2 3 4}"
STEPS="${STEPS:-600}"
[ -x "$BIN" ] || { echo "no binary at $BIN -- build first"; exit 1; }

run() {
  local label="$1"; shift
  local csvs="" coll=0
  for s in $SEEDS; do
    local c="/tmp/rs_${label//[^a-zA-Z0-9]/_}_$s.csv"
    $BIN --world forest --cell 0.25 --steps "$STEPS" --seed "$s" \
         --goal 120 150 8 --csv "$c" --out /tmp/rs "$@" 2>&1 | grep -q COLLIDED && coll=$((coll+1))
    csvs="$csvs $c"
  done
  python3 - "$label" "$coll" $csvs <<'PY'
import csv, math, statistics as st, sys
label, coll, paths = sys.argv[1], sys.argv[2], sys.argv[3:]
def wrap(d):
    while d > 180: d -= 360
    while d < -180: d += 360
    return d
G, C, A = [], [], []
for p in paths:
    try: r = [x for x in csv.DictReader(open(p)) if x.get('goalAz')]
    except OSError: continue
    if len(r) < 3: continue
    g = [float(x['goalAz']) for x in r]; c = [float(x['cmdAz']) for x in r]
    G.append(sum(abs(wrap(g[i+1]-g[i])) for i in range(len(g)-1)) / (len(g)-1))
    C.append(sum(abs(wrap(c[i+1]-c[i])) for i in range(len(c)-1)) / (len(c)-1))
    e = [float(x['e']) for x in r]; n = [float(x['n']) for x in r]
    t = sum(math.hypot(e[i+1]-e[i], n[i+1]-n[i]) for i in range(len(e)-1))
    A.append(math.hypot(e[-1]-e[0], n[-1]-n[0]) / max(1e-6, t))
if G:
    print(f"{label:<26}{st.mean(G):8.2f}{st.mean(C):8.2f}{st.mean(A):9.3f}"
          f"  [{min(A):.2f},{max(A):.2f}]{coll:>5}")
PY
}

printf "%-26s%8s%8s%9s%9s%5s\n" arm goalChurn cmdChurn advance range coll
run "none (as before)"      --lookahead 0 --goalema 1 --noreuse
run "pursuit only"          --lookahead 6 --goalema 1 --noreuse
run "reuse only"            --lookahead 0 --goalema 1
run "filter only"           --lookahead 0 --goalema 0.25 --noreuse
run "pursuit + reuse"       --lookahead 6 --goalema 1
run "all three"             --lookahead 6 --goalema 0.25

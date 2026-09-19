#!/usr/bin/env bash
# One-factor-at-a-time ablation of the steering knobs, over several seeds.
#
# WHY. Three anti-churn mechanisms (field EMA, dwell margin, reverse penalty)
# were added in one commit and the batch made progress WORSE -- advance ratio
# 0.60 -> 0.47 on the seed that was checked. A batch that moves the number the
# wrong way is uninterpretable: any one of the three could be carrying the loss
# while the other two help. So vary one at a time, and over more than one world,
# because a single forest is an anecdote and this project has already been
# burned by that twice.
#
# The reported metric is ADVANCE = net displacement / distance travelled. It is
# the one that captures the complaint: spinning is motion that does not get you
# anywhere, and advance is exactly the fraction of motion that does.
set -u
BIN="${BIN:-./build/voxel_sim}"
SEEDS="${SEEDS:-1 2 3}"
STEPS="${STEPS:-600}"
[ -x "$BIN" ] || { echo "no binary at $BIN -- build first"; exit 1; }

run() {   # run <label> <extra flags...>
  local label="$1"; shift
  local tot=0 n=0 coll=0
  for s in $SEEDS; do
    local csv="/tmp/abl_${label//[^a-zA-Z0-9]/_}_$s.csv"
    local o
    o=$($BIN --world forest --cell 0.25 --steps "$STEPS" --seed "$s" \
             --goal 120 150 8 --csv "$csv" --out /tmp/abl "$@" 2>&1)
    echo "$o" | grep -q COLLIDED && coll=$((coll+1))
    local a
    a=$(python3 - "$csv" <<'PY'
import csv,math,sys
r=list(csv.DictReader(open(sys.argv[1])))
e=[float(x['e']) for x in r]; n=[float(x['n']) for x in r]
t=sum(math.hypot(e[i+1]-e[i],n[i+1]-n[i]) for i in range(len(e)-1))
print(f"{math.hypot(e[-1]-e[0],n[-1]-n[0])/max(1e-6,t):.3f}")
PY
)
    tot=$(python3 -c "print($tot+$a)"); n=$((n+1))
  done
  printf "%-26s advance %s   collisions %d/%d\n" "$label" \
         "$(python3 -c "print(f'{$tot/$n:.3f}')")" "$coll" "$n"
}

# NOTE ON COMPARING ACROSS COMMITS. Adding trails to genForest changed what
# `--seed 1` generates -- trail generation draws from the same RNG before the
# stem loop, so every tree moved. Numbers from before that change are NOT
# comparable to numbers after it, however identical the flags look. Re-run the
# arms you care about rather than reading two tables side by side.
echo "forest, $STEPS steps, seeds: $SEEDS"
run "baseline (all off)"      --ema 1.0 --dwell 0 --revpen 0 --commit 0
run "commit only"             --ema 1.0 --dwell 0 --revpen 0 --commit 5
run "commit + ema"            --ema 0.3 --dwell 0 --revpen 0 --commit 5
run "commit + dwell"          --ema 1.0 --dwell 0.12 --revpen 0 --commit 5
run "commit + revpen"         --ema 1.0 --dwell 0 --revpen 1.2 --commit 5
run "all three"               --ema 0.3 --dwell 0.12 --revpen 1.2 --commit 5

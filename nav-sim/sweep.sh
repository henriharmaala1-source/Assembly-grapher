#!/usr/bin/env bash
# Batch runner: every world x every seed, headless, one summary table.
#
# WHY THIS EXISTS. Until now `seed` was hard-coded to 1, so every result ever
# quoted from this harness was one sample of one world presented as a property
# of the planner. This session has already been burned twice by exactly that
# mistake -- a tracker battery whose noise floor was 18 points, and a collision
# detector that could not see trees. A single run is an anecdote.
#
#   ./sweep.sh                    # 2 worlds x 5 seeds, stereo
#   ./sweep.sh --truth 8          # perfect-depth control, 8 seeds
#   ./sweep.sh "" 5 forest        # one world only
#
# Windows: run under Git Bash / WSL, or read the loop and do the same in
# PowerShell -- it is just repeated invocations of voxel_sim.exe.
set -u
BIN="${BIN:-./build/voxel_sim}"
DEPTH="${1:-}"          # "" for stereo, --truth for the control
SEEDS="${2:-5}"
WORLDS="${3:-forest city}"
STEPS="${STEPS:-900}"

[ -x "$BIN" ] || { echo "no binary at $BIN -- build first"; exit 1; }

printf "%-8s %-5s %-16s %9s %9s %9s %10s\n" \
       world seed outcome travel end-dist minClr falseFree
tot=0; hits=0; reach=0
for w in $WORLDS; do
  cell=0.25; goal="120 150 8"
  [ "$w" = city ] && { cell=0.4; goal="160 190 8"; }
  for s in $(seq 1 "$SEEDS"); do
    o=$($BIN --world "$w" --cell $cell --steps "$STEPS" --seed "$s" \
             --goal $goal $DEPTH --out "/tmp/sw_${w}_$s" 2>&1)
    out=$(echo "$o" | grep -oE 'COLLIDED|reached goal|ran out of steps' | head -1)
    printf "%-8s %-5s %-16s %9s %9s %9s %10s\n" "$w" "$s" "$out" \
      "$(echo "$o"|grep 'path travelled'|awk '{print $3}')" \
      "$(echo "$o"|grep 'distance to goal'|awk '{print $4}')" \
      "$(echo "$o"|grep 'min true'|awk '{print $4}')" \
      "$(echo "$o"|grep 'map false-free'|awk '{print $3}')"
    tot=$((tot+1))
    [ "$out" = "COLLIDED" ] && hits=$((hits+1))
    [ "$out" = "reached goal" ] && reach=$((reach+1))
  done
done
echo "---"
echo "runs $tot   collisions $hits   goals reached $reach"
echo "collision rate: $(python3 -c "print(f'{100*$hits/max(1,$tot):.0f}%')")"

#!/usr/bin/env bash
# The downward-VIO grid (downvio_sweep.cpp): every condition x 4 seeds, run
# in parallel, one CSV.   nav-sim/test/downvio_sweep.sh BUILD_DIR OUT.csv [JOBS]
set -euo pipefail
B=${1:?build dir with downvio_sweep}; OUT=${2:?out.csv}; J=${3:-$(nproc)}
T=$B/downvio_sweep
jobs=$(mktemp)
add() { for s in 1 2 3 4; do echo "$* --seed $s" >> "$jobs"; done; }
# A: tilt x field of view, both depth sources (2 m/s, 1.5 m, flat).
for d in plane stereo; do for fov in 60 90 120 140; do for tilt in 0 10 20 30 40; do
  add --block A --depth $d --fov $fov --tilt $tilt --dist 20
done; done; done
# B: rotation rate -- roll/pitch wobble on a 15 deg cruise tilt.
for fov in 60 90 140; do for wob in 10 20; do
  add --block B --depth plane --fov $fov --tilt 15 --wobble $wob --wobble-hz 1.5 --dist 20
done; done
# C: realistic FPV cruise, tilt and speed together, 5 m up, 60 fps.
for fov in 60 90 140; do for ts in "10 5" "20 10" "30 14" "40 18"; do set -- $ts
  add --block C --depth plane --fov $fov --tilt $1 --speed $2 --alt 5 --fps 60 --dist 60
done; done
# D: height.
for fov in 60 140; do for alt in 1.5 5 15; do
  add --block D --depth plane --fov $fov --tilt 20 --speed 4 --alt $alt --dist 40
done; done
# E: the ground -- clutter breaks the flat-ground assumption; lowtex is smooth.
for fov in 60 140; do for tilt in 0 30; do for d in plane stereo; do
  add --block E --depth $d --world clutter --fov $fov --tilt $tilt --dist 20
done; done; add --block E --depth plane --world lowtex --fov $fov --tilt 20 --dist 20; done
# F: an IMU attitude BIAS (on roll and pitch both) -- what an EKF's tilt
# estimate is off by in accelerating flight.
for d in plane stereo; do for fov in 60 140; do for b in 0.5 1 2; do
  add --block F --depth $d --fov $fov --tilt 0 --att-bias $b --dist 20
done; done; done
echo "$(wc -l < "$jobs") runs on $J cores"
{ echo "block,$($T --header)"; } > "$OUT"
# Each line: the block tag first (not an option of the tool), then the args.
xargs -P "$J" -L 1 bash -c 'blk=$1; shift; echo "$blk,$('"$T"' "$@")"' _ < <(sed 's/^--block //' "$jobs") >> "$OUT"
rm -f "$jobs"
echo "wrote $OUT"

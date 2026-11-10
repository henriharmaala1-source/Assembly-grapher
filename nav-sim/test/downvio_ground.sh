#!/usr/bin/env bash
# Downward VIO over REAL ground (ground_texture.hpp): every condition x 4
# seeds, parallel, one CSV.  Run from the repo root after
# nav-sim/textures/fetch_ground_textures.sh.
#   nav-sim/test/downvio_ground.sh BUILD_DIR OUT.csv [JOBS]
set -euo pipefail
B=${1:?build dir with downvio_sweep}; OUT=${2:?out.csv}; J=${3:-$(nproc)}
T=$B/downvio_sweep
jobs=$(mktemp)
add() { for s in 1 2 3 4; do echo "$* --seed $s" >> "$jobs"; done; }
# G: the ground x height x lens (20 deg tilt, 4 m/s, mono + rangefinder).
for g in noise lawn field gravel dirt sand paving park; do
  for alt in 1.5 5 15; do for fov in 60 120; do
    add --block G --ground $g --alt $alt --fov $fov --tilt 20 --speed 4 --dist 20
  done; done
done
# L: light. A high sun puts the aircraft's own shadow in view of a wide lens
# (a feature that moves WITH the aircraft); a low sun is long object shadows.
for g in field paving; do
  add --block L --ground $g --alt 1.5 --fov 120 --tilt 20 --speed 4 --dist 20 --sun-elev 70
  add --block L --ground $g --alt 1.5 --fov 120 --tilt 20 --speed 4 --dist 20 --sun-elev 70 --drone-shadow
  add --block L --ground $g --alt 1.5 --fov 120 --tilt 20 --speed 4 --dist 20 --sun-elev 20
done
# M: motion blur at FPV speed (exposure 2 ms is bright day, 6 ms overcast).
for fov in 60 120; do for e in 0 2 6; do
  add --block M --ground field --alt 5 --fov $fov --tilt 30 --speed 12 --fps 60 --dist 50 --exposure-ms $e
done; done
echo "$(wc -l < "$jobs") runs on $J cores"
{ echo "block,$($T --header)"; } > "$OUT"
xargs -P "$J" -L 1 bash -c 'blk=$1; shift; echo "$blk,$('"$T"' "$@")"' _ < <(sed 's/^--block //' "$jobs") >> "$OUT"
rm -f "$jobs"
echo "wrote $OUT"
